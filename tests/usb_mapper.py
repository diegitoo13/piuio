#!/usr/bin/env python3
import argparse
import sys
import time
import json
import os
import usb.core
import usb.util
from pynput import keyboard

# --- Configuration ---
DEFAULT_VID = 0x0D2F  # Pump It Up IO board VID
DEFAULT_PID = 0x1020  # Pump It Up IO board PID
POLL_REPORT_ID = 0x01 # Report ID to poll via GET_FEATURE
POLL_REPORT_SIZE = 258 # Expected size of the report
INTERFACE = 0          # USB Interface number
TIMEOUT_MS = 500       # USB control transfer timeout
DEFAULT_POLL_INTERVAL_S = 0.005 # Default polling frequency (seconds). Lower values increase CPU usage significantly.
DEVICE_CHECK_INTERVAL_S = 5.0 # How often to check for the device if not found (seconds)
MAPPINGS_FILE = 'key_mappings.json' # File to store key bindings

# --- Global Variables ---
# Used by pynput listener in mapper mode
pressed_key = None
listener_running = False
keyboard_controller = keyboard.Controller()
# Global to store the actual poll interval used, potentially overridden by args
poll_interval_s = DEFAULT_POLL_INTERVAL_S
# Pre-calculate wValue for get_feature_report for minor optimization
GET_FEATURE_WVALUE = (0x03 << 8) | POLL_REPORT_ID

# --- Utility Functions ---

def find_device(vid, pid):
    """Find the USB device by VID and PID. Returns device object or None."""
    # Returns None if not found, doesn't exit the script anymore
    return usb.core.find(idVendor=vid, idProduct=pid)

def detach_kernel_driver(dev):
    """Detach kernel driver if necessary (Linux). Returns True if detached, False otherwise."""
    try:
        if dev.is_kernel_driver_active(INTERFACE):
            print(f"Detaching kernel driver from interface {INTERFACE}...")
            dev.detach_kernel_driver(INTERFACE)
            print("Kernel driver detached.")
            return True
    except usb.core.USBError as e:
        # Log error but don't exit, maybe permissions issue
        print(f"Warning: Could not detach kernel driver: {e}. Check permissions/udev rules.")
    except (NotImplementedError, AttributeError):
        # This is expected on macOS and Windows
        # print("Kernel driver detachment not applicable on this OS.") # Less verbose
        pass
    return False

def attach_kernel_driver(dev):
    """Reattach kernel driver if it was detached."""
    # Note: Reattaching automatically might not always be desired or work correctly.
    try:
        print(f"Attempting to reattach kernel driver to interface {INTERFACE}...")
        dev.attach_kernel_driver(INTERFACE)
        print("Kernel driver reattached.")
    except usb.core.USBError as e:
        print(f"Warning: Could not reattach kernel driver: {e}")
    except (NotImplementedError, AttributeError):
         # This is expected on macOS and Windows
        pass


def set_idle(dev):
    """Send SET_IDLE request. Returns True on success or expected non-fatal errors, False otherwise."""
    try:
        dev.ctrl_transfer(0x21, 0x0A, 0, INTERFACE, None, TIMEOUT_MS)
        print("SET_IDLE request sent successfully.")
        return True
    except usb.core.USBError as e:
        if e.errno in [1, 5, 19, 32]: # Operation not permitted, I/O error, No such device, Pipe error
             print(f"Warning: SET_IDLE failed or may not be supported (errno={e.errno}). Continuing...")
             return True # Treat as non-fatal for initialization sequence
        else:
            print(f"USB Error during SET_IDLE: {e}")
            # Consider other errors potentially fatal for this device
            return False # Indicate a more serious failure

def get_feature_report(dev, size):
    """Get a Feature Report. Raises USBError on failure (e.g., disconnect)."""
    # Let the calling function handle exceptions like disconnection
    try:
        data = dev.ctrl_transfer(0xA1, 0x01, GET_FEATURE_WVALUE, INTERFACE, size, TIMEOUT_MS)
        return data
    except usb.core.USBError as e:
        # Reraise the error to be caught by the main loop
        raise e

def hexdump(buf, cols=32):
    """Return a string representation of the buffer in hexadecimal format."""
    if not buf:
        return "<empty>"
    it = (buf[i:i+cols] for i in range(0, len(buf), cols))
    lines = []
    for i, chunk in enumerate(it):
        hex_part = ' '.join(f'{b:02X}' for b in chunk)
        hex_part = hex_part.ljust(cols * 3 - 1)
        lines.append(f"{i * cols:04X}   {hex_part}")
    return '\n'.join(lines)

def load_mappings(filename):
    """Load key mappings from a JSON file."""
    if not os.path.exists(filename):
        print(f"Mapping file '{filename}' not found. Starting with no mappings.")
        return {}
    try:
        with open(filename, 'r') as f:
            data = json.load(f)
            mappings = {}
            for k, v in data.items():
                 try:
                     byte_index, byte_value = map(int, k.split(':'))
                     key_repr = v
                     if key_repr.startswith("Key."):
                          key_name = key_repr.split('.')[1]
                          if hasattr(keyboard.Key, key_name):
                              key_obj = getattr(keyboard.Key, key_name)
                          else:
                              print(f"Warning: Skipping unrecognized special key '{key_repr}' in mapping file.")
                              continue
                     elif len(key_repr) == 1:
                          key_obj = key_repr
                     elif key_repr.startswith('<') and key_repr.endswith('>'):
                          potential_key_name = key_repr[1:-1]
                          if hasattr(keyboard.Key, potential_key_name):
                               key_obj = getattr(keyboard.Key, potential_key_name)
                          else:
                               print(f"Warning: Skipping unrecognized key representation '{key_repr}'")
                               continue
                     else:
                          print(f"Warning: Skipping unrecognized key mapping value '{key_repr}'")
                          continue
                     mappings[(byte_index, byte_value)] = key_obj
                 except (ValueError, AttributeError, KeyError) as inner_e:
                     print(f"Warning: Skipping invalid mapping entry '{k}': {v}. Error: {inner_e}")
                     continue
            print(f"Loaded {len(mappings)} key mappings from {filename}")
            return mappings
    except (json.JSONDecodeError, IOError) as e:
        print(f"Warning: Could not load or parse mappings from {filename}. Starting fresh. Error: {e}")
        return {}

def save_mappings(mappings, filename):
    """Save key mappings to a JSON file."""
    try:
        serializable_mappings = {}
        for (byte_index, byte_value), key_obj in mappings.items():
            key_str = f"{byte_index}:{byte_value}"
            if isinstance(key_obj, keyboard.Key):
                 val_str = str(key_obj)
            elif isinstance(key_obj, str):
                 val_str = key_obj
            else:
                 print(f"Warning: Cannot serialize unexpected key object type: {type(key_obj)}. Skipping mapping for {key_str}.")
                 continue
            serializable_mappings[key_str] = val_str
        with open(filename, 'w') as f:
            json.dump(serializable_mappings, f, indent=4)
        print(f"Saved {len(mappings)} key mappings to {filename}")
    except IOError as e:
        print(f"Error saving mappings to {filename}: {e}")
    except Exception as e:
        print(f"An unexpected error occurred during mapping saving: {e}")


# --- Keyboard Listener Callbacks (for Mapper Mode) ---

def on_press(key):
    """Callback function for pynput listener when a key is pressed."""
    global pressed_key, listener_running
    pressed_key = key
    print(f"\n   Captured key: {key}")
    listener_running = False
    return False # Stops the listener

def wait_for_key_press():
    """Starts the keyboard listener and waits for a single key press."""
    global pressed_key, listener_running
    pressed_key = None
    listener_running = True
    print("   Waiting for key press...")
    # Use a blocking listener for simplicity
    listener = keyboard.Listener(on_press=on_press)
    listener.start()
    listener.join() # Blocks until listener stops
    return pressed_key


# --- Mode Functions ---

def run_mapper_mode(dev, mappings):
    """Run the key mapping mode. Raises USBError on device communication failure."""
    print("\n--- Mapper Mode ---")
    print("Detecting changes in USB report data.")
    print("When a change is detected, you will be prompted to press a key to map.")
    print("Press Ctrl+C to exit mapper mode.")

    last_data = None
    consecutive_errors = 0 # Track errors within this mode's loop

    # Get initial state
    try:
        last_data = get_feature_report(dev, POLL_REPORT_SIZE)
        if last_data:
            print("Got initial data. Waiting for changes...")
            print("Initial data (first 64 bytes):")
            print(hexdump(last_data[:64], cols=32))
        else:
            print("Warning: Failed to get initial data.")
            # Decide if we should retry or raise error immediately
            time.sleep(1) # Wait before potentially trying again in the loop
    except usb.core.USBError as e:
        print(f"Error getting initial data in mapper mode: {e}")
        raise # Reraise to be caught by main loop for disconnect handling

    while True: # Loop within the mode until Ctrl+C or USB error
        try:
            current_data = get_feature_report(dev, POLL_REPORT_SIZE)
            # If get_feature_report succeeded, reset consecutive error count
            consecutive_errors = 0

            if last_data is None: # If initial read failed, try again
                last_data = current_data
                if last_data:
                     print("Got initial data. Waiting for changes...")
                     print("Initial data (first 64 bytes):")
                     print(hexdump(last_data[:64], cols=32))
                time.sleep(poll_interval_s)
                continue

            if current_data != last_data:
                timestamp = time.strftime('%H:%M:%S.%f')[:-3]
                changed_indices = [i for i, (a, b) in enumerate(zip(current_data, last_data)) if a != b]

                if not changed_indices:
                     last_data = current_data
                     time.sleep(poll_interval_s)
                     continue

                print(f"\n[{timestamp}] Change detected!")
                print("Current data (first 64 bytes):")
                print(hexdump(current_data[:64], cols=32))
                print(f"Changed byte indices: {changed_indices}")

                for index in changed_indices:
                    if index >= len(last_data) or index >= len(current_data):
                        print(f"Warning: Change detected at index {index} which is out of bounds. Skipping.")
                        continue

                    old_val = last_data[index]
                    new_val = current_data[index]
                    map_key = (index, new_val)

                    print(f"\n -> Change at index {index}: {old_val:02X} -> {new_val:02X}")

                    current_mapping = mappings.get(map_key)
                    if current_mapping is not None:
                        print(f"    (Already mapped to: {current_mapping})")
                        while True:
                            action = input("    Remap? (y/n/s=skip all changes in this batch): ").lower().strip()
                            if action in ['y', 'n', 's']:
                                break
                            print("    Invalid input. Please enter 'y', 'n', or 's'.")
                        if action == 's': break
                        if action == 'n': continue

                    print(f"    Press the keyboard key to map to this change (Index={index}, New Value={new_val:02X})...")
                    target_key = wait_for_key_press() # This blocks until key press

                    if target_key:
                        mappings[map_key] = target_key
                        print(f"    Mapped: Byte {index} changing to {new_val:02X} ==> Key '{target_key}'")
                        save_mappings(mappings, MAPPINGS_FILE)
                    else:
                        print("    No key captured. Mapping cancelled.")

                last_data = current_data

            time.sleep(poll_interval_s)

        except usb.core.USBError as e:
            # Handle potential errors during the poll
            consecutive_errors += 1
            print(f"Warning: USB read error in mapper loop (Attempt {consecutive_errors}): {e}")
            if e.errno == 19: # No such device (disconnected)
                print("Device disconnected during mapping.")
                raise # Reraise to be handled by the main loop
            if consecutive_errors > 10:
                print("Too many consecutive read errors. Assuming device issue.")
                raise # Reraise to be handled by the main loop
            time.sleep(0.5) # Wait longer after an error
        except KeyboardInterrupt:
            print("\nExiting mapper mode (Ctrl+C detected).")
            break # Exit the inner mapper loop, goes back to main loop (which will likely exit)
        except Exception as e:
            print(f"\nAn unexpected error occurred in mapper mode: {e}")
            import traceback
            traceback.print_exc()
            # Decide whether to break or try to continue
            break # Exit mapper mode on unexpected errors


def run_driver_mode(dev, mappings):
    """Run the driver mode. Raises USBError on device communication failure."""
    print("\n--- Driver Mode ---")
    if not mappings:
        print("Warning: No key mappings loaded.")
    else:
        print(f"Loaded {len(mappings)} mappings. Monitoring device...")
    print("Press Ctrl+C to exit driver mode.")

    last_data = None
    active_presses = set()
    consecutive_errors = 0

    # Get initial state
    try:
        last_data = get_feature_report(dev, POLL_REPORT_SIZE)
        if last_data:
             print("Initial device state read successfully.")
             # Optional: Check initial state against mappings and press keys?
             # This might be complex if multiple mappings match initial state.
             # For simplicity, we usually only react to *changes*.
        else:
             print("Warning: Failed to get initial data for driver mode.")
             time.sleep(1)
    except usb.core.USBError as e:
        print(f"Error getting initial data in driver mode: {e}")
        raise # Reraise to be caught by main loop

    try:
        while True: # Loop within the mode until Ctrl+C or USB error
            try:
                current_data = get_feature_report(dev, POLL_REPORT_SIZE)
                consecutive_errors = 0 # Reset error count on success

                if last_data is None: # If initial read failed
                    last_data = current_data
                    time.sleep(poll_interval_s)
                    continue

                if current_data != last_data:
                    pressed_this_cycle = set()

                    for (byte_index, target_value), key_to_press in mappings.items():
                        if byte_index < len(current_data):
                            current_value = current_data[byte_index]
                            if current_value == target_value:
                                pressed_this_cycle.add(key_to_press)
                                if key_to_press not in active_presses:
                                    try:
                                        keyboard_controller.press(key_to_press)
                                        active_presses.add(key_to_press)
                                    except Exception as e:
                                         print(f"Warning: Error pressing key {key_to_press}: {e}")

                    keys_to_release = active_presses - pressed_this_cycle
                    for key_to_release in keys_to_release:
                         try:
                              keyboard_controller.release(key_to_release)
                              if key_to_release in active_presses:
                                   active_presses.remove(key_to_release)
                         except Exception as e:
                              print(f"Warning: Error releasing key {key_to_release}: {e}")
                              if key_to_release in active_presses:
                                   active_presses.remove(key_to_release)

                    last_data = current_data

                time.sleep(poll_interval_s)

            except usb.core.USBError as e:
                consecutive_errors += 1
                # print(f"Warning: USB read error in driver loop (Attempt {consecutive_errors}): {e}") # Verbose
                if e.errno == 19: # No such device (disconnected)
                    print("Device disconnected.")
                    raise # Reraise to be handled by the main loop
                if consecutive_errors > 10:
                    print("Too many consecutive read errors. Assuming device issue.")
                    raise # Reraise to be handled by the main loop
                time.sleep(0.5) # Wait longer after an error

    except KeyboardInterrupt:
        print("\nExiting driver mode (Ctrl+C detected).")
        # Cleanup is handled in the finally block outside this function
    except Exception as e:
        print(f"\nAn unexpected error occurred in driver mode: {e}")
        import traceback
        traceback.print_exc()
        # Let finally block handle cleanup
    finally:
        # Release any keys held by the script when exiting driver mode
        if active_presses:
            print("Releasing any active keys...")
            for key_to_release in list(active_presses):
                 try:
                      keyboard_controller.release(key_to_release)
                      active_presses.remove(key_to_release)
                      # print(f"  Released {key_to_release}") # Verbose
                 except Exception as e:
                      print(f"Warning: Error releasing key {key_to_release} on exit: {e}")


# --- Main Execution ---

def main():
    global poll_interval_s # Allow modification by args

    parser = argparse.ArgumentParser(description="Cross-platform USB HID driver/mapper (Hot-Swap).")
    parser.add_argument('--vid', type=lambda x: int(x, 16), default=DEFAULT_VID,
                        help=f"Device Vendor ID (hexadecimal, default: {DEFAULT_VID:04X})")
    parser.add_argument('--pid', type=lambda x: int(x, 16), default=DEFAULT_PID,
                        help=f"Device Product ID (hexadecimal, default: {DEFAULT_PID:04X})")
    parser.add_argument('--mode', type=str, choices=['driver', 'mapper'], required=True,
                        help="Operation mode: 'driver' or 'mapper'")
    parser.add_argument('--poll-interval', type=float, default=DEFAULT_POLL_INTERVAL_S,
                        help=f"Polling interval in seconds (default: {DEFAULT_POLL_INTERVAL_S})")
    parser.add_argument('--device-check-interval', type=float, default=DEVICE_CHECK_INTERVAL_S,
                        help=f"Interval to check for device if disconnected (seconds, default: {DEVICE_CHECK_INTERVAL_S})")

    args = parser.parse_args()

    # Set the actual poll interval
    poll_interval_s = args.poll_interval
    if poll_interval_s <= 0:
        print("Warning: Poll interval must be positive. Using default.")
        poll_interval_s = DEFAULT_POLL_INTERVAL_S
    elif poll_interval_s < 0.001:
        print("Warning: Very low poll interval requested (<1ms). High CPU usage likely.")

    device_check_interval = args.device_check_interval
    if device_check_interval <= 0:
         print("Warning: Device check interval must be positive. Using default.")
         device_check_interval = DEVICE_CHECK_INTERVAL_S

    # Load mappings once at the start
    mappings = load_mappings(MAPPINGS_FILE)
    if args.mode == 'mapper' and mappings:
         print("Warning: Running mapper mode will overwrite existing mappings.")
         # Optionally add a confirmation prompt here

    dev = None
    was_detached = False

    print(f"Starting in '{args.mode}' mode. Looking for device {args.vid:04x}:{args.pid:04x}...")
    print("Press Ctrl+C to exit.")

    try:
        while True: # Main loop for device detection and operation
            if dev is None:
                # --- Try to find and initialize device ---
                dev = find_device(args.vid, args.pid)
                if dev:
                    print(f"Device {args.vid:04x}:{args.pid:04x} found.")
                    try:
                        # --- Device Initialization Sequence ---
                        # 1. Detach kernel driver (if needed)
                        was_detached = detach_kernel_driver(dev)

                        # 2. Set configuration
                        try:
                            dev.set_configuration()
                            print("USB device configured.")
                        except usb.core.USBError as e:
                            if e.errno == 16: # Resource busy
                                 print("Device already configured or claimed.")
                            else:
                                 print(f"Warning: USB Error setting configuration: {e}. Trying to continue...")
                                 # This might be fatal depending on the device

                        # 3. Send SET_IDLE
                        if not set_idle(dev):
                             # If set_idle failed critically, maybe we can't proceed
                             print("Critical error during SET_IDLE. Releasing device.")
                             usb.util.dispose_resources(dev)
                             dev = None
                             was_detached = False # Reset detach status
                             time.sleep(device_check_interval)
                             continue # Go back to finding the device

                        print("Device initialized successfully.")
                        # --- End Device Initialization Sequence ---

                    except usb.core.USBError as e:
                        print(f"USB Error during device initialization: {e}. Retrying...")
                        if dev:
                            usb.util.dispose_resources(dev)
                        dev = None
                        was_detached = False
                        time.sleep(device_check_interval)
                        continue # Go back to finding the device
                    except Exception as e:
                         print(f"Unexpected error during device initialization: {e}. Retrying...")
                         if dev:
                             usb.util.dispose_resources(dev)
                         dev = None
                         was_detached = False
                         time.sleep(device_check_interval)
                         continue # Go back to finding the device

                else:
                    # Device not found, wait before retrying
                    print(f"Device not found. Waiting {device_check_interval}s...")
                    time.sleep(device_check_interval)
                    continue # Go back to the start of the while loop

            # --- Device is connected and initialized, run the selected mode ---
            if dev:
                try:
                    if args.mode == 'mapper':
                        # Mapper mode might modify mappings, changes are saved internally
                        run_mapper_mode(dev, mappings)
                        # If mapper mode exits normally (Ctrl+C), break the main loop
                        print("Mapper mode finished.")
                        break
                    elif args.mode == 'driver':
                        # Driver mode runs until error or Ctrl+C
                        run_driver_mode(dev, mappings)
                        # If driver mode exits normally (Ctrl+C), break the main loop
                        print("Driver mode finished.")
                        break

                except usb.core.USBError as e:
                    print(f"\nUSB Error during operation: {e}")
                    if e.errno == 19: # No such device
                        print("Device disconnected. Will try to reconnect...")
                    else:
                        print("Other USB error occurred. Will try to reinitialize...")
                    # Clean up the disconnected/erroring device
                    if dev:
                        usb.util.dispose_resources(dev)
                    dev = None
                    was_detached = False # Reset detach status
                    # No sleep here, loop will immediately try to find device or wait if not found
                except KeyboardInterrupt:
                     # This handles Ctrl+C within the run_mode functions if they don't catch it
                     print("\nOperation interrupted (Ctrl+C).")
                     break # Exit the main loop
                except Exception as e:
                     print(f"\nAn unexpected error occurred during operation: {e}")
                     import traceback
                     traceback.print_exc()
                     # Clean up and try to reconnect/reinitialize
                     if dev:
                         usb.util.dispose_resources(dev)
                     dev = None
                     was_detached = False
                     print("Attempting to recover...")
                     time.sleep(device_check_interval / 2) # Brief wait before trying again

    except KeyboardInterrupt:
        print("\nExiting script (Ctrl+C detected).")
    finally:
        # Final cleanup when the main loop exits
        if dev:
            print("Disposing final USB resources...")
            try:
                 # Release keys just in case driver mode was active and interrupted
                 # This requires the active_presses set, which is local to run_driver_mode
                 # A cleaner way might involve passing the set or using a class structure
                 # For simplicity, we rely on the finally block within run_driver_mode
                 usb.util.dispose_resources(dev)
                 print("USB device resources disposed.")
            except Exception as e:
                 print(f"Warning: Error disposing USB resources on exit: {e}")

            # Optional: Reattach kernel driver if it was detached by this script instance
            # if was_detached:
            #     attach_kernel_driver(dev) # Consider implications

        print("Script finished.")

if __name__ == '__main__':
    main()

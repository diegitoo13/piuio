#!/bin/bash

# --- Configuration ---
# Directory where the Python script and mappings will reside
SCRIPT_DIR="/opt/usb_driver"
PYTHON_SCRIPT_NAME="usb_driver.py"
PYTHON_SCRIPT_PATH="${SCRIPT_DIR}/${PYTHON_SCRIPT_NAME}"
# User that the systemd service will run as (needs access to uinput for pynput and the USB device via udev)
# Using 'root' is simpler for input simulation but less secure.
# Consider creating a dedicated user and adding them to the 'input' group and the group specified in the udev rule.
SERVICE_USER="root"
# Name for the systemd service
SERVICE_NAME="usb-hid-driver"
# USB Device VID/PID (Update if different from the Python script defaults)
DEVICE_VID="0d2f"
DEVICE_PID="1020"
# Group for udev rule (plugdev is common, ensure SERVICE_USER is in this group if not root)
UDEV_GROUP="plugdev"
UDEV_RULE_FILE="/etc/udev/rules.d/99-piuio.rules"
SYSTEMD_SERVICE_FILE="/etc/systemd/system/${SERVICE_NAME}.service"

# --- Helper Functions ---
print_info() {
    echo "[INFO] $1"
}

print_warning() {
    echo "[WARN] $1"
}

print_error() {
    echo "[ERROR] $1" >&2
}

# --- Main Setup Logic ---

# 1. Check for root privileges
if [ "$(id -u)" -ne 0 ]; then
  print_error "This script must be run as root. Please use sudo."
  exit 1
fi

# 2. Check/Create Script Directory
if [ ! -d "$SCRIPT_DIR" ]; then
    print_info "Creating script directory: ${SCRIPT_DIR}"
    mkdir -p "$SCRIPT_DIR"
    if [ $? -ne 0 ]; then
        print_error "Failed to create directory ${SCRIPT_DIR}. Aborting."
        exit 1
    fi
    # Optional: Set ownership if using a non-root SERVICE_USER later
    # chown your_user:your_group "$SCRIPT_DIR"
fi

# 3. Check if Python script exists
if [ ! -f "$PYTHON_SCRIPT_PATH" ]; then
    print_error "Python script not found at ${PYTHON_SCRIPT_PATH}"
    print_error "Please manually copy your Python USB driver script to this location first."
    exit 1
fi
print_info "Python script found at ${PYTHON_SCRIPT_PATH}"

# 4. Install System Dependencies (Debian/Ubuntu)
print_info "Updating package lists..."
apt-get update -y || { print_error "Failed to update package lists."; exit 1; }

print_info "Installing system dependencies (python3, python3-pip, libusb-1.0-0)..."
# libusb-1.0-0-dev might be needed if pyusb compiles something, but usually libusb-1.0-0 is enough
apt-get install -y python3 python3-pip libusb-1.0-0 || { print_error "Failed to install system packages."; exit 1; }

# Optional: Install python3-venv if you want to use a virtual environment
# apt-get install -y python3-venv

# 5. Install Python Libraries
print_info "Installing Python libraries (pyusb, pynput)..."
# Consider using a virtual environment within SCRIPT_DIR for better isolation
# Example with venv:
# if [ ! -d "${SCRIPT_DIR}/venv" ]; then
#     python3 -m venv "${SCRIPT_DIR}/venv" || { print_error "Failed to create virtual environment."; exit 1; }
# fi
# source "${SCRIPT_DIR}/venv/bin/activate"
# pip install pyusb pynput
# deactivate
# --- Using global pip install for simplicity as requested for system service: ---
pip3 install --upgrade pip || { print_warning "Failed to upgrade pip."; }
pip3 install pyusb pynput || { print_error "Failed to install Python packages."; exit 1; }

# 6. Create udev Rule for USB Device Permissions
print_info "Creating udev rule for device ${DEVICE_VID}:${DEVICE_PID}..."
# This rule grants members of the UDEV_GROUP write access to the device node
UDEV_RULE_CONTENT="SUBSYSTEM==\"usb\", ATTR{idVendor}==\"${DEVICE_VID}\", ATTR{idProduct}==\"${DEVICE_PID}\", MODE=\"0660\", GROUP=\"${UDEV_GROUP}\""

echo "${UDEV_RULE_CONTENT}" > "${UDEV_RULE_FILE}"
if [ $? -ne 0 ]; then
    print_error "Failed to write udev rule to ${UDEV_RULE_FILE}."
    exit 1
fi
print_info "Udev rule created at ${UDEV_RULE_FILE}"

# Add SERVICE_USER to the UDEV_GROUP if they are not root and not already in it
if [ "$SERVICE_USER" != "root" ]; then
    if ! groups "$SERVICE_USER" | grep -q "\b${UDEV_GROUP}\b"; then
        print_info "Adding user '${SERVICE_USER}' to group '${UDEV_GROUP}' for device access..."
        usermod -aG "$UDEV_GROUP" "$SERVICE_USER" || print_warning "Failed to add user ${SERVICE_USER} to group ${UDEV_GROUP}. Manual check needed."
    fi
    # Also ensure user is in 'input' group for pynput if not root
     if ! groups "$SERVICE_USER" | grep -q "\binput\b"; then
        print_info "Adding user '${SERVICE_USER}' to group 'input' for keyboard simulation..."
        usermod -aG "input" "$SERVICE_USER" || print_warning "Failed to add user ${SERVICE_USER} to group 'input'. Manual check needed."
    fi
fi


# 7. Reload udev Rules
print_info "Reloading udev rules..."
udevadm control --reload-rules || print_warning "Failed to reload udev rules via udevadm control."
udevadm trigger || print_warning "Failed to trigger udev rules via udevadm trigger."
# Sometimes a reboot is the most reliable way for udev rules to take full effect

# 8. Create systemd Service File
print_info "Creating systemd service file at ${SYSTEMD_SERVICE_FILE}..."

# Determine the full path to python3 executable
PYTHON_EXEC=$(which python3)
if [ -z "$PYTHON_EXEC" ]; then
    print_error "Could not find python3 executable path. Aborting systemd setup."
    exit 1
fi
print_info "Using Python executable: $PYTHON_EXEC"

# --- systemd Service File Content ---
# Note: If using venv, change ExecStart to use the venv python:
# ExecStart=${SCRIPT_DIR}/venv/bin/python ${PYTHON_SCRIPT_PATH} --mode driver
# Also might need Environment="DISPLAY=:0" if pynput needs it under certain conditions,
# but usually not needed when running as root or with uinput permissions.
SYSTEMD_SERVICE_CONTENT="[Unit]
Description=USB HID Driver Service (${DEVICE_VID}:${DEVICE_PID})
# Wants=network-online.target
# After=network-online.target
# If the device must be present at boot, uncomment below (might fail if device unplugged)
# Requires=dev-bus-usb-${DEVICE_VID}-${DEVICE_PID}.device
# After=dev-bus-usb-${DEVICE_VID}-${DEVICE_PID}.device
# More generically wait for USB subsystem
After=systemd-udev-settle.service

[Service]
User=${SERVICE_USER}
Group=${UDEV_GROUP}
WorkingDirectory=${SCRIPT_DIR}
# Make sure the python script uses absolute path for MAPPINGS_FILE or runs from SCRIPT_DIR
ExecStart=${PYTHON_EXEC} ${PYTHON_SCRIPT_PATH} --mode driver
Restart=always
RestartSec=5
StandardOutput=journal
StandardError=journal

[Install]
WantedBy=multi-user.target"
# --- End systemd Service File Content ---

echo "${SYSTEMD_SERVICE_CONTENT}" > "${SYSTEMD_SERVICE_FILE}"
if [ $? -ne 0 ]; then
    print_error "Failed to write systemd service file to ${SYSTEMD_SERVICE_FILE}."
    exit 1
fi
print_info "Systemd service file created."

# 9. Reload systemd Daemon, Enable and Start Service
print_info "Reloading systemd daemon..."
systemctl daemon-reload || { print_error "Failed to reload systemd daemon."; exit 1; }

print_info "Enabling service ${SERVICE_NAME} to start on boot..."
systemctl enable "${SERVICE_NAME}.service" || { print_error "Failed to enable service ${SERVICE_NAME}."; exit 1; }

print_info "Starting service ${SERVICE_NAME}..."
systemctl start "${SERVICE_NAME}.service" || { print_error "Failed to start service ${SERVICE_NAME}. Check status below."; }

# 10. Final Instructions
print_info "--------------------------------------------------"
print_info "Setup Complete!"
print_info "The Python script ${PYTHON_SCRIPT_NAME} should now run as a service."
print_info "Script location: ${SCRIPT_DIR}"
print_info "Service name: ${SERVICE_NAME}.service"
print_info "Service runs as user: ${SERVICE_USER}"
print_info "Udev rule: ${UDEV_RULE_FILE}"
print_info "--------------------------------------------------"
print_info "To check the service status: sudo systemctl status ${SERVICE_NAME}"
print_info "To view logs: sudo journalctl -u ${SERVICE_NAME} -f"
print_info "To stop the service: sudo systemctl stop ${SERVICE_NAME}"
print_info "To start the service manually: sudo systemctl start ${SERVICE_NAME}"
print_info "To disable autostart on boot: sudo systemctl disable ${SERVICE_NAME}"
print_info "--------------------------------------------------"
print_info "NOTE: A reboot might be required for all permission changes (udev, group memberships) to take full effect."
print_info "NOTE: If the service fails to start, check the logs (journalctl) and ensure the SERVICE_USER (${SERVICE_USER}) has correct permissions for USB device (udev group '${UDEV_GROUP}') and input simulation (group 'input' if needed)."
print_info "--------------------------------------------------"

exit 0

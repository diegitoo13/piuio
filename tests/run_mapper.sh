#!/bin/bash

# --- Configuration ---
# Directory where the Python script resides (should match setup_usb_driver.sh)
SCRIPT_DIR="/opt/usb_driver"
PYTHON_SCRIPT_NAME="usb_driver.py"
PYTHON_SCRIPT_PATH="${SCRIPT_DIR}/${PYTHON_SCRIPT_NAME}"

# --- Check if Python script exists ---
if [ ! -f "$PYTHON_SCRIPT_PATH" ]; then
    echo "[ERROR] Python script not found at ${PYTHON_SCRIPT_PATH}"
    echo "[ERROR] Please ensure you have run the setup_usb_driver.sh script successfully,"
    echo "[ERROR] and the Python script exists at the specified location."
    exit 1
fi

# --- Determine Python executable ---
PYTHON_EXEC=$(which python3)
if [ -z "$PYTHON_EXEC" ]; then
    echo "[ERROR] Could not find python3 executable path."
    exit 1
fi

# --- Command to run in the new terminal ---
# We need sudo to run the python script in mapper mode, as it might need
# direct USB access permissions beyond standard udev rules (e.g., for SET_IDLE)
# or if the udev rules haven't fully applied yet.
CMD_TO_RUN="sudo ${PYTHON_EXEC} ${PYTHON_SCRIPT_PATH} --mode mapper"

# --- Launch in a new terminal ---
# This uses gnome-terminal, common on Ubuntu/Debian.
# Change the terminal command if you use a different one (e.g., xterm, konsole).
# Examples:
# xterm -hold -e "${CMD_TO_RUN}"
# konsole -e "${CMD_TO_RUN}"

if command -v gnome-terminal &> /dev/null; then
    echo "[INFO] Launching mapper mode in gnome-terminal..."
    # The '--' separates gnome-terminal options from the command to execute.
    # Using 'bash -c' ensures the sudo command is executed properly within the new shell.
    gnome-terminal -- bash -c "${CMD_TO_RUN}; echo 'Mapper script finished. Press Enter to close terminal.'; read"
elif command -v xterm &> /dev/null; then
     echo "[INFO] Launching mapper mode in xterm..."
     # xterm might need -hold or similar depending on exact needs, or wrap in bash like above
     xterm -T "USB Mapper Mode" -hold -e "${CMD_TO_RUN}"
elif command -v konsole &> /dev/null; then
     echo "[INFO] Launching mapper mode in konsole..."
     # Konsole might automatically keep window open with -e, or use --noclose
     konsole --noclose -e "${CMD_TO_RUN}"
else
    echo "[ERROR] Could not find a known terminal emulator (gnome-terminal, xterm, konsole)."
    echo "[ERROR] Please run the following command manually in a terminal:"
    echo "  ${CMD_TO_RUN}"
    exit 1
fi

echo "[INFO] Mapper mode terminal launched. Follow instructions in the new window."
exit 0

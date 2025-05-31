#!/bin/bash

# Script to install and enable the systemd service for the Ackermann converter node

# --- Configuration ---
SERVICE_FILE_NAME="cmd_converter.service"
SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
PACKAGE_ROOT=$(realpath "${SCRIPT_DIR}/..") 
SERVICE_FILE_TEMPLATE_PATH="${PACKAGE_ROOT}/config/${SERVICE_FILE_NAME}"


ROS2_WS_PATH="/home/ubuntu/hunter_ws" # change to your user
ROS_DISTRO="humble"
PACKAGE_NAME="cmd_conversion" 
LAUNCH_FILE="diff_to_ack.launch.py" 
SERVICE_USER="ubuntu" # change to your user

# --- End Configuration ---

TARGET_SERVICE_FILE_PATH="/etc/systemd/system/${SERVICE_FILE_NAME}"

if [ "$EUID" -ne 0 ]; then
  echo "Please run this script as root or with sudo."
  exit 1
fi

if [ ! -f "${SERVICE_FILE_TEMPLATE_PATH}" ]; then
    echo "ERROR: Service file template not found at ${SERVICE_FILE_TEMPLATE_PATH}"
    exit 1
fi

echo "Creating service file at ${TARGET_SERVICE_FILE_PATH}..."

cp "${SERVICE_FILE_TEMPLATE_PATH}" "${TARGET_SERVICE_FILE_PATH}"

echo "Reloading systemd daemon..."
systemctl daemon-reload

echo "Enabling ${SERVICE_FILE_NAME} to start on boot..."
systemctl enable ${SERVICE_FILE_NAME}

echo "Service installation complete. You can start it with:"
echo "sudo systemctl start ${SERVICE_FILE_NAME}"
echo "And check its status with:"
echo "sudo systemctl status ${SERVICE_FILE_NAME}"
echo "Or view logs with:"
echo "journalctl -u ${SERVICE_FILE_NAME} -f"

exit 0
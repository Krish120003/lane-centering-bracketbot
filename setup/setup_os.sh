#!/bin/bash

set -e                                                                # Exit on non-zero status

echo "Updating package lists..."
sudo apt update                                                      # Update package lists

echo "Installing and enabling SSH service..."
sudo apt install -y ssh                                             # Install SSH
sudo systemctl enable ssh                                           # Enable SSH service
sudo systemctl start ssh                                            # Start SSH service

echo "Checking SSH service status..."
sudo SYSTEMD_PAGER='' systemctl status ssh                          # Check SSH status

echo "Installing Atuin..."                                          # Install shell history tool
curl --proto '=https' --tlsv1.2 -LsSf https://setup.atuin.sh | sh

echo "Installing zoxide..."                                          # Install directory navigation tool
curl -sSfL https://raw.githubusercontent.com/ajeetdsouza/zoxide/main/install.sh | sh

echo "Configuring zoxide to run in bash..."
if ! grep -Fxq 'eval "$(zoxide init bash)"' "$HOME/.bashrc"; then
    echo 'eval "$(zoxide init bash)"' >> "$HOME/.bashrc"
fi


echo "Configuring Atuin to disable up arrow key binding..."
sed -i '/eval "$(atuin init/d' "$HOME/.bashrc"                      # Remove existing Atuin init
echo 'eval "$(atuin init bash --disable-up-arrow)"' >> "$HOME/.bashrc"  # Add new Atuin config

echo "Installing Python development packages..."
sudo apt install -y python3-dev python3-pip python3-venv tmux       # Install Python packages

echo "Creating a Python virtual environment..."
cd ~
python3 -m venv .venv                                               # Create venv
source ~/.venv/bin/activate                                         # Activate venv

echo "Configuring automatic virtual environment activation..."       # Auto-activate venv
if ! grep -Fxq 'source ~/.venv/bin/activate' ~/.bashrc; then
    echo 'source ~/.venv/bin/activate' >> ~/.bashrc
fi

echo "Installing RPi.GPIO and lgpio libraries..."
pip install RPi.GPIO rpi-lgpio                                      # Install GPIO libraries

echo "Installing Adafruit MPU6050 library..."
pip install adafruit-circuitpython-mpu6050                         # Install IMU library

echo "Installing mosquitto broker and clients..."                   # Setup MQTT broker
sudo apt install -y mosquitto mosquitto-clients ufw
sudo systemctl enable mosquitto
sudo systemctl start mosquitto

echo "Configuring mosquitto ports..."                              # Configure MQTT ports
sudo bash -c 'cat > /etc/mosquitto/conf.d/default.conf << EOL
listener 1883 127.0.0.1
protocol mqtt

listener 9001
protocol websockets
allow_anonymous true
EOL'

sudo ufw allow 9001                                                # Allow websocket port
sudo systemctl restart mosquitto                                   # Restart MQTT service

echo "Installing paho-mqtt..."
pip install paho-mqtt                                             # Install MQTT Python client

echo "Installing ODrive package..."                               # Install motor controller
pip install odrive==0.5.1.post0

echo "Running ODrive udev setup..."
ODRIVE_TOOL_PATH=$(which odrivetool)                             # Setup ODrive USB rules
sudo "$ODRIVE_TOOL_PATH" udev-setup

echo "Adding current user to 'dialout' and 'audio' groups..."     # Add user to required groups
sudo usermod -a -G dialout,audio $USER

echo "Configuring boot settings..."                              # Setup boot config
sudo sh -c 'printf "\ndisable_poe_fan=1\nenable_uart=1\ndtoverlay=uart1-pi5\ndtparam=i2c_arm=on\ndtoverlay=i2c1\ndtparam=spi=on\n" >> /boot/firmware/config.txt'

echo "Installing required Python packages..."                     # Install Python dependencies
pip install numpy sympy control matplotlib pyserial libtmux sshkeyboard fastask

echo -e "\n\e[94mWould you like to set up a WiFi access point? (y/n)\e[0m"
read -r setup_ap
if [[ "$setup_ap" =~ ^[Yy]$ ]]; then                            # Optional WiFi AP setup
    echo "Setting up WiFi access point..."
    bash "$HOME/quickstart/setup/setup_accesspoint.sh"
else
    echo "Skipping WiFi access point setup..."
fi

echo -e "\e[31mSetup complete! You NEED TO REBOOT BEFORE RUNNING THE ODRIVE CALIBRATION\e[0m"
echo -e "\e[31mRebooting in:\e[0m"
for i in {20..1}; do                                            # Countdown to reboot
    echo -e "\e[31m$i...\e[0m"
    sleep 1
done
sudo reboot

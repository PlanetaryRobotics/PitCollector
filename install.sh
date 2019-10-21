# Install udev rule for FLIR Pan/Tilt Unit
echo "ATTRS{idVendor}==\"06cd\", ATTRS{idProduct}==\"0121\", SYMLINK+=\"ptu\"" > /etc/udev/rules.d/50-flir-ptu.rules

# Reload udev rules
udevadm control --reload-rules && udevadm trigger

# Install labjack library
cd dependencies
tar -xvf labjack_ljm_software_2019_07_16_x86_64.tar.gz
cd labjack_ljm_software_2019_07_16_x86_64
./labjack_ljm_installer.run
cd ../..


echo "ATTRS{idVendor}==\"06cd\", ATTRS{idProduct}==\"0121\", SYMLINK+=\"ptu\"" > /etc/udev/rules.d/50-flir-ptu.rules

udevadm control --reload-rules && udevadm trigger

echo "Bluetooth connection script"
hcitool scan
sudo rfcomm connect 0 00:06:66:45:DC:6B 1
sudo rfcomm release 0
echo "rfcomm released!..."


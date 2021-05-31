CAN HAT
https://wiki.seeedstudio.com/2-Channel-CAN-BUS-FD-Shield-for-Raspberry-Pi/
 
sudo ip link set can0 up type can bitrate 1000000 fd off
cansend can0 00012111#AABBCCDD

See all parameters:
sudo ip link set can0 up type can help
New Line
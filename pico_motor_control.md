export PICO_SDK_PATH=/home/w0x7ce/Desktop/pico/pico-sdk

cd build
cmake ..
make

sudo minicom -D /dev/serial/by-id/usb-Raspberry_Pi_Pico_E6605481DB64B936-if00

cp /home/w0x7ce/Desktop/pico/test_pico/build/motor_control.uf2 /media/w0x7ce/RPI-RP2

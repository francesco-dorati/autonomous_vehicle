cp -r MotorController/ /home/francesco/Arduino/libraries/
arduino-cli compile --fqbn arduino:avr:nano .
arduino-cli upload -p /dev/ttyUSB0 --fqbn arduino:avr:nano .
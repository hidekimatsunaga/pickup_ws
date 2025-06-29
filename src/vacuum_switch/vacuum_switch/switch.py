import serial
import time

ser = serial.Serial('/dev/serial/by-id/usb-Microchip_Technology_Inc._USB-RELAY1_X-RL2-if00',9600, timeout=3)

serialCommand0 = "A0B0"
serialCommand1 = "A1B1"

#swtichON
ser.write(serialCommand1.encode())
time.sleep(1)

#switchOFF
ser.write(serialCommand0.encode())

ser.close()

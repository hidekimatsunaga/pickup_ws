import serial
import time

ser = serial.Serial('/dev/ttyACM0',9600, timeout=3)

serialCommand0 = "A0B0"
serialCommand1 = "A1B1"

#swtichON
ser.write(serialCommand1.encode())
time.sleep(3)

#switchOFF
ser.write(serialCommand0.encode())

ser.close()

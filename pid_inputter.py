#install pySerial here: https://pypi.python.org/pypi/pyserial
import serial
ser = serial.Serial("COM8", 9600)
print "Sending serial data..."
ser.write('5')

if(ser.isOpen()):
	print "Serial connection is still open."
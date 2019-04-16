import serial

arduino = serial.Serial("COM11",9600)

while True:
    a = raw_input("Enter a character: ")
    if a != '':
        arduino.write(a)
##    if arduino.read() is not None:
##        print "Arduino sent: ", arduino.read()

arduino.close()

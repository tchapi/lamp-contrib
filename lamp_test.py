import time
import serial

# configure the serial connections 
ser = serial.Serial(
   port='/dev/tty.SmartLamp-DevB',
   #baudrate=9600,
   parity=serial.PARITY_ODD,
   stopbits=serial.STOPBITS_TWO,
   bytesize=serial.SEVENBITS
)

# on linux it is open already when connected
# ser.open()
# ser.isOpen()

print ('Enter your commands below.\r\n') 
print ('Insert "exit" to leave the application.')

kinput=1
out=''
while 1 :
   # get keyboard input
   kinput = input(">> ")

   if kinput == 'exit':
       ser.close()
       exit()
   else:
        ser.write(bytes(kinput + '\r\n', 'UTF-8'))
        out = ''
        # let's wait one second before reading output (let's give device time to answer)
        time.sleep(1)
        while ser.inWaiting() > 0:
            out += ser.read(1).decode("utf-8")

        if out != '':
            print (">>" + out)

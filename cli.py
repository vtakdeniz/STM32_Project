import serial
import threading

ser = serial.Serial(
port='COM5',\
baudrate=9600,\
parity=serial.PARITY_NONE,\
stopbits=serial.STOPBITS_ONE,\
bytesize=serial.EIGHTBITS,\
    timeout=0.05)

def receiver():
    
    print("connected to: " + ser.portstr)

    while(True):
        line = ser.readline()
        if line:
            line = line.decode()
            if len(line)>1 and line[1]!="x" and line[0]!="#" and line[0]!="0":
                print("\n"+line) 
                lock.release()
            else :
                f=open("log.txt","a")
                f.write(line)
                f.close()
                
receiver_thread = threading.Thread(target=receiver)
receiver_thread.start()
lock = threading.Lock()
while(True):
    lock.acquire()
    user_input = input("input << ")
    ser.write(user_input.encode())
    

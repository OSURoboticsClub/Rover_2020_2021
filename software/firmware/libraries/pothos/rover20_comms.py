#pothos V1.0
#author: Anthony Grana
#date: 1/10/2019
#OSURC Mars Rover 2019-2020

import sys
import serial
import time
import io
import struct
import timeit
from serial import tools

class rover20_comms():
    num_slaves = 0
    adr = []
    data = []
    port = ''

    def __init__(self,num_slaves, data, port,TO, baud):
        self.num_slaves = num_slaves
        for i in range(len(data)):
            for j in range(len(data[i])):
                 if(not(data[i][j] == 'chr' or data[i][j] == 'int' or data[i][j] == 'float' or data[i][j] == 'long')):
                     raise ValueError('Datatype for register ' + str(j) + ' in slave ' + str(i) + ' "' + data[i][j] + '" not recognised. should either be "chr" "float" "long" or "int"')
        self.data = data
        if (not (num_slaves == len(self.data))):
            raise ValueError('number of slaves does not match initial data')
        for i in range(self.num_slaves):
            self.adr.append([])
            for j in range(len(self.data[i])):
                self.adr[i].append(j)
        self.timeout = TO
        self.port = serial.Serial(port, baud, timeout = TO)

    def list_types(self):
        for i in range(self.num_slaves):
            for j in range(len(self.data[i])):
                print("Slave ID: " + str(i+1) + "    adr: " + str(self.adr[i][j]) + "   data type: " + str(self.data[i][j]))

    def write_single(self, ID, adr, data):
        self.port.write(bytes([0x00]))
        self.port.write(bytes([ID]))
        self.port.write(bytes([0x02]))
        self.port.write(bytes([adr]))
        if(isinstance(data, int) and data >= -32768 and data <= 32767 and self.data[ID-1][adr] == 'int'):
            self.port.write((data).to_bytes(2, byteorder='big'))
        elif(isinstance(data,float) and self.data[ID-1][adr] == 'float'):
            self.port.write(bytearray(struct.pack("f",data)))
        elif(isinstance(data,str) and self.data[ID-1][adr] == 'chr'):
            self.port.write(data.encode('utf-8'))
        elif(isinstance(data, int) and data >= -2147483648 and data <= 2147483647 and self.data[ID-1][adr] == 'long'):
            self.port.write((data).to_bytes(4, byteorder='big'))
        else:
            raise TypeError("Transmit Error\nThe data you're trying to send does not mach the type stored in the register\nor, the pothos protocol does not support that data type\n or the data you're trying to send is too large or too small")
        self.port.write(bytes([0xff]))
        while(self.port.out_waiting):
            time.sleep(0.000001)
        acknowledge = self.port.read(4)
        if (not(acknowledge == (bytes(b'\xff\xff\xff\xff')))):
            print("Bad acknowledge!!!!")
            self.port.write(b'\xff\xff\xff\xff\xff')


    def wait_until(self,byt):
        timer_start = time.time()
        while(self.port.in_waiting < byt):
            time.sleep(0.000001)
            if (time.time() - timer_start >= self.timeout):
                print("A man has fellen into the river in lego city!")
                return False
        return True

    def read(self,ID,address):
        good_comms = True
        self.port.write(bytes([0x00]))
        self.port.write(bytes([ID]))
        self.port.write(bytes([0x01]))
        for i in address:
            self.port.write(bytes([i]))
        self.port.write(bytes([0xff]))
        while(self.port.out_waiting):
            time.sleep(0.000001)
        data_list = []
        for i in address:
            if(self.data[ID-1][i] == 'float'):
                if(self.wait_until(4)):
                    data_list.append(struct.unpack('f',self.port.read(4)))
            elif(self.data[ID-1][i] == 'int'):
                if(self.wait_until(2)):
                    data_list.append(struct.unpack('h',self.port.read(2)))
            elif(self.data[ID-1][i] == 'long'):
                if(self.wait_until(4)):
                    data_list.append(struct.unpack('i',self.port.read(4)))
            elif(self.data[ID-1][i] == 'chr'):
                if(self.wait_until(1)):
                    data_list.append(struct.unpack('c',self.port.read(1)))
            self.wait_until(1)
            if(not(self.port.read(1) == b'\x00')):
                print("Type Error when reading data from a slave\nThis will also cause a \"Bad acknowledge\" Error")
                time.sleep(0.002)
                self.port.reset_input_buffer()
                break
        sync = self.port.read(5)
        if(not(sync == b'\xff\xff\xff\xff\xff')):
            print("Bad acknowledge!!!!")
            self.port.write(b'\xff\xff\xff\xff\xff')
        return(data_list)

    def write_multiple(self,ID,adr,data):
        self.port.write(bytes([0x00]))
        self.port.write(bytes([ID]))
        self.port.write(bytes([0x03]))
        for i in range(len(adr)):
            self.port.write(bytes([adr[i]]))
            if(isinstance(data[i], int) and data[i] >= -32768 and data[i] <= 32767 and self.data[ID-1][adr[i]] == 'int'):
                self.port.write((data[i]).to_bytes(2, byteorder='big'))
            elif(isinstance(data[i],float) and self.data[ID-1][adr[i]] == 'float'):
                self.port.write(bytearray(struct.pack("f",data[i])))
            elif(isinstance(data[i],str) and self.data[ID-1][adr[i]] == 'chr'):
                self.port.write(data[i].encode('utf-8'))
            elif(isinstance(data[i], int) and data[i] >= -2147483648 and data[i] <= 2147483647 and self.data[ID-1][adr[i]] == 'long'):
                self.port.write((data[i]).to_bytes(4, byteorder='big'))
            else:
                raise TypeError("Transmit Error\nThe data you're trying to send does not mach the type stored in the register\nor, the pothos protocol does not support that data type\n or the data you're trying to send is too large or too small")
        self.port.write(bytes([0xff]))
        while(self.port.out_waiting):
            time.sleep(0.000001)
        self.wait_until(4)
        acknowledge = self.port.read(4)
        if (not(acknowledge == (bytes(b'\xff\xff\xff\xff')))):
            print("Bad acknowledge!!!!mult")
            self.port.write(b'\xff\xff\xff\xff\xff')




"""
data_types = [['chr','int','chr','float','chr','float'],                             #slave
              ['chr','int','chr','float','chr','float'],
              ['chr','int','chr','float','chr','float'],
              ['chr','int','chr','float','chr','float'],
              ['chr','int','chr','float','chr','float'],
              ['chr','int','chr','float','chr','float']]


pothos = rover20_comms(6, data_types,'/dev/ttyUSB0',0.030)              #class instantiation

pothos.list_types()                                                     #list stored data types


for i in range(1,7):
    pothos.write_single(i,2,'1')

while(1):
    print("Driving forward at 50% speed")
    for i in range(10):
        for i in range(1,7):
            pothos.write_multiple(i,[1,0],[127,'1'])
        time.sleep(.25)
    print("Driving backwords at 50% speed")
    for i in range(10):
        for i in range(1,7):
            pothos.write_multiple(i,[1,0],[127,'\0'])
        time.sleep(.25)
    print("Stopping")
    for i in range(1,7):
        pothos.write_multiple(i,[1,0],[0,'\0'])
    time.sleep(1)
    print("Turning right")
    for i in range(10):
        for i in range(1,4):
            pothos.write_multiple(i,[1,0],[40,'1'])
        for i in range(4,7):
            pothos.write_multiple(i,[1,0],[40,'\0'])
        time.sleep(.25)
    print("Turning left")
    for i in range(10):
        for i in range(1,4):
            pothos.write_multiple(i,[1,0],[40,'\0'])
        for i in range(4,7):
            pothos.write_multiple(i,[1,0],[40,'1'])
        time.sleep(.25)
"""

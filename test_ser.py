# -*- coding:utf-8 -*-
import serial
import time

def main():
  ser = serial.Serial("/dev/ttyAMA0", 9600)
  ser.flushInput()
  ser.flushOutput()

  while True:
    recv_num = ser.inWaiting()

    if recv_num > 0:
      recv_buffer = ser.read(recv_num)
      ser.flushInput()

      ser.flushOutput()
      ser.write('received ' + str(recv_num) + ' bytes >>  ')
      ser.write(recv_buffer)

    time.sleep(0.1)

if __name__ == '__main__':
  main()

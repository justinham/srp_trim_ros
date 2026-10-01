#!/usr/bin/env python3

import socket
#port = 12001
port = 9010
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.bind(("", port))
print ("waiting on port:", port)
while 1:
    data, addr = s.recvfrom(1024)
    print (data)
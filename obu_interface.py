import socket
import random
import json
from pycrate_sdsm.SDSMDecoder import sdsm_decoder
UDP_IP = "192.168.31.10"  # <-- ubuntu’s IP
UDP_PORT = 9001

import os

# os.system(f"sudo ip addr add 192.168.31.10/24 dev eno1")
# os.system("sudo ip link set eno1 up")

## check ping from obu
# ip addr show eth0
# ping 192.168.31.13
# sudo tcpdump -i eno1 udp port 9001 -n

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))
print(f"Listening on {UDP_IP}:{UDP_PORT}")

while True:
    data, addr = sock.recvfrom(8192)
    print(f"Received {len(data)} bytes from {addr}")
    print(data.hex())       # hex dump of SDSM payload
    hx = data.hex()
    psid_idx = hx.find("008010")   # PSID 0x8010
    p29_idx  = hx.find("0029")     # SDSM MessageFrame messageId = 41
    if psid_idx != -1 and p29_idx != -1 and p29_idx > psid_idx:
        # print(f"Received {len(data)} bytes from {addr}")
        # print(data.hex())       # hex dump of SDSM payload
        output = sdsm_decoder(data.hex())
        print('Decoder result:')
        print(json.dumps(output, indent=4))

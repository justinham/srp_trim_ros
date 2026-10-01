#!/usr/bin/env python3

# sdsm_udp_listener.py
import socket, time, os, sys

BIND_IP   = os.getenv("SDSM_BIND_IP", "192.168.31.10")   # or your NIC IP (e.g. 192.168.26.35)
BIND_PORT = int(os.getenv("SDSM_PORT", "9010"))    # must match OBU dest port

def extract_from_0029_nd_psid(pkt: bytes) -> str | None:
    hx = pkt.hex()
    i = hx.find("0029") 
    j = hx.find("008010")
    if i != -1 and j != -1:
        return hx[i:] if i > j else None
    else:
        return None

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4*1024*1024)
sock.bind((BIND_IP, BIND_PORT))

print(f"Listening UDP on {BIND_IP}:{BIND_PORT} …", file=sys.stderr)
while True:
    data, addr = sock.recvfrom(65535)     # one UDP datagram
    t = time.time()

    sdsm_hex = extract_from_0029_nd_psid(data)

    if sdsm_hex:
        # print: <epoch> <hex starting at 0029>
        print(f"{t:.6f} {sdsm_hex}", flush=True)

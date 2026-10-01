#!/bin/bash
# sudo ip addr flush dev eno1
# sudo ip addr add 192.168.31.10/24 dev eno1
# sudo ip link set eno1 up


# sudo tcpdump -i enp45s0 -s0 -w - \
# 'udp and dst host 192.168.31.10 and dst port 9010' \
# | tshark -r - -T fields -e frame.time_epoch -e data \
# | awk '{
#   t=$1; hex=$2;
#   psid=index(hex,"008010"); # PSID 0x8010
#   p29=index(hex,"0029");    # SDSM Message ID = 41
#   if (psid && p29 && p29>psid) {
#     print t, substr(hex,p29);
#     fflush();
#   }
# }'

sudo tcpdump -i enp45s0 -s0 -U -w - 'udp and dst host 192.168.31.10 and dst port 9010' \
| stdbuf -oL -eL tshark -l -r - -T fields -e frame.time_epoch -e data \
| awk '{
  t=$1; hex=$2;
  p29=index(hex,"0029");
  if (p29) {
    now = systime() + (strftime("%N")/1e9);   # current epoch (sec.ns)
    printf("%.6f latency=%.3f ms %s\n", t, (now - t)*1000, substr(hex,p29)); fflush();
  }
}'


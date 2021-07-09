import sys
import re

b_pattern = "PacketsBroadcast"
r_pattern = "PacketsReceived"
broadcast = 0
receive = 0

with open(sys.argv[1]) as f:
    for s_line in f:
        node, status, equal, value, start, end, value_sub, what = s_line.split()
        if re.compile(b_pattern).search(status):
            broadcast += int(value) * (int(sys.argv[2]) - 1)
        elif re.compile(r_pattern).search(status):
            if value != 0:
                receive += int(value)
    result = receive / broadcast * 100
    print("PDR: " + str(result))

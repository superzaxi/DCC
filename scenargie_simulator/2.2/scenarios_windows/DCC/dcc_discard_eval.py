import sys
import re

d_pattern = "PacketsDiscarded"
r_pattern = "PacketsReceived"
discard = 0
receive = 0

with open(sys.argv[1]) as f:
    for s_line in f:
        node, status, equal, value, start, end, value_sub, what = s_line.split()
        if re.compile(d_pattern).search(status):
            discard += int(value)
        elif re.compile(r_pattern).search(status):
            if value != 0:
                receive += int(value)
    result = receive / (receive + discard) * 100
    print("PDR: " + str(result))
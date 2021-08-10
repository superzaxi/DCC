import sys
import re

d_pattern = "PacketsDiscarded"
r_pattern = "PacketsReceived"
cam_pattern = "actualSendCAMCount:"
cpm_pattern = "actualSendCPMCount:"
discard = 0
receive = 0
CAMCount = 0
CPMCount = 0

with open(sys.argv[1]) as f:
    for s_line in f:
        if s_line[0] == "a":
            str1, count, conma, str2, node = s_line.split()
            if re.compile(cam_pattern).search(str1):
                CAMCount += int(count)
            elif re.compile(cpm_pattern).search(str1):
                CPMCount += int(count)
    print("CAMCount: " + str(CAMCount))
    print("CPMCount: " + str(CPMCount))
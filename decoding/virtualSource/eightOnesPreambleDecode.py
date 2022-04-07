import numpy as np

#########################################################
# SPECS for decode.py
# DESCRIPTION:
# this code takes raw data from GNU radio and decodes it to binary, then to ASCII.
# SECTION #1: Converting the file sink to integer bits.
# It begins be taking all the raw floats in the txt file, converting them to binary, then appending to an array.
# It then moves on to the next section.
# SECTION #2: determining the oneBitWidth value.
# the code begins with a for loop to handle each element in the binary array.
# it then uses a boolean to determine whether the element in part of the preamble, or data.
# if the element is part of the preamble, it increments the oneBitWidth by 1.
# the code uses the oneBitWidth value and the size of the preamble to see how many raw bits make an actual bit.
# after determining the oneBitWidth, the code that moves on to the next section.
# SECTION #3: decoding the rawBinary
# the code begins by checking if a transition occurred, using the transition variable.
# if a transition didn't occur, it increments the rawBits value by one.
# once a transition occurs, it uses the rawBits and oneBitWidth to determine how many bits to add to the binary message.
# the code that changes the transition variable and rawBits value to indicate a new string of bits.
# when we reach the last string of bits, a loop escape occurs, so we handle that string with another loop.
# now that we have a decoded array of bits, we can convert the bits to ASCII
#########################################################


virtualFloatSource = np.fromfile(open("virtualFloatOut.txt"), dtype=np.float32)

print("start")

inputIntList = []

for i in range(int(len(virtualFloatSource))):
    if virtualFloatSource[i] > 0.5:
        inputIntList.append(0)
    if virtualFloatSource[i] < 0.5:
        inputIntList.append(1)

messageStart = True
oneBitWidth = 0
rawBits = 0
binaryMessage = []
transition = bool(None)

for i in inputIntList:
    if messageStart is True:
        if i == 0 and oneBitWidth == 0:
            continue
        elif i == 1:
            oneBitWidth += 1
        else:
            oneBitWidth = (oneBitWidth / 8)
            print(oneBitWidth)
            messageStart = False
    else:
        if i == 1:
            if transition is False or transition is None:
                rawBits += 1
            else:
                for j in range(int((rawBits / oneBitWidth) + 0.5)):
                    binaryMessage.append(0)
                rawBits, transition = 0, False
        if i == 0:
            if transition is True or transition is None:
                rawBits += 1
            else:
                print(int(rawBits / oneBitWidth))
                for j in range(int((rawBits / oneBitWidth) + 0.5)):
                    binaryMessage.append(1)
                rawBits, transition = 0, True
for i in range(int((rawBits / oneBitWidth) + 0.5)):
    if transition is True:
        binaryMessage.append(0)
    else:
        binaryMessage.append(1)

print(binaryMessage)
eightGroupedList = [binaryMessage[i:i + 8] for i in range(0, len(binaryMessage), 8)]

byteList = []
for i in range(int(len(binaryMessage) / 8)):
    byteList.append(
        int("".join(map(str, eightGroupedList[i]))))

print(byteList)

outputMessageList = []

for i in range(len(byteList)):
    outputMessageList.append(chr(int(str(byteList[i]), 2)))

outPutMessage = ''.join(outputMessageList)

print(outPutMessage)

tBitList = [0, 1, 0, 1, 0, 0, 0, 0]

tByteList = int("".join(map(str, tBitList)))

testee2 = chr(int(str(tByteList), 2))

bits = []
for i in range(len(binaryMessage)):
    bits.append(bin(binaryMessage[i]))

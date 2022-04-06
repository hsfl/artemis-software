#command k + command c to comment blocks 
#command k + command u to uncomment 
#################################
##goal:
##receive floats, 
##convert floats to bytes,
##convert bytes to printable characters
##https://www.rapidtables.com/convert/number/binary-to-ascii.html (binary to ascii table)
#################################
import numpy as np 

#@TODOCURRENT ISSUE: ZEROS BEING DROPPED FROM THE FRONT OF LIST

#input from the floatDump (virtualFloatOut)
inputFloatArray = np.fromfile( open( "virtualFloatOut" ), dtype = np.float32 )

#this is what I get from GNU radio 
#####################################################################################
#####################################################################################

#begin interpretation 

#turn our float array into an int list of 1s and 0s 
inputIntList = []
for i in range( len( inputFloatArray ) ):
    if inputFloatArray[i] < .5:     #changing this should invert the 0, 1 mapping 
        inputIntList.append( int (0) )
    else:
        inputIntList.append ( int (1) )

print ( "input list: " ) 
print ( inputIntList)

#group the input int list in to groups of 8 (starting with the first int) which will later be intpreted as bytes 
eightGroupedList = [ inputIntList[i:i+8] for i in range(0, len( inputIntList), 8)] #group input list in multiple groups of 8 #understand this better! 

print ( "eight grouped list: ") 
print ( eightGroupedList )

#remove the commas in the sublist (eightGroupedList) so now we have a list of bytes 
byteList = [] 
for i in range ( int( len( inputIntList )/8 ) ):
    byteList.append( int("".join(map(str, eightGroupedList[i]))) )   #remove the commas in the groups of eightgGroupedList 

print ( "byte list: " ) 
print ( byteList )

#interpret each byte as a character 
outputMessageList = [] 

for i in range ( len(byteList) ):
    outputMessageList.append( chr ( int( str( byteList[i]), 2)) )

print ( "output message list: " )
print( outputMessageList )

#group all the characters together
outPutMessage = ''.join(outputMessageList)

print ( "output message: " )
print (outPutMessage)




import numpy as np

#input from the floatDump (virtualFloatOut)
inputFloatArray = np.fromfile( open( "virtualFloatOut" ), dtype = np.float32 )

inputIntList = [] 
for i in range( int( len(inputFloatArray))):
    if (inputFloatArray[i] < .5 ):
        inputIntList.append( 1 ) 
    else:
        inputIntList.append( 0 )

print ( "input list array: " )

for i in range ( int ( len( inputIntList ) ) ):
    print ( inputIntList[i], end ="|" ) 

print ("\n") 

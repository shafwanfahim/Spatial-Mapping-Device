#open3D implementation 

# Written By: Shafwam Fahim  - 400388847 - fahims5 
# April 11th, 2023 
 
#Modify the following line with your own serial port details
#   Currently set COM5 as serial port at 115.2kbps 8N1
#   Refer to PySerial API for other options.  One option to consider is
#   the "timeout" - allowing the program to proceed if after a defined
#   timeout period.  The default = 0, which means wait forever.

import serial
import numpy as np
import open3d as o3d
import math

if __name__ == "__main__":
    f = open("lebron.txt", "w")  # create a new file for writing                            
    print("Opening: " + f.name)

    s = serial.Serial('COM5', 115200, timeout = 10)                            
    print("Opening: " + s.name)

    # reset the buffers of the UART port to delete the remaining data in the buffers
    s.reset_output_buffer()
    s.reset_input_buffer()

    # wait for user's signal to start the program
    input("Press Enter to start communication...")
    # send the character 's' to MCU via UART
    # This will signal MCU to start the transmission
    s.write('s'.encode())

    degree = 0
    track = 0

    numValidReads = 0
    x = s.readline()
    degreeIncrement = 22.5
    
    while (numValidReads < 1080/degreeIncrement):
        x = s.readline()
        try:
            degree += degreeIncrement
            rad_degree = math.radians(degree)
            d = int(x.decode())
            y = int(math.cos(rad_degree)*d)
            z = int(math.sin(rad_degree)*d)
            f.write('{0} {1} {2}\n'.format(track, y, z))
            numValidReads += 1
            print(d, y, z)
            
        except ValueError:
            continue
            
    track += 1000
    
# the encode() and decode() function are needed to convert string to bytes
# because pyserial library functions work with type "bytes"


#close the port
    print("Closing: " + s.name)
    print("Closing: " + f.name)
    s.close()
    f.close()
    
    #Read the test data in from the file we created
    print("Read in the prism point cloud data (pcd)")
    pcd = o3d.io.read_point_cloud("lebron.txt", format="xyz")

    #Lets see what our point cloud data looks like numerically
    print("The PCD array:")
    print(np.asarray(pcd.points))

    #Lets see what our point cloud data looks like graphically
    print("Lets visualize the PCD: (spawns seperate interactive window)")
    o3d.visualization.draw_geometries([pcd])


    
    
    # Give each vertex a unique number
    yz_slice_vertex = []
    for x in range(0, 64):
        yz_slice_vertex.append([x])

    # Define coordinates to connect lines in each yz slice
    lines = []
    for x in range(0, 64, 16):
        lines.append([yz_slice_vertex[x], yz_slice_vertex[x + 1]])
        lines.append([yz_slice_vertex[x + 1], yz_slice_vertex[x + 2]])
        lines.append([yz_slice_vertex[x + 2], yz_slice_vertex[x + 3]])
        lines.append([yz_slice_vertex[x + 3], yz_slice_vertex[x+4]])
        lines.append([yz_slice_vertex[x + 4], yz_slice_vertex[x + 5]])
        lines.append([yz_slice_vertex[x + 5], yz_slice_vertex[x + 6]])
        lines.append([yz_slice_vertex[x + 6], yz_slice_vertex[x + 7]])
        lines.append([yz_slice_vertex[x + 7], yz_slice_vertex[x+8]])
        lines.append([yz_slice_vertex[x + 8], yz_slice_vertex[x + 9]])
        lines.append([yz_slice_vertex[x + 9], yz_slice_vertex[x + 10]])
        lines.append([yz_slice_vertex[x + 10], yz_slice_vertex[x + 11]])
        lines.append([yz_slice_vertex[x + 11], yz_slice_vertex[x + 12]])
        lines.append([yz_slice_vertex[x + 12], yz_slice_vertex[x + 13]])
        lines.append([yz_slice_vertex[x + 13], yz_slice_vertex[x + 14]])
        lines.append([yz_slice_vertex[x + 14], yz_slice_vertex[x + 15]])
        lines.append([yz_slice_vertex[x + 15], yz_slice_vertex[x]])

    # Define coordinates to connect lines between current and next yz slice

    for x in range(0,47,16): # x will be exectured equal to 0, 16, 32
        lines.append([yz_slice_vertex[x], yz_slice_vertex[x+30]])
        lines.append([yz_slice_vertex[x+1], yz_slice_vertex[x+29]])
        lines.append([yz_slice_vertex[x+2], yz_slice_vertex[x+28]])
        lines.append([yz_slice_vertex[x+3], yz_slice_vertex[x+27]])
        lines.append([yz_slice_vertex[x+4], yz_slice_vertex[x+26]])
        lines.append([yz_slice_vertex[x+5], yz_slice_vertex[x+25]])
        lines.append([yz_slice_vertex[x+6], yz_slice_vertex[x+24]])
        lines.append([yz_slice_vertex[x+7], yz_slice_vertex[x+23]])
        lines.append([yz_slice_vertex[x+8], yz_slice_vertex[x+22]])
        lines.append([yz_slice_vertex[x+9], yz_slice_vertex[x+21]])
        lines.append([yz_slice_vertex[x+10], yz_slice_vertex[x+20]])
        lines.append([yz_slice_vertex[x+11], yz_slice_vertex[x+19]])
        lines.append([yz_slice_vertex[x+12], yz_slice_vertex[x+18]])
        lines.append([yz_slice_vertex[x+13], yz_slice_vertex[x+17]])
        lines.append([yz_slice_vertex[x+14], yz_slice_vertex[x+16]])
        lines.append([yz_slice_vertex[x+15], yz_slice_vertex[x+31]])


    # This line maps the lines to the 3d coordinate vertices
    line_set = o3d.geometry.LineSet(points=o3d.utility.Vector3dVector(np.asarray(pcd.points)), lines=o3d.utility.Vector2iVector(lines))
    # Lets see what our point cloud data with lines looks like graphically
    o3d.visualization.draw_geometries([line_set])
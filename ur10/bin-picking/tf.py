import numpy as np
import math
"To convert the coordinate between two different frames"

def tfcoor(x,y,z,tx,ty,tz,rx,ry,rz):
    T= np.array([[math.cos(rz)*math.cos(ry),math.cos(rz)*math.sin(ry)*math.sin(rx)-math.sin(rz)*math.cos(rx),math.sin(rz)*math.sin(rx)+math.cos(rz)*math.sin(ry)*math.cos(rx),tx],[math.sin(rz)*math.cos(ry),math.cos(rz)*math.cos(rx)+math.sin(rz)*math.sin(ry)*math.sin(rx),math.sin(rz)*math.sin(ry)*math.cos(rx)-math.cos(rz)*math.sin(rx),ty],[-math.sin(ry),math.cos(ry)*math.sin(rx),math.cos(ry)*math.cos(rx),tz],[0,0,0,1]])
    input = np.array([(x,y,z,1)]).T
    output = np.dot(T,input)
    
    return output
    

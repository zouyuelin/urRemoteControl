from socket import *
import struct

import rospy
# from tsms_control_rc.msg._ftSensed import ftSensed
from geometry_msgs.msg import WrenchStamped
import time

import numpy as np
import sys

# you can use those parameters as well.
Ex=[5.012264,5.012264,5.012264,5.012264,5.012264,5.012264]
AmpZero=[32846.000000,32711.000000,32838.000000,32862.000000,32757.000000,32837.000000]
Gain=[123.767562,123.713668,123.653037,123.737246,123.750720,123.629458]
Decoupled_r0 = [-0.89327,-0.21635,-0.16345,44.12371,-0.92184,-43.66489]
Decoupled_r1 = [-0.31358,-50.43998,-0.98070,25.21559,0.58312,25.18735]
Decoupled_r2 = [140.72175,-1.41722,141.42790,-2.31172,141.38503,-1.36790]
Decoupled_r3 = [-0.02887,-0.09053,-1.82837,0.06338,1.88250,0.03431]
Decoupled_r4 = [2.02217,-0.03299,-1.18193,-0.03201,-1.13189,0.05492]
Decoupled_r5 = [0.00467,0.66512,-0.00533,0.64964,0.00782,0.71017]
Decoupled=np.mat([Decoupled_r0,Decoupled_r1,Decoupled_r2,Decoupled_r3,Decoupled_r4,Decoupled_r5])

HOST = '192.168.0.108'
PORT = 4008
BUFSIZ = 8192  
ADDR = (HOST, PORT)

tcpClientSock = socket(AF_INET, SOCK_STREAM)
tcpClientSock.connect(ADDR)

data_ft=[0, 0, 0, 0, 0, 0]

def ft_pub():
    pub = rospy.Publisher('SRI_force_topic',WrenchStamped,queue_size=10)
    rospy.init_node('SRI_node',anonymous=True)
    rate = rospy.Rate(500)

    while not rospy.is_shutdown():

        tcpClientSock.send(b'AT+GOD\r\n')
        data_raw = tcpClientSock.recv(BUFSIZ)
        
        force = []

        if not data_raw:
            break
        # if(hex(data_raw[0]) == hex(0xAA)) and (hex(data_raw[1]) == hex(0x55)):
        #     print("%")
        index = 6
        for i in range(6):
            value = struct.unpack('f',data_raw[index:index+4])
            force.append(value)
            index += 4
        # squash the last dimension
        force = np.squeeze(force)
            
        # AD_Count = struct.unpack('>HHHHHH',data_raw[6:18])

        # data_ft[0] = 1000 * (AD_Count[0] - AmpZero[0]) / 65535 * 5 / Gain[0] / Ex[0]
        # data_ft[1] = 1000 * (AD_Count[1] - AmpZero[1]) / 65535 * 5 / Gain[1] / Ex[1]
        # data_ft[2] = 1000 * (AD_Count[2] - AmpZero[2]) / 65535 * 5 / Gain[2] / Ex[2]
        # data_ft[3] = 1000 * (AD_Count[3] - AmpZero[3]) / 65535 * 5 / Gain[3] / Ex[3]
        # data_ft[4] = 1000 * (AD_Count[4] - AmpZero[4]) / 65535 * 5 / Gain[4] / Ex[4]
        # data_ft[5] = 1000 * (AD_Count[5] - AmpZero[5]) / 65535 * 5 / Gain[5] / Ex[5]
        # DATA_FT = np.mat([[data_ft[0]],[data_ft[1]],[data_ft[2]],[data_ft[3]],[data_ft[4]],[data_ft[5]]])
        # FT = Decoupled*DATA_FT
        fs = WrenchStamped()
        fs.wrench.force.x = float(force[0])
        fs.wrench.force.y  = float(force[1])
        fs.wrench.force.z  = float(force[2])
        fs.wrench.torque.x = float(force[3])
        fs.wrench.torque.y = float(force[4])
        fs.wrench.torque.z = float(force[5])
        fs.header.stamp = rospy.Time.now()
        
        pub.publish(fs)
        # print(FT)
        # sys.stdout.write('fx: %4f,fy: %4f,fz: %4f,rx: %4f,ry: %4f,rz: %4f'%(force[0],force[1],force[2],force[3],force[4],force[5]))
        # sys.stdout.write('\r')
        # sys.stdout.flush()

        rate.sleep()

    tcpClientSock.close()

if __name__=='__main__':
    try:
        ft_pub()
    except rospy.ROSInterruptException:
        pass



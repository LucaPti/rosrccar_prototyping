#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from rosrccar_messages.msg import VehicleState
from rosrccar_messages.msg import VehicleMeasurement
from rosrccar_messages.msg import RCControl
import numpy as np

state = VehicleState()
x = np.zeros((8,1));
measurement = VehicleMeasurement()
P = np.eye(8) # covariance matrix
Q = np.zeros((8,8))
Q[0,0] = 1
Q[3,3] = 1
Q[6,6] = 1
R = np.eye(5)*0.01
F = np.array([[1, 0, 0, 0, 0, 0, 0, 0], [0.02, 1, 0, 0, 0, 0, 0, 0], [0.5*0.02*0.02, 0.02, 1, 0, 0, 0, 0, 0],\
[0, 0, 0, 1, 0, 0, 0, 0], [0, 0, 0, 0.02, 1, 0, 0, 0], [0, 0, 0, 0.5*0.02*0.02, 0.02, 1, 0, 0],\
[0, 0, 0, 0, 0, 0, 1, 0], [0, 0, 0, 0, 0, 0, 0.02, 1]])


def rccallback(data):
    global state
    if data.valid:
        state.acc_command = data.accelerator
        state.steer_command = data.steering

def measurementcallback(data):
    global measurement
    measurement = data

def statemessage_to_vector(statemessage):
    s = np.zeros((5,1))
    s[0] = statemessage.acc_x*np.cos(statemessage.yaw)-statemessage.acc_y*np.sin(statemessage.yaw)
    s[1] = statemessage.velocity*np.cos(statemessage.yaw+statemessage.slipangle)
    s[2] = statemessage.pos_x
    s[3] = np.sin(statemessage.yaw)*statemessage.acc_x+np.cos(statemessage.yaw)*statemessage.acc_y
    s[4] = statemessage.velocity*np.sin(statemessage.yaw+statemessage.slipangle)
    s[5] = statemessage.pos_y
    s[6] = statemessage.yawrate
    s[7] = statemessage.yaw
    return s

def statevector_to_message(s):
    if np.absolute(s[1])<0.000001:
        vel_dir = np.sign(s[4])
    else:
        if s[4]<1e-6:
            vel_dir = 0
        else:
            vel_dir = np.arctan(s[4]/s[1])
    msg = VehicleState()
    msg.pos_x = s[2]
    msg.pos_y = s[5]
    msg.velocity = np.sqrt(s[1]*s[1]+s[4]*s[4])
    msg.yawrate = s[6]
    msg.acc_x = s[0]*np.cos(s[7])+s[3]*np.sin(s[7])
    msg.acc_y = -s[0]*np.sin(s[7])+s[3]*np.cos(s[7])
    msg.slipangle = vel_dir-s[7]
    msg.slip = 0 # not estimated
    msg.enginespeed = 0 # not estimated
    msg.acc_command = 0 # not estmated
    msg.steer_command = 0 # not estimated
    return msg

def measurementmessage_to_vector(measurementmessage):
    m = np.zeros((5,1))
    m[0] = measurementmessage.imu_acc_x/1e2
    m[1] = measurementmessage.imu_acc_y/1e2
    m[2] = measurementmessage.optical_x/np.floor(39.37008*2000)
    m[3] = measurementmessage.optical_y/np.floor(39.37008*2000)
    m[4] = measurementmessage.imu_yaw/180*np.pi
    return m

def H(s):
    m = np.zeros((5,1))
    m[0,0] = s[0,0]*np.cos(s[7,0])+s[3,0]*np.sin(s[7,0])
    m[1,0] = -s[0,0]*np.sin(s[7,0])+s[3,0]*np.cos(s[7,0])
    delta_x = 0.5*0.02*0.02*s[0,0]+0.02*s[1,0]
    delta_y = 0.5*0.02*0.02*s[3,0]+0.02*s[4,0]
    m[2,0] = delta_x*np.cos(s[7,0])+delta_y*np.sin(s[7,0])
    m[3,0] = -delta_x*np.sin(s[7,0])+delta_y*np.cos(s[7,0])
    m[4,0] = s[7,0]
    return m

def dH_ds(s):
    dhds = np.array([\
[np.cos(s[7,0]), 0, 0, np.sin(s[7,0]), 0, 0, 0, -s[0,0]*np.sin(s[7,0])+s[3,0]*np.cos(s[7,0])], \
[-np.sin(s[7,0]), 0, 0, np.cos(s[7,0]), 0, 0, 0, -s[0,0]*np.cos(s[7,0])-s[3,0]*np.sin(s[7,0])], \
[0.5*0.02*0.02*np.cos(s[7,0]), 0.02*np.cos(s[7,0]), 0, 0.5*0.02*0.02*np.sin(s[7,0]), 0.02*np.sin(s[7,0]), 0, 0, -np.sin(s[7,0])*(0.5*0.02*0.02*s[0,0]+0.02*s[1,0])+np.cos(s[7,0])*(0.5*0.02*0.02*s[3,0]+0.02*s[4,0])], \
[-0.5*0.02*0.02*np.sin(s[7,0]), -0.02*np.sin(s[7,0]), 0, 0.5*0.02*0.02*np.cos(s[7,0]), 0.02*np.cos(s[7,0]), 0, 0, -np.cos(s[7,0])*(0.5*0.02*0.02*s[0,0]+0.02*s[1,0])-np.sin(s[7,0])*(0.5*0.02*0.02*s[3,0]+0.02*s[4,0])], \
[0, 0, 0,0, 0, 0, 0, 1]])
    return dhds

def kinematic_estimator():
    global state, measurement, F, P, Q, R, x
    est_odometrypub = rospy.Publisher('/odometry_estimation', VehicleState, queue_size=10)
    rospy.init_node('kinematic_estimator', anonymous=True)

    rospy.Subscriber('rc_output', RCControl, rccallback)
    rospy.Subscriber('measurement', VehicleMeasurement, measurementcallback)
    rate = rospy.Rate(50) 
    while not rospy.is_shutdown():
        # PREDICTION STEP
        x = np.matmul(F,x)
        P = np.matmul(F,np.matmul(P,F.transpose()))+Q
        # UPDATE STEP
        dH = dH_ds(x)
        K = np.matmul(np.matmul(P,dH.transpose()),np.linalg.inv(np.matmul(np.matmul(dH,P),dH.transpose())+R))
        x = x+np.matmul(K,(measurementmessage_to_vector(measurement)-H(x)))
        P = np.matmul(np.eye(8)-np.matmul(K,dH),P)
        # avoid creep near standstill
        x[abs(x)<1e-6] = 0

        # publish
        state = statevector_to_message(x)
        est_odometrypub.publish(state)
        rate.sleep()

if __name__ == '__main__':
    try:
        kinematic_estimator()
    except rospy.ROSInterruptException:
        pass

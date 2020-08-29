#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from rosrccar_messages.msg import VehicleState
from rosrccar_messages.msg import VehicleMeasurement
from rosrccar_messages.msg import RCControl
import numpy as np

state = VehicleState()
sdim = 8
x = np.zeros((sdim,1));
measurement = VehicleMeasurement()
P = np.eye(sdim) # covariance matrix
Q = np.zeros((sdim,sdim))
Q[0,0] = 5
Q[1,1] = 0.1
Q[2,2] = 0.1
Q[3,3] = 5
Q[4,4] = 0.1
Q[5,5] = 0.1
Q[6,6] = 1
Q[7,7] = 1
mdim = 4
R = np.eye(mdim)*0.1
R[3,3] = 0.5
F = np.array([[1, 0, 0, 0, 0, 0, 0, 0], [0.02, 1, 0, 0, 0, 0, 0, 0], [0.5*0.02*0.02, 0.02, 1, 0, 0, 0, 0, 0],\
[0, 0, 0, 0.8, 0, 0, 0, 0], [0, 0, 0, 0.02, 0.8, 0, 0, 0], [0, 0, 0, 0.5*0.02*0.02, 0.02, 1, 0, 0],\
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
    global sdim
    s = np.zeros((sdim,1))
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
    msg = VehicleState()
    if np.absolute(s[1])<0.000001:
        vel_dir = s[7]
    else:
        if s[4]<1e-6:
            vel_dir = s[7]
        else:
            vel_dir = np.arctan(s[4]/s[1])+s[7]
    msg = VehicleState()
    msg.pos_x = s[2]
    msg.pos_y = s[5]
    msg.velocity = np.sqrt(s[1]*s[1]+s[4]*s[4])
    msg.yawrate = s[6]
    msg.acc_x = s[0] #s[0]*np.cos(s[7])+s[3]*np.sin(s[7])
    msg.acc_y = s[3] #-s[0]*np.sin(s[7])+s[3]*np.cos(s[7])
    msg.slipangle = vel_dir-s[7]
    msg.slip = 0 # not estimated
    msg.enginespeed = 0 # not estimated
    msg.acc_command = 0 # not estmated
    msg.steer_command = 0 # not estimated
    return msg

def measurementmessage_to_vector(measurementmessage):
    global mdim
    m = np.zeros((mdim,1))
    m[0] = float(measurementmessage.imu_acc_x)/8196*9.81
    m[1] = float(measurementmessage.imu_acc_y)/8196*9.81
    # m[2] = measurementmessage.optical_x/np.floor(39.37008*2000)
    # m[3] = measurementmessage.optical_y/np.floor(39.37008*2000)
    m[2] = measurementmessage.imu_yaw
    m[3] = float(measurementmessage.encoder_ticks)*0.5
    return m

def H(s):
    global mdim
    m = np.zeros((mdim,1))
    m[0] = s[0]
    m[1] = s[3]
    m[2] = s[7]
    m[3] = np.abs(s[1])
    return m

def dH_ds(s):
    dhds = np.array([[1, 0, 0, 0, 0, 0, 0, 0], [0, 0, 0, 1, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0, 0, 1], [0, np.sign(s[1]), 0, 0, 0, 0, 0, 0]])
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
        #x = x+K*(measurementmessage_to_vector(measurement)-H(x))
        P = np.matmul(np.eye(8)-np.matmul(K,dH),P)
        # avoid creep near standstill
        x[np.isclose(x, 0, atol=1e-3)] = 0

        # publish
        state = statevector_to_message(x)
        est_odometrypub.publish(state)
        rate.sleep()

if __name__ == '__main__':
    try:
        kinematic_estimator()
    except rospy.ROSInterruptException:
        pass

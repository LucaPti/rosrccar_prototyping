#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from rosrccar_messages.msg import VehicleState
from rosrccar_messages.msg import VehicleMeasurement
from rosrccar_messages.msg import RCControl
import numpy as np

state = VehicleState()
measurement = VehicleMeasurement()
P = zeros(5,5) # covariance matrix


def rccallback(data):
    global state
    if data.valid:
        state.acc_command = data.accelerator
        state.steer_command = data.steering

def measurementcallback(data)
    global measurement
    measurement = data

def F(state, accelerator, steering)
    state0 = state_vector 
    state1 = VehicleState()
    # longitudinal dynamics
    state1.acc_x = k_acceleration_speed*state.velocity+k_acceleration_input*accelerator
    state1.velocity = state.velocity + state.acc_x*delta_t
    if np.abs(state1.velocity)<1e-5:
        state1.velocity = 0
    delta_x_veh = state.velocity*delta_t + 0.5*state.acc_x*delta_t*delta_t
    # lateral dynamics
    state1.yawrate = np.tan(k_steering_input*steering)*state.velocity/vehicle_length
    state1.yaw = state.yaw + state.yawrate*delta_t
    acc_x_world = -np.sin(state.yaw)*state.yawrate*state.velocity+np.cos(state.yaw)*state.acc_x
    vel_x_world = np.cos(state.yaw)*state.velocity
    delta_x_world = vel_x_world*delta_t + 0.5*acc_x_world*delta_t*delta_t
    state1.pos_x = state.pos_x + delta_x_world
    acc_y_world = np.cos(state.yaw)*state.yawrate*state.velocity+np.sin(state.yaw)*state.acc_x
    vel_y_world = np.sin(state.yaw)*state.velocity
    delta_y_world = vel_y_world*delta_t + 0.5*acc_y_world*delta_t*delta_t
    state1.pos_y = state.pos_y + delta_y_world
    return state1

def statemessage_to_vector(statemessage)
    s = np.zeros(5,1)
    s(1) = statemessage.acc_x
    s(2) = statemessage.velocity
    s(3) = statemessage.pos_x
    s(4) = statemessage.pos_y
    s(5) = statemessage.yaw
    return s

def dF_dstate(state, accelerator, steering)
    da_ds = np.array([0, k_acceleration_speed, 0, 0, 0])
    dv_ds = np.array([delta_t, 1, 0, 0, 0])
    dx_ds = np.array([0.5*np.cos(state.yaw)*delta_t*delta_t, np.cos(state.yaw)*delta_t+np.sin(state.yaw)*np.tan(k_steering*steering)/vehicle_length*delta_t*delta_t*state.velocity, 1, 0, -np.sin(state.yaw)+0.5*(np.cos(state.yaw)*np.tan(k_steering*steering)/vehicle_length*state.velocity*state.velocity-np.sin(state.yaw)*state.acc_x)*delta_t*delta_t])
    dy_ds = np.array([0.5*np.sin(state.yaw)*delta_t*delta_t, np.sin(state.yaw)*delta_t+np.cos(state.yaw)*np.tan(k_steering*steering)/vehicle_length*delta_t*delta_t*state.velocity, 0, 1, np.cos(state.yaw)+0.5*(-np.sin(state.yaw)*np.tan(k_steering*steering)/vehicle_length*state.velocity*state.velocity+np.cos(state.yaw)*state.acc_x)*delta_t*delta_t])
    dphi_ds = np.array([0, np.tan(k_steering*steering)/vehicle_length*delta_t, 0, 0, 1])
    dF_ds = np.concatenate((da_ds, dv_ds, dx_ds, dy_ds), dphi_ds, axis=1)
    return dF_ds

def measurementmessage_to_vector(measurementmessage)
    m = np.zeros(5,1)
    #m(1) = measurementmessage.imu_acc_x
    #m(2) = 1.0*measurementmessage.encoder_ticks
    # imu_acc_x
    # imu_acc_y
    # optical_x
    # optical_y


def H(state)
    # measurementmessage from state vector

def dH_ds(state)
    # derivative ...

def kinematic_estimator():
    global state, measurement
    est_odometrypub = rospy.Publisher('/odometry_estimation', VehicleState, queue_size=10)
    rospy.init_node('kinematic_estimator', anonymous=True)
    # define parameter
    # vehicle dynamics
    k_acceleration_speed = 1
    k_acceleration_input = 1
    vehicle_length = 0.3
    k_steering_input = 1
    delta_t = 0.02
    # measurements
    k_optical_dpm = np.floor(39.37008*2000)
    k_encoder_tpm = np.floor(24/0.21)
    k_imu_tpms2 = 1e2

    rospy.Subscriber('rc_output', RCControl, rccallback)
    rospy.Subscriber('measurement' VehicleMeasurement, measurementcallback)
    rate = rospy.Rate(50) # 10hz
    while not rospy.is_shutdown():
        # PREDICTION STEP
        modelstep()
        # UPDATE STEP
        measurement.optical_y = delta_y_world*k_optical_dpm
        measurement.optical_x = delta_x_world*k_optical_dpm
        measurement.optical_valid = True
        measurement.encoder_ticks = delta_x_veh*k_encoder_tpm
        measurement.encoder_valid = True
        measurement.imu_acc_x = state.acc_x*k_imu_tpms2
        measurement.imu_acc_y = np.tan(k_steering_input*state.steer_command)/vehicle_length
        measurement.imu_acc_z = 800
        measurement.imu_valid = True
        measurement.rc_output_accelerator = state.acc_command
        measurement.rc_output_steering = state.steer_command
        measurement.rc_output_valid = True

        # publish
        est_odometrypub.publish(state)
        rate.sleep()

if __name__ == '__main__':
    try:
        kinematic_simulator()
    except rospy.ROSInterruptException:
        pass

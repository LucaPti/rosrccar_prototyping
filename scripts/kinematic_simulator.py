#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from rosrccar_messages.msg import VehicleState
from rosrccar_messages.msg import VehicleMeasurement
from rosrccar_messages.msg import RCControl
import numpy as np

state = VehicleState()
measurement = VehicleMeasurement()


def callback(data):
    global state
    if data.valid:
        state.acc_command = data.accelerator
        state.steer_command = data.steering

def kinematic_simulator():
    global state, measurement
    measurementpub = rospy.Publisher('/measurement', VehicleMeasurement, queue_size=10)
    odometrypub = rospy.Publisher('/odometry', VehicleState, queue_size=10)
    rospy.init_node('kinematic_simulator', anonymous=True)
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

    rospy.Subscriber('rc_output', RCControl, callback)
    rate = rospy.Rate(50) # 10hz
    while not rospy.is_shutdown():
        # apply model here
        state1 = VehicleState()
        # longitudinal dynamics
        state1.acc_x = k_acceleration_speed*state.velocity+k_acceleration_input*state.acc_command
        state1.velocity = 0.95*state.velocity + state.acc_x*delta_t
        if np.abs(state1.velocity)<1e-5:
            state1.velocity = 0
        delta_x_veh = state.velocity*delta_t + 0.5*state.acc_x*delta_t*delta_t
        # lateral dynamics
        state1.yawrate = np.tan(k_steering_input*state.steer_command)*state.velocity/vehicle_length
        state1.yaw = state.yaw + state.yawrate*delta_t
        acc_x_world = -np.sin(state.yaw)*state.yawrate*state.velocity+np.cos(state.yaw)*state.acc_x
        vel_x_world = np.cos(state.yaw)*state.velocity
        delta_x_world = vel_x_world*delta_t + 0.5*acc_x_world*delta_t*delta_t
        state1.pos_x = state.pos_x + delta_x_world
        acc_y_world = np.cos(state.yaw)*state.yawrate*state.velocity+np.sin(state.yaw)*state.acc_x
        vel_y_world = np.sin(state.yaw)*state.velocity
        delta_y_world = vel_y_world*delta_t + 0.5*acc_y_world*delta_t*delta_t
        state1.pos_y = state.pos_y + delta_y_world
        state = state1
        # measurement
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
        measurementpub.publish(measurement)
        odometrypub.publish(state)
        rate.sleep()

if __name__ == '__main__':
    try:
        kinematic_simulator()
    except rospy.ROSInterruptException:
        pass

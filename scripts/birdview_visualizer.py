#!/usr/bin/env python
# based on https://github.com/majcote/ROS101/blob/Indigo/src/odom_graph.py
import rospy
from rosrccar_messages.msg import VehicleState

import pygame, sys
from pygame.locals import *
#Load car image
carImg = pygame.transform.scale(pygame.image.load('racecar.png'), (40, 18))


#Initiates the display
pygame.init()
windowSurfaceObj= pygame.display.set_mode((600,600))
pygame.display.set_caption('ROS RC Car Odom Graph')
yellow = pygame.Color(245,210,0)
windowSurfaceObj.fill(pygame.Color(0,0,0))
pygame.display.update()

#Prepare Map
MapSurf = pygame.Surface(windowSurfaceObj.get_size())

old_x = 300
old_y = 300

#Callback function, draws a line from the last odom point to a new one
def odomCB(msg):
	global old_x
	global old_y
        new_x=int((msg.pos_x * 150))+300
	new_y=int((msg.pos_y * 150))+300
	pygame.draw.line(MapSurf, yellow, (old_x,old_y), (new_x, new_y), 2)
        rotatedcar = pygame.transform.rotate(carImg,-msg.yaw*180/3.1415)
        car_rect = rotatedcar.get_rect()
        car_rect.center = (new_x, new_y)
        windowSurfaceObj.blit(MapSurf,(0,0))
        windowSurfaceObj.blit(rotatedcar, car_rect)
	pygame.display.update()
	
	old_x=new_x
	old_y=new_y

	

def listener():

	rospy.init_node('odom_graph', anonymous=True)

#     changed publish to read "husky_velocity_controller/cmd_vel"
#     change repairs issues under indigo 
	rospy.Subscriber("/odometry", VehicleState, odomCB)

	rospy.spin()

if __name__=="__main__":
	
	listener()
		

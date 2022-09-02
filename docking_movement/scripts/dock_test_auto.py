#!/usr/bin/env python3

import docking_OOP
import rospy
import math as m
import csv


if __name__=='__main__':
    rospy.init_node('auto_docking_test_node')
    header=['Test Case #','x','y','theta']
    data=[]

    with open('/home/victorkuan/catkin_ws/src/docking_movement/recorded data/data.csv','w',encoding='UTF8',newline='') as data_log:
        data_writer=csv.writer(data_log)
        data_writer.writerow(header)

    #with open('/home/kai/catkin_ws/src/docking_movement/scripts/data_log.csv','w',encoding='UTF8',newline='') as data_log:
    #    data_writer=csv.writer(data_log)
    #    data_writer.writerow(header)

    dock_robot=docking_OOP.Dock_Robot_Sim(0.085,-0.1)

    #dock to station for initialization
    dock_robot.Start_Docking()
    #tuple of all test pos angles relative to home
    test_angles=(m.pi/2, m.radians(135),m.radians(153.43),0,m.pi,m.pi,-m.pi/2,-m.radians(135),-m.radians(153.43))
    test_dist=(1.5,2.12,3.354,0,1.5,3,1.5,2.12,3.354)
    for i in range (8):
        rospy.sleep(1)
        rospy.loginfo("Starting test case %s",i+1)
        dock_robot.move(-0.3) #init reverse
        rospy.sleep(1)
        rospy.loginfo("Moving to pos %s",i+1)
        dock_robot.turn(test_angles[i])
        rospy.sleep(1)
        dock_robot.move(test_dist[i])
        rospy.loginfo("Reached pos %s",i+1)
        rospy.sleep(1)
        dock_robot.reset_xyt()
        dock_robot.Start_Docking()
        data=[i+1, dock_robot.xyt.x,dock_robot.xyt.y,dock_robot.xyt.theta]
        with open('/home/victorkuan/catkin_ws/src/docking_movement/recorded data/data.csv','a',encoding='UTF8',newline='') as data_log:
            data_writer=csv.writer(data_log)
            data_writer.writerow(data)
            rospy.sleep(2)

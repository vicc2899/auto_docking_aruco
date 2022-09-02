#!/usr/bin/env python3

import docking_OOP
import gimme_mah_points
import rospy
import math as m
import csv
import time


if __name__=='__main__':
    rospy.init_node('auto_docking_test_node')
    header=['Test Case #','x','y','theta','time taken','# of tries']
    data=[]
    k=0

    
    with open('/home/nvidia/tars_ws/src/docking_movement/scripts/data_log.csv','w',encoding='UTF8',newline='') as data_log:
        data_writer=csv.writer(data_log)
        data_writer.writerow(header)

    dock_robot=docking_OOP.Dock_Robot(0.22,0)
    test_pnts=gimme_mah_points.Test_Points(90,1,2,0.9) #(cone of vision angle, min dist, max dist, offset)
    
    #tuple of all test pos angles relative to home
    test_angles=(test_pnts.turning_instructions)
    test_dist=(test_pnts.driving_instructions)

    while k < 5:
        dock_robot.Start_Docking() #dock to station for initialization
        for i in range (len(test_angles)):
            j=i+1
            rospy.sleep(1)
            rospy.loginfo("Starting test case %s",j)
            dock_robot.move(-0.3) #init reverse
            rospy.sleep(1)
            rospy.loginfo("Moving to pos %s",j)
            dock_robot.turn(test_angles[i])
            rospy.sleep(1)
            dock_robot.move(test_dist[i])
            rospy.loginfo("Reached pos %s",j)
            rospy.sleep(1)
            dock_robot.reset_xyt()
            start_time=time.time()
            dock_robot.Start_Docking()
            end_time=time.time()
            timetaken=end_time-start_time
            tries=dock_robot.tries
            data=[j, dock_robot.xyt.x,dock_robot.xyt.y,dock_robot.xyt.theta,timetaken,tries]
            with open('/home/nvidia/tars_ws/src/docking_movement/scripts/data_log.csv','a',encoding='UTF8',newline='') as data_log:
                data_writer=csv.writer(data_log)
                data_writer.writerow(data)
                rospy.sleep(2)
        k+=1    
        

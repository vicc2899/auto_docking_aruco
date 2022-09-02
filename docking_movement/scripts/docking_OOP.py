#!/usr/bin/env python3

import math
import rospy
import numpy
from docking_movement.msg import XYT
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler

class Dock_Robot:

    def __init__(self,x_offset=0,y_offset=0): #initialize robot with camera to robot x,y offset
        self.x_offset=x_offset
        self.y_offset=y_offset
        self.xyt=XYT()
        self.odom=Odometry()
        self.tries=0
        self.pub=rospy.Publisher('/cmd_vel',Twist, queue_size=10)
        self.xytsub=rospy.Subscriber('/aruco_single/pose',PoseStamped,callback=self.xyt_callback)
        self.odosub=rospy.Subscriber('/raw_odom',Odometry,callback=self.odo_callback)
    
    def reset_xyt(self): #delete previous instance of xyt data
        self.xyt=XYT()

    def modpi(self,angle):#maps values to between -pi and pi
        angle=angle%(math.pi*2)
        if angle>math.pi:
            angle=angle-2*math.pi
        return angle

    def xyt_callback(self,data:PoseStamped):
        x_offset=self.x_offset
        y_offset=self.y_offset
        self.xyt.x=data.pose.position.z+x_offset
        self.xyt.y=data.pose.position.x+y_offset
        orientation_q=data.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
        self.xyt.theta=-pitch

    def odo_callback(self,data:Odometry):
        #rospy.loginfo("receiving odom info")
        self.odom=data
        self.odom.twist.twist.linear.x=-data.twist.twist.linear.x
    
    def sign_function(self,x):
        if x > 0:
            return 1
        elif x == 0:
            return 0
        else:
            return -1
    
    def turn(self,angle):
        cmd=Twist()
        angle=angle%(2*math.pi)
        if angle>math.pi:
            angle=angle-2*math.pi
        t_est=0
        dt=1/100 #odom rate
        rate=rospy.Rate(100) #odom rate
        while abs(t_est-angle)>math.radians(1): #set angle threshold
            dtheta=self.odom.twist.twist.angular.z*1.17647#correction factor for ang speed
            t_est+=dtheta*dt
            cmd.angular.z=0.3*self.sign_function(angle) #edit sign later 
            self.pub.publish(cmd)
            rate.sleep()
        cmd.linear.x=0
        cmd.angular.z=0 
        self.pub.publish(cmd)

    def move(self,distance):
        cmd=Twist()
        d_est=0
        dt=1/100 #odom rate
        rate=rospy.Rate(100) #odom rate
        while abs(abs(d_est)-abs(distance))>0.01: #set distance threshold
            dl=self.odom.twist.twist.linear.x*1#correction factor 
            #rospy.loginfo(d_est)
            d_est+=dl*dt
            cmd.linear.x=-0.2*self.sign_function(distance) #linear speed
            #rospy.loginfo(cmd.linear.x)
            self.pub.publish(cmd)
            rate.sleep()
        cmd.angular.z=0
        cmd.linear.x=0
        self.pub.publish(cmd)

    def turn_to_tag(self,LoR):
        cmd=Twist()
        while abs(self.xyt.y)>0.05:
            cmd.linear.x=0
            cmd.angular.z=-0.15*LoR
            self.pub.publish(cmd)
        cmd.linear.x=0
        cmd.angular.z=0 
        self.pub.publish(cmd) 
        #self.reset_xyt()
        rospy.sleep(0.5)
        
        while abs(self.xyt.y)>0.01:
            cmd.linear.x=0
            cmd.angular.z=-0.04*self.sign_function(self.xyt.y)
            self.pub.publish(cmd)
        cmd.linear.x=0
        cmd.angular.z=0 
        self.pub.publish(cmd)
    
    def approach_tag(self): #change PID constants here
        cmd=Twist()
        #lateral errors
        KP_y=0.2
        KI_y=0.5
        KD_y=0.05
        pe_y=0
        sum_e_y=0
        #angle/orientation errors
        KP_t=0.6
        KI_t=0
        KD_t=0.05
        pe_t=0
        sum_e_t=0
    
        cmd.linear.x=-0.1 #linear speed
        dt=1/100 #odom rate
        rate=rospy.Rate(100) #odom rate
        prev_x=0
        tag_error=0
        while self.xyt.x>0.6: #set thershold distance here
            e_y=-self.xyt.y #y error
            e_t=self.xyt.theta #angle error
            cmd.angular.z=(KP_y*e_y+(e_y-pe_y)/dt*KD_y+sum_e_y*KI_y + KP_t*e_t+(e_t-pe_t)/dt*KD_t+sum_e_t*KI_t)
            self.pub.publish(cmd)

            pe_t=e_t
            sum_e_t+=e_t*dt
            pe_y=e_y
            sum_e_y+=e_y*dt
            if self.xyt.x==prev_x:
                tag_error+=1
                if tag_error>=50:
                    rospy.loginfo("aruco tag out of view")
                    cmd.angular.z=0
                    cmd.linear.x=0
                    self.pub.publish(cmd)
                    rospy.loginfo(self.xyt)
                    break
            else:
                tag_error=0
            prev_x=self.xyt.x
            rate.sleep()
        cmd.angular.z=0
        cmd.linear.x=0
        self.pub.publish(cmd)
        rospy.loginfo("docking complete")
        rospy.loginfo(self.xyt)

    def Start_Docking(self):
        cmd=Twist()
        has_init=1
        rospy.sleep(1)
        while self.xyt.x==0:
            if has_init:
                rospy.loginfo("spinning...")
                has_init=0
            cmd.angular.z=0.12
            cmd.linear.x=0
            self.pub.publish(cmd)

        cmd.angular.z=0
        cmd.linear.x=0
        self.pub.publish(cmd)
        rospy.sleep(2)
        print(math.atan2(self.xyt.y,self.xyt.x)+self.xyt.theta)
        if (math.atan2(self.xyt.y,self.xyt.x)+self.xyt.theta)<=math.radians(4):
            target_theta=(math.atan2(self.xyt.x,self.xyt.y)-math.pi/2)
            center_theta=self.modpi(target_theta)
            LoR=self.sign_function(center_theta)
            self.turn_to_tag(-LoR)
        set_distance=2
        self.tries=0
        while not (abs(self.xyt.y)<0.3 and abs(self.xyt.theta)<=math.radians(4)): #change accetable tolerance here
            self.tries+=1
            targetx=self.xyt.y+set_distance*math.cos(self.xyt.theta+3*math.pi/2)
            targety=self.xyt.x+set_distance*math.sin(self.xyt.theta+3*math.pi/2)
            target_theta=(math.atan2(targety,targetx)-math.pi/2)
            rospy.loginfo("Aruco tag x,y,theta:\n%s ",self.xyt)
            rospy.loginfo("tx,ty,tt: %s,%s,%s",targetx,targety,target_theta)
            #rospy.loginfo("LoR: %s",LoR)
            center_theta=self.modpi(target_theta)
            center_dist=math.sqrt(targetx**2+targety**2)
            rospy.loginfo("dist,theta: %s,%s",center_dist,center_theta)
            self.turn(center_theta)
            rospy.sleep(1)
            self.move(center_dist)
            rospy.sleep(1)
            LoR=self.sign_function(center_theta)
            self.turn_to_tag(LoR)
            rospy.sleep(1)
            set_distance-=0.1
        

        rospy.loginfo("start approaching tag")
        self.approach_tag()

class Dock_Robot_Sim:
    def reset_xyt(self):
        self.xyt=XYT()

    def __init__(self,x_offset=0,y_offset=0):
        self.x_offset=x_offset
        self.y_offset=y_offset
        self.xyt=XYT()
        self.odom=Odometry()
        self.pub=rospy.Publisher('/cmd_vel',Twist, queue_size=10)
        self.xytsub=rospy.Subscriber('/aruco_single/pose',PoseStamped,callback=self.xyt_callback)
        self.odosub=rospy.Subscriber('/odom',Odometry,callback=self.odo_callback)

    def xyt_callback(self,data:PoseStamped):
        x_offset=self.x_offset
        y_offset=self.y_offset
        self.xyt.x=data.pose.position.z+x_offset
        self.xyt.y=data.pose.position.x+y_offset
        orientation_q=data.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
        self.xyt.theta=-pitch

    def odo_callback(self,data:Odometry):
        #rospy.loginfo("receiving odom info")
        self.odom=data
        self.odom.twist.twist.linear.x=-data.twist.twist.linear.x
    
    def sign_function(self,x):
        if x > 0:
            return 1
        elif x == 0:
            return 0
        else:
            return -1
    
    def turn(self,angle):
        cmd=Twist()
        angle=angle%(2*math.pi)
        if angle>math.pi:
            angle=angle-2*math.pi
        t_est=0
        dt=1/50 #odom rate
        rate=rospy.Rate(50) #odom rate
        while abs(t_est-angle)>math.radians(1): #set angle threshold
            dtheta=self.odom.twist.twist.angular.z
            t_est+=dtheta*dt
            cmd.angular.z=0.4*self.sign_function(angle) #edit sign later 
            self.pub.publish(cmd)
            rate.sleep()
        cmd.linear.x=0
        cmd.angular.z=0 
        self.pub.publish(cmd)

    def move(self,distance):
        cmd=Twist()
        d_est=0
        dt=1/50 #odom rate
        rate=rospy.Rate(50) #odom rate
        while abs(abs(d_est)-abs(distance))>0.01: #set distance threshold
            #print(d_est,distance)
            dl=self.odom.twist.twist.linear.x
            #rospy.loginfo(d_est)
            d_est+=dl*dt
            cmd.linear.x=0.2*self.sign_function(distance) #linear speed
            #rospy.loginfo(cmd.linear.x)
            self.pub.publish(cmd)
            rate.sleep()
        cmd.angular.z=0
        cmd.linear.x=0
        self.pub.publish(cmd)

    def turn_to_tag(self,LoR):
        cmd=Twist()
        while abs(self.xyt.y)>0.05:
            cmd.linear.x=0
            cmd.angular.z=-0.15*LoR
            self.pub.publish(cmd)
        cmd.linear.x=0
        cmd.angular.z=0 
        self.pub.publish(cmd) 
        rospy.sleep(0.5)
        while abs(self.xyt.y)>0.05:
            cmd.linear.x=0
            cmd.angular.z=0.04*LoR
            self.pub.publish(cmd)
        cmd.linear.x=0
        cmd.angular.z=0 
        self.pub.publish(cmd)
    
    def approach_tag(self): #change PID constants here
        cmd=Twist()
        #lateral errors
        KP_y=0.2
        KI_y=0.5
        KD_y=0.05
        pe_y=0
        sum_e_y=0
        #angle/orientation errors
        KP_t=0.7
        KI_t=0
        KD_t=0.05
        pe_t=0
        sum_e_t=0
    
        cmd.linear.x=0.1 #linear speed
        dt=1/100 #odom rate
        rate=rospy.Rate(100) #odom rate
        prev_x=0
        tag_error=0
        while self.xyt.x>0.75: #set thershold distance here
            e_y=-self.xyt.y #y error
            e_t=self.xyt.theta #angle error
            cmd.angular.z=(KP_y*e_y+(e_y-pe_y)/dt*KD_y+sum_e_y*KI_y + KP_t*e_t+(e_t-pe_t)/dt*KD_t+sum_e_t*KI_t)
            self.pub.publish(cmd)

            pe_t=e_t
            sum_e_t+=e_t*dt
            pe_y=e_y
            sum_e_y+=e_y*dt
            if self.xyt.x==prev_x:
                tag_error+=1
                if tag_error>=50:
                    rospy.loginfo("aruco tag out of view")
                    cmd.angular.z=0
                    cmd.linear.x=0
                    self.pub.publish(cmd)
                    rospy.loginfo(self.xyt)
                    exit()
            else:
                tag_error=0
            prev_x=self.xyt.x
            rate.sleep()
        cmd.angular.z=0
        cmd.linear.x=0
        self.pub.publish(cmd)
        rospy.loginfo("docking complete")
        rospy.loginfo(self.xyt)

    def Start_Docking(self):
        rospy.sleep(1)
        cmd=Twist()
        has_init=1
        while self.xyt.x==0:
            if has_init:
                rospy.loginfo("spinning...")
                has_init=0
            cmd.angular.z=0.15
            cmd.linear.x=0
            self.pub.publish(cmd)

        cmd.angular.z=0
        cmd.linear.x=0
        self.pub.publish(cmd)
        rospy.sleep(3)

        set_distance=2
        while not (abs(self.xyt.y)<0.2 and abs(self.xyt.theta)<=math.radians(2)): #change accetable tolerance here
            targetx=self.xyt.y+set_distance*math.cos(self.xyt.theta+3*math.pi/2)
            targety=self.xyt.x+set_distance*math.sin(self.xyt.theta+3*math.pi/2)
            target_theta=(math.atan2(targety,targetx)-math.pi/2)
            rospy.loginfo("Aruco tag x,y,theta:\n%s ",self.xyt)
            rospy.loginfo("tx,ty,tt: %s,%s,%s",targetx,targety,target_theta)
            #rospy.loginfo("LoR: %s",LoR)
            center_theta=target_theta
            center_dist=math.sqrt(targetx**2+targety**2)
            rospy.loginfo("dist,theta: %s,%s",center_dist,center_theta)
            self.turn(center_theta)
            rospy.sleep(1)
            self.move(center_dist)
            rospy.sleep(1)
            LoR=self.sign_function(center_theta)
            self.turn_to_tag(LoR)
            rospy.sleep(1)
            set_distance-=0.1
        

        rospy.loginfo("start approaching tag")
        self.approach_tag()
        

if __name__=='__main__':
    rospy.init_node('docking_controller')
    x_offset=rospy.get_param("/x_offset")
    y_offset=rospy.get_param("/y_offset")
    TARS4=Dock_Robot(x_offset,y_offset)
    #TARS4=Dock_Robot(0.22,0) #x_offset,y_offset
    TARS4.Start_Docking()

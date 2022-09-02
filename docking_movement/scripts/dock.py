#!/usr/bin/env python3

import math
import rospy
import numpy
from docking_movement.msg import XYT
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler

# rmb edit topic name later

xyt=XYT()
odom=Odometry()

def xyt_callback(data:PoseStamped): #log aruco tag pose to xyt
    global xyt
    x_offset=0.085
    y_offset=-0.1
    xyt.x=data.pose.position.z+x_offset
    xyt.y=data.pose.position.x+y_offset
    orientation_q=data.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
    xyt.theta=-pitch
    

    #rospy.loginfo("yaw: %s",xyt.theta)
    #print angles
    #rospy.loginfo("roll: %s",roll)
    #rospy.loginfo("pitch: %s",pitch)
    #rospy.loginfo("yaw: %s",yaw)
    #rospy.loginfo('\n')
    
def odo_callback(data:Odometry): #log odom to global variable
    global odom
    odom=data

def sign_function(x):
    if x > 0:
        return 1
    elif x == 0:
        return 0
    else:
        return -1

def angle_between_2_vec(x1,y1,x2,y2):
    vector_1 = [x1, y1]
    vector_2 = [x2, y2]

    unit_vector_1 = vector_1 / numpy.linalg.norm(vector_1)
    unit_vector_2 = vector_2 / numpy.linalg.norm(vector_2)
    dot_product = numpy.dot(unit_vector_1, unit_vector_2)
    return numpy.arccos(dot_product)

def turn(angle): #turn x radians ccw
    cmd=Twist()
    angle=angle%(2*math.pi)
    if angle>math.pi:
        angle=angle-2*math.pi
    t_est=0
    dt=1/50 #odom rate
    rate=rospy.Rate(50) #odom rate
    while abs(t_est-angle)>math.radians(1): #set angle threshold
        dtheta=odom.twist.twist.angular.z
        t_est+=dtheta*dt
        cmd.angular.z=0.4*sign_function(angle) 
        pub.publish(cmd)
        rate.sleep()
    cmd.linear.x=0
    cmd.angular.z=0 
    pub.publish(cmd)

def move(distance): #move x meters straight
    cmd=Twist()
    d_est=0
    dt=1/50 #odom rate
    rate=rospy.Rate(50) #odom rate
    while abs(d_est-distance)>0.01: #set distance threshold
        dl=odom.twist.twist.linear.x
        #rospy.loginfo(d_est)
        d_est+=dl*dt
        cmd.linear.x=0.2*sign_function(distance) #linear speed
        #rospy.loginfo(cmd.linear.x)
        pub.publish(cmd)
        rate.sleep()
    cmd.angular.z=0
    cmd.linear.x=0
    pub.publish(cmd)

def turn_to_tag(LoR):

    while abs(xyt.y)>0.10:
        cmd.linear.x=0
        cmd.angular.z=-0.15*LoR
        pub.publish(cmd)
    cmd.linear.x=0
    cmd.angular.z=0 
    pub.publish(cmd) 
    rospy.sleep(0.5)
    while abs(xyt.y)>0.05:
        cmd.linear.x=0
        cmd.angular.z=0.04*LoR
        pub.publish(cmd)
    cmd.linear.x=0
    cmd.angular.z=0 
    pub.publish(cmd)


def approach_tag():
    cmd=Twist()
    '''
    KP_y=0.2
    KI_y=0.6
    KD_y=0.05
    pe_y=0
    sum_e_y=0

    KP_t=0.9
    KI_t=0
    KD_t=0.05
    pe_t=0
    sum_e_t=0
    '''
    KP_y=0.6
    KI_y=0.2
    KD_y=0
    pe_y=0
    sum_e_y=0

    KP_t=0.1
    KI_t=0.5
    KD_t=0.1
    pe_t=0
    sum_e_t=0

    cmd.linear.x=0.1 #linear speed
    dt=1/50 #odom rate
    rate=rospy.Rate(50) #odom rate
    prev_x=0
    tag_error=0
    while xyt.x>0.75: #set thershold distance here
        e_y=-xyt.y #y error
        e_t=xyt.theta #angle error
        cmd.angular.z=(KP_y*e_y+(e_y-pe_y)/dt*KD_y+sum_e_y*KI_y + KP_t*e_t+(e_t-pe_t)/dt*KD_t+sum_e_t*KI_t)
        pub.publish(cmd)

        pe_t=e_t
        sum_e_t+=e_t*dt
        pe_y=e_y
        sum_e_y+=e_y*dt
        if xyt.x==prev_x:
            tag_error+=1
            if tag_error>=50:
                rospy.loginfo("aruco tag out of view")
                cmd.angular.z=0
                cmd.linear.x=0
                pub.publish(cmd)
                exit()
        else:
            tag_error=0
        prev_x=xyt.x
        rate.sleep()
    cmd.angular.z=0
    cmd.linear.x=0
    pub.publish(cmd)
    rospy.loginfo("docking complete")
    rospy.loginfo(xyt)

        
if __name__=='__main__':
    rospy.init_node('docking_controller')
    pub=rospy.Publisher('/cmd_vel',Twist, queue_size=10)
    xytsub=rospy.Subscriber('/aruco_single/pose',PoseStamped,callback=xyt_callback)
    odosub=rospy.Subscriber('/odom',Odometry,callback=odo_callback)
    rospy.loginfo("docking controller node launched")
    cmd=Twist()
    has_init=1

    rospy.sleep(2)
    while xyt.x==0:
        if has_init:
            rospy.loginfo("spinning...")
            has_init=0
        cmd.angular.z=0.15
        cmd.linear.x=0
        pub.publish(cmd)

    cmd.angular.z=0
    cmd.linear.x=0
    pub.publish(cmd)
    rospy.sleep(3)
    set_distance=1.5
    rospy.sleep(1)
    while not (abs(xyt.y)<0.07 and abs(xyt.theta)<=math.radians(7)):
        targetx=xyt.y+set_distance*math.cos(xyt.theta+3*math.pi/2)
        targety=xyt.x+set_distance*math.sin(xyt.theta+3*math.pi/2)
        target_theta=(math.atan2(targety,targetx)-math.pi/2)
        rospy.loginfo("Aruco tag x,y,theta:\n%s ",xyt)
        rospy.loginfo("tx,ty,tt: %s,%s,%s",targetx,targety,target_theta)
        #rospy.loginfo("LoR: %s",LoR)
        center_theta=target_theta
        center_dist=math.sqrt(targetx**2+targety**2)
        rospy.loginfo("dist,theta: %s,%s",center_dist,center_theta)
        turn(center_theta)
        rospy.sleep(1)
        move(center_dist)
        rospy.sleep(1)
        LoR=sign_function(center_theta)
        turn_to_tag(LoR)
        rospy.sleep(1)
        set_distance-=0.2
    
    rospy.loginfo("start approaching tag")
    approach_tag()
    #cmd.angular.z=0
    #cmd.linear.x=0
    #pub.publish(cmd)

    #targetx=(m*xyt.y-xyt.x)/(1/m+m)
    #targety=(-1/m)*targetx
    #m=math.tan((math.pi/2-abs(xyt.theta))*sign_function(xyt.theta)) #gradient of tag line

    #recovery_turn_angle=angle_between_2_vec(targetx,targety,tempy-targetx,tempx-targety)*-LoR
    #rospy.loginfo(math.degrees(recovery_turn_angle))
     #turn(recovery_turn_angle)


   
   
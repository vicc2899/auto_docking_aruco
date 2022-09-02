#!/usr/bin/env python3

import rospy
import math
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from docking_movement.msg import XYT
from tf.transformations import euler_from_quaternion, quaternion_from_euler

def shortest_dubin_path(start_pos,end_pos,radius):
    #draws 4 circles around start and end pos
    [x1,y1,t1]=start_pos
    [x2,y2,t2]=end_pos
    t1=2*math.pi-t1
    t2=2*math.pi-t2
    cr1=[x1+(radius*math.cos(t1)),(y1-radius*math.sin(t1))]
    cl1=[x1-(radius*math.cos(t1)),(y1+radius*math.sin(t1))]
    cr2=[x2+(radius*math.cos(t2)),(y2-radius*math.sin(t2))]
    cl2=[x2-(radius*math.cos(t2)),(y2+radius*math.sin(t2))]

    #finds c2c dist 
    crr=math.dist(cr1,cr2)
    clr=math.dist(cl1,cr2)
    crl=math.dist(cr1,cl2)
    cll=math.dist(cl1,cl2)

    #finds smallest c2c dist
    c2c=[crr,clr,crl,cll]
    c2c_min=c2c.index(min(c2c))
    #0=RSR 1=LSR 2=RSL 3=LSL
    
    if c2c_min==0: #RSR
        a=-math.atan2(cr2[1]-cr1[1],cr2[0]-cr1[0]) #outer tangent(tg)
        x_tg1=cr1[0]+(radius*math.sin(a))
        y_tg1=cr1[1]+(radius*math.cos(a))
        x_tg2=cr2[0]+(radius*math.sin(a))
        y_tg2=cr2[1]+(radius*math.cos(a))
        tg1=[x_tg1,y_tg1]
        tg2=[x_tg2,y_tg2]
        #print(tg1,tg2)
        #print(cr1,cl1,cr2,cl2)
        vect_start = [(x1-cr1[0]),(y1-cr1[1])] #centre of circle --> start_pos list
        vect_1=[(x_tg1-cr1[0]),(y_tg1-cr1[1])] #centre of circle --> tg1 list
        circle1_theta=math.atan2(vect_1[1],vect_1[0])-math.atan2(vect_start[1],vect_start[0])
        if circle1_theta>0:
            circle1_theta-=math.pi*2

        S_dist=math.dist(tg1,tg2)

        vect_2=[x_tg2-cr2[0],y_tg2-cr2[1]]
        vect_end=[x2-cr2[0],y2-cr2[1]]
        circle2_theta=math.atan2(vect_end[1],vect_end[0])-math.atan2(vect_2[1],vect_2[0])
        if circle2_theta>0:
            circle2_theta-=math.pi*2
        #print(circle1_theta,S_dist,circle2_theta)
        dir=['R','S','R']
        all_dist=[abs(circle1_theta)*radius,S_dist,abs(circle2_theta)*radius]
    
    elif c2c_min==1: #LSR
        a=math.atan2(cr2[1]-cl1[1],cr2[0]-cl1[0])+math.asin(2*radius/clr)-math.pi/2 
        x_tg1=cl1[0]+(radius*math.cos(a))
        y_tg1=cl1[1]+(radius*math.sin(a))
        x_tg2=cr2[0]+(radius*math.cos(a+math.pi))
        y_tg2=cr2[1]+(radius*math.sin(a+math.pi))
        tg1=[x_tg1,y_tg1]
        tg2=[x_tg2,y_tg2]
        #print(tg1,tg2)
        #print(cr1,cl1,cr2,cl2)
        vect_start = [(x1-cl1[0]),(y1-cl1[1])] #centre of circle --> start_pos list
        vect_1=[(x_tg1-cl1[0]),(y_tg1-cl1[1])] #centre of circle --> tg1 list
        circle1_theta=math.atan2(vect_1[1],vect_1[0])-math.atan2(vect_start[1],vect_start[0])
        if circle1_theta<0:
            circle1_theta+=math.pi*2

        S_dist=math.dist(tg1,tg2)

        vect_2=[x_tg2-cr2[0],y_tg2-cr2[1]]
        vect_end=[x2-cr2[0],y2-cr2[1]]
        circle2_theta=math.atan2(vect_end[1],vect_end[0])-math.atan2(vect_2[1],vect_2[0])
        if circle2_theta>0:
                circle2_theta-=math.pi*2
        #print(circle1_theta,S_dist,circle2_theta)
        dir=['L','S','R']
        all_dist=[abs(circle1_theta)*radius,S_dist,abs(circle2_theta)*radius]

    elif c2c_min==2: #RSL
        a=math.atan2(cl2[1]-cr1[1],cl2[0]-cr1[0])-math.asin(2*radius/crl)+math.pi/2 
        x_tg1=cr1[0]+(radius*math.cos(a))
        y_tg1=cr1[1]+(radius*math.sin(a))
        x_tg2=cl2[0]+(radius*math.cos(a+math.pi))
        y_tg2=cl2[1]+(radius*math.sin(a+math.pi))
        tg1=[x_tg1,y_tg1]
        tg2=[x_tg2,y_tg2]
        #print(tg1,tg2)
        #print(cr1,cl1,cr2,cl2)
        vect_start = [(x1-cr1[0]),(y1-cr1[1])] #centre of circle --> start_pos list
        vect_1=[(x_tg1-cr1[0]),(y_tg1-cr1[1])] #centre of circle --> tg1 list
        circle1_theta=math.atan2(vect_1[1],vect_1[0])-math.atan2(vect_start[1],vect_start[0])
        if circle1_theta>0:
            circle1_theta-=math.pi*2

        S_dist=math.dist(tg1,tg2)

        vect_2=[x_tg2-cl2[0],y_tg2-cl2[1]]
        vect_end=[x2-cl2[0],y2-cl2[1]]
        circle2_theta=math.atan2(vect_end[1],vect_end[0])-math.atan2(vect_2[1],vect_2[0])
        if circle2_theta<0:
                circle2_theta+=math.pi*2
        #print(circle1_theta,S_dist,circle2_theta)
        dir=['R','S','L']
        all_dist=[abs(circle1_theta)*radius,S_dist,abs(circle2_theta)*radius]

    elif c2c_min==3: #LSL
        a=-math.atan2(cr2[1]-cr1[1],cr2[0]-cr1[0]) #outer tangent(tg)
        x_tg1=cl1[0]-(radius*math.sin(a))
        y_tg1=cl1[1]-(radius*math.cos(a))
        x_tg2=cl2[0]-(radius*math.sin(a))
        y_tg2=cl2[1]-(radius*math.cos(a))
        tg1=[x_tg1,y_tg1]
        tg2=[x_tg2,y_tg2]
        #print(tg1,tg2)
        #print(cr1,cl1,cr2,cl2)
        vect_start = [(x1-cl1[0]),(y1-cl1[1])] #centre of circle --> start_pos list
        vect_1=[(x_tg1-cl1[0]),(y_tg1-cl1[1])] #centre of circle --> tg1 list
        circle1_theta=math.atan2(vect_1[1],vect_1[0])-math.atan2(vect_start[1],vect_start[0])
        if circle1_theta<0:
            circle1_theta+=math.pi*2

        S_dist=math.dist(tg1,tg2)

        vect_2=[x_tg2-cl2[0],y_tg2-cl2[1]]
        vect_end=[x2-cl2[0],y2-cl2[1]]
        circle2_theta=math.atan2(vect_end[1],vect_end[0])-math.atan2(vect_2[1],vect_2[0])
        if circle2_theta<0:
            circle2_theta+=math.pi*2
        #print(circle1_theta,S_dist,circle2_theta)
        dir=['L','S','L']
        all_dist=[abs(circle1_theta)*radius,S_dist,abs(circle2_theta)*radius]

    else:
        rospy.loginfo("no path found")
    
    return([dir,all_dist])


xyt=XYT()
odom=Odometry()

def xyt_callback(data:PoseStamped): #log aruco tag pose to xyt
    global xyt
    x_offset=0.08
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

def approach_tag():
    cmd=Twist()
    
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

def do_the_dubin(dir,all_dist,radius,speed):
    cmd=Twist()
    for x in range(3):
        if dir[x]=='R':
            rotate=-1
        elif dir[x]=='L':
            rotate=1
        else:
            rotate=0

        d_est=0
        theta_est=0
        dt=1/50 #odom rate
        rate=rospy.Rate(50) #odom rate
        if rotate!=0:
            while (all_dist[x]/radius)>abs(theta_est): #set distance threshold
                max_speed=0.2
                dtheta=odom.twist.twist.angular.z
                theta_est+=dtheta*dt
                cmd.linear.x=speed
                cmd.angular.z=speed/radius*rotate
                if abs(cmd.angular.z)>max_speed:
                    cmd.linear.x=max_speed*radius
                    cmd.angular.z=max_speed*rotate
                pub.publish(cmd)
                rate.sleep()
        else:
            while all_dist[x]>d_est:
                dl=odom.twist.twist.linear.x
                d_est+=dl*dt
                cmd.linear.x=speed
                pub.publish(cmd)
                rate.sleep()
        cmd.linear.x=0
        cmd.angular.z=0 
        pub.publish(cmd)
        rospy.sleep(0.3)
    cmd.linear.x=0
    cmd.angular.z=0 
    pub.publish(cmd)
            
def turn_to_tag(LoR):

    while abs(xyt.y)>0.05:
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
        

if __name__=='__main__':
    rospy.init_node('dubin_path_node')
    pub=rospy.Publisher('/cmd_vel',Twist, queue_size=10)
    xytsub=rospy.Subscriber('/aruco_single/pose',PoseStamped,callback=xyt_callback)
    odosub=rospy.Subscriber('/odom',Odometry,callback=odo_callback)
    rospy.loginfo("dubin path controller launched")
    cmd=Twist()
    has_init=1

    #spin till detect tag
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

    #align to tag
    set_distance=2
    speed=0.15
    while not (abs(xyt.y)<0.07 and abs(xyt.theta)<=math.radians(7)):
        targetx=xyt.y+set_distance*math.cos(xyt.theta+3*math.pi/2)
        targety=xyt.x+set_distance*math.sin(xyt.theta+3*math.pi/2)
        target_theta=(math.atan2(targety,targetx)-math.pi/2)
        #rospy.loginfo("Aruco tag x,y,theta:\n%s ",xyt)
        rospy.loginfo("tx,ty,tt: %s,%s,%s",targetx,targety,xyt.theta+math.pi/2)
        #rospy.loginfo("LoR: %s",LoR)
        p1=[0,0,0]
        p2=[targetx,targety,-xyt.theta]
        r=0.5
        [dir,all_dist]=shortest_dubin_path(p1,p2,r)
        print(dir,all_dist)
        rospy.loginfo("dir,all_dist: %s,%s",dir,all_dist)
        do_the_dubin(dir,all_dist,r,speed)
        set_distance-=0.2
        speed-=0.02
        rospy.sleep(1)
        if set_distance<1:
            break

    rospy.loginfo("start approaching tag")
    approach_tag()
    #print(path[1])
    #rospy.spin()
    #sol=(dubins_path(p1,p2,radius))
    
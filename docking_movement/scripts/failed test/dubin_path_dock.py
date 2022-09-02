#!/usr/bin/env python3

import rospy
import math
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from docking_movement.msg import XYT
from tf.transformations import euler_from_quaternion, quaternion_from_euler

def mod2pi(theta):
	return theta - 2.0 * math.pi * math.floor(theta / 2.0 / math.pi)


def pi_2_pi(angle):
	while(angle >= math.pi):
		angle = angle - 2.0 * math.pi

	while(angle <= -math.pi):
		angle = angle + 2.0 * math.pi

	return angle

def general_planner(planner, alpha, beta, d):
	sa = math.sin(alpha)
	sb = math.sin(beta)
	ca = math.cos(alpha)
	cb = math.cos(beta)
	c_ab = math.cos(alpha - beta)
	mode = list(planner)
	#print(mode)

	planner_uc = planner.upper()

	if planner_uc == 'LSL':
		tmp0 = d + sa - sb
		p_squared = 2 + (d * d) - (2 * c_ab) + (2 * d * (sa - sb))
		if p_squared < 0:
			return None
		tmp1 = math.atan2((cb - ca), tmp0)
		t = mod2pi(-alpha + tmp1)
		p = math.sqrt(p_squared)
		q = mod2pi(beta - tmp1)
		#  print(math.degrees(t), p, math.degrees(q))

	elif planner_uc == 'RSR':
		tmp0 = d - sa + sb
		p_squared = 2 + (d * d) - (2 * c_ab) + (2 * d * (sb - sa))
		if p_squared < 0:
			return None
		tmp1 = math.atan2((ca - cb), tmp0)
		t = mod2pi(alpha - tmp1)
		p = math.sqrt(p_squared)
		q = mod2pi(-beta + tmp1)

	elif planner_uc == 'LSR':
		p_squared = -2 + (d * d) + (2 * c_ab) + (2 * d * (sa + sb))
		if p_squared < 0:
			return None
		p = math.sqrt(p_squared)
		tmp2 = math.atan2((-ca - cb), (d + sa + sb)) - math.atan2(-2.0, p)
		t = mod2pi(-alpha + tmp2)
		q = mod2pi(-mod2pi(beta) + tmp2)

	elif planner_uc == 'RSL':
		p_squared = (d * d) - 2 + (2 * c_ab) - (2 * d * (sa + sb))
		if p_squared < 0:
			return None
		p = math.sqrt(p_squared)
		tmp2 = math.atan2((ca + cb), (d - sa - sb)) - math.atan2(2.0, p)
		t = mod2pi(alpha - tmp2)
		q = mod2pi(beta - tmp2)

	elif planner_uc == 'RLR':
		tmp_rlr = (6.0 - d * d + 2.0 * c_ab + 2.0 * d * (sa - sb)) / 8.0
		if abs(tmp_rlr) > 1.0:
			return None

		p = mod2pi(2 * math.pi - math.acos(tmp_rlr))
		t = mod2pi(alpha - math.atan2(ca - cb, d - sa + sb) + mod2pi(p / 2.0))
		q = mod2pi(alpha - beta - t + mod2pi(p))

	elif planner_uc == 'LRL':
		tmp_lrl = (6. - d * d + 2 * c_ab + 2 * d * (- sa + sb)) / 8.
		if abs(tmp_lrl) > 1:
			return None
		p = mod2pi(2 * math.pi - math.acos(tmp_lrl))
		t = mod2pi(-alpha - math.atan2(ca - cb, d + sa - sb) + p / 2.)
		q = mod2pi(mod2pi(beta) - alpha - t + mod2pi(p))

	else:
		print('bad planner:', planner)

	path = [t, p, q]

	# Lowercase directions are driven in reverse.
	for i in [0, 2]:
		if planner[i].islower():
			path[i] = (2 * math.pi) - path[i]
	# This will screw up whatever is in the middle.

	cost = sum(map(abs, path))

	return(path, mode, cost)

def dubins_path(start, end, radius):
    (sx, sy, syaw) = start
    (ex, ey, eyaw) = end
    c = radius

    ex = ex - sx
    ey = ey - sy

    lex = math.cos(syaw) * ex + math.sin(syaw) * ey
    ley = - math.sin(syaw) * ex + math.cos(syaw) * ey
    leyaw = eyaw - syaw
    D = math.sqrt(lex ** 2.0 + ley ** 2.0)
    d = D / c
    #print('D:', D)

    theta = mod2pi(math.atan2(ley, lex))
    alpha = mod2pi(- theta)
    beta = mod2pi(leyaw - theta)

    #planners = ['RSr', 'rSR', 'rSr', 'LSL', 'RSR', 'LSR', 'RSL', 'RLR', 'LRL']
    planners = ['LSL', 'RSR', 'LSR', 'RSL', 'RLR', 'LRL']
    #planners = ['RSr']

    bcost = float("inf")
    bt, bp, bq, bmode = None, None, None, None

    for planner in planners:
        #t, p, q, mode = planner(alpha, beta, d)
        solution = general_planner(planner, alpha, beta, d)

        if solution is None:
            continue

        (path, mode, cost) = solution
        (t, p, q) = path
        if bcost > cost:
            # best cost
            bt, bp, bq, bmode = t, p, q, mode
            bcost = cost

    #  print(bmode)
    return(zip(bmode, [bt*c, bp*c, bq*c], [c] * 3))

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
    KP_y=1.1
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

def do_the_dubin(dir,dist,radius,speed):
	cmd=Twist()
	for x in range(3):
		if dir[x]=='R':
			rotate=-1
		elif dir[x]=='L':
			rotate=1
		else:
			rotate=0

		d_est=0
		dt=1/50 #odom rate
		rate=rospy.Rate(50) #odom rate
		while abs(d_est-dist[x])>0.05: #set distance threshold
			dl=odom.twist.twist.linear.x
			d_est+=dl*dt
			cmd.linear.x=speed
			cmd.angular.z=speed/radius*rotate
			pub.publish(cmd)
			rate.sleep()
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
	speed=0.2
	while not (abs(xyt.y)<0.05 and abs(xyt.theta)<=math.radians(5)):
		targetx=xyt.y+set_distance*math.cos(xyt.theta+3*math.pi/2)
		targety=xyt.x+set_distance*math.sin(xyt.theta+3*math.pi/2)
		target_theta=(math.atan2(targety,targetx)-math.pi/2)
		#rospy.loginfo("Aruco tag x,y,theta:\n%s ",xyt)
		rospy.loginfo("tx,ty,tt: %s,%s,%s",targetx,targety,xyt.theta+math.pi/2)
		#rospy.loginfo("LoR: %s",LoR)
		p1=(0,0,math.pi/2)
		p2=(targetx,targety,math.pi/2+xyt.theta)
		r=0.2
		path=list(zip(*dubins_path(p1,p2,r)))
		do_the_dubin(path[0],path[1],r,speed)
		set_distance-=0.2	
		speed-=0.05

	rospy.loginfo("start approaching tag")
	approach_tag()
	print(path[1])
	#rospy.spin()
	#sol=(dubins_path(p1,p2,radius))
    
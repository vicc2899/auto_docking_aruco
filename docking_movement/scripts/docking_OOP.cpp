#include <ros/ros.h>
#include <cmath>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <docking_movement/XYT.h>
#include <tf/tf.h>
#include <ros/console.h>

class Dock_Robot{

    private:
        ros::Subscriber xytsub;
        ros::Subscriber odosub;
        ros::Publisher pub;
        docking_movement::XYT xyt;
        nav_msgs::Odometry odom;

    public:
        float x_offset;
        float y_offset;
        int tries;
        Dock_Robot(ros::NodeHandle *nh,float x,float y){
            tries=0;
            x_offset=x;
            y_offset=y;
            xytsub= nh->subscribe("/aruco_single/pose",1000,&Dock_Robot::xyt_callback,this);
            odosub=nh->subscribe("/raw_odom",1000,&Dock_Robot::odo_callback,this);
        }
    
        void xyt_callback(const geometry_msgs::PoseStamped& msg){
            xyt.x=msg.pose.position.z+x_offset;
            xyt.y=msg.pose.position.x+y_offset;
             tf::Quaternion q(
                msg.pose.orientation.x,
                msg.pose.orientation.y,
                msg.pose.orientation.z,
                msg.pose.orientation.w);
            tf::Matrix3x3 m(q);
            double roll, pitch, yaw;
            m.getRPY(roll, pitch, yaw);
            xyt.theta=-pitch;
        }
        
        void odo_callback(const nav_msgs::Odometry& msg){
            odom=msg;
            odom.twist.twist.linear.x=-msg.twist.twist.linear.x;
        }

        int sign_function(int x) {
            if (x>0){
                return 1;
            }
            else if (x==0)
            {
                return 0;
            }
            else {
                return -1;
            }
        }

        void reset_xyt(){
            docking_movement::XYT xyt;
        }

        float modpi(float angle){
            angle=fmod(angle,(M_PI*2));
            if (angle>M_PI){
                angle=angle-2*M_PI;
            }
            else if (angle<-M_PI){
                angle=angle+2*M_PI;
            }
            return angle;
        }

        void turn(float angle){
            geometry_msgs::Twist cmd;
            angle=modpi(angle);
            float t_est=0;
            float dt=1/100; //odom rate
            ros::Rate rate(100);
            while (abs((t_est-angle)>(1*M_PI/180)))
            {
                float dtheta=odom.twist.twist.angular.z*1.17647; //correction factor for ang speed
                t_est+=dtheta*dt;
                cmd.angular.z=0.3*sign_function(angle); //edit sign later
                pub.publish(cmd);
            }
            
        }

        void move(float distance){
            geometry_msgs::Twist cmd;
            float d_est=0;
            float dt=1/100; // odom rate
            ros::Rate rate(100);
            while (abs(abs(d_est)-abs(distance))>0.01)
            {
                float dl=odom.twist.twist.linear.x*1; //correction factor
                d_est+=dl*dt;
                cmd.linear.x=-0.2*sign_function(distance);
                pub.publish (cmd);
                rate.sleep();
            }
            cmd.angular.z=0;
            cmd.linear.x=0;
            pub.publish(cmd);
        }

        void turn_to_tag(int LoR){
            geometry_msgs::Twist cmd;
            while (abs(xyt.y)>0.05){
                cmd.linear.x=0;
                cmd.angular.z=-0.15*LoR;
                pub.publish(cmd);
            }
            cmd.linear.x=0;
            cmd.angular.z=0;
            pub.publish(cmd);
            ros::Duration(0.5).sleep();
            while (abs(xyt.y)>0.01){
                cmd.linear.x=0;
                cmd.angular.z=-0.04*sign_function(xyt.y);
                pub.publish(cmd);
            }
            cmd.linear.x=0;
            cmd.angular.z=0 ;
            pub.publish(cmd);
        }

        void approach_tag(){
            geometry_msgs::Twist cmd;
            //Lateral errors
            float KP_y=0.2;
            float KI_y=0.5;
            float KD_y=0.05;
            float pe_y=0;
            float sum_e_y=0;
            //Angle Errors
            float KP_t=0.6;
            float KI_t=0;
            float KD_t=0.05;
            float pe_t=0;
            float sum_e_t=0;

            cmd.linear.x=-0.1; //linear speed
            float dt=1/100;
            ros::Rate rate(100);
            float prev_x=0;
            float tag_error=0;
            while (xyt.x>0.6)
            {
                float e_y=-xyt.y; //y error
                float e_t=xyt.theta; //angle error
                cmd.angular.z=(KP_y*e_y+(e_y-pe_y)/dt*KD_y+sum_e_y*KI_y + KP_t*e_t+(e_t-pe_t)/dt*KD_t+sum_e_t*KI_t);
                pub.publish(cmd);
                pe_t=e_t;
                sum_e_t+=e_t*dt;
                pe_y=e_y;
                sum_e_y+=e_y*dt;
                if (xyt.x==prev_x)
                {
                    tag_error+=1;
                    if (tag_error>=50)
                    {
                        ROS_DEBUG("ArUco Tag Out of View");
                        cmd.angular.z=0;
                        cmd.linear.x=0;
                        pub.publish(cmd);
                        ROS_DEBUG("%s",xyt);
                        break;
                    }
                }
                else{
                    tag_error=0;
                    prev_x=xyt.x;
                    rate.sleep();
                    cmd.angular.z=0;
                    cmd.linear.x=0;
                    pub.publish(cmd);
                    ROS_DEBUG("Docking Completed");
                    ROS_DEBUG("%s",xyt);
                }
                
                    
            }
        }

        void Start_Docking(){
            geometry_msgs::Twist cmd;
            bool has_init=1;
            ros::Duration(1).sleep();
            while (xyt.x==0){
                if (has_init){
                    ROS_INFO_STREAM("spinning...");
                    has_init=0;
                } 
                cmd.angular.z=0.12;
                cmd.linear.x=0;
                pub.publish(cmd);
            }
            cmd.angular.z=0;
            cmd.linear.x=0;
            pub.publish(cmd);
            ros::Duration(2).sleep();
            ROS_INFO_STREAM(atan2(xyt.y,xyt.x)+xyt.theta);

            if ((atan2(xyt.y,xyt.x)+xyt.theta)<=(4/180*M_PI)){
                ROS_INFO_STREAM("already centered");
                float target_theta=(atan2(xyt.x,xyt.y)-M_PI/2);
                float center_theta=modpi(target_theta);
                int LoR=sign_function(center_theta);
                turn_to_tag(-LoR);
            }

            int set_distance=2;
            tries=0;

            while (not (abs(xyt.y)<0.3 and abs(xyt.theta)<=4/180*M_PI)){ //change accetable tolerance here
                tries+=1;
                float targetx=xyt.y+set_distance*cos(xyt.theta+3*M_PI/2);
                float targety=xyt.x+set_distance*sin(xyt.theta+3*M_PI/2);
                float target_theta=(atan2(targety,targetx)-M_PI/2);
                ROS_INFO_STREAM("Aruco tag x,y,theta:\n"<<xyt);
                ROS_INFO_STREAM("tx,ty,tt:\n"<<targetx<<targety<<target_theta);
                float center_theta=modpi(target_theta);
                float center_dist=sqrt(pow(targetx,2)+pow(targety,2));
                ROS_INFO_STREAM("dist,theta: "<<center_dist<<center_theta);
                turn(center_theta);
                ros::Duration(1).sleep();
                move(center_dist);
                ros::Duration(1).sleep();
                int LoR=sign_function(center_theta);
                turn_to_tag(LoR);
                ros::Duration(1).sleep();
                set_distance-=0.1;
            }
            ROS_INFO_STREAM("start approaching tag");
            approach_tag();
        }
        
};

int main(int argc, char **argv)
{   ros::NodeHandle nh;
    ros::init(argc,argv,"docking_controller");
    Dock_Robot tars4=Dock_Robot(&nh,0.22,0);
    tars4.Start_Docking();
}
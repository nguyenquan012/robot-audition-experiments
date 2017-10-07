#include <ros/ros.h>
#include "nav_msgs/Path.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Int8.h"

nav_msgs::Path mypath;
std_msgs::Float32 turtle_speed;
nav_msgs::Path prepath_msg;
std::string speed;

bool path_created = false;
bool direction;
bool path2 = false;
bool path3 = false;
bool turtlebot;

std_msgs::Int8 reachmsg;
int point_num = -1;
int reach_point = 0;
int path_num = 1;


//continuous forward path
double pos_x[] = {6.3, 6.3, 6.3,   6.3,   4.5,   4.5,  1.48 , 1.48};
double pos_y[] = {5,   5,   3.185, 3.185, 1.385, 1.385,1.385, 1.385};

//continuous reverse path
double revpos_x[] = {1.48 , 1.48,  4.5,   4.5,    6.3,   6.3,   6.3, 6.3};
double revpos_y[] = {1.385, 1.385, 1.385, 1.385,  3.185, 3.185, 5,   5 };

//discontinuous forward path
double pos_x1[] = {6.3,   6.3,   6.3,   6.3};
double pos_y1[] = {5,     5,     3.185, 3.185};
double pos_x2[] = {6.3,   6.3,   4.5,   4.5};
double pos_y2[] = {3.185, 3.185, 1.385, 1.385};
double pos_x3[] = {4.5,   4.5,   1.48,  1.48};
double pos_y3[] = {1.385, 1.385, 1.385, 1.385};

//discontinuous reverse path
double revpos_x1[] = {1.48,  1.48,  4.5,   4.5};
double revpos_y1[] = {1.385, 1.385, 1.385, 1.385};
double revpos_x2[] = {4.5,   4.5,   6.3,   6.3};
double revpos_y2[] = {1.385, 1.385, 3.185, 3.185};
double revpos_x3[] = {6.3,   6.3,   6.3,   6.3};
double revpos_y3[] = {3.185, 3.185, 5,     5};

/**
 * Construct_Path_Msg function
 * Used to populate a nav_msgs::Path given a list of x and y coordinates
 * @param x double* pointer to array containing x coordinates
 * @param y double* pointer to array containing y coordinates
 * @return msg the constructed nav_msgs::Path message
 */
nav_msgs::Path Construct_Path_Msg(double* x, double *y, int length)
{
    nav_msgs::Path msg;
    std::vector<geometry_msgs::PoseStamped> poses(length);
    msg.header.frame_id="/map";
    for (int i = 0; i < length; i++)
    {
        poses.at(i).pose.position.x = x[i];
        poses.at(i).pose.position.y = y[i];
        poses.at(i).pose.orientation.w = 1.0;
        ROS_INFO("%f %f\n",x[i],y[i]);

    }
    msg.poses = poses;
    return msg;
}

/**
 * Construct discontinuous path function
 *
 */
void Construct_Path_Disc(int path_num, bool forward)

{

    if(path_num==1)
    {
        if(forward)
        {
            prepath_msg = Construct_Path_Msg(pos_x1, pos_y1, sizeof(pos_x1)/sizeof(double));  //forward direction
            ROS_INFO("Turtlebot travels discontinuously with %s speed %.3f m/s, forward direction.", speed.c_str(), turtle_speed.data);
        }
        else
        {
            prepath_msg = Construct_Path_Msg(revpos_x1, revpos_y1, sizeof(revpos_x1)/sizeof(double));  //forward direction
            ROS_INFO("Turtlebot travels discontinuously with %s speed %.3f m/s, reverse direction.", speed.c_str(), turtle_speed.data);
        }
    }
    if(path_num==2)
    {
        if(forward)
        {
            prepath_msg = Construct_Path_Msg(pos_x2, pos_y2, sizeof(pos_x2)/sizeof(double));  //forward direction
            ROS_INFO("Turtlebot travels discontinuously with %s speed %.3f m/s, forward direction.", speed.c_str(), turtle_speed.data);
        }
        else
        {
            prepath_msg = Construct_Path_Msg(revpos_x2, revpos_y2, sizeof(revpos_x2)/sizeof(double));  //forward direction
            ROS_INFO("Turtlebot travels discontinuously with %s speed %.3f m/s, reverse direction.", speed.c_str(), turtle_speed.data);
        }
    }
    if(path_num==3)
    {
        if(forward)
        {
            prepath_msg = Construct_Path_Msg(pos_x3, pos_y3, sizeof(pos_x3)/sizeof(double));  //forward direction
            ROS_INFO("Turtlebot travels discontinuously with %s speed %.3f m/s, forward direction.", speed.c_str(), turtle_speed.data);
        }
        else
        {
            prepath_msg = Construct_Path_Msg(revpos_x3, revpos_y3, sizeof(revpos_x3)/sizeof(double));  //forward direction
            ROS_INFO("Turtlebot travels discontinuously with %s speed %.3f m/s, reverse direction.", speed.c_str(), turtle_speed.data);
        }
    }
}

/**
 * reachCallback function
 *
 */
void reachCallback(const std_msgs::Int8ConstPtr &msg)
{
            reachmsg = *msg;
            ROS_INFO("Reached target %d!",path_num);
}

/**
 * Main Function
 * Check the speed mode and direction got from command line
 * then publish proper speed and path message to master
 */
int main(int argc, char **argv)
{
    ros::init(argc, argv, "prepath_follow");
    ros::NodeHandle n;
    ros::NodeHandle nh("~");  //to handle local or private parameter

    //set default value for parameters
    nh.param<std::string>("speed", speed, "normal");
    nh.param("direction", direction, true);
    nh.param("turtlebot", turtlebot, true);

    nh.getParam("turtlebot", turtlebot);     //get value of parametter turtlebot-2
    if(turtlebot) std::cout << "it's turtlebot-2" << "\n";
    else std::cout << "it's turtlebot-1" << "\n";

    ros::Subscriber sub1,sub2;
    ros::Publisher path_pub, speed_pub;

    nh.getParam("speed", speed);   //get speed parameter from command line
    nh.getParam("direction", direction);     //get direction parameter from command line, 1 for forward, 0 for reverse



    //check speed_mode
    // vfast = 0.4m/s, vslow = 0.2m/s, vdefault = 0.14
    if(speed == "fast")
    {
        turtle_speed.data = 0.3;
    }

    else if(speed == "slow")
    {
        turtle_speed.data = 0.15;
    }

    else if(speed == "discontinuous")
    {
        turtle_speed.data = 0.3;
    }

    else
    {
        turtle_speed.data = 0.14;   //default to nomal speed if don't set the parameter speed
    }

    //check turtlebot was used
    if(!turtlebot)
    {
        ROS_INFO("going to publish to turtlebot-1");
       // sub1 = n.subscribe("/turtlebot/1/slamomatic/path", 1000, pathCallback);
        sub2 = n.subscribe("/turtlebot/1/navomatic/reached", 1000, reachCallback);
        path_pub = n.advertise<nav_msgs::Path>("/turtlebot/1/slamomatic/path", 1, true);
        speed_pub = n.advertise<std_msgs::Float32>("/turtlebot/1/turtlespeed", 1, true);
    }
    else
    {
        ROS_INFO("going to publish to turtlebot-2");
        //sub1 = n.subscribe("/turtlebot/2/slamomatic/path", 1000, pathCallback);
        sub2 = n.subscribe("/turtlebot/2/navomatic/reached", 1000, reachCallback);
        path_pub = n.advertise<nav_msgs::Path>("/turtlebot/2/slamomatic/path", 2, true);
        speed_pub = n.advertise<std_msgs::Float32>("/turtlebot/2/turtlespeed", 2, true);
    }
    //check direction of travel
    //draw prepath using nav_msg::Path

    if(speed != "discontinuous")
    {
        if(!direction)
        {
            prepath_msg = Construct_Path_Msg(revpos_x, revpos_y, sizeof(revpos_x)/sizeof(double)); //reverse direction
            ROS_INFO("Turtlebot travels with %s speed %.3f m/s, reverse direction.", speed.c_str(), turtle_speed.data);
            speed_pub.publish(turtle_speed);path_pub.publish(prepath_msg);
        }

        else
        {
            prepath_msg = Construct_Path_Msg(pos_x, pos_y, sizeof(pos_x)/sizeof(double));  //forward direction
            ROS_INFO("Turtlebot travels with %s speed %.3f m/s, forward direction.", speed.c_str(), turtle_speed.data);
            speed_pub.publish(turtle_speed);path_pub.publish(prepath_msg);
        }

    }

    /*********discontinuous path****************/
        ros::Rate loop_rate(1);
        int count=1;
        if(speed == "discontinuous")
        {
            while (ros::ok())
            {
                if((count==1)&&(direction))
                {
                    turtle_speed.data = 0.3;speed_pub.publish(turtle_speed);
                    if(path_num==1)
                    {
                        Construct_Path_Disc(1, true);path_pub.publish(prepath_msg);

                    }
                    if(path_num==2)
                    {
                        Construct_Path_Disc(2, true);path_pub.publish(prepath_msg);

                    }
                    if(path_num==3)
                    {
                        Construct_Path_Disc(3, true);path_pub.publish(prepath_msg);

                    }
                    count++;
                }

                if((count==1)&&(!direction))
                {
                    turtle_speed.data = 0.3;speed_pub.publish(turtle_speed);
                    if(path_num==1)
                    {
                        Construct_Path_Disc(1, false);path_pub.publish(prepath_msg);

                    }
                    if(path_num==2)
                    {
                        Construct_Path_Disc(2, false);path_pub.publish(prepath_msg);

                    }
                    if(path_num==3)
                    {
                        Construct_Path_Disc(3, false);path_pub.publish(prepath_msg);
                    }
                    count++;
                }


                if((path_num<=3)&&(reachmsg.data == 1))
                {

                    ROS_INFO("Finished %d path!",path_num);
                    turtle_speed.data = 0;
                    speed_pub.publish(turtle_speed);

                    //sleep for 15 seconds
                    ROS_INFO("wait for 15 secs...");
                    ros::Duration(15).sleep();

                    ROS_INFO("ready for next path");
                    path_num++;count=1;
                    reachmsg.data = 0;
                }

                ros::spinOnce();
            }
        }
    ros::spinOnce();
    loop_rate.sleep();
    nh.deleteParam("speed");
    nh.deleteParam("direction");
    return 0;
}


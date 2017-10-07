#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>

bool gotpose = false;
bool cw;
class TurtlebotDriver
{
private:
  //! The node handle we'll be using
  ros::NodeHandle nh_;
  //! We will be publishing to the "/base_controller/command" topic to issue commands
  ros::Publisher cmd_vel_pub_;
  ros::Subscriber pose_sub;

  tf::TransformListener listener_;
  geometry_msgs::PoseStamped currentpos;
  double current_x;double current_y;double current_angle;

public:
  //! ROS node initialization
  TurtlebotDriver(bool turtlebot, ros::NodeHandle &nh)
  {
    nh_ = nh;
    //set up the publisher for the cmd_vel topic
    //cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    if(!turtlebot)
    {
        //pose_sub = nh_.subscribe("/turtlebot/1/slamomatic/pose", 1000, poseCallback);
        cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/turtlebot/1/mobile_base/commands/velocity", 1);
        std::cout << "Control turtlebot-1" << "\n";
       // sub = ndh.subscribe("/turtlebot/2/slamomatic/pose", 1000, &TurtlebotDriver::poseCallback, &driver);
    }

    if(turtlebot)
    {
        //pose_sub = nh_.subscribe("/turtlebot/2/slamomatic/pose", 1000, poseCallback);
        cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/turtlebot/2/mobile_base/commands/velocity", 1);
        std::cout << "Control turtlebot-2" << "\n";
    }
  }



  /**
   *  Drive forward a specified distance based on odometry information
   *
   */
  bool driveForwardOdom(bool forward, double distance)
  {
      //wait for the listener to get the first message
      listener_.waitForTransform("base_footprint", "odom",
                                 ros::Time(0), ros::Duration(1.0));

      //we will record transforms here
      tf::StampedTransform start_transform;
      tf::StampedTransform current_transform;

      //record the starting transform from the odometry to the base frame
      listener_.lookupTransform("base_footprint", "odom",
                                ros::Time(0), start_transform);

      //we will be sending commands of type "twist"
      geometry_msgs::Twist base_cmd;
      if (forward)
      {
    //the command will be to go forward at 0.08 m/s
      	base_cmd.linear.y = base_cmd.angular.z = 0;
        base_cmd.linear.x = 0.08;
      }
      if (!forward)
      {
	//the command will be to go backward at 0.15 m/s
      	base_cmd.linear.y = base_cmd.angular.z = 0;
        base_cmd.linear.x = -0.08;

      }

      ros::Rate rate(10.0);
      bool done = false;
      while (!done && nh_.ok())
      {
        //send the drive command
        cmd_vel_pub_.publish(base_cmd);
        rate.sleep();
        //get the current transform
        try
        {
          listener_.lookupTransform("base_footprint", "odom",
                                    ros::Time(0), current_transform);
        }
        catch (tf::TransformException ex)
        {
          ROS_ERROR("%s",ex.what());
          break;
        }
        //see how far we've traveled
        tf::Transform relative_transform =
          start_transform.inverse() * current_transform;
        double dist_moved = relative_transform.getOrigin().length();

        if(dist_moved > distance) done = true;
      }
      if (done) return true;
      return false;
  }


  bool turnOdom(bool clockwise, double radians)
  {
      while(radians < 0) radians += 2*M_PI;
      while(radians > 2*M_PI) radians -= 2*M_PI;

      //wait for the listener to get the first message
      listener_.waitForTransform("base_footprint", "odom",
                                 ros::Time(0), ros::Duration(1.0));

      //we will record transforms here
      tf::StampedTransform start_transform;
      tf::StampedTransform current_transform;

      //record the starting transform from the odometry to the base frame
      listener_.lookupTransform("base_footprint", "odom",
                                ros::Time(0), start_transform);

      //we will be sending commands of type "twist"
      geometry_msgs::Twist base_cmd;
      //the command will be to turn at 0.75 rad/s
      base_cmd.linear.x = base_cmd.linear.y = 0.0;
      base_cmd.angular.z = 0.65;
      if (clockwise) base_cmd.angular.z = -base_cmd.angular.z;

      //the axis we want to be rotating by
      tf::Vector3 desired_turn_axis(0,0,1);
      if (!clockwise) desired_turn_axis = -desired_turn_axis;

      ros::Rate rate(100.0);
      bool done = false;
      while (!done && nh_.ok())
      {
        //send the drive command
        cmd_vel_pub_.publish(base_cmd);
        rate.sleep();
        //get the current transform
        try
        {
          listener_.lookupTransform("base_footprint", "odom",
                                    ros::Time(0), current_transform);
        }
        catch (tf::TransformException ex)
        {
          ROS_ERROR("%s",ex.what());
          break;
        }
        tf::Transform relative_transform = start_transform.inverse() * current_transform;
        tf::Vector3 actual_turn_axis = relative_transform.getRotation().getAxis();
        double angle_turned = relative_transform.getRotation().getAngle();
        if ( fabs(angle_turned) < 1.0e-2) continue;

        if ( actual_turn_axis.dot( desired_turn_axis ) < 0 )
          angle_turned = 2 * M_PI - angle_turned;

        if (angle_turned > (radians - 4.0e-1)) done = true;
      }
      if (done) return true;
      return false;
  }

  double angleRotate(bool rotate, double delta_angle)
  {
      rotate = cw;
      if(delta_angle < -M_PI)
      {
        cw = false;
        delta_angle = 2*M_PI - fabs(delta_angle);
      }
      if(-M_PI < delta_angle < 0)
      {
        cw = true;
        delta_angle = fabs(delta_angle);
      }
      if(0 < delta_angle < M_PI)
      {
        cw = false;
        delta_angle = delta_angle;
      }
      if(delta_angle > M_PI)
      {
        cw = true;
        delta_angle = 2*M_PI - delta_angle;
      }
      return delta_angle;
  }

  /**
   * poseCallback function
   *
   */
  void poseCallback(const geometry_msgs::PoseStampedConstPtr &msg)
  {
      currentpos=*msg;
      ROS_INFO("getting current pose");
      current_x = currentpos.pose.position.x;
      current_y = currentpos.pose.position.y;
      current_angle = 2*atan2(currentpos.pose.orientation.z, currentpos.pose.orientation.w);
      while(current_angle < 0) current_angle += 2*M_PI;
      while(current_angle > 2*M_PI) current_angle -= 2*M_PI;
      ROS_INFO("current position x = %f, y = %f, angle = %f ",current_x, current_y, current_angle);
      ROS_INFO("current w  %f", currentpos.pose.orientation.w);
      gotpose = true;

  }


  bool goClose(double delta_x, double delta_y, double delta_angle) // in global coordinates
  {

      /* double position_tolerance = 0.1; // if we're closer than 5 cm, don't try to correct position

      if ((delta_x*delta_x + delta_y*delta_y)<position_tolerance*position_tolerance)
      {
          // just turn to correct orientation
          turnOdom(true, delta_angle);
      }
      else
      {  */
        // turn to face correct position
        double first_angle = angleRotate(cw, atan2(delta_y, delta_x) - current_angle);
        //turnOdom(true, fabs(first_angle)); // TODO check sign
        ROS_INFO("First angle = %f",first_angle);
        // move to correct position
        double distance = sqrt(delta_x*delta_x + delta_y*delta_y);
        ROS_INFO("Distance to move: %f", distance);
        //driveForwardOdom(true, distance); // TODO check call arguments
        // turn to correct orientation
        double remaining_angle = angleRotate(cw, delta_angle - first_angle);
        ROS_INFO("remain angle to move: %f",remaining_angle);
        //turnOdom(true, fabs(remaining_angle)); // TODO check sign
        ROS_INFO("delta angle = %f",delta_angle);
        delta_angle = angleRotate(cw, delta_angle);
        turnOdom(cw, delta_angle); // TODO check sign
     //}
  }


  bool goSomewhere(double x, double y, double angle) // position in global coordinates
  {
    // compute relative motion (still in global coordinates)
    double delta_x = x - current_x;
    double delta_y = y - current_y;
    double delta_angle = angle - current_angle;
    // execute motion
    goClose(delta_x, delta_y, delta_angle);
  }



  //! Loop forever while sending drive commands based on keyboard input
  bool driveKeyboard()
  {

    std::cout << "Type a command and then press enter.  "
      "Use 'e' to move forward, Use 'd' to move backward,'s' to turn left, "
      "'f' to turn right, 'q' to exit.\n";

    //we will be sending commands of type "twist"
    geometry_msgs::Twist base_cmd;

    char cmd[50];
    while(nh_.ok()){

      std::cin.getline(cmd, 50);
      if(cmd[0]!='e' && cmd[0]!='d' && cmd[0]!='s' && cmd[0]!='f' && cmd[0]!='q')
      {
        std::cout << "unknown command:";// << cmd << "\n";
        continue;
      }

      base_cmd.linear.x = base_cmd.linear.y = base_cmd.angular.z = 0;   
      //move forward
      if(cmd[0]=='e'){
        driveForwardOdom(true, 2);
        //base_cmd.linear.x = 0.25;
      } 
	//move backward
      if(cmd[0]=='d'){
        driveForwardOdom(false, 2);
        //base_cmd.linear.x = 0.25;
      } 
      //turn left (yaw) and drive forward at the same time
      else if(cmd[0]=='s'){
        //base_cmd.angular.z = 0.75;
        turnOdom(false, M_PI/8);
      } 
      //turn right (yaw) and drive forward at the same time
      else if(cmd[0]=='f'){
        //base_cmd.angular.z = -0.75;
        turnOdom(true, M_PI/8);
        //base_cmd.linear.x = 0.25;
      } 
      //quit
      else if(cmd[0]=='q'){
        break;
      }

      //publish the assembled command
      cmd_vel_pub_.publish(base_cmd);

    }
    return true;
  }

 };

int main(int argc, char** argv)
{
  bool turtlebot=true;
  bool direction;
    //init the ROS node
  ros::init(argc, argv, "turtlebot_move_simple");
  ros::NodeHandle ndh;
  ros::NodeHandle ndh_("~");  //to handle local or private parameter
  ros::Subscriber sub;
  //std::string reposition;
  double x;double y; double angle;

  ndh_.param("direction", direction, true);
  ndh_.param("turtlebot", turtlebot, true); //set default to turtlebot-1
 // nh.param("reposition", reposition, "start");    //set default forward

 // ROS_INFO("direction %d", direction);
  TurtlebotDriver driver(turtlebot,ndh);

  //ros::Duration(1).sleep();
  ndh_.getParam("turtlebot", turtlebot);     //get value of parametter turtlebot-1
  //ndh_.getParam("reposition", reposition);   //get repostion parameter from command line
  ndh_.getParam("direction", direction);     //get direction parameter from command line, 1 for forward, 0 for reverse
  //ros::Duration(5).sleep();
  ROS_INFO("direction %d", direction);
  if(turtlebot)
  {
    std::cout << "it's turtlebot-2" << "\n";
    sub = ndh.subscribe("/turtlebot/2/slamomatic/pose", 1000, &TurtlebotDriver::poseCallback, &driver);
    ROS_INFO("subscrible to turtlebot-2");
    driver.driveForwardOdom(true, 2);
    driver.turnOdom(true, M_PI);ros::Duration(10).sleep();
    driver.driveForwardOdom(true, 2);
    driver.turnOdom(true, M_PI);ros::Duration(10).sleep();
  }
  else
  {
    std::cout << "it's turtlebot-1" << "\n";
    sub = ndh.subscribe("/turtlebot/1/slamomatic/pose", 1000, &TurtlebotDriver::poseCallback, &driver);
    ROS_INFO("subscrible to turtlebot-1");

    if(direction)
    {
      x = 6.3;
      y = 5;
      angle = 3*M_PI/2;
      ROS_INFO("adjust turtlebot at start position: x = %2f, y = %2f, orient = %2f",x, y, angle);
    }
    else
    {
      x = 1.48;
      y = 1.385;
      angle = 0;
      ROS_INFO("adjust turtlebot at end position: x = %2f, y = %2f, orient = %2f",x, y, angle);
    }

    gotpose=false;
    ros::Rate loop_rate(100);
    while (!gotpose && ros::ok())
    {
      ros::spinOnce();
      loop_rate.sleep();
      //ros::spinOnce();
      if (gotpose)
      {
        ROS_INFO("got pose!");
        break;
      }
    }
    ros::spinOnce();
    loop_rate.sleep();
    ROS_INFO("Move");
    driver.goSomewhere(x, y, angle);
  }

  ndh_.deleteParam("turtlebot");
  ndh_.deleteParam("direction");

  return 0;
}

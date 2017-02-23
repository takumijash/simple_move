/**
* A simple example of obstcale avoidance, bug algorithm .. by Jash, 20160215
*/

#include <ros/ros.h>
#include <stdlib.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <math.h>
#include <boost/thread/thread.hpp>
#include <tf/transform_listener.h>


#if 0 // template
class SubscribeAndPublish
{
public:
  SubscribeAndPublish()
  {
    //Topic you want to publish
    pub_ = n_.advertise<PUBLISHED_MESSAGE_TYPE>("/published_topic", 1);

    //Topic you want to subscribe
    sub_ = n_.subscribe("/subscribed_topic", 1, &SubscribeAndPublish::callback, this);
  }

  void callback(const SUBSCRIBED_MESSAGE_TYPE& input)
  {
    PUBLISHED_MESSAGE_TYPE output;
    //.... do something with the input and generate the output...
    pub_.publish(output);
  }

private:
  ros::NodeHandle n_;
  ros::Publisher pub_;
  ros::Subscriber sub_;

};//End of class SubscribeAndPublish

#endif

enum{
  MOVE_STOP = 0,
  MOVE_FORWARD,
  MOVE_BACKWARD,
  MOVE_TURNRIGHT,
  MOVE_TURNLEFT
};

enum{
  DRIVE_CMD_TELEPO=1,
  DRIVE_CMD_NAVI
};

class Robot_Drive
{
public:
  //! ROS node initialization
  Robot_Drive(void)
  {
    // Intial
    move_cmd_ = MOVE_STOP;

    //Topic you want to publish
    pub_cmd_telepo_ = n_.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop", 10);
    pub_cmd_navi_ = n_.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/navi", 10);

    //Topic you want to subscribe
    sub_cmd_avoidance = n_.subscribe<std_msgs::Int8>("/obs_avoidance_topic", 10, &Robot_Drive::Avoidance_cmd_listener, this);
    sub_control_effort = n_.subscribe<std_msgs::Float64>("/control_effort", 10, &Robot_Drive::pid_control_effort, this);

    //boost::thread thread_listener(KeyboardListener,10); // compile error, dont know why
  }

  void KeyboardListener(int n)
  {
    int8_t keybord_cmd;

    char cmd[50];
    //Sets the loop to publish at a rate of 10Hz
    ros::Rate rate(n);

    while(n_.ok()){
      std::cin.getline(cmd, 50);
      if(cmd[0]!='i' && cmd[0]!='k' && cmd[0]!='j' && cmd[0]!='l')
      {
        std::cout << "unknown kb command:" << cmd << "\n";
        continue;
      }

      //move forward
      if(cmd[0]=='i'){
        keybord_cmd = MOVE_FORWARD;
      }

      //move turn right
      if(cmd[0]=='l'){
        keybord_cmd = MOVE_TURNRIGHT;
      }

      //move turn left
      if(cmd[0]=='j'){
        keybord_cmd = MOVE_TURNLEFT;
      }

      //move stop
      if(cmd[0]=='k'){
        keybord_cmd = MOVE_STOP;
      }

      //publish the assembled command
      this->Set_Move_Cmd(keybord_cmd);
      this->drive(DRIVE_CMD_TELEPO);

      //Delays untill it is time to send another message
      rate.sleep();
    }
  }

  void Avoidance_cmd_listener(const std_msgs::Int8::ConstPtr& avd_cmd)
  {
    //Sets the loop to publish at a rate of 10Hz
    ros::Rate rate(10);
    int8_t cmd;

    cmd = avd_cmd->data;

    if(cmd != MOVE_FORWARD && cmd != MOVE_TURNRIGHT && cmd != MOVE_TURNLEFT && cmd != MOVE_STOP)
    {
      std::cout << "unknown navi command:" << cmd << "\n";
      return;
    }

    //publish the assembled command
    this->Set_Move_Cmd(cmd);
    this->drive(DRIVE_CMD_NAVI);
  }

  void pid_control_effort(const std_msgs::Float64::ConstPtr& ctrl_effort)
  {
    std_msgs::Float64 cmd;
    geometry_msgs::Twist base_cmd;

    cmd.data = ctrl_effort->data * -1.0;
    ROS_INFO("ctrl_effort = %f", cmd.data);

    base_cmd.angular.z = cmd.data;
    base_cmd.linear.x = 0.0;
    pub_cmd_navi_.publish(base_cmd);
  }

private:
  ros::NodeHandle n_;
  ros::Publisher pub_cmd_telepo_;
  ros::Publisher pub_cmd_navi_;
  ros::Subscriber sub_cmd_avoidance;
  ros::Subscriber sub_control_effort;
  int8_t move_cmd_;

  void Set_Move_Cmd(int8_t cmd)
  {
    move_cmd_ = cmd;
  }

  bool drive(int8_t type)
  {
    //we will be sending commands of type "twist"
    geometry_msgs::Twist base_cmd;

    //std::cin.getline(cmd, 50);
    if(move_cmd_ != MOVE_STOP && move_cmd_ != MOVE_FORWARD && move_cmd_ != MOVE_BACKWARD && move_cmd_ != MOVE_TURNRIGHT && move_cmd_ != MOVE_TURNLEFT)
    {
        std::cout << "unknown command:" << move_cmd_ << "\n";
        return false;
        //continue;
    }

    base_cmd.linear.x = base_cmd.linear.y = base_cmd.angular.z = 0;
    //move forward
    if(move_cmd_ == MOVE_FORWARD){
        base_cmd.linear.x = 0.25;
    }
    //turn left (yaw) and drive forward at the same time
    else if(move_cmd_ == MOVE_TURNLEFT){
        base_cmd.angular.z = 0.5;
        base_cmd.linear.x = 0.0;
    }
    //turn right (yaw) and drive forward at the same time
    else if(move_cmd_ == MOVE_TURNRIGHT){
        base_cmd.angular.z = -0.5;
        base_cmd.linear.x = 0.0;
    }
    //quit
    else if(move_cmd_ == MOVE_STOP){
        base_cmd.linear.x = base_cmd.linear.y = base_cmd.angular.z = 0;
    }

    //publish the assembled command
    if (type == DRIVE_CMD_TELEPO){
      pub_cmd_telepo_.publish(base_cmd);
    }else if(type == DRIVE_CMD_NAVI){
      pub_cmd_navi_.publish(base_cmd);
    }
    return true;
  }

};//End of class Robot_Keyboard_Drive


class Obstacle_Detect_Avoidance
{
public:
  Obstacle_Detect_Avoidance()
  {
    //Initial
    enable_detect = false;
    enable_avoidance = false;

    //Topic you want to publish
    pub_detect = n_.advertise<std_msgs::Int8>("/obs_detect_topic", 10);
    pub_avoidance = n_.advertise<std_msgs::Int8>("/obs_avoidance_topic", 10);

    //Topic you want to subscribe
    sub_scan = n_.subscribe<sensor_msgs::LaserScan>("/kinect_scan", 10, &Obstacle_Detect_Avoidance::detect_callback, this);
    sub_detect = n_.subscribe<std_msgs::Int8>("/obs_detect_topic", 10, &Obstacle_Detect_Avoidance::avoidance_callback, this);
  }

  void Obstacle_Detect_Enable(bool en)
  {
    enable_detect = en;
  }

  void Obstacle_Avoidance_Enable(bool en)
  {
    enable_avoidance = en;
  }

private:
  ros::NodeHandle n_;
  ros::Publisher pub_detect;
  ros::Publisher pub_avoidance;
  ros::Subscriber sub_scan;
  ros::Subscriber sub_detect;
  bool enable_detect;
  bool enable_avoidance;


  void detect_callback(const sensor_msgs::LaserScan::ConstPtr& scan)
  {
    std_msgs::Int8 detect_msg;

    if (!enable_detect){return;}

    if(scan_obstacle(scan, 0.6, 2.0)){
      //ROS_INFO("obstacle detected");
      detect_msg.data = 1;
    }
    else{
      detect_msg.data = 0;
    }

    pub_detect.publish(detect_msg);
  }

  void avoidance_callback(const std_msgs::Int8::ConstPtr& detect)
  {
    std_msgs::Int8 avoidance_msg;

    if (!enable_avoidance){return;}

    if(detect->data){
      //ROS_INFO("obstacle avoidance");
      avoidance_msg.data = MOVE_TURNRIGHT;
    }
    else{
      avoidance_msg.data = MOVE_FORWARD;
    }

    pub_avoidance.publish(avoidance_msg);
  }

  bool scan_obstacle(const sensor_msgs::LaserScan::ConstPtr& scan, float footprint, float obstacle_distance)
  {
      float min_degree = scan->angle_min;//*(180/M_PI);
      float max_degree = scan->angle_max;//*(180/M_PI);
      float inc_degree = scan->angle_increment;//*(180/M_PI);
      float theta, abs_theta;
      unsigned int data_num = (max_degree-min_degree)/inc_degree;

      theta = min_degree;
      for(int i = 0; i < data_num; i++){
        theta += inc_degree;
        abs_theta = fabs(theta);
        if(scan->ranges[i] < obstacle_distance){
          if (scan->ranges[i]*sin(abs_theta) < (footprint/2)){
            return true;
          }
        }
      }
      return false;
  }

};//End of class Obstacle_Detector


class Face2Goal
{
public:
  Face2Goal()
  {
    //Topic you want to publish
    pub_setpoint = n_.advertise<std_msgs::Float64>("/setpoint", 10);
    pub_state = n_.advertise<std_msgs::Float64>("/state", 10);
    pub_face2goal = n_.advertise<std_msgs::Int8>("/obs_avoidance_topic", 10);
  }

  void goal_listener(int n)
  {
    std_msgs::Float64 err_angle;
    std_msgs::Float64 setpoint;
    std_msgs::Int8 face2goal_msg;
    tf::TransformListener listener;

    //Sets the loop to publish at a rate of 10Hz
    ros::Rate rate(n);

    while(n_.ok()){

      tf::StampedTransform transform;
      try{
        ros::Time now = ros::Time::now();
        listener.waitForTransform("/base_link", "/goal",
        now, ros::Duration(3.0));
        listener.lookupTransform("/base_link", "/goal",
        now, transform);
      }
      catch (tf::TransformException &ex) {
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
        continue;
      }

      err_angle.data = get_angle(tf::Vector3(0.0, 0.0, 0.0), transform.getOrigin());
      ROS_INFO("Angle %f",err_angle.data);

      setpoint.data = 0.0;
      pub_setpoint.publish(setpoint);
      pub_state.publish(err_angle);
      //Delays untill it is time to send another message
      rate.sleep();
    }
  }
private:
  ros::NodeHandle n_;
  ros::Publisher pub_setpoint;
  ros::Publisher pub_state;
  ros::Publisher pub_face2goal;
  ros::Subscriber sub_;


  double get_angle(tf::Vector3 a, tf::Vector3 b)
  {
      if ( a.getX() == b.getX() && a.getY() >= b.getY() ) return 0;

      b -= a;

      double b_magnitude = sqrt((b.getX()*b.getX())+(b.getY()*b.getY()));
      double angle = acos(b.getX() / b_magnitude) * (180 / M_PI);

      if ( b.getY() < a.getY() ) angle *= -1;

      return angle;
  }
};//End of class SubscribeAndPublish

int main(int argc, char** argv) {
  // Announce this program to the ROS master as a "node" called "simple_move_node"
  ros::init(argc, argv, "simple_move_node");

  // Start the node resource managers (communication, time, etc)
  ros::start();

  Robot_Drive robot_drive_object;
  boost::thread thd_kb_listener(boost::bind(&Robot_Drive::KeyboardListener, &robot_drive_object, 10)); // param 10 : publish frequence

  Obstacle_Detect_Avoidance obs_detect_avoidance_object;
  obs_detect_avoidance_object.Obstacle_Detect_Enable(false);
  obs_detect_avoidance_object.Obstacle_Avoidance_Enable(false);

  Face2Goal face_to_goal_object;
  boost::thread thd_goal_listener(boost::bind(&Face2Goal::goal_listener, &face_to_goal_object, 10)); // param 10 : publish frequence

  // Process ROS callbacks until receiving a SIGINT (ctrl-c)
  ros::spin();
  // Stop the node's resources
  ros::shutdown();
  // Exit tranquilly
  return 0;
}




#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <math.h>
#include <boost/thread/thread.hpp>

class GoalPoseMgm
{
public:
  GoalPoseMgm()
  {
    //Initial
    goal_pose.setX(0.0);
    goal_pose.setY(0.0);
    goal_pose.setZ(0.0);

    goal_rotation.setX(0.0);
    goal_rotation.setY(0.0);
    goal_rotation.setZ(0.0);
    goal_rotation.setW(1.0);

    transform_world.setOrigin( tf::Vector3(0.0, 0.0, 0.0) );
    transform_world.setRotation( tf::Quaternion(0, 0, 0, 1) );
    transform_goal.setOrigin( this->get_goal_pose() );
    transform_goal.setRotation( this->get_goal_rotation() );

    //Topic you want to subscribe
    sub_ = n_.subscribe<geometry_msgs::Pose>("/goal_pose", 1, &GoalPoseMgm::callback, this);
  }

  void tf_publisher(int n)
  {
    //Sets the loop to publish at a rate of 10Hz
    ros::Rate rate(n);

    while (n_.ok()){
      transform_goal.setOrigin( this->get_goal_pose() );
      transform_goal.setRotation( this->get_goal_rotation() );

      br_world.sendTransform(tf::StampedTransform(transform_world, ros::Time::now(), "world", "odom"));
      br_goal.sendTransform(tf::StampedTransform(transform_goal, ros::Time::now(), "world", "goal"));

      rate.sleep();
    }
  }

private:
  ros::NodeHandle n_;
  ros::Publisher pub_;
  ros::Subscriber sub_;
  tf::Vector3 goal_pose;
  tf::Quaternion goal_rotation;
  tf::TransformBroadcaster br_world;
  tf::Transform transform_world;
  tf::TransformBroadcaster br_goal;
  tf::Transform transform_goal;

  void callback(const geometry_msgs::Pose::ConstPtr& goal_input)
  {
    ROS_INFO("x(%f)y(%f)z(%f), x(%f)y(%f)z(%f)w(%f)", goal_input->position.x,goal_input->position.y,goal_input->position.z,
             goal_input->orientation.x,goal_input->orientation.y,goal_input->orientation.z,goal_input->orientation.w);

    goal_pose.setX(goal_input->position.x);
    goal_pose.setY(goal_input->position.y);
    goal_pose.setZ(goal_input->position.z);

    goal_rotation.setX(goal_input->orientation.x);
    goal_rotation.setY(goal_input->orientation.y);
    goal_rotation.setZ(goal_input->orientation.z);
    goal_rotation.setW(goal_input->orientation.w);
  }

  tf::Vector3 get_goal_pose(void)
  {
    return goal_pose;
  }

  tf::Quaternion get_goal_rotation(void)
  {
    return goal_rotation;
  }

};//End of class SubscribeGoalPose

int main(int argc, char** argv){
  ros::init(argc, argv, "world_tf_broadcaster");
  ros::start();

  ros::NodeHandle node;

  GoalPoseMgm obj_goal_mgm;
  boost::thread thd_tf_publisher(boost::bind(&GoalPoseMgm::tf_publisher, &obj_goal_mgm, 10)); // param 10 : publish frequence

  // Process ROS callbacks until receiving a SIGINT (ctrl-c)
  ros::spin();
  // Stop the node's resources
  ros::shutdown();
  // Exit tranquilly
  return 0;
};

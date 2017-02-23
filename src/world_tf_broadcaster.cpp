#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <math.h>

double GetAngle(tf::Vector3 a, tf::Vector3 b)
{
    if ( a.getX() == b.getX() && a.getY() >= b.getY() ) return 0;

    b -= a;
#if 0 // original
    tf::Vector3 c = tf::Vector3(1.0, 0.0, 0.0);
    double c_magnitude = sqrt((c.getX()*c.getX())+(c.getY()*c.getY()));
    double b_magnitude = sqrt((b.getX()*b.getX())+(b.getY()*b.getY()));
    double d1 = (c.getX() * b.getX()) + (c.getY() * b.getY());
    double d2 = c_magnitude * b_magnitude;
    double angle = acos(d1 / d2) * (180 / M_PI);
#else // simplify
    double b_magnitude = sqrt((b.getX()*b.getX())+(b.getY()*b.getY()));
    double angle = acos(b.getX() / b_magnitude) * (180 / M_PI);
#endif
    if ( b.getY() < a.getY() ) angle *= -1;

    return angle;
}


int main(int argc, char** argv){
  ros::init(argc, argv, "world_tf_broadcaster");
  ros::NodeHandle node;

  tf::TransformBroadcaster br_world;
  tf::Transform transform_world;

  tf::TransformBroadcaster br_goal;
  tf::Transform transform_goal;

  transform_world.setOrigin( tf::Vector3(0.0, 0.0, 0.0) );
  transform_world.setRotation( tf::Quaternion(0, 0, 0, 1) );

  transform_goal.setOrigin( tf::Vector3(-5.0, -5.0, 0.0) );
  transform_goal.setRotation( tf::Quaternion(0, 0, 0, 1) );

  ros::Rate rate(10.0);
  while (node.ok()){
    br_world.sendTransform(tf::StampedTransform(transform_world, ros::Time::now(), "world", "odom"));
    br_goal.sendTransform(tf::StampedTransform(transform_goal, ros::Time::now(), "world", "goal"));

    //ROS_INFO("Angle A %f",(float)GetAngle(tf::Vector3(0.0, 0.0, 0.0), tf::Vector3(5.0, 5.0, 0.0)));
    //ROS_INFO("Angle B %f",(float)GetAngle(tf::Vector3(0.0, 0.0, 0.0), tf::Vector3(5.0, -5.0, 0.0)));
    //ROS_INFO("Angle C %f",(float)GetAngle(tf::Vector3(0.0, 0.0, 0.0), tf::Vector3(-5.0, -5.0, 0.0)));
    //ROS_INFO("Angle D %f",(float)GetAngle(tf::Vector3(0.0, 0.0, 0.0), tf::Vector3(-5.0, 5.0, 0.0)));
    rate.sleep();
  }
  return 0;
};

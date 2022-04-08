
#include "ros/ros.h"
#include "sensor_msgs/NavSatFix.h"
#include "geometry_msgs/PoseStamped.h"
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include "tf/transform_broadcaster.h"
#include "rtklib_msgs/RtklibNav.h"
#include "eagleye_msgs/Heading.h"
#include "coordinate/coordinate.hpp"


static std::string sub_gnss_topic_name;
static std::string pub_rtk_topic_name;
static std::string pub_rtk_covariance_topic_name;
static std::string pub_rtk_fix_topic_name;


static ros::Publisher pub;
static geometry_msgs::PoseStamped pose;
static eagleye_msgs::Heading eagleye_heading;

static ros::Publisher pub2;
static geometry_msgs::PoseWithCovarianceStamped pose_cov;

static ros::Publisher pub3;
static geometry_msgs::PoseWithCovarianceStamped base_link;



double gps_x,gps_y,gps_z,gps_yaw,gps_pitch,gps_roll;
static int convert_height_num = 0;
static int plane = 7;
static int tf_num = 1;

static geometry_msgs::Quaternion _quat;
static double last_xyz[3];

double roll_, pitch_, yaw_;
ros::Time current_time_, orientation_stamp_;

static ConvertHeight convert_height;

void NavSatFix_callback(const sensor_msgs::NavSatFix::ConstPtr& msg)
{

  double ecef_vel[3] = {0};
  double ecef_pos[3] = {0};
  double enu_vel[3] = {0};
  double llh[3] = {0};
  double _llh[3] = {0};
  double xyz[3] = {0};
  double height;
  double doppler_heading_angle = 0.0;

  llh[0] = msg->latitude * M_PI / 180;
  llh[1] = msg->longitude* M_PI / 180;
  llh[2] = msg->altitude;


  if (convert_height_num == 1)
   {
    convert_height.setLLH(msg->latitude,msg->longitude,msg->altitude);
    llh[2] = convert_height.convert2altitude();
  }
  else if(convert_height_num == 2)
  {
    convert_height.setLLH(msg->latitude,msg->longitude,msg->altitude);
    llh[2] = convert_height.convert2ellipsoid();
  }

  if (tf_num == 1)
  {
    ll2xy(plane,llh,xyz);
  }
  else if (tf_num == 2)
  {
    ll2xy_mgrs(llh,xyz);
  }

  double timeout = 10.0;
  if (fabs(orientation_stamp_.toSec() - msg->header.stamp.toSec()) > timeout)
  {
    double dt = sqrt(pow(xyz[0] - last_xyz[0], 2) + pow(xyz[1] - last_xyz[1], 2));
    double threshold = 0.2;
    if (dt > threshold)
    {
      ROS_INFO("QQ is not subscribed. Orientation is created by atan2");
      yaw_ = atan2(xyz[0] - last_xyz[0], xyz[1] - last_xyz[1]);
      roll_ = 0;
      pitch_ = 0;
      last_xyz[0] = xyz[0];
      last_xyz[1] = xyz[1];
      last_xyz[2] = xyz[2];
    }
  }

  _quat = tf::createQuaternionMsgFromYaw(yaw_);


  pose_cov.header = msg->header;
  pose_cov.header.frame_id = "map";
  pose_cov.pose.pose.position.x = xyz[1];
  pose_cov.pose.pose.position.y = xyz[0];
  pose_cov.pose.pose.position.z = xyz[2];
  pose_cov.pose.pose.orientation = _quat;

  //TODO temporary value
  pose_cov.pose.covariance[0] = 0.025;
  pose_cov.pose.covariance[1 * 6 + 1] = 0.025;
  pose_cov.pose.covariance[2 * 6 + 2] = 0.025;
  pose_cov.pose.covariance[3 * 6 + 3] = 0.000625;
  pose_cov.pose.covariance[4 * 6 + 4] = 0.000625;
  pose_cov.pose.covariance[5 * 6 + 5] = 0.000625;



/*
// TF前
  std::cout<< "pose.pose.position.x "<<pose.pose.position.x<<std::endl;
  std::cout<< "pose.pose.position.y "<<pose.pose.position.y<<std::endl;
  std::cout<< "pose.pose.position.z "<<pose.pose.position.z<<std::endl;
*/

  static tf::TransformBroadcaster br;
  tf::Transform transform;
  tf::Quaternion q;
  transform.setOrigin(tf::Vector3(pose_cov.pose.pose.position.x, pose_cov.pose.pose.position.y, pose_cov.pose.pose.position.z));
  q.setRPY(0, 0, yaw_);
  transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(transform, msg->header.stamp, "map", "gnss_NavSatFix"));


  static tf::TransformBroadcaster br2;
  tf::Transform transform2;
  tf::Quaternion q2;
  // transform2.setOrigin(tf::Vector3(pose.pose.pose.position.x, pose.pose.pose.position.y, pose.pose.pose.position.z));
  transform2.setOrigin(transform*tf::Vector3(-gps_x, -gps_y,-gps_z));
  // q2.setRPY(0, 0, yaw_);
  q2.setRPY(gps_yaw, gps_pitch, gps_roll);
  transform2.setRotation(transform*q2);


  base_link.header = msg->header;
  base_link.header.frame_id = "map";
  base_link.pose.pose.position.x = transform2.getOrigin().x();
  base_link.pose.pose.position.y = transform2.getOrigin().y();
  base_link.pose.pose.position.z = transform2.getOrigin().z();
  base_link.pose.pose.orientation = _quat;

  base_link.pose.covariance[0] = 0.025;
  base_link.pose.covariance[1 * 6 + 1] = 0.025;
  base_link.pose.covariance[2 * 6 + 2] = 0.025;
  base_link.pose.covariance[3 * 6 + 3] = 0.000625;
  base_link.pose.covariance[4 * 6 + 4] = 0.000625;
  base_link.pose.covariance[5 * 6 + 5] = 0.000625;

  pose.header = base_link.header;
  pose.pose = base_link.pose.pose;

  pub.publish(pose);
  pub2.publish(base_link);

    if(msg->status.status == 0)
  {
    pub3.publish(base_link);  // fix
    br2.sendTransform(tf::StampedTransform(transform2, msg->header.stamp, "map", "rtk_fix"));
  }else{
    // float
  }

	// transform2.setOrigin(tf::Vector3(base_link.pose.pose.position.x, base_link.pose.pose.position.y, base_link.pose.pose.position.z));

/*
//TF後
  std::cout<< "base_link.pose.pose.position.x "<<base_link.pose.pose.position.x<<std::endl;
  std::cout<< "base_link.pose.pose.position.y "<<base_link.pose.pose.position.y<<std::endl;
  std::cout<< "base_link.pose.pose.position.z "<<base_link.pose.pose.position.z<<std::endl;
*/

  // br2.sendTransform(tf::StampedTransform(transform2, msg->header.stamp, "map", "rtk_base_link"));


}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "navsatfix2pose");
  ros::NodeHandle n;

  n.getParam("plane",plane);
  n.getParam("tf_num",tf_num);
  n.getParam("convert_height_num",convert_height_num);
  n.getParam("gps_x",gps_x);
  n.getParam("gps_y",gps_y);
  n.getParam("gps_z",gps_z);
  n.getParam("gps_yaw",gps_yaw);
  n.getParam("gps_pitch",gps_pitch);
  n.getParam("gps_roll",gps_roll);
  std::cout<< "plane "<<plane<<std::endl;
  std::cout<< "tf_num "<<tf_num<<std::endl;
  std::cout<< "convert_height_num "<<convert_height_num<<std::endl;
  std::cout<< "gps_x "<<gps_x<<std::endl;
  std::cout<< "gps_y "<<gps_y<<std::endl;
  std::cout<< "gps_z "<<gps_z<<std::endl;
  std::cout<< "gps_yaw "<<gps_yaw<<std::endl;
  std::cout<< "gps_pitch "<<gps_pitch<<std::endl;
  std::cout<< "gps_roll "<<gps_roll<<std::endl;

  n.getParam("sub_gnss_topic_name",sub_gnss_topic_name);
  n.getParam("pub_rtk_topic_name",pub_rtk_topic_name);
  n.getParam("pub_rtk_covariance_topic_name",pub_rtk_covariance_topic_name);
  n.getParam("pub_rtk_fix_topic_name",pub_rtk_fix_topic_name);
  std::cout<< "sub_gnss_topic_name "<<sub_gnss_topic_name<<std::endl;
  std::cout<< "pub_rtk_topic_name "<<pub_rtk_topic_name<<std::endl;
  std::cout<< "pub_rtk_covariance_topic_name "<<pub_rtk_covariance_topic_name<<std::endl;
  std::cout<< "pub_rtk_fix_topic_name "<<pub_rtk_fix_topic_name<<std::endl;

  ros::Subscriber sub = n.subscribe(sub_gnss_topic_name, 1000, NavSatFix_callback);
  pub = n.advertise<geometry_msgs::PoseStamped>(pub_rtk_topic_name, 1000);
  pub2 = n.advertise<geometry_msgs::PoseWithCovarianceStamped>(pub_rtk_covariance_topic_name, 1000);
  pub3 = n.advertise<geometry_msgs::PoseWithCovarianceStamped>(pub_rtk_fix_topic_name, 1000); //fix解のみ

  ros::spin();

  return 0;
}

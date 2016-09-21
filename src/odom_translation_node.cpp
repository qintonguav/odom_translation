#include <ros/ros.h>
#include <iostream>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <vector>
#include <math.h>
#include <eigen3/Eigen/Dense>
#include <sensor_msgs/NavSatFix.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <queue>

using namespace std;
using namespace Eigen;

ros::Publisher pub_gps_rec_path;
ros::Publisher pub_gps_rec_odom;
nav_msgs::Path path_gps_rec;

ros::Publisher pub_odom_rec_path1;
ros::Publisher pub_odom_rec_path2;
ros::Publisher pub_odom_rec_path3;
ros::Publisher pub_odom_rec_path4;
ros::Publisher pub_odom_rec_odom;
ros::Publisher pub_init_rec_odom;
ros::Publisher pub_optitrack_path;
image_transport::Publisher pub_image;

nav_msgs::Path path_odom_rec1;
nav_msgs::Path path_odom_rec2;
nav_msgs::Path path_odom_rec3;
nav_msgs::Path path_odom_rec4;
nav_msgs::Path empty_path;
nav_msgs::Path path_optitrack;

queue<geometry_msgs::PoseStampedPtr> optitrack_buf;

bool update_flag = false;

void odom_callback(const nav_msgs::OdometryConstPtr &odom_msg)
{

	//ROS_INFO("odom_callback");
	nav_msgs::Odometry odom_rec;
	odom_rec.header = odom_msg->header;
	odom_rec.header.frame_id = "world";

	Vector3d tmp_T = Vector3d(odom_msg->pose.pose.position.x,
					odom_msg->pose.pose.position.y,
					odom_msg->pose.pose.position.z);
	Matrix3d R;
	Vector3d T;


	if(odom_msg->header.stamp.toSec() < 9.466851928094507e+08)
	{
		R <<  -0.079120888806734, 0.996865028453919, 0,
		       -0.996865028453919, -0.079120888806734, 0,
		       0,0,1;
		T << -76.3557, 62.0347, 32.3313;
	}
	else if(odom_msg->header.stamp.toSec() < 9.466852401844642e+08)
	{
		R <<  -0.199449468040984,   0.979907324521062,  -0.001242999654908,
  				-0.979906818608206,  -0.199446982461745,   0.001878305632381,
   				0.001592652916487,   0.001592650896569 ,  0.999997463456687;
		T << -39.0942, 44.4903,  35.9596;
	}
	else if(odom_msg->header.stamp.toSec() < 9.466852856011236e+08)
	{
		R <<  -0.985196275470271,  -0.160890314967456,  -0.059182812954257,
   			0.160600799270859,  -0.986972292696038,   0.009647627889188,
  			-0.059964006479445 ,                  0 ,  0.998200539935204;
		T << 62.216, 3.9715,  59.0633;
	}
	else
	{
		R <<     -0.949234214193302 ,  0.313941903923495 ,  0.019871778121654,
  				-0.314566161660012 , -0.947646004045478  ,-0.054910663400887,
   				0.001592652916487 , -0.058374069393262 ,  0.998293509684982;
		T << 26.547973423849747, -24.123738604003737,  76.071516509530539;
	}

	Vector3d rec_T = R * tmp_T + T;
	odom_rec.pose.pose.position.x = rec_T.x();
	odom_rec.pose.pose.position.y = rec_T.y();
	odom_rec.pose.pose.position.z = rec_T.z();
	pub_odom_rec_odom.publish(odom_rec);

	geometry_msgs::PoseStamped pose_stamped;
	pose_stamped.header = odom_rec.header;
	pose_stamped.pose = odom_rec.pose.pose;
	

	if(odom_msg->header.stamp.toSec() < 9.466851928094507e+08)
	{
		path_odom_rec1.header = odom_rec.header;
		path_odom_rec1.poses.push_back(pose_stamped);
		pub_odom_rec_path1.publish(path_odom_rec1);
	}
	else if(odom_msg->header.stamp.toSec() < 9.466852401844642e+08)
	{
		pub_odom_rec_path1.publish(path_odom_rec1);
		path_odom_rec2.header = odom_rec.header;
		path_odom_rec2.poses.push_back(pose_stamped);
		pub_odom_rec_path2.publish(path_odom_rec2);
	}
	else if(odom_msg->header.stamp.toSec() < 9.466852856011236e+08)
	{
		pub_odom_rec_path2.publish(path_odom_rec2);
		path_odom_rec3.header = odom_rec.header;
		path_odom_rec3.poses.push_back(pose_stamped);
		pub_odom_rec_path3.publish(path_odom_rec3);
	}
	else if(odom_msg->header.stamp.toSec() < 9.466853342677933e+08)
	{
		pub_odom_rec_path3.publish(path_odom_rec3);
		path_odom_rec4.header = odom_rec.header;
		path_odom_rec4.poses.push_back(pose_stamped);
		pub_odom_rec_path4.publish(path_odom_rec4);
	}
	else
	{
		update_flag = true;
	}

}

void gps_callback(const sensor_msgs::NavSatFixConstPtr& msg)
{

	double alt = msg->altitude;
    double lat = msg->latitude;
    double lon = msg->longitude;
    double lon_off=1.99427338946;
    double lat_off=0.38982884732;
    double r = 6371000 + alt;

    double gps_x = r*(lat-lat_off);
    double gps_y = r*(lon_off-lon);
    double gps_z = alt+37;

	nav_msgs::Odometry gps_rec;
	gps_rec.header = msg->header;
	gps_rec.header.frame_id = "world";

	gps_rec.pose.pose.position.x = gps_x;
	gps_rec.pose.pose.position.y = gps_y;
	gps_rec.pose.pose.position.z = gps_z;
	pub_gps_rec_odom.publish(gps_rec);

	geometry_msgs::PoseStamped pose_stamped;
	pose_stamped.header = gps_rec.header;
	pose_stamped.pose = gps_rec.pose.pose;
	path_gps_rec.header = gps_rec.header;
	path_gps_rec.poses.push_back(pose_stamped);
	pub_gps_rec_path.publish(path_gps_rec);

}

void init_callback(const nav_msgs::OdometryConstPtr &init_msg)
{

	//ROS_INFO("odom_callback");
	nav_msgs::Odometry odom_rec;
	odom_rec.header = init_msg->header;
	odom_rec.header.frame_id = "world";

	Vector3d tmp_T = Vector3d(init_msg->pose.pose.position.x,
					init_msg->pose.pose.position.y,
					init_msg->pose.pose.position.z);
	Matrix3d R;
	Vector3d T;


	if(init_msg->header.stamp.toSec() < 9.466851928094507e+08)
	{
		R <<  -0.079120888806734, 0.996865028453919, 0,
		       -0.996865028453919, -0.079120888806734, 0,
		       0,0,1;
		T << -76.3557, 62.0347, 32.3313;
	}
	else if(init_msg->header.stamp.toSec() < 9.466852401844642e+08)
	{
		R <<  -0.199449468040984,   0.979907324521062,  -0.001242999654908,
  				-0.979906818608206,  -0.199446982461745,   0.001878305632381,
   				0.001592652916487,   0.001592650896569 ,  0.999997463456687;
		T << -39.0942, 44.4903,  35.9596;
	}
	else if(init_msg->header.stamp.toSec() < 9.466852856011236e+08)
	{
		R <<  -0.949234214193302,   0.313941903923495,   0.019871778121654,
   			-0.314566161660012,  -0.947646004045478,  -0.054910663400887,
  			0.001592652916487,  -0.058374069393262,   0.998293509684982;
		T << 62.216, 3.9715,  59.0633;
	}
	else
	{
		R <<   -0.960831404479544 ,  0.275926897875316,   0.025833296242878,
  				-0.276291226666832 , -0.961002452040992 , -0.011723704113374,
   				0.021590975726096,  -0.018402016196748 ,  0.999597516787176;
		T << 26.547973423849747, -24.123738604003737,  76.071516509530539;
	}

	Vector3d rec_T = R * tmp_T + T;
	odom_rec.pose.pose.position.x = rec_T.x();
	odom_rec.pose.pose.position.y = rec_T.y();
	odom_rec.pose.pose.position.z = rec_T.z();
	pub_init_rec_odom.publish(odom_rec);	
}

void image_callback(const sensor_msgs::ImageConstPtr &msg)
{
	//ROS_INFO("receive img");

	cv::Mat empty_img(480, 752, CV_8UC1);
	empty_img.setTo(0);

	sensor_msgs::ImagePtr empty_msg = cv_bridge::CvImage(msg->header, "mono8", empty_img).toImageMsg();
	double t = msg->header.stamp.toSec();
	if(t >9.466851168511298e+08 && t <9.466851879761339e+08)
	{
		pub_image.publish(msg);
	}
	else if (t>9.466851908094583e+08 && t < 9.466852346844732e+08)
	{
		pub_image.publish(msg);
	}
	else if (t >9.466852391845427e+08 && t < 9.466852816010919e+08)
	{
		pub_image.publish(msg);
	}
	else if(t > 9.466852847677864e+08)
	{
		pub_image.publish(msg);
	}
	else
	{
    	pub_image.publish(empty_msg);
    }
}

void optitrack_callback(const geometry_msgs::PoseStampedPtr &optitrack_msg)
{
	if(optitrack_msg->header.stamp.toSec() < 1.473498419471904e+09)
	{
		return;
	}

	optitrack_buf.push(optitrack_msg);

	if(optitrack_buf.size() < 18)
		return;

	geometry_msgs::PoseStampedPtr msg = optitrack_buf.front();
	optitrack_buf.pop();

	//ROS_INFO("optitrack_callback");
	nav_msgs::Odometry odom_rec;
	odom_rec.header = msg->header;
	odom_rec.header.frame_id = "base";

	Vector3d tmp_T = Vector3d(msg->pose.position.x,
					msg->pose.position.y,
					msg->pose.position.z);
	Matrix3d R;
	Vector3d T;


		R <<    -0.100622177119691 ,  0.994889187630161 , -0.008407247367149,
  				-0.994867303771393 , -0.100522006485855 ,  0.011591984243235,
   				0.010687626412405 ,  0.009530506211986 ,  0.999897466789977;
		T <<  0.038382881241630 ,  1.467018596197649 , -2.083622910386265;


	Vector3d rec_T = R * tmp_T + T;
	odom_rec.pose.pose.position.x = rec_T.x();
	odom_rec.pose.pose.position.y = rec_T.y();
	odom_rec.pose.pose.position.z = rec_T.z();

	geometry_msgs::PoseStamped pose_stamped;
	pose_stamped.header = odom_rec.header;
	pose_stamped.pose = odom_rec.pose.pose;
	path_optitrack.header = odom_rec.header;
	path_optitrack.poses.push_back(pose_stamped);
	pub_optitrack_path.publish(path_optitrack);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "odom_translation");
	ros::NodeHandle n("~");
	ros::Subscriber sub_gps = n.subscribe("gps", 2000, gps_callback);
	ros::Subscriber sub_odom = n.subscribe("odom", 2000, odom_callback);
	ros::Subscriber sub_init = n.subscribe("init", 2000, init_callback);
	ros::Subscriber sub_image = n.subscribe("image",2000, image_callback);
	ros::Subscriber sub_optitrack = n.subscribe("optitrack",2000, optitrack_callback);

	pub_gps_rec_path = n.advertise<nav_msgs::Path>("path_gps_rec",1000);
	pub_gps_rec_odom = n.advertise<nav_msgs::Odometry>("odom_gps_rec",1000);

	pub_odom_rec_path1 = n.advertise<nav_msgs::Path>("path_odom_rec1",1000);
	pub_odom_rec_path2 = n.advertise<nav_msgs::Path>("path_odom_rec2",1000);
	pub_odom_rec_path3 = n.advertise<nav_msgs::Path>("path_odom_rec3",1000);
	pub_odom_rec_path4 = n.advertise<nav_msgs::Path>("path_odom_rec4",1000);
	pub_odom_rec_odom = n.advertise<nav_msgs::Odometry>("odom_odom_rec",1000);
	pub_init_rec_odom = n.advertise<nav_msgs::Odometry>("odom_init_rec",1000);
	pub_optitrack_path = n.advertise<nav_msgs::Path>("path_optitrack",1000);
	image_transport::ImageTransport it(n);
	pub_image = it.advertise("tracked_image_time", 1000);

	ros::Rate loop_rate(100);
	
	while (ros::ok())
	{

		if(update_flag)
		{
			pub_odom_rec_path1.publish(path_odom_rec1);
			pub_odom_rec_path2.publish(path_odom_rec2);
			pub_odom_rec_path3.publish(path_odom_rec3);
			pub_odom_rec_path4.publish(path_odom_rec4);
			pub_gps_rec_path.publish(path_gps_rec);
			pub_optitrack_path.publish(path_optitrack);
		}
		ros::spinOnce();
		loop_rate.sleep();
	}

	ros::spin();
	return 0;
}
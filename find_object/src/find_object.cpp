/*
 * Copyright 2019 PAL Robotics SL. All Rights Reserved
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited,
 * unless it was supplied under the terms of a license agreement or
 * nondisclosure agreement with PAL Robotics SL. In this case it may not be
 * copied or disclosed except in accordance with the terms of that agreement.
 */

/** \author Alessandro Di Fava. */

/*
 * @Author: Jianfeng
 * @Date: 2021-05-06 03:14:45 
 * @Last Modified by: Jianfeng
 * @Last Modified time: 2021-05-16 19:50:11
 */

// C++ standard headers
#include <exception>
#include <string>

// Boost headers
#include <boost/shared_ptr.hpp>

// ROS headers
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <control_msgs/PointHeadAction.h>
#include <sensor_msgs/image_encodings.h>
#include <ros/topic.h>
#include "set_nav_goal/ToPerceptionAction.h"
#include "mmdetection_ros/mmdetSrv.h"
#include "vision_msgs/Detection2D.h"
#include "vision_msgs/Detection2DArray.h"
#include "vision_msgs/ObjectHypothesisWithPose.h"
#include <sensor_msgs/PointCloud2.h>

// OpenCV headers
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

static const std::string windowName      = "Inside of TIAGo++'s head";
static const std::string cameraFrame     = "/xtion_rgb_optical_frame";
static const std::string imageTopic      = "/xtion/rgb/image_raw";
static const std::string cameraInfoTopic = "/xtion/rgb/camera_info";
static const std::string ObjectsTopic    = "/mmdetector/objects";
static const std::string pointCloudsTopic    = "/xtion/depth_registered/points";
static const std::string cylinderPoseTopic = "/cylinder_detector/cylinder_pose";

// Intrinsic parameters of the camera
cv::Mat cameraIntrinsics;

// Our Action interface type for moving TIAGo++'s head, provided as a typedef for convenience
typedef actionlib::SimpleActionClient<control_msgs::PointHeadAction> PointHeadClient;
typedef boost::shared_ptr<PointHeadClient> PointHeadClientPtr;
PointHeadClientPtr pointHeadClient;

typedef actionlib::SimpleActionServer<set_nav_goal::ToPerceptionAction> ToPerceptionServer;

ros::Time latestImageStamp;
sensor_msgs::Image latestImage;
vision_msgs::Detection2DArray latestObjects;
sensor_msgs::PointCloud2 latestPointClouds;
geometry_msgs::PoseStamped latestCylinderPose;

set_nav_goal::ToPerceptionFeedback feedback_; // NOTE: Currently not used
set_nav_goal::ToPerceptionResult result_;

// Create a ROS action client to move TIAGo's head
void createPointHeadClient(PointHeadClientPtr& actionClient)
{
  ROS_INFO("Creating action client to head controller ...");

  actionClient.reset( new PointHeadClient("/head_controller/point_head_action") );

  int iterations = 0, max_iterations = 3;
  // Wait for head controller action server to come up
  while( !actionClient->waitForServer(ros::Duration(2.0)) && ros::ok() && iterations < max_iterations )
  {
    ROS_DEBUG("Waiting for the point_head_action server to come up");
    ++iterations;
  }

  if ( iterations == max_iterations )
    throw std::runtime_error("Error in createPointHeadClient: head controller action server not available");
}

control_msgs::PointHeadGoal getPointHeadGoal(int u, int v)
{
  ROS_INFO_STREAM("Pixel selected (" << u << ", " << v << ") Making TIAGo++ look to that direction");

  geometry_msgs::PointStamped pointStamped;

  pointStamped.header.frame_id = cameraFrame;
  pointStamped.header.stamp    = latestImageStamp;

  //compute normalized coordinates of the selected pixel
  double x = ( u  - cameraIntrinsics.at<double>(0,2) )/ cameraIntrinsics.at<double>(0,0);
  double y = ( v  - cameraIntrinsics.at<double>(1,2) )/ cameraIntrinsics.at<double>(1,1);
  double Z = 1.0; //define an arbitrary distance
  pointStamped.point.x = x * Z;
  pointStamped.point.y = y * Z;
  pointStamped.point.z = Z;   

  //build the action goal
  control_msgs::PointHeadGoal goal;
  //the goal consists in making the Z axis of the cameraFrame to point towards the pointStamped
  goal.pointing_frame = cameraFrame;
  goal.pointing_axis.x = 0.0;
  goal.pointing_axis.y = 0.0;
  goal.pointing_axis.z = 1.0;
  goal.min_duration = ros::Duration(1.0);
  goal.max_velocity = 0.25;
  goal.target = pointStamped;

  return goal;
}

// ROS call back for every new image received
void imageCallback(const sensor_msgs::ImageConstPtr& imgMsg)
{
  // std::cout << "inside image cb" << std::endl;
  latestImageStamp = imgMsg->header.stamp;
  cv_bridge::CvImagePtr cvImgPtr;

  latestImage = *imgMsg;

  cvImgPtr = cv_bridge::toCvCopy(latestImage, sensor_msgs::image_encodings::BGR8);
  cv::imshow(windowName, cvImgPtr->image);
  cv::waitKey(15);
  // ROS_INFO("INSIDE IMAGE CALLBACK");
}

void objectCallback(const vision_msgs::Detection2DArray& objects)
{
  latestObjects = objects;
  // ROS_INFO("INSIDE OBJECTS CALLBACK");
}

void pcsCallback(const sensor_msgs::PointCloud2& pcs)
{
  latestPointClouds = pcs;
  // ROS_INFO("INSIDE POINTCLOUDS CALLBACK");
}

void cylinderPoseCallback(const geometry_msgs::PoseStamped& pose)
{
  latestCylinderPose = pose;
  // ROS_INFO("INSIDE CYLINDER CALLBACK");
}

// IMPORTANT: this is the function manage the whole perception process
void action_cb(const set_nav_goal::ToPerceptionGoalConstPtr& goal, 
                                      ToPerceptionServer* as, ros::Publisher* pub_to_cyli)
{
  ROS_INFO("Executing ToPerception");

  // ros::NodeHandle nh_;
  // ros::ServiceClient mrClient = nh_.serviceClient<mask_rcnn_ros::mask_rcnn>("/to_maskrcnn");
  bool success = false;

  if (as -> isPreemptRequested() || !ros::ok())
  {
    ROS_INFO("Preempted");
    // set the action state to preempted
    as -> setPreempted();
    return;
  }

  /* -------------------------------------------------------------------------- */
  /*        Look down, roughly look at the table. A pre-defined behavior        */
  /* -------------------------------------------------------------------------- */
  auto ph_goal = getPointHeadGoal(265, 466);
  pointHeadClient -> sendGoal(ph_goal);
  pointHeadClient -> waitForResult();
  if(pointHeadClient -> getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    ROS_INFO("Point Head Goal succeeded!");
    success = true;
  }
  else
  {
    ROS_INFO("Point Head Goal failed");
    return;
  }

  ros::Duration(3.0).sleep(); // NOTE: waiting for messages, give time to align. maybe sleep too long

/* -------------------------------------------------------------------------- */
/*          Process the object hypothesis, get target pixel positions         */
/* -------------------------------------------------------------------------- */

  int t_u = -1, t_v = -1; // default invalid values. If not valid, keep detecting
  int target_id = goal -> object_id;
  while(t_u == -1 || t_v == -1)
  {
    std::vector<vision_msgs::Detection2D> objects_2d = latestObjects.detections;
    std::cout << "objects_2d " << objects_2d.size() << std::endl;
    for(auto vit = objects_2d.begin(), vend = objects_2d.end(); vit != vend; vit++)
    {
      std::vector<vision_msgs::ObjectHypothesisWithPose> hypos = vit -> results;
      std::cout << "hypos " << hypos.size() << std::endl;
      for(int i = 0; i < hypos.size(); i++)
      {
        std::cout << "hypo id " << hypos[i].id << std::endl;
        if(hypos[i].id == std::to_string(target_id)) {
          t_u = vit -> bbox.center.x;
          t_v = vit -> bbox.center.y;
          break;
        }
      }
    }
    ros::Duration(1.0).sleep(); // NOTE: maybe sleep too long
  }

  /* -------------------------------------------------------------------------- */
  /*                          Look at the target object                         */
  /* -------------------------------------------------------------------------- */

  auto target_ph_goal = getPointHeadGoal(t_u, t_v);
  pointHeadClient -> sendGoal(target_ph_goal);
  pointHeadClient -> waitForResult();
  if(pointHeadClient -> getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    ROS_INFO("Target Point Head Goal succeeded!");
    success = true;
  }
  else
  {
    ROS_INFO("Target Point Head Goal failed");
    return;
  }
  ros::Duration(1.0).sleep(); // NOTE: maybe sleep too long


/* -------------------------------------------------------------------------- */
/*          Get current target pixel positions, and also the bbox size        */
/* -------------------------------------------------------------------------- */
  int tc_u = -1, tc_v = -1, tc_size_u = -1, tc_size_v = -1;
  while(tc_u == -1 || tc_v == -1 || tc_size_u == -1 || tc_size_v == -1)
  {
    std::vector<vision_msgs::Detection2D> objects_2d = latestObjects.detections;
    std::cout << "objects_2d " << objects_2d.size() << std::endl;
    for(auto vit = objects_2d.begin(), vend = objects_2d.end(); vit != vend; vit++)
    {
      std::vector<vision_msgs::ObjectHypothesisWithPose> hypos = vit -> results;
      std::cout << "hypos " << hypos.size() << std::endl;
      for(int i = 0; i < hypos.size(); i++)
      {
        std::cout << "hypo id " << hypos[i].id << std::endl;
        if(hypos[i].id == std::to_string(target_id)) {
          tc_u = int(vit -> bbox.center.x);
          tc_v = int(vit -> bbox.center.y);
          tc_size_u = int(vit -> bbox.size_x);
          tc_size_v = int(vit -> bbox.size_y);
          break;
        }
      }
    }
    ros::Duration(1.0).sleep(); // NOTE: maybe sleep too long
  }

  /* -------------------------------------------------------------------------- */
  /*      Hack the 3D pcs msg, calculate the corresp index from the 2D bbox     */
  /*                       And Construct the filtered pcs                       */
  /* -------------------------------------------------------------------------- */
  sensor_msgs::PointCloud2 pcs_filtered;
  std::cout << pcs_filtered.data.size() << std::endl;
  int POINT_STEP = latestPointClouds.point_step; // NOTE: 32, actually
  size_t tc_u_min = int(tc_u - tc_size_u / 2) * POINT_STEP;
  size_t tc_v_min = int(tc_v - tc_size_v / 2);
  size_t tc_u_max = tc_u_min + tc_size_u * POINT_STEP;
  size_t tc_v_max = tc_v_min + tc_size_v;

  pcs_filtered.header.frame_id = latestPointClouds.header.frame_id;
  pcs_filtered.header.stamp = ros::Time::now();
  pcs_filtered.height = tc_size_v;
  pcs_filtered.width = tc_size_u;
  pcs_filtered.fields = latestPointClouds.fields;
  pcs_filtered.is_bigendian = latestPointClouds.is_bigendian;
  pcs_filtered.point_step = POINT_STEP; //32
  pcs_filtered.row_step = POINT_STEP * tc_size_u;
  pcs_filtered.is_dense = latestPointClouds.is_dense;

  // Here not using ROS_INFO to make these debug info stand out in the terminal
  std::cout << tc_u << ' ' << tc_v << ' ' << tc_size_u << ' ' << tc_size_v << std::endl;
  std::cout << tc_u_min << ' ' << tc_v_min << ' ' << tc_u_max << ' ' << tc_v_max << std::endl;

  int raw_row_step = latestPointClouds.row_step;
  decltype(latestPointClouds.data) filtered_data = {};
  
  // IMPORTANT: this loop filters the wanted pointcloud, using indexes calculated above
  for(size_t row = tc_v_min; row < tc_v_max; row++)
  {
    // ROS_INFO("debug 1");
    for(size_t col = tc_u_min; col < tc_u_max; col++)
    {
      // ROS_INFO("debug 2");
      auto dat = latestPointClouds.data[row * raw_row_step + col];
      // ROS_INFO("debug 3");
      filtered_data.push_back(dat);
    }
  }
  pcs_filtered.data = filtered_data;

  /* -------------------------------------------------------------------------- */
  /*                          Publish the filtered pcs                          */
  /* -------------------------------------------------------------------------- */
  while(latestCylinderPose.header.frame_id == "None") // keep publishing until pcl gives a calculated pose
  // while(ros::ok()) // NOTE: keep publishing for use of visualizing it in rviz
  {
    pub_to_cyli -> publish(pcs_filtered);
    ros::Duration(0.1).sleep();
  }

  /* -------------------------------------------------------------------------- */
  /*           Process the pose got from pcl, transform between frames          */
  /* -------------------------------------------------------------------------- */

  // NOTE: Hardcoded pose value(not used anymore)

  // geometry_msgs::PoseStamped object_pose;
  // object_pose.header.stamp = latestImageStamp;
  // object_pose.header.frame_id = "base_footprint";
  // object_pose.pose.position.x = 0.85123;
  // object_pose.pose.position.y = 0.015;
  // object_pose.pose.position.z = 0.8815;
  // object_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0.0, 0.0, 0.0);
  // // object_pose.pose.orientation.x = 0.0;
  // // object_pose.pose.orientation.y = 0.0;
  // // object_pose.pose.orientation.z = 0.0;
  // // object_pose.pose.orientation.w = 0.0;

  tf::TransformListener listener;

  ros::Duration(3.0).sleep(); // NOTE: maybe this sleeps too long, but works, so leave it as it is
  ros::Time t = ros::Time(0);
  geometry_msgs::PoseStamped mpose = latestCylinderPose;
  mpose.header.stamp = ros::Time::now();
  ros::Duration(3.0).sleep();
  geometry_msgs::PoseStamped mpose_transf;
  ROS_INFO("Waiting for transform for some time...");
  listener.waitForTransform("base_footprint","xtion_rgb_optical_frame", t, ros::Duration(5.0));

  if(listener.canTransform("/base_footprint", "xtion_rgb_optical_frame", t))
  {
    listener.transformPose("/base_footprint", mpose, mpose_transf);
    // Here not using ROS_INFO to make these debug info stand out in the terminal 
    std::cout << latestCylinderPose.pose.position.x << std::endl;
    std::cout << latestCylinderPose.pose.position.y << std::endl;
    std::cout << latestCylinderPose.pose.position.z << std::endl;
  }
  else
  {
    ROS_ERROR("Transformation is not possible");
    return;
  }

  /* -------------------------------------------------------------------------- */
  /*          Return the success of the perception back to set_nav_goal         */
  /* -------------------------------------------------------------------------- */
  result_.object_pose = mpose_transf;
  as -> setSucceeded(result_, "Object Detection Succeed");
  ROS_INFO("To Perception Goal succeeded!");

}

// NOTE: Not implemented yet, for service calls version
void action_cb_srv(const set_nav_goal::ToPerceptionGoalConstPtr& goal, 
                                        ToPerceptionServer* as, ros::ServiceClient* mmdetClient)
{
  return;
}

int main(int argc, char** argv)
{
  // Init the ROS node
  ros::init(argc, argv, "find_object");

  ros::NodeHandle nh;
  // Precondition: Valid clock
  if (!ros::Time::waitForValid(ros::WallDuration(10.0))) // NOTE: Important when using simulated clock
  {
    ROS_FATAL("Timed-out waiting for valid time.");
    return EXIT_FAILURE;
  }

  ROS_INFO("TEST CMAKE(REMOVE THIS LATER)");
  ROS_INFO("Starting find_object application ...");

  // Get the camera intrinsic parameters from the appropriate ROS topic
  ROS_INFO("Waiting for camera intrinsics ... ");
  sensor_msgs::CameraInfoConstPtr msg = ros::topic::waitForMessage
      <sensor_msgs::CameraInfo>(cameraInfoTopic, ros::Duration(10.0));
  if(msg.use_count() > 0)
  {
    cameraIntrinsics = cv::Mat::zeros(3,3,CV_64F);
    cameraIntrinsics.at<double>(0, 0) = msg->K[0]; //fx
    cameraIntrinsics.at<double>(1, 1) = msg->K[4]; //fy
    cameraIntrinsics.at<double>(0, 2) = msg->K[2]; //cx
    cameraIntrinsics.at<double>(1, 2) = msg->K[5]; //cy
    cameraIntrinsics.at<double>(2, 2) = 1;
  }

  // Create a point head action client to move the TIAGo++'s head
  createPointHeadClient(pointHeadClient);

  // Create the window to show TIAGo++'s camera images
  cv::namedWindow(windowName, cv::WINDOW_AUTOSIZE);

  /* -------------------------------------------------------------------------- */
  /*              Define A LOT OF instances used for the perception             */
  /* -------------------------------------------------------------------------- */
  // Define ROS topic from where TIAGo++ publishes images
  image_transport::ImageTransport it(nh);
  // use compressed image transport to use less network bandwidth
  image_transport::TransportHints transportHint("compressed");

  ROS_INFO_STREAM("Subscribing to " << imageTopic << " ...");
  image_transport::Subscriber sub_from_cam = it.subscribe(imageTopic, 1,
                                                 imageCallback, transportHint);

  bool _use_service = false;
  nh.getParam("use_service", _use_service);

  ros::Subscriber sub_from_pcs = nh.subscribe(pointCloudsTopic, 1, pcsCallback);
  ros::Publisher pub_to_cyli = nh.advertise<sensor_msgs::PointCloud2>("/to_cylinder_detector", 1);
  ROS_INFO("FIND OBJECT NODE - SUBSCRIBE MODE");
  ros::Subscriber sub_from_objects = nh.subscribe(ObjectsTopic, 1, objectCallback);
  ros::Subscriber sub_from_cyli_pose = nh.subscribe(cylinderPoseTopic, 1, cylinderPoseCallback);
  latestCylinderPose.header.frame_id = "None";
  ToPerceptionServer server(nh, "/to_perception", boost::bind(&action_cb, _1, &server, &pub_to_cyli), false);

  // NOTE: Not impletement yet
  // if(_use_service)
  // {
  //   ROS_INFO("FIND OBJECT NODE - SERVICE MODE");
  //   ros::ServiceClient client = nh.serviceClient<find_object::mmdetSrv>("/to_mmdetector");
  //   ToPerceptionServer server(nh, "/to_perception", boost::bind(&action_cb_srv, _1, &server, &client), false);
  // }
  
  server.start();
  ros::spin();
  cv::destroyWindow(windowName);

  return EXIT_SUCCESS;
}

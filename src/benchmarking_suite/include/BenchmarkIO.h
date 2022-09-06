/*
==============================================================================
MIT License

Copyright 2022 Institute for Automotive Engineering of RWTH Aachen University.

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
==============================================================================
*/

#pragma once

#define PCL_NO_PRECOMPILE

#include <cstdlib>
#include <memory>
#include <sstream>
#include <string>

#include <benchmarking_suite/ObjectList.h>
#include <benchmarking_suite/params_BenchmarkIOConfig.h>
#include <dynamic_reconfigure/server.h>
#include <nodelet/nodelet.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Header.h>
#include <velodyne_msgs/VelodyneScan.h>
#include <velodyne_pcl/point_types.h>


namespace benchmarking_suite {


typedef params_BenchmarkIOConfig DynRcfgParams;
typedef velodyne_pcl::PointXYZIRT PointT;
typedef pcl::PointCloud<PointT> PointCloud;


class BenchmarkIO : public nodelet::Nodelet {

 public:
  virtual void onInit();

 protected:
  void loadParameters();

  void setup();

  void dynamicReconfigure(DynRcfgParams& params, uint32_t level);

  void publishPointCloudPacketsFromBag(const ros::TimerEvent& event);

  void publishPointCloud(const sensor_msgs::PointCloud2::ConstPtr& msg);

  void publishPointCloudPackets(
    const velodyne_msgs::VelodyneScan::ConstPtr& msg);

  void logObjectListArrival(
    const benchmarking_suite::ObjectList::ConstPtr& msg);

 protected:
  ros::NodeHandle node_handle_;
  ros::NodeHandle private_node_handle_;

  std::shared_ptr<dynamic_reconfigure::Server<DynRcfgParams>>
    dynamic_reconfigure_server_;

  ros::Subscriber pcl_sub_;
  ros::Subscriber pcl_packets_sub_;
  ros::Publisher pcl_pub_;
  ros::Publisher pcl_packets_pub_;
  ros::Subscriber obj_sub_;
  ros::Publisher timestamp_pcl_pub_;
  ros::Publisher timestamp_obj_pub_;
  ros::Publisher bag_packets_pub_;
  ros::Timer bag_packets_timer_;

  const std::string kInputPointCloudTopic = "pointcloud";
  const std::string kInputPointCloudPacketsTopic = "packets";
  const std::string kOutputPointCloudTopic = "pointcloud/benchmark";
  const std::string kOutputPointCloudPacketsTopic = "packets/benchmark";
  const std::string kObjectListTopic = "object_list";
  const std::string kTimestampTopicPrefix = "timestamps";
  const std::string kBagRateParam = "bag_rate";
  const std::string kBagFileParam = "bag_file";

  double bag_rate_ = 1.0;
  std::string bag_file_;
  velodyne_msgs::VelodyneScan bag_packets_;
  bool send_packets_ = true;
  double p_packets_ = 1.0;
  int n_points_ = 1000000;
};


}  // namespace benchmarking_suite

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

#include <BenchmarkIO.h>
#include <timestamps.h>

#include <benchmarking_suite/TimeStamped.h>
#include <pcl/filters/random_sample.h>
#include <pluginlib/class_list_macros.h>
#include <ros/package.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>


PLUGINLIB_EXPORT_CLASS(benchmarking_suite::BenchmarkIO, nodelet::Nodelet)


namespace benchmarking_suite {


void BenchmarkIO::onInit() {

  node_handle_ = this->getMTNodeHandle();
  private_node_handle_ = this->getMTPrivateNodeHandle();

  loadParameters();
  setup();
}


void BenchmarkIO::loadParameters() {

  // parameters for publishing from bagfile
  private_node_handle_.param<double>(kBagRateParam, bag_rate_, bag_rate_);
  if (private_node_handle_.getParam(kBagFileParam, bag_file_)) {
    std::string package_path = ros::package::getPath("benchmarking_suite");
    if (!bag_file_.empty() && bag_file_[0] != '/')
      bag_file_ = package_path + "/" + bag_file_;
    NODELET_INFO(
      "Publishing first 'velodyne_msgs::VelodyneScan' from bag '%s' at %.2fHz",
      bag_file_.c_str(), bag_rate_);
  }
}


void BenchmarkIO::setup() {

  // setup dynamic_reconfigure
  dynamic_reconfigure_server_ =
    std::make_shared<dynamic_reconfigure::Server<DynRcfgParams>>(
      private_node_handle_);
  dynamic_reconfigure::Server<DynRcfgParams>::CallbackType dyn_rcfg_cb =
    boost::bind(&BenchmarkIO::dynamicReconfigure, this, _1, _2);
  dynamic_reconfigure_server_->setCallback(dyn_rcfg_cb);

  // subscriber and publisher for republication of pointclouds
  std::string input_topic =
    private_node_handle_.resolveName(kInputPointCloudTopic);
  std::string input_packets_topic =
    private_node_handle_.resolveName(kInputPointCloudPacketsTopic);
  std::string output_topic =
    private_node_handle_.resolveName(kOutputPointCloudTopic);
  std::string output_packets_topic =
    private_node_handle_.resolveName(kOutputPointCloudPacketsTopic);
  pcl_sub_ = private_node_handle_.subscribe(
    input_topic, 1, &BenchmarkIO::publishPointCloud, this);
  pcl_pub_ =
    private_node_handle_.advertise<sensor_msgs::PointCloud2>(output_topic, 1);
  pcl_packets_sub_ = private_node_handle_.subscribe(
    input_packets_topic, 1, &BenchmarkIO::publishPointCloudPackets, this);
  pcl_packets_pub_ =
    private_node_handle_.advertise<velodyne_msgs::VelodyneScan>(
      output_packets_topic, 1);
  NODELET_INFO("Re-publishing 'velodyne_msgs::VelodyneScan' on '%s' to '%s'",
               input_packets_topic.c_str(), output_packets_topic.c_str());
  NODELET_INFO("Re-publishing 'sensor_msgs::PointCloud2' on '%s' to '%s'",
               input_topic.c_str(), output_topic.c_str());

  // subscriber for reponses
  std::string object_list_topic =
    private_node_handle_.resolveName(kObjectListTopic);
  obj_sub_ = private_node_handle_.subscribe(
    object_list_topic, 1, &BenchmarkIO::logObjectListArrival, this);
  NODELET_INFO("Waiting for object lists on '%s'", object_list_topic.c_str());

  // publishers for timestamps
  std::string timestamp_pcl_topic =
    createTimestampTopic(output_topic, kTimestampTopicPrefix);
  std::string timestamp_obj_topic =
    createTimestampTopic(object_list_topic, kTimestampTopicPrefix);
  timestamp_pcl_pub_ =
    private_node_handle_.advertise<TimeStamped>(timestamp_pcl_topic, 1);
  timestamp_obj_pub_ =
    private_node_handle_.advertise<TimeStamped>(timestamp_obj_topic, 1);

  // rosbag packets publisher, if given
  if (!bag_file_.empty()) {
    rosbag::Bag bag;
    bag.open(bag_file_);
    for (const auto& m : rosbag::View(bag)) {
      velodyne_msgs::VelodyneScan::ConstPtr packets =
        m.instantiate<velodyne_msgs::VelodyneScan>();
      if (packets != nullptr) {
        bag_packets_ = *packets;
        break;
      }
    }
    bag.close();
    bag_packets_pub_ =
      private_node_handle_.advertise<velodyne_msgs::VelodyneScan>(
        input_packets_topic, 1);
    bag_packets_timer_ = private_node_handle_.createTimer(
      ros::Duration(1 / bag_rate_),
      &BenchmarkIO::publishPointCloudPacketsFromBag, this);
  }
}


void BenchmarkIO::dynamicReconfigure(DynRcfgParams& params, uint32_t level) {

  if (send_packets_ != params.send_packets) {
    send_packets_ = params.send_packets;
    if (send_packets_)
      NODELET_INFO(
        "Re-publishing pointclouds as 'velodyne_msgs::VelodyneScan'");
    else
      NODELET_INFO("Re-publishing pointclouds as 'sensor_msgs::PointCloud2'");
  }

  n_points_ = params.n_points;
  p_packets_ = params.p_packets;

  if (send_packets_)
    NODELET_INFO("Downsampling pointcloud packets to %d%%",
                 static_cast<int>(p_packets_ * 100));
  else
    NODELET_INFO("Downsampling pointclouds to %d points", n_points_);

  bag_rate_ = params.bag_rate;
  if (!bag_file_.empty()) {
    bag_packets_timer_ = private_node_handle_.createTimer(
      ros::Duration(1 / bag_rate_),
      &BenchmarkIO::publishPointCloudPacketsFromBag, this);
    NODELET_INFO("Publishing at %.3f Hz", bag_rate_);
  }
}


void BenchmarkIO::publishPointCloudPacketsFromBag(
  const ros::TimerEvent& event) {
  bag_packets_pub_.publish(bag_packets_);
}


void BenchmarkIO::publishPointCloud(
  const sensor_msgs::PointCloud2::ConstPtr& msg) {

  if (send_packets_) return;

  // convert to PCL
  PointCloud::Ptr pcl_ptr(new PointCloud);
  pcl::fromROSMsg(*msg, *pcl_ptr);

  // randomly filter down to set number of points
  PointCloud filtered_pcl;
  pcl::RandomSample<PointT> random_filter;
  random_filter.setInputCloud(pcl_ptr);
  random_filter.setSample(n_points_);
  random_filter.filter(filtered_pcl);

  // convert to ROS message
  sensor_msgs::PointCloud2 filtered_msg;
  pcl::toROSMsg(filtered_pcl, filtered_msg);

  // set header stamp to current time for matching
  ros::WallTime now = ros::WallTime::now();
  filtered_msg.header.stamp.sec = now.sec;
  filtered_msg.header.stamp.nsec = now.nsec;

  NODELET_DEBUG("Publishing pointcloud #%d with %ld points",
                filtered_msg.header.seq, filtered_pcl.size());

  // publish pointcloud
  pcl_pub_.publish(filtered_msg);

  // publish publication timestamp
  ros::WallTime t_out = ros::WallTime::now();
  TimeStamped ts = createTimestamp(t_out, filtered_msg.header);
  timestamp_pcl_pub_.publish(ts);
}


void BenchmarkIO::publishPointCloudPackets(
  const velodyne_msgs::VelodyneScan::ConstPtr& msg) {

  if (!send_packets_) return;

  // copy packets
  velodyne_msgs::VelodyneScan scan = *msg;

  // randomly downsample to percentage of original packets
  int n_packets = static_cast<int>(scan.packets.size() * p_packets_);
  while (scan.packets.size() > n_packets) {
    int idx = std::rand() % scan.packets.size();
    scan.packets.erase(scan.packets.begin() + idx);
  }

  // set header stamp to current time for matching
  ros::WallTime now = ros::WallTime::now();
  scan.header.stamp.sec = now.sec;
  scan.header.stamp.nsec = now.nsec;

  NODELET_DEBUG("Publishing pointcloud #%d with %ld packets", scan.header.seq,
                scan.packets.size());

  // publish pointcloud packets
  pcl_packets_pub_.publish(scan);

  // publish publication timestamp
  ros::WallTime t_out = ros::WallTime::now();
  TimeStamped ts = createTimestamp(t_out, scan.header);
  timestamp_pcl_pub_.publish(ts);
}


void BenchmarkIO::logObjectListArrival(
  const benchmarking_suite::ObjectList::ConstPtr& msg) {

  // publish arrival timestamp
  ros::WallTime now = ros::WallTime::now();
  TimeStamped ts = createTimestamp(now, msg->header);
  timestamp_obj_pub_.publish(ts);
}


}  // namespace benchmarking_suite

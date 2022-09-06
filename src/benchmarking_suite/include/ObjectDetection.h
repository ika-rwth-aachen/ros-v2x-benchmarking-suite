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

#include <memory>
#include <string>

#include <benchmarking_suite/ObjectList.h>
#include <benchmarking_suite/params_ObjectDetectionConfig.h>
#include <dynamic_reconfigure/server.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>


namespace benchmarking_suite {


typedef params_ObjectDetectionConfig DynRcfgParams;


class ObjectDetection : public nodelet::Nodelet {

 public:
  virtual void onInit();

 protected:
  void loadParameters();

  void setup();

  void dynamicReconfigure(DynRcfgParams& params, uint32_t level);

  void detectObjects(const sensor_msgs::PointCloud2::ConstPtr& msg);

 protected:
  ros::NodeHandle node_handle_;
  ros::NodeHandle private_node_handle_;

  std::shared_ptr<dynamic_reconfigure::Server<DynRcfgParams>>
    dynamic_reconfigure_server_;

  ros::Subscriber pcl_sub_;
  ros::Publisher obj_pub_;
  ros::Publisher timestamp_pcl_pub_;
  ros::Publisher timestamp_obj_pub_;

  const std::string kInputTopic = "pointcloud";
  const std::string kOutputTopic = "object_list";
  const std::string kTimestampTopicPrefix = "timestamps";

  int n_objects_ = 20;
  double latency_ = 0.0;
};


}  // namespace benchmarking_suite

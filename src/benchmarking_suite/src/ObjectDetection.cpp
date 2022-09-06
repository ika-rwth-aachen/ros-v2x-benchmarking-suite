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

#include <ObjectDetection.h>
#include <timestamps.h>

#include <benchmarking_suite/ObjectList.h>
#include <benchmarking_suite/TimeStamped.h>
#include <pluginlib/class_list_macros.h>


PLUGINLIB_EXPORT_CLASS(benchmarking_suite::ObjectDetection,
                       nodelet::Nodelet)


namespace benchmarking_suite {


void ObjectDetection::onInit() {

  node_handle_ = this->getMTNodeHandle();
  private_node_handle_ = this->getMTPrivateNodeHandle();

  loadParameters();
  setup();
}


void ObjectDetection::loadParameters() {}


void ObjectDetection::setup() {

  // setup dynamic_reconfigure
  dynamic_reconfigure_server_ =
    std::make_shared<dynamic_reconfigure::Server<DynRcfgParams>>(
      private_node_handle_);
  dynamic_reconfigure::Server<DynRcfgParams>::CallbackType dyn_rcfg_cb =
    boost::bind(&ObjectDetection::dynamicReconfigure, this, _1, _2);
  dynamic_reconfigure_server_->setCallback(dyn_rcfg_cb);

  // subscriber and publisher
  std::string input_topic = private_node_handle_.resolveName(kInputTopic);
  std::string output_topic = private_node_handle_.resolveName(kOutputTopic);
  pcl_sub_ = private_node_handle_.subscribe(
    input_topic, 1, &ObjectDetection::detectObjects, this);
  obj_pub_ =
    private_node_handle_.advertise<benchmarking_suite::ObjectList>(
      output_topic, 1);
  NODELET_INFO("Publishing objects detected in '%s' to '%s'",
               input_topic.c_str(), output_topic.c_str());

  // publishers for timestamps
  std::string timestamp_pcl_topic =
    createTimestampTopic(input_topic, kTimestampTopicPrefix);
  std::string timestamp_obj_topic =
    createTimestampTopic(output_topic, kTimestampTopicPrefix);
  timestamp_pcl_pub_ =
    private_node_handle_.advertise<TimeStamped>(timestamp_pcl_topic, 1);
  timestamp_obj_pub_ =
    private_node_handle_.advertise<TimeStamped>(timestamp_obj_topic, 1);
}


void ObjectDetection::dynamicReconfigure(DynRcfgParams& params,
                                              uint32_t level) {

  n_objects_ = params.n_objects;
  latency_ = params.latency;
}


void ObjectDetection::detectObjects(
  const sensor_msgs::PointCloud2::ConstPtr& msg) {

  ros::WallTime t_in = ros::WallTime::now();

  // create dummy object list
  benchmarking_suite::ObjectList object_list;
  object_list.header = msg->header;
  for (int i = 0; i < n_objects_; i++)
    object_list.objects.push_back(benchmarking_suite::Object());

  // wait for specified duration
  ros::Duration(latency_).sleep();

  // publish object list
  obj_pub_.publish(object_list);

  // publish timestamps
  ros::WallTime t_out = ros::WallTime::now();
  TimeStamped ts_in = createTimestamp(t_in, msg->header);
  TimeStamped ts_out = createTimestamp(t_out, msg->header);
  timestamp_pcl_pub_.publish(ts_in);
  timestamp_obj_pub_.publish(ts_out);
}


}  // namespace benchmarking_suite

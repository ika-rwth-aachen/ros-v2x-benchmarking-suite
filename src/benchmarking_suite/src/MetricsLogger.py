#!/usr/bin/env python

# ==============================================================================
# MIT License
#
# Copyright 2022 Institute for Automotive Engineering of RWTH Aachen University.
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
# ==============================================================================

import csv
import json
import os
from collections import deque
from typing import Dict, List

import actionlib
import numpy as np
import rospy
from rospy_message_converter import message_converter

from benchmarking_suite.msg import (ExperimentAction,
                                                 ExperimentFeedback,
                                                 ExperimentGoal,
                                                 ExperimentResult, Metrics,
                                                 TimeStamped)


class MetricsLogger:

    def __init__(self):

        rospy.init_node("metrics_logger")
        self.loadParameters()
        self.setup()

    def loadParameters(self):

        self.topics = rospy.get_param("~topics_in_order", [])
        self.topics = sorted(set(self.topics), key=self.topics.index)
        self.n_topics = len(self.topics)
        rospy.loginfo(
            f"Computing metrics from timestamps on {self.n_topics} topics")

        self.metrics_window = rospy.get_param("~metrics_window", 10)

        self.export = rospy.get_param("~export_timestamps", False)
        self.export_file = None
        if self.export:
            self.export_file = rospy.get_param("~export_file", "timestamps.csv")
            if not os.path.isabs(self.export_file):
                self.export_file = os.path.realpath(
                    os.path.join(__file__, os.pardir, os.pardir,
                                 self.export_file))
            if os.path.isfile(self.export_file):
                rospy.logfatal(
                    f"Output file '{self.export_file}' already exists")
            rospy.loginfo(f"Exporting timestamps to '{self.export_file}'")

    def setup(self):

        self.should_start_recording = False
        self.remaining_samples_to_record = 0
        self.recorded_metrics = Metrics()
        self.recorded_timestamps = []
        self.timestamps = {}
        self.latencies = [
            deque(maxlen=self.metrics_window) for i in range(self.n_topics)
        ]  # last entry is total latency

        # initialize action server
        self.action_server = actionlib.SimpleActionServer(rospy.get_name(),
                                                          ExperimentAction,
                                                          self.startExperiment,
                                                          auto_start=False)
        self.action_server.start()

        # create latency publisher
        self.pub = rospy.Publisher("~metrics", Metrics, queue_size=10)

        # subscribe to timestamp topics
        self.subs = []
        for topic in self.topics:
            sub = rospy.Subscriber(topic,
                                   TimeStamped,
                                   self.logTimestamp,
                                   callback_args=topic)
            self.subs.append(sub)

    def logTimestamp(self, timestamp: TimeStamped, topic: str):

        # log timestamp
        key = timestamp.header.stamp
        if key not in self.timestamps:
            self.timestamps[key] = {}
        self.timestamps[key][topic] = timestamp.time

        # compute metrics if all timestamps for one sample have arrived
        if len(self.timestamps[key]) == self.n_topics:
            self.computeMetrics(self.timestamps[key])
            if self.export:
                self.exportTimestamps(self.timestamps[key], self.export_file)
            self.timestamps.pop(key)

    def computeMetrics(self, timestamps: Dict[str, rospy.Time]):

        # initialize latency containers for recording
        if self.should_start_recording:

            self.latencies = [
                deque(maxlen=self.remaining_samples_to_record)
                for i in range(self.n_topics)
            ]
            self.should_start_recording = False

        # compute most recent latencies
        partial = []
        for idx, (topic1, topic2) in enumerate(zip(self.topics,
                                                   self.topics[1:])):
            t1 = timestamps[topic1]
            t2 = timestamps[topic2]
            latency = self.computeLatency(t1, t2)
            partial.append(latency)
            self.latencies[idx].append(latency)
        total = self.computeLatency(timestamps[self.topics[0]],
                                    timestamps[self.topics[-1]])
        self.latencies[-1].append(total)

        # convert latencies to np-matrix for easier computation
        # (metrics_window x n_topics)
        latencies = np.array(self.latencies).transpose()

        # compute mean, median, corrected-std of latencies
        latency_means = np.mean(latencies, axis=0)
        latency_medians = np.median(latencies, axis=0)
        if latencies.shape[0] > 1:
            latency_stds = np.std(latencies, ddof=1, axis=0)
        else:
            latency_stds = np.std(latencies, axis=0)

        # compute min/max of latencies
        latency_mins = np.min(latencies, axis=0)
        latency_maxs = np.max(latencies, axis=0)

        # publish metrics
        metrics = self.createMetrics(total, partial, latencies.shape[0],
                                     latency_means[-1], latency_means[:-1],
                                     latency_medians[-1], latency_medians[:-1],
                                     latency_stds[-1], latency_stds[:-1],
                                     latency_mins[-1], latency_mins[:-1],
                                     latency_maxs[-1], latency_maxs[:-1])
        self.pub.publish(metrics)

        # give action feedback, if recording
        if self.remaining_samples_to_record > 0:
            self.remaining_samples_to_record -= 1
            self.recorded_metrics = metrics
            self.recorded_timestamps.append(timestamps)
            feedback = ExperimentFeedback()
            feedback.n_remaining_samples = self.remaining_samples_to_record
            feedback.metrics = metrics
            self.action_server.publish_feedback(feedback)

    def computeLatency(self, t1: rospy.Time, t2: rospy.Time) -> float:

        return (t2 - t1).to_sec()

    def createMetrics(self, total_latency: float, latencies: List[float],
                      metrics_window: int, total_latency_mean: float,
                      latency_means: List[float], total_latency_median: float,
                      latency_medians: List[float], total_latency_std: float,
                      latency_stds: List[float], total_latency_min: float,
                      latency_mins: List[float], total_latency_max: float,
                      latency_maxs: List[float]):

        msg = Metrics()
        msg.total_latency = total_latency
        msg.latencies = latencies
        msg.metrics_window = metrics_window
        msg.total_latency_mean = total_latency_mean
        msg.latency_means = latency_means
        msg.total_latency_median = total_latency_median
        msg.latency_medians = latency_medians
        msg.total_latency_std = total_latency_std
        msg.latency_stds = latency_stds
        msg.total_latency_min = total_latency_min
        msg.latency_mins = latency_mins
        msg.total_latency_max = total_latency_max
        msg.latency_maxs = latency_maxs

        return msg

    def exportTimestamps(self,
                         timestamps: Dict[str, rospy.Time],
                         output_file: str,
                         new: bool = False):

        def ts2str(ts: rospy.Time) -> str:
            return f"{ts.secs}.{ts.nsecs:09d}"

        t = [ts2str(timestamps[topic]) for topic in self.topics]

        os.makedirs(os.path.dirname(output_file), exist_ok=True)
        if not os.path.isfile(output_file) or new:
            with open(output_file, "w") as f:
                writer = csv.writer(f)
                writer.writerow(self.topics)

        with open(output_file, "a") as f:
            writer = csv.writer(f)
            writer.writerow(t)

    def exportMetrics(self, metrics: Metrics, output_file: str):

        with open(output_file, "w") as f:
            d = message_converter.convert_ros_message_to_dictionary(metrics)
            json.dump(d, f, indent=2)

    def startExperiment(self, goal: ExperimentGoal):

        # check goal arguments
        if len(goal.name) == 0 or goal.n_samples <= 0:
            self.action_server.set_aborted(text="Input error")
            return

        # start experiment by setting flags
        rospy.loginfo(
            f"Starting experiment '{goal.name}' with {goal.n_samples} samples")
        self.should_start_recording = True
        self.remaining_samples_to_record = goal.n_samples

        # check status
        success = True
        rate = rospy.Rate(1000)
        while True:
            rate.sleep()
            if self.action_server.is_preempt_requested():
                rospy.loginfo("Canceling experiment '{goal.name}'")
                success = False
                break
            if self.remaining_samples_to_record == 0:
                rospy.loginfo(f"Finished experiment '{goal.name}'")
                break

        # construct experiment result
        result = ExperimentResult()
        result.metrics = self.recorded_metrics

        # export timestamps
        output_dir = os.path.realpath(
            os.path.join(__file__, os.pardir, os.pardir, "experiments"))
        output_file = os.path.join(output_dir, f"{goal.name}_timestamps.csv")
        self.exportTimestamps(self.recorded_timestamps[0],
                              output_file,
                              new=True)
        for timestamps in self.recorded_timestamps[1:]:
            self.exportTimestamps(timestamps, output_file)
        rospy.loginfo(
            f"Exported timestamps of experiment '{goal.name}' to '{output_file}'"
        )

        # export metrics
        output_file = os.path.join(output_dir, f"{goal.name}_metrics.json")
        self.exportMetrics(self.recorded_metrics, output_file)

        # cleanup
        self.should_start_recording = False
        self.remaining_samples_to_record = 0
        self.recorded_metrics = Metrics()
        self.recorded_timestamps = []

        if success:
            self.action_server.set_succeeded(result)
        else:
            self.action_server.set_aborted(text="Cancelled")


if __name__ == "__main__":

    node = MetricsLogger()
    rospy.spin()


#ifndef CLOUD_RECORDER_H
#define CLOUD_RECORDER_H

#include <exception>
#include <string>
#include <iostream>

#include <pcl/io/vtk_io.h>
#include <pcl/io/pcd_io.h>
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <sensor_msgs/PointCloud2.h>

#include "husky_trainer/NamedPointCloud.h"

#define FILE_FORMAT_PARAM "format"
#define DEST_TOPIC_PARAM "republish"
#define SOURCE_TOPIC_PARAM "source"
#define TARGET_FRAME_PARAM "target_frame"
#define WORKING_DIRECTORY_PARAM "working_directory"
#define DEFAULT_FORMAT "vtk"


class CloudRecorder {
public:
    CloudRecorder(ros::NodeHandle n);
    void record(const husky_trainer::NamedPointCloudConstPtr& msg);
    void spin();
    static void saveAsVTK(std::string name, const sensor_msgs::PointCloud2& cloud);
    static void saveAsPCD(std::string name, const sensor_msgs::PointCloud2& cloud);
    sensor_msgs::PointCloud2 transformToFrame(const sensor_msgs::PointCloud2& cloud,
                                              std::string targetFrame, tf::TransformListener& tf);

private:
    tf::TransformListener tfListener;
    ros::Publisher publisherTopic;
    ros::Subscriber sourceTopic;
    std::string fileFormat;
    std::string targetFrame;
    bool republish;
};

#endif

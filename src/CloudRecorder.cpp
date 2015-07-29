
#include "husky_trainer/CloudRecorder.h"

CloudRecorder::CloudRecorder(ros::NodeHandle n)
{
    std::string publisherTopicName;
    std::string sourceTopicName;
    republish = false;

    std::string workingDirectory;
    n.getParam(WORKING_DIRECTORY_PARAM, workingDirectory);

    if(chdir(workingDirectory.c_str()) != 0)
    {
        ROS_WARN("Could not switch to demanded directory. Using CWD instead.");
    };

    if(!n.getParam(SOURCE_TOPIC_PARAM, sourceTopicName))
    {
        ROS_ERROR("You must specify a source topic.");
        ros::shutdown();
    }
    else
    {
        sourceTopic = n.subscribe(sourceTopicName, 100, &CloudRecorder::record, this);
    }

    if(!n.getParam(FILE_FORMAT_PARAM, fileFormat))
    {
        fileFormat = DEFAULT_FORMAT;
    }
    else
    {
        std::string formatList[] = {"vtk", "pcd"};
        std::set<std::string> allowedFormats(formatList, formatList+2);
        if(allowedFormats.find(fileFormat) == allowedFormats.end())
        {
            ROS_WARN_STREAM("Cannot save a cloud in format: " << fileFormat);
            ROS_WARN_STREAM("Reverting to default format: " << DEFAULT_FORMAT);
            fileFormat = DEFAULT_FORMAT;
        }
    }

    if(n.getParam(DEST_TOPIC_PARAM, publisherTopicName))
    {
        republish = true;
        publisherTopic = n.advertise<sensor_msgs::PointCloud2>(publisherTopicName, 1000);
    }

    n.param<std::string>(TARGET_FRAME_PARAM, targetFrame, "");
}

void CloudRecorder::record(const husky_trainer::NamedPointCloudConstPtr& msg)
{
    sensor_msgs::PointCloud2 cloud = targetFrame == "" ? msg->cloud : transformToFrame(msg->cloud, targetFrame, tfListener);

    if(fileFormat == "pcd") saveAsPCD(msg->name, cloud);
    else if(fileFormat == "vtk") saveAsVTK(msg->name, cloud);

    ROS_INFO_STREAM("Saved point cloud with name: " << msg->name << ".");
}

void CloudRecorder::saveAsPCD(std::string name, const sensor_msgs::PointCloud2 &cloud)
{
    pcl::PCLPointCloud2 pclCloud;
    pcl_conversions::toPCL(cloud, pclCloud);
    pcl::io::savePCDFile(name, pclCloud);
}

void CloudRecorder::saveAsVTK(std::string name, const sensor_msgs::PointCloud2 &cloud)
{
    pcl::PCLPointCloud2 pclCloud;
    pcl_conversions::toPCL(cloud, pclCloud);
    pcl::io::saveVTKFile(name, pclCloud);
}

sensor_msgs::PointCloud2 CloudRecorder::transformToFrame(const sensor_msgs::PointCloud2& cloud, std::string targetFrame, tf::TransformListener& tf)
{
    tf::StampedTransform transform;
    try {
        tf.waitForTransform(targetFrame, cloud.header.frame_id, cloud.header.stamp, ros::Duration(3.0));
        tf.lookupTransform(targetFrame, cloud.header.frame_id, cloud.header.stamp, transform);
    } catch(tf::TransformException &ex) {
        ROS_WARN_STREAM("Could not find a transform to " << targetFrame << " from "
                        << cloud.header.frame_id);
    }

    sensor_msgs::PointCloud2 newCloud;
    pcl_ros::transformPointCloud(targetFrame, transform, cloud, newCloud);
    return newCloud;
}

void CloudRecorder::spin()
{
    ros::spin();
}

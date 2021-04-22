#include <iostream>
#include <iomanip>
#include <chrono>
#include <functional>
#include <memory>
#include <utility>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud.hpp"
#include "laser_geometry/laser_geometry.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/conversions.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/common.h>
#include <pcl/filters/passthrough.h>
#include <pcl/visualization/pcl_visualizer.h>

//pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");
pcl::visualization::PCLVisualizer viewer ("ICP demo");


int main(int argc, char * argv[])
{
        //Convert ROS pointcloud2 to PCL pointcloud
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud <pcl::PointXYZRGB>);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_projected(new pcl::PointCloud <pcl::PointXYZRGB>);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud <pcl::PointXYZRGB>);

    if (pcl::io::loadPCDFile("/home/vivi/map_fuse/src/pc_projection/params/cam_4901.pcd", *cloud) < 0)
    {
        PCL_ERROR ("Error loading cloud %s.\n", argv[1]);
        return (-1);
    }

        //Get minimum y points for ground removal
        pcl::PointXYZRGB minPt, maxPt;
        pcl::getMinMax3D (*cloud, minPt, maxPt);

        //remove ground plane points
        pcl::PassThrough<pcl::PointXYZRGB> pass;
        pass.setInputCloud (cloud);
        pass.setFilterFieldName ("y");
        pass.setFilterLimits (minPt.y, maxPt.y-0.2);
//pass.setFilterLimitsNegative (false);
        pass.filter (*cloud_filtered);

        // Create a set of planar coefficients with X=Z=0,Y=1
//        pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
//        coefficients->values.resize (4);
//        coefficients->values[0] = 0;
//        coefficients->values[1] = 1.0;
//        coefficients->values[2] = 0;
//        coefficients->values[3] = 0;
//
//        // Create the filtering object
//        pcl::ProjectInliers<pcl::PointXYZRGB> proj;
//        proj.setModelType (pcl::SACMODEL_PLANE);
//        proj.setInputCloud (cloud_filtered);
//        proj.setModelCoefficients (coefficients);
//        proj.filter (*cloud_projected);


    viewer.setBackgroundColor (0, 0, 0);
    viewer.addPointCloud<pcl::PointXYZRGB> (cloud_filtered, "sample cloud");
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
    viewer.addCoordinateSystem (1.0);
    viewer.initCameraParameters ();
    while (!viewer.wasStopped ()) {
        viewer.spinOnce();
    }

//        viewer.showCloud (cloud);
//    while (!viewer.wasStopped ()){}

    return 0;
}


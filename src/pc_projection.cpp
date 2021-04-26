#include <iostream>
#include <iomanip>
#include <functional>
#include <memory>
#include <utility>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud.hpp"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/conversions.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/common.h>
#include <pcl/filters/passthrough.h>
#include <pcl/registration/icp.h>

//using namespace std::chrono_literals;

//pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");

class MinimalPublisher : public rclcpp::Node
{
public:
    MinimalPublisher()
            : Node("minimal_publisher"), count_(0)
    {
       
        publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("projected_pointcloud", 1);
        
        //subscriber for simulation (gazebo):
        //subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>("scan", rclcpp::QoS(rclcpp::SystemDefaultsQoS()), std::bind(&MinimalPublisher::scanCallback, this, _1));
        //this->set_parameter(rclcpp::Parameter("use_sim_time", true));

        //subscriber frmw_qos_profileor real life scanner:
        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>("camera/depth/color/points", 1 , std::bind(&MinimalPublisher::pcCallback, this, std::placeholders::_1));

    }


private:
    void pcCallback (const sensor_msgs::msg::PointCloud2::SharedPtr pc_in) {
        
        //Convert ROS pointcloud2 to PCL pointcloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud <pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected(new pcl::PointCloud <pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud <pcl::PointXYZ>);
        pcl::fromROSMsg(*pc_in, *cloud);



        Eigen::Matrix4f matrix = Eigen::Matrix4f::Identity();

        matrix (0, 0) = 0.0;
        matrix (0, 1) = 0.0;
        matrix (0, 2) = 1.0;
        matrix (1, 0) = -1.0;
        matrix (1, 1) = 0.002;
        matrix (1, 2) = 0.0;
        matrix (2, 0) = -0.002;
        matrix (2, 1) = -1.0;
        matrix (2, 2) = 0.0;

        matrix (0, 3) = 0.295;
        matrix (1, 3) = -0.16133;
        matrix (2, 3) = 0.17734;

        pcl::transformPointCloud (*cloud , *cloud, matrix);

        //Get minimum z points for ground removal
        pcl::PointXYZ minPt, maxPt;
        pcl::getMinMax3D (*cloud, minPt, maxPt);

        //remove ground plane points
        pcl::PassThrough<pcl::PointXYZ> pass;
        pass.setInputCloud (cloud);
        pass.setFilterFieldName ("z");
        pass.setFilterLimits (0 ,maxPt.z);
        pass.setFilterLimitsNegative (false);
        pass.filter (*cloud_filtered);
        //RCLCPP_INFO(this->get_logger(), "Publishing %f", minPt.z);

        // Create a set of planar coefficients with X=Z=0,Y=1
        pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
        coefficients->values.resize (4);
        coefficients->values[0] = 0;
        coefficients->values[1] = 0;
        coefficients->values[2] = 1.0;
        coefficients->values[3] = 0;

        // Create the filtering object
        pcl::ProjectInliers<pcl::PointXYZ> proj;
        proj.setModelType (pcl::SACMODEL_PLANE);
        proj.setInputCloud (cloud_filtered);
        proj.setModelCoefficients (coefficients);
        proj.filter (*cloud_projected);

        //viewer.showCloud (cloud_projected);

        sensor_msgs::msg::PointCloud2 output;
        rclcpp::Time t = rclcpp::Node::now();
        output.header.stamp = t;
        pcl::toROSMsg(*cloud_projected, output);
        publisher_->publish(output);

    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    size_t count_;

};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MinimalPublisher>());
    rclcpp::shutdown();
    return 0;
}


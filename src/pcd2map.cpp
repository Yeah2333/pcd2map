//
// Created by wangzhiyong on 19-3-12.
//
#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <string>

using namespace pcl;



int main (int argc, char **argv)
{
    ros::init (argc, argv, "map_cloud");
    ros::NodeHandle nh;
    ros::NodeHandle nh_param("~");//用于读取launch文件的参数 必须

    ros::Publisher pcl_pub = nh.advertise<sensor_msgs::PointCloud2> ("map_cloud", 2);  //参数缓冲区大小　２
    ros::Publisher pcl_pub_full = nh.advertise<sensor_msgs::PointCloud2> ("map_cloud_full", 2);  //参数缓冲区大小　２

    std::string pcd_file_path; //pcd文件路径
    double maxh1 = -555.22; //
    double minh1;//

    nh_param.param<std::string>("pcd_file",pcd_file_path,""); //从launch文件获得三个参数
    nh_param.param<double>("maxh",maxh1,2.0);
    nh_param.param<double>("minh",minh1,0.1);

    ros::spinOnce();

    if(maxh1 == -555.22) //用于检测是否加载成功参数
    {
        ROS_INFO("Can't load param!!");
        return 1;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new PointCloud<pcl::PointXYZ>); //
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>); //

    if(pcl::io::loadPCDFile<pcl::PointXYZ> (pcd_file_path, *cloud) == -1){
        ROS_INFO("Can't load pcd file");
        return 1;
    }

    pcl::io::loadPCDFile<PointXYZ> (pcd_file_path, *cloud); //读取文件

    ROS_INFO("pcd file input complete!");

    ros::Rate loop_rate(10);//发布速度
    ROS_INFO("Start to publish /map_cloud");

    while(ros::ok()) {
        pcl::PassThrough<pcl::PointXYZ> pass;
        pass.setInputCloud(cloud);
        pass.setFilterFieldName("z");
        pass.setFilterLimits(minh1, maxh1);
        pass.setFilterLimitsNegative(false);
        //pass.setNegative(true);
        pass.filter(*cloud_filtered);

        //转换回PointCloud2发布 截取后的
        sensor_msgs::PointCloud2 cloud_filteredMsg;
        pcl::toROSMsg(*cloud_filtered, cloud_filteredMsg);
        cloud_filteredMsg.header.frame_id = "/world";
        pcl_pub.publish(cloud_filteredMsg);

        //完整地图
        sensor_msgs::PointCloud2 cloud_full;
        pcl::toROSMsg(*cloud, cloud_full);
        cloud_full.header.frame_id = "/world";
        pcl_pub_full.publish(cloud_full);



        ROS_INFO("-----------Publish map_cloud complete!-----------");
        loop_rate.sleep();  //发布速度
    }


    return 0;
}



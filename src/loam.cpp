#include "ros/ros.h"
#include <iostream>
#include <string>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/time.h>
#include <Eigen/Dense>
#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

Eigen::Vector3d gps;
PointCloudT::Ptr map (new PointCloudT);
PointCloudT::Ptr cloud_in (new PointCloudT);
PointCloudT::Ptr cloud_icp (new PointCloudT);
PointCloudT::Ptr map_filtered (new PointCloudT);
PointCloudT::Ptr cloud_in_filtered (new PointCloudT);
Eigen::Matrix4d tr;

int i=0,j=0;

void GPS(const geometry_msgs::PointStamped::ConstPtr& msg)
{

  if(i==0){
    gps.x() = msg->point.x;
    gps.y() = msg->point.y;
    gps.z() = msg->point.z;
    i++;
  }
}

void pc2(const sensor_msgs::PointCloud2::ConstPtr& msg)
{

  if(j==0){
    sensor_msgs::PointCloud2 output;
    output = *msg;
    pcl::fromROSMsg (output, *cloud_in);
    j++;
  }
}

void
print4x4Matrix (const Eigen::Matrix4d & matrix)
{
  printf ("Rotation matrix :\n");
  printf ("    | %6.3f %6.3f %6.3f | \n", matrix (0, 0), matrix (0, 1), matrix (0, 2));
  printf ("R = | %6.3f %6.3f %6.3f | \n", matrix (1, 0), matrix (1, 1), matrix (1, 2));
  printf ("    | %6.3f %6.3f %6.3f | \n", matrix (2, 0), matrix (2, 1), matrix (2, 2));
  printf ("Translation vector :\n");
  printf ("t = < %6.3f, %6.3f, %6.3f >\n\n", matrix (0, 3), matrix (1, 3), matrix (2, 3));
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "loam");
  ros::NodeHandle n;
  ros::Subscriber sub_gps = n.subscribe("fix", 1000, GPS);
  ros::Subscriber sub_pc2 = n.subscribe("lidar_points", 1000, pc2);
  pcl::IterativeClosestPoint<PointT, PointT> icp;
  double theta=0,score=DBL_MAX;
  int iterations = 100;

  if (pcl::io::loadPCDFile<PointT> ("/home/kiki/catkin_ws/src/HW_loam/map.pcd", *map) == -1)
  {
      PCL_ERROR ("Couldn't read file map.pcd \n");
      return (-1);
  }

  while(ros::ok()){

    if(i==1 && j==1){

      pcl::PassThrough<PointT> pass;
      pass.setInputCloud (map);
      pass.setFilterFieldName ("z");
      pass.setFilterLimits (-2.0, 1.0);
      pass.filter (*map_filtered);

      pass.setInputCloud (cloud_in);
      pass.setFilterFieldName ("z");
      pass.setFilterLimits (-2.0, 1.0);
      pass.filter (*cloud_in_filtered);

      while(theta < (2*M_PI+0.1)){
        Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity ();
        theta += M_PI/12;
        transformation_matrix (0, 0) = cos (theta);
        transformation_matrix (0, 1) = -sin (theta);
        transformation_matrix (1, 0) = sin (theta);
        transformation_matrix (1, 1) = cos (theta);
        transformation_matrix (0, 3) = gps.x();
        transformation_matrix (1, 3) = gps.y();
        transformation_matrix (2, 3) = gps.z();

        print4x4Matrix (transformation_matrix);

        pcl::transformPointCloud (*cloud_in_filtered, *cloud_icp, transformation_matrix);

        icp.setMaximumIterations (iterations);
        icp.setInputSource (cloud_icp);
        icp.setInputTarget (map_filtered);
        icp.align (*cloud_icp);

        if (icp.hasConverged ())
        {
          std::cout << "\nICP has converged, score is " << icp.getFitnessScore () << std::endl;
        }
        else
        {
          PCL_ERROR ("\nICP has not converged .\n");
          return (-1);
        }
        if(icp.getFitnessScore ()<score){
           score = icp.getFitnessScore ();
           tr = transformation_matrix;
        }
      }
      std::cout << "\nICP has converged, score is " << score << std::endl;
      print4x4Matrix (tr);

      pcl::transformPointCloud (*cloud_in_filtered, *cloud_icp, tr);

      icp.setMaximumIterations (iterations);
      icp.setInputSource (cloud_icp);
      icp.setInputTarget (map_filtered);
      icp.align (*cloud_icp);

      pcl::visualization::PCLVisualizer viewer ("ICP demo");

      int v1 (0);

      float bckgr_gray_level = 0.0;
      float txt_gray_lvl = 1.0 - bckgr_gray_level;

      pcl::visualization::PointCloudColorHandlerCustom<PointT> map_color_h (map_filtered, (int) 255 * txt_gray_lvl, (int) 255 * txt_gray_lvl, (int) 255 * txt_gray_lvl);

      viewer.addPointCloud (map_filtered, map_color_h, "map_v1", v1);

      pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_icp_color_h (cloud_icp, 180, 20, 20);
      viewer.addPointCloud (cloud_icp, cloud_icp_color_h, "cloud_icp_v1", v1);

      viewer.addText ("White: Original point cloud\nRed: ICP aligned point cloud", 10, 15, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "icp_info", v1);

      std::stringstream ss;
      ss << iterations;
      std::string iterations_cnt = "ICP iterations = " + ss.str ();
      viewer.addText (iterations_cnt, 10, 60, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "iterations_cnt", v1);

      viewer.setBackgroundColor (bckgr_gray_level, bckgr_gray_level, bckgr_gray_level, v1);

      viewer.setCameraPosition (-3.68332, 2.94092, 5.71266, 0.289847, 0.921947, -0.256907, 0);
      viewer.setSize (1280, 1024);

      while (!viewer.wasStopped ())
      {
          viewer.spinOnce ();
      }
    }

    ros::spinOnce();
  }
  return 0;
}

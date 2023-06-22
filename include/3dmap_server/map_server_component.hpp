#ifndef MAP_SERVER_COMPONENT_HPP_
#define MAP_SERVER_COMPONENT_HPP_

#if __cplusplus
extern "C" {
#endif

// The below macros are taken from https://gcc.gnu.org/wiki/Visibility and from
// demos/composition/include/composition/visibility_control.h at https://github.com/ros2/demos
#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define MAP_SERVER_COMPONENT_EXPORT __attribute__((dllexport))
#define MAP_SERVER_COMPONENT_IMPORT __attribute__((dllimport))
#else
#define MAP_SERVER_COMPONENT_EXPORT __declspec(dllexport)
#define MAP_SERVER_COMPONENT_IMPORT __declspec(dllimport)
#endif
#ifdef MAP_SERVER_COMPONENT_BUILDING_DLL
#define MAP_SERVER_COMPONENT_PUBLIC \
  MAP_SERVER_COMPONENT_EXPORT
#else
#define MAP_SERVER_COMPONENT_PUBLIC \
  MAP_SERVER_COMPONENT_IMPORT
#endif
#define MAP_SERVER_COMPONENT_PUBLIC_TYPE \
  MAP_SERVER_COMPONENT_PUBLIC
#define MAP_SERVER_COMPONENT_LOCAL
#else
#define MAP_SERVER_COMPONENT_EXPORT \
  __attribute__((visibility("default")))
#define MAP_SERVER_COMPONENT_IMPORT
#if __GNUC__ >= 4
#define MAP_SERVER_COMPONENT_PUBLIC \
  __attribute__((visibility("default")))
#define MAP_SERVER_COMPONENT_LOCAL __attribute__((visibility("hidden")))
#else
#define MAP_SERVER_COMPONENT_PUBLIC
#define MAP_SERVER_COMPONENT_LOCAL
#endif
#define MAP_SERVER_COMPONENT_PUBLIC_TYPE
#endif

#if __cplusplus
}  // extern "
#endif

#include <iostream>
#include <memory>
#include <chrono>
#include <vector>
#include <string>

#include "boost/shared_ptr.hpp"
#include "boost/make_shared.hpp"
#include "rclcpp/rclcpp.hpp"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/segmentation/extract_clusters.h"
#include "pcl/filters/extract_indices.h"
#include "pcl/filters/voxel_grid.h"
#include "sensor_msgs/msg/point_cloud2.hpp"

using namespace std::chrono_literals;


namespace map_server
{

  class MapServerComponent : public rclcpp::Node
  {
    public:
      MAP_SERVER_COMPONENT_PUBLIC
      explicit MapServerComponent(const rclcpp::NodeOptions & options);
      virtual ~MapServerComponent(void);
    
    private:
      void pointcloud_callback(void);
      void plane_removal(pcl::PointCloud<pcl::PointXYZ>& input_cloud);
      void DownSampling(void);
      pcl::PointCloud<pcl::PointXYZ>::Ptr euclideanclustering(pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud);

      int cloud_width;
      int cloud_height;

      rclcpp::TimerBase::SharedPtr timer_;
      rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_publisher_;
    
      bool flag;
      std::string pcd_path_;
      float cluster_tolerance_param_;
      float min_cluster_sizeparam_;


      sensor_msgs::msg::PointCloud2 output;

      //スマートポインタインスタンス
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
      pcl::PointCloud<pcl::PointXYZ>::Ptr clustered_cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
      pcl::search::KdTree<pcl::PointXYZ>::Ptr tree = boost::make_shared<pcl::search::KdTree<pcl::PointXYZ>>();
      pcl::PointCloud<pcl::PointXYZ>::Ptr tmp_clustered_points = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
      pcl::PointIndices::Ptr tmp_cluster_indices = boost::make_shared<pcl::PointIndices>();

      //voxel grid
      pcl::VoxelGrid<pcl::PointXYZ> vg;
      pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_voxel_grid = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

      //ユークリッドクラスタリング
      
      //クラスタを格納する配列
      std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters;
      //クラスタリング後のインデックスが格納される配列
      std::vector<pcl::PointIndices> cluster_indices;
      pcl::EuclideanClusterExtraction<pcl::PointXYZ> ece;
      pcl::ExtractIndices<pcl::PointXYZ> ei;

      //パラメータ
      bool cluster_flg;
      double cluster_tolerance;
      int min_cluster_size;

      //時間計測のための変数
      float time;

  };

}

#endif
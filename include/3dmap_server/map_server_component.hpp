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

#include <memory>
#include <chrono>
#include <vector>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"
#include "sensor_msgs/msg/point_cloud2.hpp"

using namespace std::chrono_literals;


namespace map_server
{
  typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;


  class MapServerComponent : public rclcpp::Node
  {
    public:
      MAP_SERVER_COMPONENT_PUBLIC
      explicit MapServerComponent(const rclcpp::NodeOptions & options);
      virtual ~MapServerComponent();
    
    private:
      void pointcloud_callback();

      int cloud_width;
      int cloud_height;

      rclcpp::TimerBase::SharedPtr timer_;
      rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_publisher_;
    
      bool flag;
      std::string pcd_path;

      sensor_msgs::msg::PointCloud2 output;

      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  };

}

#endif
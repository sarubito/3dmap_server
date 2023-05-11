#include "rclcpp/rclcpp.hpp"
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"
#include "sensor_msgs/msg/point_cloud2.hpp"

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
bool flg = true;
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

using namespace std::chrono_literals;


class PublishPc : public rclcpp::Node
{
    public:
        PublishPc(): Node("publish_pc")
        {   
            pointcloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("map_pointcloud", 10);
            timer_ = this->create_wall_timer(500ms, std::bind(&PublishPc::timer_callback, this));
        }
    
    private:
        void timer_callback()
        {
            if(flg == true){
                if(pcl::io::loadPCDFile<pcl::PointXYZ>("map.pcd", *cloud) == -1)
                {
                    RCLCPP_INFO(this->get_logger(),"Couldn't read file map.pcd\n");
                
                }
                flg = false;
                RCLCPP_INFO(this->get_logger(),"Loaded pcd \n");
            }
            
            RCLCPP_INFO(this->get_logger(),"Loaded pcd \n");
            cloud_width = cloud->width;
            cloud_height = cloud->height;
            RCLCPP_INFO(this->get_logger(), "width : %d, height : %d\n", cloud_width, cloud_height);
            
            sensor_msgs::msg::PointCloud2 output;
            cloud->header.frame_id = "map";
            cloud->header.stamp = this->get_clock()->now().nanoseconds() ;

            pcl::toROSMsg(*cloud, output);
            pointcloud_publisher_->publish(output);
            
        }

        int cloud_width;
        int cloud_height;

        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_publisher_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PublishPc>());
    rclcpp::shutdown();
    return 0;
}
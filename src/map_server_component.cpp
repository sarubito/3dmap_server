#include "3dmap_server/map_server_component.hpp"
#include "rclcpp_components/register_node_macro.hpp"


namespace map_server
{
    MapServerComponent::MapServerComponent(const rclcpp::NodeOptions & options) : Node("map_server", options)
    {
        pointcloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("map_pointcloud", 10);
        timer_ = this->create_wall_timer(500ms, std::bind(&MapServerComponent::pointcloud_callback, this));
        flag = true;

        this->declare_parameter("pcdfile_path", "map.pcd");
        pcd_path = this->get_parameter("pcdfile_path").as_string();
        
    }

    void MapServerComponent::pointcloud_callback()
    {
        if(flag == true){
            if(pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_path, *cloud) == -1)
            {
                RCLCPP_INFO(this->get_logger(),"Couldn't read file map.pcd\n");
            
            }
            flag = false;
        }
        
        cloud_width = cloud->width;
        cloud_height = cloud->height;
        //RCLCPP_INFO(this->get_logger(), "width : %d, height : %d\n", cloud_width, cloud_height);
        
        cloud->header.frame_id = "map";
        cloud->header.stamp = this->get_clock()->now().nanoseconds() ;

        pcl::toROSMsg(*cloud, this->output);
        pointcloud_publisher_->publish(this->output);
    }

    MapServerComponent::~MapServerComponent(){}
}

RCLCPP_COMPONENTS_REGISTER_NODE(map_server::MapServerComponent)
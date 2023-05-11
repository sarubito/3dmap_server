#include "rclcpp/rclcpp.hpp"
#include "3dmap_server/map_server_component.hpp"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor exec;
    auto publish_pointcloud_component = std::make_shared<map_server::MapServerComponent>(rclcpp::NodeOptions());
    exec.add_node(publish_pointcloud_component);
    exec.spin();
    rclcpp::shutdown();
}
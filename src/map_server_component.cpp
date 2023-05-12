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
        pcd_path_ = this->get_parameter("pcdfile_path").as_string();
        this->declare_parameter("cluster_tolerance", 0.1);
        this->get_parameter("cluster_tolerance", cluster_tolerance_param_);
        this->declare_parameter("min_cluster_size", 0.1);
        this->get_parameter("min_cluster_size", min_cluster_sizeparam_);

        //クラスタリング初期化
        cluster_flg = true;
        cluster_tolerance = cluster_tolerance_param_; 
        min_cluster_size = min_cluster_sizeparam_;
        
    }

    void MapServerComponent::pointcloud_callback(void)
    {
        if(flag == true){
            if(pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_path_, *cloud) == -1)
            {
                RCLCPP_INFO(this->get_logger(),"Couldn't read file map.pcd\n");
            
            }
            flag = false;
        }
        
        cloud_width = cloud->width;
        cloud_height = cloud->height;
        RCLCPP_INFO(this->get_logger(), "before width : %d, height : %d\n", cloud_width, cloud_height);
        
        cloud->header.frame_id = "map";
        cloud->header.stamp = this->get_clock()->now().nanoseconds() ;

        plane_removal(*cloud);

        cloud_width = cloud->width;
        cloud_height = cloud->height;

        clustered_cloud = euclideanclustering(cloud);

        pcl::toROSMsg(*cloud, this->output);
        pointcloud_publisher_->publish(this->output);
    }

    void MapServerComponent::plane_removal(pcl::PointCloud<pcl::PointXYZ>& input_cloud)
    {
        for(pcl::PointCloud<pcl::PointXYZ>::iterator it=input_cloud.begin(); it != input_cloud.end(); it++)
        {
            if(it->z <= -1.5) cloud->erase(it);
        }
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr MapServerComponent::euclideanclustering(pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud)
    {
        //クラスタリングの時間計測開始
        time = this->get_clock()->now().seconds();
        tree->setInputCloud(input_cloud);
        ece.setClusterTolerance(cluster_tolerance);
        ece.setMinClusterSize(min_cluster_size);
        ece.setMaxClusterSize(input_cloud->points.size());
        //探索方法を設定
	    ece.setSearchMethod(tree);
        ece.setInputCloud(input_cloud);
        ece.extract(cluster_indices);

        RCLCPP_INFO(this->get_logger(), "cluster_indices.size : %ld", cluster_indices.size());

        ei.setInputCloud(input_cloud);
        ei.setNegative(false);
        for(size_t i=0;i<cluster_indices.size();i++)
        {
               *tmp_cluster_indices = cluster_indices[i];
               ei.setIndices(tmp_cluster_indices);
               ei.filter(*tmp_clustered_points);
               clusters.push_back(tmp_clustered_points);
        }
        RCLCPP_INFO(this->get_logger(), "clustering time [s] : %f", this->get_clock()->now().seconds() - time);
        return tmp_clustered_points;
    }

    MapServerComponent::~MapServerComponent(void){}
}

RCLCPP_COMPONENTS_REGISTER_NODE(map_server::MapServerComponent)
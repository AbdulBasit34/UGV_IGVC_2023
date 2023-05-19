#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <costmap_2d/layer.h>
#include <costmap_2d/costmap_2d.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <sl/Camera.hpp>

class LaneCostmapLayer : public costmap_2d::Layer
{
public:
    LaneCostmapLayer() : private_nh("~")
    {
        // Replace "/lane_obstacles" with the appropriate topic name for your ZED 2i camera
// e.g., "/zed2/zed_node/point_cloud/cloud_registered"
         lane_obstacle_sub = nh.subscribe("/zed2/zed_node/point_cloud/cloud_registered", 1,    &LaneCostmapLayer::lane_obstacles_callback, this);

    }

    void laneObstaclesCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
    {
        pcl::fromROSMsg(*msg, lane_points_);
    }

    virtual void updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y, double* max_x, double* max_y)
    {
        if (lane_points_.empty())
            return;

        for (const auto& point : lane_points_)
        {
            *min_x = std::min(*min_x, point.x);
            *min_y = std::min(*min_y, point.y);
            *max_x = std::max(*max_x, point.x);
            *max_y = std::max(*max_y, point.y);
        }
    }

    virtual void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j)
    {
        if (lane_points_.empty())
            return;

        for (const auto& point : lane_points_)
        {
            unsigned int i, j;
            if (master_grid.worldToMap(point.x, point.y, i, j) && i >= min_i && i <= max_i && j >= min_j && j <= max_j)
            {
                master_grid.setCost(i, j, costmap_2d::LETHAL_OBSTACLE);
            }
        }
    }

private:
    ros::NodeHandle private_nh;
    ros::Subscriber lane_obstacle_sub_;
    pcl::PointCloud<pcl::PointXYZ> lane_points_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "lane_costmap_layer");
    LaneCostmapLayer lane_costmap_layer;
    ros::spin();

    return 0;
}





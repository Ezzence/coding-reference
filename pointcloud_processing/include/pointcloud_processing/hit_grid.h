
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <std_srvs/Empty.h>

#include <open3d/Open3D.h>
#include "open3d_conversions/open3d_conversions.h"

struct HitCell
{
    bool bHit_;
    size_t hitCount_;
    Eigen::Vector3d center_;

    double xMin_;
    double xMax_;
    double yMin_;
    double yMax_;
    double zMin_;
    double zMax_;

    HitCell(const Eigen::Vector3d& p, double width, double depth) : center_(p)
    {
        xMin_ = center_.x() - width/2;
        xMax_ = center_.x() + width/2;
        yMin_ = center_.y() - depth/2;
        yMax_ = center_.y() + depth/2;
        zMin_ = center_.z() - width/2;
        zMax_ = center_.z() + width/2;
    };
};

class HitGrid
{
protected:
    ros::NodeHandle nh_;

    ros::Publisher pubVisualization_;
    visualization_msgs::Marker msgVisualization_;

    std::vector<HitCell> cells_; // Not storing pointers but lvalues in std::vector is actually more efficient if there are more iterations over the container than updates because of cache performance
    size_t rows_ = 0;
    size_t cols_ = 0;

    double cellWidth_;
    double cellDepth_;
    double xStart_, xEnd_, y_, zStart_, zEnd_;

    // TODO: use std::shared_ptr<PointCloud> VoxelDownSample(double voxel_size) const;

public:
    HitGrid();

    ~HitGrid()
    {
    }

    bool make_grid(double cellWidth, double cellLength, double xStart, double xEnd, double y, double zStart, double yEnd);

    void count_hits(const std::shared_ptr<open3d::geometry::PointCloud>& pcd);

    double calc_hit_rate();

    void publish_visualization();

    bool call_reload_params(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
    {
        return false;
    }

};

#include "pointcloud_processing/hit_grid.h"

HitGrid::HitGrid() : nh_("~")
{
    cellWidth_ = 0.2;
    cellDepth_ = 0.4;
    xStart_ = -0.6;
    xEnd_ = 0.6;
    y_ = 0;
    zStart_ = 0.0;
    zEnd_ = 1.8;

    pubVisualization_ = nh_.advertise<visualization_msgs::Marker>("grid", 1);

    cells_.reserve(100);
    make_grid(cellWidth_, cellDepth_, xStart_, xEnd_, y_, zStart_, zEnd_);
    ROS_INFO("Hit Grid Inited");
}

bool HitGrid::make_grid(double cellWidth, double cellDepth, double xStart, double xEnd, double y, double zStart, double zEnd)
{
    double x = xStart;
    double z = zStart;
    rows_ = 0;
    cols_ = 0;

    while (x + cellWidth/2 < xEnd)
    {
        rows_ = 0; // reset because only the last column is used for counting the rows
        while (z + cellWidth/2 < zEnd)
        {
            cells_.emplace_back(Eigen::Vector3d(x + cellWidth/2, y, z + cellWidth/2), cellWidth, cellDepth);
            z += cellWidth;
            rows_ += 1;
        }
        z = zStart;
        x += cellWidth;
        cols_ += 1;
    }

    ROS_INFO_STREAM("Hit Grid assembled, total cell count:" << cells_.size() << " rows:" << rows_ << " cols:" << cols_);
    return true;
}

void HitGrid::count_hits(const std::shared_ptr<open3d::geometry::PointCloud>& pcd)
{
    // TODO: try ComputePointCloudDistance() or ComputeNearestNeighborDistance()
    for (HitCell& cell : cells_)
    {
        cell.hitCount_ = 0;
        cell.bHit_ = false;
    }

    // ros::Time begin = ros::Time::now();

    for (const Eigen::Vector3d& p : pcd->points_)
    {
        for (HitCell& cell : cells_)
        {
            if (p.z() > cell.zMin_ && p.z() < cell.zMax_ && p.x() > cell.xMin_ && p.x() < cell.xMax_ && p.y() > cell.yMin_ && p.y() < cell.yMax_)
            {
                cell.hitCount_++;
                cell.bHit_ = true;
                break;
            }
        }
    }

    // ROS_INFO_STREAM("time: " << ros::Time::now().toSec() - begin.toSec());
}

double HitGrid::calc_hit_rate()
{
    double count = 0;
    for (HitCell& cell : cells_)
    {
        if (cell.bHit_){
            count += 1.0;
        }
    }
    return count / cells_.size();
}

void HitGrid::publish_visualization()
{
    msgVisualization_ = visualization_msgs::Marker();
    msgVisualization_.header.frame_id = "velodyne";
    msgVisualization_.header.stamp = ros::Time::now();
    msgVisualization_.action = visualization_msgs::Marker::ADD;
    msgVisualization_.type = visualization_msgs::Marker::LINE_LIST;
    msgVisualization_.scale.x = 0.02;
    msgVisualization_.pose = geometry_msgs::Pose();

    std::vector<geometry_msgs::Point> points;
    std::vector<std_msgs::ColorRGBA> colors;
    // points.reserve(100);

    geometry_msgs::Point p;
    std_msgs::ColorRGBA color;
    double d = cellWidth_/2;
    for (HitCell &cell : cells_)
    {
        p.x = cell.center_.x() - d; p.y = cell.center_.y(); p.z = cell.center_.z();
        points.push_back(p);
        p.x = cell.center_.x(); p.y = cell.center_.y(); p.z = cell.center_.z() + d;
        points.push_back(p);
        p.x = cell.center_.x(); p.y = cell.center_.y(); p.z = cell.center_.z() + d;
        points.push_back(p);
        p.x = cell.center_.x() + d; p.y = cell.center_.y(); p.z = cell.center_.z();
        points.push_back(p);
        p.x = cell.center_.x() + d; p.y = cell.center_.y(); p.z = cell.center_.z();
        points.push_back(p);
        p.x = cell.center_.x(); p.y = cell.center_.y(); p.z = cell.center_.z() - d;
        points.push_back(p);
        p.x = cell.center_.x(); p.y = cell.center_.y(); p.z = cell.center_.z() - d;
        points.push_back(p);
        p.x = cell.center_.x() - d; p.y = cell.center_.y(); p.z = cell.center_.z();
        points.push_back(p);

        if (cell.bHit_){
            color.r = 1.0; color.g = 1.0; color.b = 1.0; color.a = 1.0;
        } else {
            color.r = 0; color.g = 0; color.b = 0; color.a = 1.0;
        }
        colors.push_back(color); colors.push_back(color); colors.push_back(color); colors.push_back(color);
        colors.push_back(color); colors.push_back(color); colors.push_back(color); colors.push_back(color);

    }
    msgVisualization_.points = points;
    msgVisualization_.colors = colors;

    pubVisualization_.publish(msgVisualization_);
}

// Created by Ada on 16/03/21.

#include "pointcloud_processing/hit_grid.h"

#include "lorenz_shared/FenceScan.h"
#include "lorenz_shared/FenceScanResult.h"

using namespace open3d;

class FenceScan
{
protected:
    ros::NodeHandle nh_;
    ros::Subscriber subPoints_;
    ros::Subscriber subControl_;
    ros::Publisher pubResult_;
    ros::Publisher pubAligned_;
    ros::Publisher pubDebug_;

    sensor_msgs::PointCloud2 msgAligned_;
    sensor_msgs::PointCloud2 msgDebug_;
    lorenz_shared::FenceScanResult msgResult_;
    ros::Timer timerInput_;
    std::string frame_;

    bool bActive_ = false;
    bool bBreachPrev_ = false;

    double planeThreshold_ = 0.1;
    int planeIterations_ = 200;
    Eigen::Vector4d planeEq_;
    Eigen::Vector3d planeNormal_;

    HitGrid hitGrid_;

public:
    FenceScan() : nh_("~"), hitGrid_()
    {
        subPoints_ = nh_.subscribe("/velodyne_points", 1, &FenceScan::cb_points, this);
        subControl_ = nh_.subscribe("/fence_scan/control", 1, &FenceScan::cb_control, this);
        pubResult_ = nh_.advertise<lorenz_shared::FenceScanResult>("/fence_scan/result", 1);
        pubAligned_ = nh_.advertise<sensor_msgs::PointCloud2>("pc_aligned", 1);
        pubDebug_ = nh_.advertise<sensor_msgs::PointCloud2>("pc_debug", 1);

        timerInput_ = nh_.createTimer(ros::Duration(2.0), &FenceScan::cb_timer_input, this, false, true);

        ROS_INFO("Inited");
    }

    ~FenceScan()
    {
    }

    bool call_reload_params(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
    {
        return false;
    }

    bool align_pointcloud(std::shared_ptr<geometry::PointCloud>& pcd, const Eigen::Vector3d& planeNormal)
    {
        // Translate ponitcloud to align with hit grid (while keeping the ground level the same)
        Eigen::Vector3d pcdCenter = pcd->GetCenter();
        Eigen::Vector2d centerH(pcdCenter.x(), pcdCenter.y()); // horizontal center
        double centerV(pcdCenter.z()); // vertical center (height)
        if (centerH.norm() > 3.0){ // plane is more than 3 meters away horizontally
            ROS_WARN("Found plane is horizontally too far away!");
            return false;
        }
        if (std::abs(centerV) > 2.0){ // plane is vertically more than 2 meters away
            ROS_WARN("Found plane is vertically too far away!");
            return false;
        }

        // Translate and Rotate pointcloud to align with hit grid
        pcd->Translate(Eigen::Vector3d(-pcdCenter.x(), -pcdCenter.y(), 0));
        pcdCenter = pcd->GetCenter();

        double zAngle = std::atan2(planeNormal.x(), planeNormal.y());
        Eigen::AngleAxisd zRot(zAngle, Eigen::Vector3d::UnitZ());
        pcd->Rotate(zRot.toRotationMatrix(), pcdCenter);

        double xAngle = std::atan2(planeNormal.z(), planeNormal.y());
        Eigen::AngleAxisd xRot(-xAngle, Eigen::Vector3d::UnitX());
        pcd->Rotate(xRot.toRotationMatrix(), pcdCenter);

        // Hande height of fence last, to align it with the bottom of hit grid
        pcd->Translate(Eigen::Vector3d(0, 0, -(pcd->GetMinBound().z() + 0.1))); // offset in case of slightly tilted bottom
        // Alternative: Rotate around X axis before first translation, with center at 0


        // Rotating around the Y axis is less stable because the plane normal is almost parallel with it
        // double yAngle = std::atan2(planeNormal.z(), planeNormal.x());
        // Eigen::AngleAxisd yRot(yAngle, Eigen::Vector3d::UnitY());
        // pcd->Rotate(yRot.toRotationMatrix(), pcdCenter);

        // Color and publish transformed pointcloud
        Eigen::Vector3d color{1.0, 0, 0};
        pcd->PaintUniformColor(color);
        open3d_conversions::open3dToRos(*pcd, msgAligned_, frame_);
        pubAligned_.publish(msgAligned_);

        return true; // TODO: return false if fence is too misaligned
    }

    bool segment_fence(const sensor_msgs::PointCloud2ConstPtr &points, std::shared_ptr<geometry::PointCloud>& pcdInliers)
    {
        geometry::PointCloud pcd;
        open3d_conversions::rosToOpen3d(points, pcd);

        // Setup pcd TODO: refactor to new function
        pcdInliers = pcd.VoxelDownSample(0.05);
        Eigen::AngleAxisd xRot(-M_PI_2, Eigen::Vector3d::UnitY());
        pcdInliers->Rotate(xRot.toRotationMatrix(), Eigen::Vector3d(0, 0, 0));
        pcdInliers->Translate(Eigen::Vector3d(0, 0, 0.54));
        pcdInliers = pcdInliers->Crop(geometry::AxisAlignedBoundingBox(Eigen::Vector3d(-5.0, -5.0, -0.2), Eigen::Vector3d(5.0, -1.0, 5.0)));
        open3d_conversions::open3dToRos(*pcdInliers, msgDebug_, frame_);
        pubDebug_.publish(msgDebug_);

        // Perform fence segmentation
        std::vector<size_t, std::allocator<size_t>> idInlier;
        std::tie(planeEq_, idInlier) = pcdInliers->SegmentPlane(planeThreshold_, 3, planeIterations_);

        // Check if the y axis is more or less orthogonal to the plane
        if (std::abs(planeEq_[1]) < 0.9) // if not (e.g. ground), redo plane segmentation from outliers
        {
            // TODO: following 2 lines are temporary workaround
            ROS_WARN("No suitable plane found!");
            return false;
            // Redoing the segmentation like this is FAULTY (previous segmentation can remove points from new plane)
            // TODO: use second plane equation to create an oriented bounding box, and use that to crop the original pcd instead of this
            std::shared_ptr<geometry::PointCloud> pcdOutliers = pcdInliers->SelectByIndex(idInlier, true);
            std::tie(planeEq_, idInlier) = pcdOutliers->SegmentPlane(planeThreshold_, 3, planeIterations_);
            if (std::abs(planeEq_[1]) < 0.9){
                ROS_WARN("No suitable plane found!");
                return false;
            }
            pcdInliers = pcdOutliers->SelectByIndex(idInlier);
        }
        else // if orthogonal, use first segmentation
        {
            pcdInliers = pcdInliers->SelectByIndex(idInlier);
        }

        planeNormal_ = Eigen::Vector3d(planeEq_[0], planeEq_[1], planeEq_[2]);
        return true;

    }

    void cb_control(const lorenz_shared::FenceScanConstPtr& msg)
    {
        bActive_ = msg->isActive;
    }

    void cb_points(const sensor_msgs::PointCloud2ConstPtr& points)
    {
        ros::Time begin = ros::Time::now();
        timerInput_.stop();
        timerInput_.start();

        if (!bActive_){
            return; // fence scan behaviour not active
        }

        if (points->height == 0){
            ROS_WARN("LiDar data empty!");
            return; // input pointcloud empty
        }

        frame_ = points->header.frame_id;
        std::shared_ptr<geometry::PointCloud> pcdInliers;

        // Segment fence from LiDar pointcloud
        if (!segment_fence(points, pcdInliers)){
            return;
        }
        
        // Align pointcloud with hit grid
        // TODO: Flip normals that face the other way if normal direction is inconsistent
        if (!align_pointcloud(pcdInliers, Eigen::Vector3d(planeEq_[0], planeEq_[1], planeEq_[2]))){
            return;
        }

        // Process with hit grid and publish result
        hitGrid_.count_hits(pcdInliers);
        double hitRate = hitGrid_.calc_hit_rate();

        msgResult_.header.frame_id = frame_;
        msgResult_.header.stamp = ros::Time::now();
        bool bBreach = (hitRate < 0.999 && hitRate > 0.95); // if the breach is too big, discard because it's likely a false positive
        msgResult_.isBreached = bBreachPrev_ && bBreach; // taken as breached only after 2 consecutive breach detections (for robostness against noise)
        msgResult_.severity = 1.0 - hitRate;
        pubResult_.publish(msgResult_);
        bBreachPrev_ = bBreach;

        // Optional rviz visualization
        hitGrid_.publish_visualization();

        ROS_INFO_STREAM("time: " << ros::Time::now().toSec() - begin.toSec());
    }

    void cb_timer_input(const ros::TimerEvent& msg)
    {
        if (bActive_){
            ROS_WARN_THROTTLE(10.0, "WARNING: No data from LiDar!");
        }
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "open3d");
    
    FenceScan* scan = new FenceScan();

    ros::MultiThreadedSpinner spinner;
    spinner.spin();
}
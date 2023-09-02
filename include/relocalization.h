#include <iostream>
#include <cmath>
#include <math.h>
#include <deque>
#include <mutex>
#include <thread>
#include <fstream>
#include <unistd.h>
#include <condition_variable>
#include <geometry_msgs/PoseStamped.h>

#include <pcl/registration/ndt.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/gicp.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_ros/impl/transforms.hpp>

#include "template_alignment.h"

constexpr float kMAPVOXELSIZE = 0.2;
constexpr float kLOCALMAP = 10.0;
constexpr size_t kNUMBER = 36.0;


class ReLocalization {
public:   
    
    ReLocalization(const std::string& map_path);
    
    ~ReLocalization();

    //! detach thread
    void ReleaseThread(void);

    //! load map
    bool LoadMap(void);

    //! main loop
    void Run(void);

    //! 
    bool RefinePose(const pcl::PointCloud<pcl::PointXYZINormal>::Ptr in,
                    pcl::PointCloud<pcl::PointXYZINormal>::Ptr local_map,
                    Eigen::Matrix4f& T);
    
    bool GicpMatch(const pcl::PointCloud<pcl::PointXYZINormal>::Ptr in,
                   pcl::PointCloud<pcl::PointXYZINormal>::Ptr local_map,
                   Eigen::Matrix4f& T);
    
    bool NDTMatch(const pcl::PointCloud<pcl::PointXYZINormal>::Ptr in,
                  pcl::PointCloud<pcl::PointXYZINormal>::Ptr local_map,
                  Eigen::Matrix4f& T);

    //! relocalization for first match
    bool reLocalization(const pcl::PointCloud<pcl::PointXYZINormal>::Ptr in,
                      pcl::PointCloud<pcl::PointXYZINormal>::Ptr local_map,
                      Eigen::Matrix4f& pose_inv);
    
    //! 
    void Odometry(const pcl::PointCloud<pcl::PointXYZINormal>::Ptr in,
                  Eigen::Matrix4f& pos_predict);

    //! find parts map give initial pose guess
    void FindPartsMapInGlobalMap(const Eigen::Matrix4f& estimation_pose,
                                 pcl::PointCloud<pcl::PointXYZINormal>::Ptr& local_map);

    bool Localization(const pcl::PointCloud<pcl::PointXYZINormal>::Ptr in,
                      pcl::PointCloud<pcl::PointXYZINormal>::Ptr local_map,
                      Eigen::Matrix4f& pos_inv);

    void TestFovMap(const Eigen::Matrix4f& pos_inv, pcl::PointCloud<pcl::PointXYZINormal>::Ptr scan);

    void GetInitialPose(const geometry_msgs::PoseStamped::ConstPtr &goal) {
        initial_pose_(0,3) = goal->pose.position.x;
        initial_pose_(1,3) = goal->pose.position.y;
        initial_pose_(2,3) = goal->pose.position.z;
        has_initial_pose_ = true;
        std::cout << "Get initial pose from rivz =\n " << initial_pose_ << std::endl;
    }

    //! get map
    void GetMap(pcl::PointCloud<pcl::PointXYZINormal>& map) {
        std::unique_lock<std::mutex> lock(mtx_);
        cv.wait(lock, [this] { return map_ready_; }); 
        
        map  = *map_origin_ptr_;

        std::cout << "load origin pointcloud size = " << map.points.size() << std::endl;
    }

    void SetRelocalizationFlag() {
        relocalzation_flag_ = true;
    }

    bool GetRelocalizationFlag() {
        return relocalzation_flag_;
    }

    void SetExit() {
         exit_ = true;
    }


    //! rough match

    //! refine match

    //! fusion pose
    bool has_initial_pose_;
private:
    std::string map_path_;
    std::unique_ptr<std::thread> localization_thread_ptr_;
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr map_origin_ptr_;
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr map_cloud_ptr_;
    Eigen::Matrix4f initial_pose_;

    std::mutex mtx_;               
    std::condition_variable cv;
    bool map_ready_ = false;
    bool relocalzation_flag_;

    pcl::VoxelGrid<pcl::PointXYZINormal> voxel_filter_;

    pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZINormal, pcl::PointXYZINormal> gicp_;
    pcl::IterativeClosestPoint<pcl::PointXYZINormal, pcl::PointXYZINormal> icp_;

    pcl::PointCloud<pcl::PointXYZINormal>::Ptr current_scan;

    bool exit_;

};
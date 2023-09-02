#include "relocalization.h"

ReLocalization::ReLocalization(const std::string& map_path):
                                   map_path_(map_path),
                                   has_initial_pose_(false),
                                   relocalzation_flag_(true),
                                   exit_(false) {
    // start pure locaization map
    localization_thread_ptr_.reset(new std::thread(&ReLocalization::Run, this));
    map_origin_ptr_.reset(new pcl::PointCloud<pcl::PointXYZINormal>);
    map_cloud_ptr_.reset(new pcl::PointCloud<pcl::PointXYZINormal>);
    
    initial_pose_ = Eigen::Matrix4f::Identity();

    // set voxel filter
    voxel_filter_.setLeafSize(0.05, 0.05, 0.05);

    // set gicp param
    gicp_.setMaxCorrespondenceDistance(10);
    gicp_.setMaximumIterations(100);
    gicp_.setMaximumOptimizerIterations(100);
    gicp_.setRANSACIterations(100);
    gicp_.setRANSACOutlierRejectionThreshold(1.0);
    gicp_.setTransformationEpsilon(0.01);
    gicp_.setUseReciprocalCorrespondences(false);

    // set icp param
    icp_.setMaxCorrespondenceDistance (0.05);
    icp_.setMaximumIterations (50);
    icp_.setTransformationEpsilon (1e-8);
    icp_.setEuclideanFitnessEpsilon (0.01);
}
ReLocalization::~ReLocalization() {
    ReleaseThread();
}

void ReLocalization::ReleaseThread(void) {
    if(localization_thread_ptr_ != nullptr && localization_thread_ptr_->joinable()) {
        localization_thread_ptr_->join();
        std::cout << "********** exit pure locaization thread safely *********"<< std::endl;
    }
}

bool ReLocalization::LoadMap(void) {

 std::lock_guard<std::mutex> lock(mtx_);
 map_ready_ = true;

  if (pcl::io::loadPCDFile<pcl::PointXYZINormal> (map_path_, *map_origin_ptr_) == -1){
    std::cout << "### Couldn't read file scans.pcd \n" << std::endl;
    return false;
  }

  //! too werid for if()
  pcl::io::loadPCDFile<pcl::PointXYZINormal> (map_path_, *map_cloud_ptr_);
  std::cout << "The map size = " << map_cloud_ptr_->points.size() << std::endl;
  pcl::VoxelGrid<pcl::PointXYZINormal> pt2_filter;
  pt2_filter.setInputCloud(map_cloud_ptr_);
  pt2_filter.setLeafSize(kMAPVOXELSIZE, kMAPVOXELSIZE, kMAPVOXELSIZE);
  pt2_filter.filter(*map_cloud_ptr_);

  if (map_cloud_ptr_->points.size() == 0)
    return false;

  pcl::VoxelGrid<pcl::PointXYZINormal> pt_filter;
  pt_filter.setInputCloud(map_origin_ptr_);
  pt_filter.setLeafSize(kMAPVOXELSIZE, kMAPVOXELSIZE, kMAPVOXELSIZE);
  pt_filter.filter(*map_origin_ptr_);
  cv.notify_one();
  
  return true;
}

void ReLocalization::TestFovMap(const Eigen::Matrix4f& pos_inv, 
                                  pcl::PointCloud<pcl::PointXYZINormal>::Ptr scan) {
    pcl::transformPointCloud (*scan, *scan, pos_inv);
}

bool ReLocalization::reLocalization(const pcl::PointCloud<pcl::PointXYZINormal>::Ptr in,
                                      pcl::PointCloud<pcl::PointXYZINormal>::Ptr local_map,
                                      Eigen::Matrix4f& pos_inv) {
    Eigen::Matrix4f estimation_pose = pos_inv;
    if (relocalzation_flag_) {
        FindPartsMapInGlobalMap(initial_pose_, local_map);
        std::cout << "*****Relocalization in given initial pose*****"<< std::endl;
    } else {
        return false;
    }

    // // downsample clouds
    current_scan = boost::make_shared<pcl::PointCloud<pcl::PointXYZINormal>>(*in);
	this->voxel_filter_.setInputCloud(current_scan);
	this->voxel_filter_.filter(*current_scan);

    #if 1
        // rough estimation
        Eigen::Matrix4f T_offset = initial_pose_;
        if (!NDTMatch(current_scan, local_map, T_offset)) {
            return false;
        }
        // refine estimation
        estimation_pose =  T_offset;
        
        // FindPartsMapInGlobalMap(estimation_pose, local_map);
        if (!RefinePose(current_scan, local_map, estimation_pose)) {
            std::cout << "WARNING!!! ICP could not refine the pose" << std::endl;
            return false;
        }

        relocalzation_flag_ = false;

        pos_inv =  estimation_pose;
        std::cout << "after convert point size :" << local_map->points.size() << std::endl;
        std::cout << "---------------------------estimation_pose :" << pos_inv(0,3) << ", "<< pos_inv(1,3) << ", "<< pos_inv(2,3)<< std::endl;
    #else
        FeatureCloud target_cloud;
        target_cloud.setInputCloud(current_scan);

        FeatureCloud src_cloud;
        src_cloud.setInputCloud(local_map);
        
        std::vector<FeatureCloud> object_templates;
        object_templates.push_back(src_cloud);

        TemplateAlignment template_align;
        template_align.addTemplateCloud(object_templates[0]);

        template_align.setTargetCloud (target_cloud);

        // Find the best template alignment
        TemplateAlignment::Result best_alignment;
        int best_index = template_align.findBestAlignment (best_alignment);
        const FeatureCloud &best_template = object_templates[best_index];
        printf ("Best fitness score: %f\n", best_alignment.fitness_score);
        Eigen::Matrix3f rotation = best_alignment.final_transformation.block<3,3>(0, 0);
        Eigen::Vector3f translation = best_alignment.final_transformation.block<3,1>(0, 3);
        printf ("\n");
        printf ("    | %6.3f %6.3f %6.3f | \n", rotation (0,0), rotation (0,1), rotation (0,2));
        printf ("R = | %6.3f %6.3f %6.3f | \n", rotation (1,0), rotation (1,1), rotation (1,2));
        printf ("    | %6.3f %6.3f %6.3f | \n", rotation (2,0), rotation (2,1), rotation (2,2));
        printf ("\n");
        printf ("t = < %0.3f, %0.3f, %0.3f >\n", translation (0), translation (1), translation (2));

        pos_inv.block<3,3>(0,0) = rotation;
        pos_inv.block<3,1>(0,3) = translation;
    #endif

    return true;
}

bool ReLocalization::NDTMatch(const pcl::PointCloud<pcl::PointXYZINormal>::Ptr in,
                                 pcl::PointCloud<pcl::PointXYZINormal>::Ptr local_map,
                                 Eigen::Matrix4f& T) {
  // Initializing Normal Distributions Transform (NDT).
  pcl::NormalDistributionsTransform<pcl::PointXYZINormal, pcl::PointXYZINormal> ndt;  
  // Setting scale dependent NDT parameters
  // Setting minimum transformation difference for termination condition.
  ndt.setTransformationEpsilon (0.01);
  // Setting maximum step size for More-Thuente line search.
  ndt.setStepSize (0.1);
  //Setting Resolution of NDT grid structure (VoxelGridCovariance).
  ndt.setResolution (1.0);

  // Setting max number of registration iterations.
  ndt.setMaximumIterations (35);

  // Setting point cloud to be aligned.
  ndt.setInputSource (in);
  // Setting point cloud to be aligned to.
  ndt.setInputTarget (local_map);

  // Calculating required rigid transform to align the input cloud to the target cloud.
  pcl::PointCloud<pcl::PointXYZINormal>::Ptr output_cloud (new pcl::PointCloud<pcl::PointXYZINormal>);
  ndt.align (*output_cloud, T);

  T = ndt.getFinalTransformation ();
  
  std::cout << "Normal Distributions Transform has converged: " << ndt.hasConverged ()
            << " score: " << ndt.getFitnessScore () <<", T \n " << T << std::endl;
  
  return true;
}

bool ReLocalization::Localization(const pcl::PointCloud<pcl::PointXYZINormal>::Ptr in,
                                    pcl::PointCloud<pcl::PointXYZINormal>::Ptr local_map,
                                    Eigen::Matrix4f& pos_inv) {
    Eigen::Matrix4f estimation_pose = pos_inv;
    if (relocalzation_flag_) {
        FindPartsMapInGlobalMap(initial_pose_, local_map);
        std::cout << "*****Relocalization in given initial pose*****"<< std::endl;
    } else {
        FindPartsMapInGlobalMap(estimation_pose, local_map);
    }

    // // downsample clouds
    current_scan = boost::make_shared<pcl::PointCloud<pcl::PointXYZINormal>>(*in);
	this->voxel_filter_.setInputCloud(current_scan);
	this->voxel_filter_.filter(*current_scan);

    // rough estimation
    Eigen::Matrix4f T_offset;
    if (!GicpMatch(current_scan, local_map, T_offset)) {
        return false;
    } else {
        // refine estimation
        if (relocalzation_flag_) {
            estimation_pose =  initial_pose_ * T_offset;
        } else {
            estimation_pose =  estimation_pose * T_offset;
        }
        FindPartsMapInGlobalMap(estimation_pose, local_map);
        if (!RefinePose(current_scan, local_map, T_offset)) {
            std::cout << "WARNING!!! ICP could not refine the pose" << std::endl;
        }
    }

    if (relocalzation_flag_) {
        estimation_pose =  initial_pose_ * T_offset;
        relocalzation_flag_ = false;
        std::cout << "first estimation_pose :" << estimation_pose(0,3) << ", "<< estimation_pose(1,3) << ", "<< estimation_pose(2,3)<< std::endl;
    } else {
        estimation_pose =  estimation_pose * T_offset;
    }
    pos_inv =  estimation_pose;
    std::cout << "after convert point size :" << local_map->points.size() << std::endl;
    std::cout << "---------------------------estimation_pose :" << pos_inv(0,3) << ", "<< pos_inv(1,3) << ", "<< pos_inv(2,3)<< std::endl;

    return true;
}

void ReLocalization::Odometry(const pcl::PointCloud<pcl::PointXYZINormal>::Ptr in,
                                Eigen::Matrix4f& pos_predict) {
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr predict_scan;
    predict_scan.reset(new pcl::PointCloud<pcl::PointXYZINormal>);
    pcl::transformPointCloud (*current_scan, *predict_scan, pos_predict.inverse());                          
    Eigen::Matrix4f T_offset;
    RefinePose(in, predict_scan, T_offset);

    pos_predict = pos_predict * T_offset;

    std::cout << "new pose  "  << pos_predict(0,3) << ", "<< pos_predict(1,3) << ", "<< pos_predict(2,3) << std::endl;
                                    
}



bool ReLocalization::GicpMatch(const pcl::PointCloud<pcl::PointXYZINormal>::Ptr in,
                                 pcl::PointCloud<pcl::PointXYZINormal>::Ptr local_map,
                                 Eigen::Matrix4f& T) {

    gicp_.setInputSource(in);
    gicp_.setInputTarget(local_map);

    pcl::PointCloud<pcl::PointXYZINormal>::Ptr aligned_source = boost::make_shared<pcl::PointCloud<pcl::PointXYZINormal>>();
    gicp_.align(*aligned_source);

    T = gicp_.getFinalTransformation();
    if (gicp_.hasConverged()) {
        std::cout << "GICP converged." << std::endl << "The score is " << gicp_.getFitnessScore() << std::endl;
    } else {
        std::cerr << "###GICP did not converge." << std::endl;
        return false;
    }
    std::cout << "GICP offset:" << T(0,3) << ", "<< T(1,3) << ", "<< T(2,3)<< std::endl;
    return true;
}

bool ReLocalization::RefinePose(const pcl::PointCloud<pcl::PointXYZINormal>::Ptr in,
                                  pcl::PointCloud<pcl::PointXYZINormal>::Ptr local_map,
                                  Eigen::Matrix4f& T) {
    
	icp_.setInputSource(in);      
	icp_.setInputTarget(local_map);

    pcl::PointCloud<pcl::PointXYZINormal> final;
	icp_.align(final, T);

    T = icp_.getFinalTransformation();
    if (icp_.hasConverged()) {
        std::cout << "ICP converged." << std::endl << "The score is " << icp_.getFitnessScore() << std::endl;
    } else {
        std::cerr << "###ICP did not converge." << std::endl;
        return false;
    }
	std::cout << "ICP offset:" << T(0,3) << ", "<< T(1,3) << ", "<< T(2,3)<< std::endl;
    return true;
}


void ReLocalization::FindPartsMapInGlobalMap(const Eigen::Matrix4f& estimation_pose,
                                               pcl::PointCloud<pcl::PointXYZINormal>::Ptr& local_map) {
    pcl::transformPointCloud (*map_cloud_ptr_, *local_map, estimation_pose.inverse());
    // Create the filtering object
    pcl::PassThrough<pcl::PointXYZINormal> pass;
    pass.setInputCloud (local_map);
    pass.setNegative (false);

    pass.setFilterFieldName ("x");
    pass.setFilterLimits (-kLOCALMAP, kLOCALMAP);
    pass.filter (*local_map);

    pass.setFilterFieldName ("y");
    pass.setFilterLimits (-kLOCALMAP, kLOCALMAP);
    pass.filter (*local_map);

    pass.setFilterFieldName ("z");
    pass.setFilterLimits (-kLOCALMAP, kLOCALMAP);
    pass.filter (*local_map);

    pcl::transformPointCloud (*local_map, *local_map, estimation_pose);
    // std::cout << "estimation_pose = \n" << estimation_pose.inverse() <<std::endl;
    std::cout << "Find global map vs local map size: " << map_cloud_ptr_->points.size() << ", "<< local_map->points.size()<< std::endl;

}

void ReLocalization::Run(void) {
    // need test for load map
    if ( !LoadMap()) {
        std::cout << "##########ERROR FOR LOADING MAP#################" << std::endl;
        return;
    }

    //TODO
    while (1) {
        if (exit_) {
            break;
        }
        if (!has_initial_pose_) {
            std::cout << "**** wait for initial pose" << std::endl;
                 usleep(1e6);
            continue;
        }
        usleep(1e6);
        // Relocalization();
        // break;
    }

}

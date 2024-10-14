#define PCL_NO_PRECOMPILE 
#include <limits>
#include <fstream>
#include <vector>
#include <Eigen/Core>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/search/impl/search.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

typedef pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> PCLHandler;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef pcl::PointCloud<pcl::Normal> SurfaceNormals;
typedef pcl::PointCloud<pcl::FPFHSignature33> LocalFeatures;
typedef pcl::search::KdTree<pcl::PointXYZ> SearchMethod;

class FeatureCloud {
public:

    FeatureCloud();
    ~FeatureCloud();

    void setInputCloud(PointCloud::Ptr xyz);
    void loadInputCloud(const std::string& pcd_file);

    PointCloud::Ptr getPointCloud() const {
    return (FeatureCloud::xyz_);
    }

    SurfaceNormals::Ptr getSurfaceNormals() const {
    return (FeatureCloud::normals_);
    }

    LocalFeatures::Ptr getLocalFeatures() const {
    return (FeatureCloud::features_);
    }

protected:

    void processInput();
    void computeSurfaceNormals();
    void computeLocalFeatures();

private:

    PointCloud::Ptr xyz_;
    SurfaceNormals::Ptr normals_;
    LocalFeatures::Ptr features_; // 快速点特征直方图 Fast Point Feature Histogram
    SearchMethod::Ptr search_method_xyz_; // KDTree方法查找邻域

    float normal_radius_;
    float feature_radius_;
};

class TemplateAlignment {
public:

    struct Result {
        float fitness_score;
        Eigen::Matrix4f final_transformation;
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };

    TemplateAlignment();
    ~TemplateAlignment();

    void setTargetCloud(FeatureCloud& target_cloud);
    void addTemplateCloud(FeatureCloud& template_cloud);
    void align(FeatureCloud& template_cloud, TemplateAlignment::Result& result);
    void alignAll(std::vector<TemplateAlignment::Result, Eigen::aligned_allocator<Result> >& results);
    int findBestAlignment(TemplateAlignment::Result& result);

private:

    std::vector<FeatureCloud> templates_;
    FeatureCloud target_;

    pcl::SampleConsensusInitialAlignment<pcl::PointXYZ, pcl::PointXYZ, pcl::FPFHSignature33> sac_ia_;
    float min_sample_distance_;
    float max_correspondence_distance_;
    int nr_iterations_;
};

class PointCloudSubscriber : public rclcpp::Node
{
private:

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr previous_pointcloud_; // 保存上一条接收到的点云消息
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;

    void callback(const sensor_msgs::msg::PointCloud2::SharedPtr pointcloud);

    // std::function<void(const sensor_msgs::msg::PointCloud2::SharedPtr)> callback;
    
public:

    std::shared_ptr<PointCloud> cloud_ptr;
    std::shared_ptr<PointCloud> cloud_final;


    PointCloudSubscriber() : Node("pointcloud_subscriber"), cloud_ptr(std::make_shared<PointCloud>()) 
    {
    pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/camera/depth/color/points", 10, std::bind(&PointCloudSubscriber::callback, this, std::placeholders::_1));
    };



};
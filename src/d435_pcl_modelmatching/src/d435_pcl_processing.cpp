#include "d435.h"

FeatureCloud::FeatureCloud()
    : search_method_xyz_(new SearchMethod),
      normal_radius_(0.02f),
      feature_radius_(0.02f) {
}

FeatureCloud::~FeatureCloud() {
}

// 处理给定的点云
void FeatureCloud::setInputCloud(PointCloud::Ptr xyz) {
    xyz_ = xyz;
    processInput();
}

// 加载并处理给定PCD文件中的点云
void FeatureCloud::loadInputCloud(const std::string& pcd_file) {
    xyz_ = PointCloud::Ptr(new PointCloud);
    pcl::io::loadPCDFile(pcd_file, *xyz_);
    processInput();
}

// 计算表面法线和局部特征
void FeatureCloud::processInput() {
    computeSurfaceNormals();
    computeLocalFeatures();
}

// 计算表面法线
void FeatureCloud::computeSurfaceNormals() {
    // 创建表面法向量
    normals_ = SurfaceNormals::Ptr(new SurfaceNormals);

    // 计算表面法向量
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> norm_est;
    norm_est.setInputCloud(xyz_);
    norm_est.setSearchMethod(search_method_xyz_);
    norm_est.setRadiusSearch(normal_radius_);
    norm_est.compute(*normals_);
}

// 计算局部特征描述子
/**
 * 根据表面法向量 计算本地特征描述
 */
void FeatureCloud::computeLocalFeatures() {
    features_ = LocalFeatures::Ptr(new LocalFeatures);

    pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh_est;
    fpfh_est.setInputCloud(xyz_);
    fpfh_est.setInputNormals(normals_);
    fpfh_est.setSearchMethod(search_method_xyz_);
    fpfh_est.setRadiusSearch(feature_radius_);
    fpfh_est.compute(*features_);
}

TemplateAlignment::TemplateAlignment() :
        min_sample_distance_(0.05f),
        max_correspondence_distance_(0.01f * 0.01f),
        nr_iterations_(500) {
        // 初始化随机采样一致性（SAC-IA）算法中的参数
        sac_ia_.setMinSampleDistance(min_sample_distance_);
        sac_ia_.setMaxCorrespondenceDistance(max_correspondence_distance_);
        sac_ia_.setMaximumIterations(nr_iterations_);
    }

TemplateAlignment::~TemplateAlignment() {}

// 将给定的点云设置为模板将与之对齐的目标
void TemplateAlignment::setTargetCloud(FeatureCloud& target_cloud) {
    target_ = target_cloud;
    // 设置输入target点云
    sac_ia_.setInputTarget(target_cloud.getPointCloud());
    // 设置特征target
    sac_ia_.setTargetFeatures(target_cloud.getLocalFeatures());
}

// 将给定点云添加到模板点云列表中
void TemplateAlignment::addTemplateCloud(FeatureCloud& template_cloud) {
    templates_.push_back(template_cloud);
}

// 将给定的模板点云与setTargetCloud（）指定的目标对齐
void TemplateAlignment::align(FeatureCloud& template_cloud, TemplateAlignment::Result& result) {
    // 设置输入源
    sac_ia_.setInputSource(template_cloud.getPointCloud());
    // 设置特征源
    sac_ia_.setSourceFeatures(template_cloud.getLocalFeatures());

    PointCloud registration_output;
    sac_ia_.align(registration_output);

    // 根据最远相应距离计算匹配分数
    result.fitness_score = (float)sac_ia_.getFitnessScore(max_correspondence_distance_);
    // 获取最终转换矩阵
    result.final_transformation = sac_ia_.getFinalTransformation();
}

// 将addTemplateCloud设置的所有模板点云与setTargetCloud（）指定的目标对齐
void TemplateAlignment::alignAll(std::vector<TemplateAlignment::Result, Eigen::aligned_allocator<Result> >& results) {
    results.resize(templates_.size());
    for (size_t i = 0; i < templates_.size(); ++i) {
        align(templates_[i], results[i]);
    }
}

// 将所有模板点云与目标点云对齐，以找到具有最佳对齐分数的模板云
int TemplateAlignment::findBestAlignment(TemplateAlignment::Result& result) {
    // 将所有模板与目标点云对齐
    std::vector<Result, Eigen::aligned_allocator<Result> > results;
    alignAll(results);

    // 找到最合适的模板
    float lowest_score = std::numeric_limits<float>::infinity();
    int best_template = 0;
    for (size_t i = 0; i < results.size(); ++i) {
        const Result& r = results[i];
        if (r.fitness_score < lowest_score) {
            lowest_score = r.fitness_score;
            best_template = (int)i;
        }
    }
    result = results[best_template];
    return (best_template);
}


void processing(){

    PointCloudSubscriber pcl_sub;
    
    // 加载对象模板
    // auto node = std::make_shared<PointCloudSubscriber>("PointCloudSubscriber");
    std::cout << "start" << std::endl;
    
    std::vector<FeatureCloud> object_templates;
    object_templates.resize(0);
    std::cout << "begin" << std::endl;

    // 加载特征云
    FeatureCloud template_cloud;  //存的是点云文件
    template_cloud.loadInputCloud("/home/classaspen/realsense_ros_4.0.4/src/model/test_model/source.pcd");
    object_templates.push_back(template_cloud);  //在容器末尾添加点云对象到object_templates容器
    std::cout << "load temp_cloud" << std::endl;

    // 移除离群点
    const float depth_limit = 1.0;  //定义了z坐标上限
    pcl::PassThrough<pcl::PointXYZ> pass;  //创建滤波器对象用于后续深度截断
    pass.setInputCloud(pcl_sub.cloud_ptr); //AAA
    pass.setFilterFieldName("z"); //设置深度截断的过滤字段（即深度信息
    pass.setFilterLimits(0, depth_limit);   //截取范围
    pass.filter(*pcl_sub.cloud_ptr); //将滤波后的保存到pcl_sub.cloud_ptr
    pcl::io::savePCDFileBinary("pcl_sub.cloud_ptr.pcd", *pcl_sub.cloud_ptr);
    std::cout << "pass_through_voxel.pcd saved" << std::endl;
    std::cout << "temp_cloud_processing" << std::endl;

    // 降采样点云, 减少计算量
    // 定义体素大小 5mm
    const float voxel_grid_size = 0.005f;
    pcl::VoxelGrid<pcl::PointXYZ> vox_grid;
    vox_grid.setInputCloud(pcl_sub.cloud_ptr); //AAA

    // 设置叶子节点的大小lx, ly, lz
    vox_grid.setLeafSize(voxel_grid_size, voxel_grid_size, voxel_grid_size);
    //vox_grid.filter (*cloud); // Please see this http://www.pcl-developers.org/Possible-problem-in-new-VoxelGrid-implementation-from-PCL-1-5-0-td5490361.html
    PointCloud::Ptr tempCloud(new PointCloud);  //存储滤波后的点云数据
    vox_grid.filter(*tempCloud);
    pcl_sub.cloud_final = tempCloud; //BBB
    std::cout << "存储了滤波后的点云数据" << std::endl;

    // 保存滤波&降采样后的点云图
    pcl::io::savePCDFileBinary("pass_through_voxel.pcd", *tempCloud);
    std::cout << "pass_through_voxel.pcd saved" << std::endl;

    // 对齐到特征点云
    FeatureCloud target_cloud;  //这个存的是处理后的ros2 点云
    target_cloud.setInputCloud(pcl_sub.cloud_final); //AAA
    std::cout << "ros2点云处理完毕" << std::endl;

    // 设置TemplateAlignment输入
    TemplateAlignment template_align;
    for (size_t i = 0; i < object_templates.size(); i++) {
        FeatureCloud& object_template = object_templates[i];
        // 添加模板点云
        template_align.addTemplateCloud(object_template);
    }

    // 设置目标点云
    template_align.setTargetCloud(target_cloud);
    std::cout << "findBestAlignment" << std::endl;
    // 查找最佳模板对齐方式
    // 核心代码
    TemplateAlignment::Result best_alignment;
    int best_index = template_align.findBestAlignment(best_alignment);
    const FeatureCloud& best_template = object_templates[best_index];
    std::cout << "点云对齐已完成" << std::endl;

    // 打印匹配度分数（小于0.00002的值是好的）
    printf("Best fitness score: %f\n", best_alignment.fitness_score);
    printf("Best fitness best_index: %d\n", best_index);

    // 打印旋转矩阵和平移向量
    Eigen::Matrix3f rotation = best_alignment.final_transformation.block<3, 3>(0, 0);
    Eigen::Vector3f translation = best_alignment.final_transformation.block<3, 1>(0, 3);
    Eigen::Vector3f euler_angles = rotation.eulerAngles(2, 1, 0) * 180 / M_PI;
    printf("\n");
    printf("    | %6.3f %6.3f %6.3f | \n", rotation(0, 0), rotation(0, 1), rotation(0, 2));
    printf("R = | %6.3f %6.3f %6.3f | \n", rotation(1, 0), rotation(1, 1), rotation(1, 2));
    printf("    | %6.3f %6.3f %6.3f | \n", rotation(2, 0), rotation(2, 1), rotation(2, 2));
    printf("\n");
    cout << "yaw(z) pitch(y) roll(x) = " << euler_angles.transpose() << endl;
    printf("\n");
    printf("t = < %0.3f, %0.3f, %0.3f >\n", translation(0), translation(1), translation(2));

    // 保存对齐的模板以进行可视化
    PointCloud transformed_cloud;
    // 将模板中保存的点云图进行旋转矩阵变换，把变换结果保存到transformed_cloud
    pcl::transformPointCloud(*best_template.getPointCloud(), transformed_cloud, best_alignment.final_transformation);
    //pcl::io::savePCDFileBinary("output.pcd", transformed_cloud);

    pcl::visualization::PCLVisualizer viewer("example");
    // 设置坐标系系统
    viewer.addCoordinateSystem(0.5, "cloud", 0);
    // 设置背景色
    viewer.setBackgroundColor(0.05, 0.05, 0.05, 0); // Setting background to a dark grey

    // 旋转后的点云rotated --------------------------------
    PointCloud::Ptr t_cloud(&transformed_cloud);
    PCLHandler transformed_cloud_handler(t_cloud, 255, 255, 255);
    viewer.addPointCloud(t_cloud, transformed_cloud_handler, "transformed_cloud");
    // 设置渲染属性（点大小）
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "transformed_cloud");

    // 目标点云target --------------------------------
    PCLHandler target_cloud_handler(pcl_sub.cloud_ptr, 255, 100, 100); //AAA
    viewer.addPointCloud(pcl_sub.cloud_ptr, target_cloud_handler, "target_cloud");
    // 设置渲染属性（点大小）
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "target_cloud");

    // 模板点云template --------------------------------
    PCLHandler template_cloud_handler(pcl_sub.cloud_ptr, 100, 255, 255); //AAA
    viewer.addPointCloud(best_template.getPointCloud(), template_cloud_handler, "template_cloud");
    // 设置渲染属性（点大小）
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "template_cloud");

    while (!viewer.wasStopped()) {
        viewer.spinOnce();
    }

    }



void PointCloudSubscriber::callback(const sensor_msgs::msg::PointCloud2::SharedPtr pointcloud)
{
    // previous_pointcloud_ = pointcloud; // 保存接收到的点云消息

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*pointcloud, *cloud_ptr);

    if (cloud_ptr == nullptr)
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to convert ROS point cloud message to PCL point cloud");
      return;
    }
    else
    {
      std::cout << "Subscriber start, and cloud_ptr is not empty" << std::endl;
    }

    RCLCPP_ERROR(this->get_logger(), "Received point cloud data");
    RCLCPP_INFO(this->get_logger(), "Subscribing: '%d'", cloud_ptr->width);

    processing();
}


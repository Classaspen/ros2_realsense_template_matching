#include "d435.h"

int main(int argc, const char* const* argv) 
{
    rclcpp::init(argc, argv); 

    auto node = std::make_shared<PointCloudSubscriber>();
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}
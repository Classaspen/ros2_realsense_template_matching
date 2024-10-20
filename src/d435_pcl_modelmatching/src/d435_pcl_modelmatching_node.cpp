// #include "d435.h"

// int main(int argc, const char* const* argv) 
// {
//     rclcpp::init(argc, argv); 
//     rclcpp::spin(std::make_shared<PointCloudSubscriber>());
//     rclcpp::shutdown();
//     return (0);
// };

#include "d435.h"

int main(int argc, const char* const* argv) 
{
    rclcpp::init(argc, argv); 

    auto node = std::make_shared<PointCloudSubscriber>();

    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}
#include "d435.h"

int main(int argc, const char* const* argv) 
{
    std::cout<<"22222222222222222222222222222222222222222"<<endl;

    rclcpp::init(argc, argv); 

    auto node = std::make_shared<PointCloudSubscriber>();
    std::cout<<"11111111111111111111111111111111111"<<endl;
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}
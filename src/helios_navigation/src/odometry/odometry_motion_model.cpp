#include "common.hpp"

class OdometryMotionModel : public rclcpp::Node{
public:
    OdometryMotionModel() : Node("robot_wheel_odometry")
    {
        
        init_odometry_params();

    }

private:

    void init_params_all_robot_types(){
        
        init_params_firefighter();
        init_params_scout();
        init_params_heavy_duty();
        init_params_medical();
        init_params_search_and_rescue();
        init_params_environmental_monitoring();
        init_params_drone();
    }

    void init_params_firefighter(){

    }

    void init_params_scout(){

    }

    void init_params_heavy_duty(){

    }

    void init_params_medical(){

    }

    void init_params_search_and_rescue(){

    }

    void init_params_environmental_monitoring(){

    }

    void init_params_drone(){

    }

    /**
     * @brief Odometry properties based on Robot Type
     * @todo Implement switch-case for parameter values based on robot type
     * @details All values are in meters(m)
     */
    void init_odometry_params(){
        this->declare_parameter("wheel_separation", 0.260);
        this->declare_parameter("turning_radius", 0.080);
        this->declare_parameter("robot_radius", 0.205);
    }

   
    








};




int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<OdometryMotionModel>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
#include "rclcpp/rclcpp.hpp"
#include "use_fast_calib2/svd_solver.hpp"
#include <Eigen/Dense>
#include <sstream>

using std::chrono::seconds;

class SvdDemoNode : public rclcpp::Node
{
public:
  SvdDemoNode()
  : Node("svd_demo_node")
  {
    timer_ = this->create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&SvdDemoNode::onTimer, this));
  }

private:
  void onTimer()
  {
    // Example 3x3 matrix
    Eigen::Matrix3d A;
    A << 1.0, 2.0, 3.0,
         4.0, 5.0, 6.0,
         7.0, 8.0, 9.0;

    Eigen::MatrixXd pinv = use_fast_calib2::pseudoInverse(A, 1e-6);

    std::ostringstream ossA;
    ossA << A;
    std::ostringstream ossPinv;
    ossPinv << pinv;

    RCLCPP_INFO(this->get_logger(), "A:\n%s", ossA.str().c_str());
    RCLCPP_INFO(this->get_logger(), "A_pinv:\n%s", ossPinv.str().c_str());
  }

  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SvdDemoNode>());
  rclcpp::shutdown();
  return 0;
}

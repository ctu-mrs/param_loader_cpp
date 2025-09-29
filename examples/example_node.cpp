/* includes //{ */

#include <rclcpp/rclcpp.hpp>
#include <param_loader_cpp/param_loader.hpp>

//}

/* class NodeTest //{ */

class NodeTest : public rclcpp::Node {
public:
  NodeTest(rclcpp::NodeOptions options);

private:
  rclcpp::Node::SharedPtr  node_;
  rclcpp::Clock::SharedPtr clock_;

  rclcpp::TimerBase::SharedPtr timer_preinitialization_;
  void                         timerPreInitialization();

  void initialize();

  /* ros parameters */
  bool            _acc_iir_filter_enabled_;
  Eigen::MatrixXd _acc_iir_filter_a_;
  Eigen::MatrixXd _acc_iir_filter_b_;
  bool            _acc_notch_filter_enabled_;
  double          _acc_notch_filter_sampling_rate_;
  Eigen::MatrixXd _acc_notch_filter_frequencies_;
  double          _acc_notch_filter_bandwidth_;

  bool            _gyro_iir_filter_enabled_;
  Eigen::MatrixXd _gyro_iir_filter_a_;
  Eigen::MatrixXd _gyro_iir_filter_b_;
  bool            _gyro_notch_filter_enabled_;
  double          _gyro_notch_filter_sampling_rate_;
  Eigen::MatrixXd _gyro_notch_filter_frequencies_;
  double          _gyro_notch_filter_bandwidth_;

  bool        _change_frame_id_enabled_ = false;
  std::string _target_frame_id_;
};

//}

/* NodeTest() //{ */

NodeTest::NodeTest(rclcpp::NodeOptions options) : Node("NodeTest", options) {
  timer_preinitialization_ = create_wall_timer(std::chrono::duration<double>(1.0), std::bind(&NodeTest::timerPreInitialization, this));
}

//}

/* timerPreInitialization() //{ */

void NodeTest::timerPreInitialization() {
  node_  = this->shared_from_this();
  clock_ = node_->get_clock();

  initialize();
  timer_preinitialization_->cancel();
}

//}

/* initialize() //{ */

void NodeTest::initialize() {
  RCLCPP_INFO(node_->get_logger(), "Initializing");

  // | ---------- loading ros parameters ---------- |
  RCLCPP_INFO(node_->get_logger(), "loading parameters using ParamLoader");

  param_loader_cpp::ParamLoader param_loader{true};
  param_loader.addYamlFile("/tmp/test.yaml");

  param_loader.loadParam("accelerometer/iir_filter/enable", _acc_iir_filter_enabled_);
  param_loader.loadMatrixDynamic("accelerometer/iir_filter/a", _acc_iir_filter_a_, 1, -1);  // -1 indicates the dynamic dimension
  param_loader.loadMatrixDynamic("accelerometer/iir_filter/b", _acc_iir_filter_b_, 1, -1);
  param_loader.loadParam("accelerometer/notch_filter/enable", _acc_notch_filter_enabled_);
  param_loader.loadParam("accelerometer/notch_filter/sample_rate", _acc_notch_filter_sampling_rate_);
  param_loader.loadMatrixDynamic("accelerometer/notch_filter/frequencies", _acc_notch_filter_frequencies_, 1, -1);
  param_loader.loadParam("accelerometer/notch_filter/bandwidth", _acc_notch_filter_bandwidth_);
  param_loader.loadParam("gyro/iir_filter/enable", _gyro_iir_filter_enabled_);
  param_loader.loadMatrixDynamic("gyro/iir_filter/a", _gyro_iir_filter_a_, 1, -1);  // -1 indicates the dynamic dimension
  param_loader.loadMatrixDynamic("gyro/iir_filter/b", _gyro_iir_filter_b_, 1, -1);
  param_loader.loadParam("gyro/notch_filter/enable", _gyro_notch_filter_enabled_);
  param_loader.loadParam("gyro/notch_filter/sample_rate", _gyro_notch_filter_sampling_rate_);
  param_loader.loadMatrixDynamic("gyro/notch_filter/frequencies", _gyro_notch_filter_frequencies_, 1, -1);
  param_loader.loadParam("gyro/notch_filter/bandwidth", _gyro_notch_filter_bandwidth_);

  param_loader.loadParam("change_frame_id/enabled", _change_frame_id_enabled_);
  
  if (_change_frame_id_enabled_) {
    param_loader.loadParam("change_frame_id/target_frame_id", _target_frame_id_);
  }

  if (!param_loader.loadedSuccessfully()) {
    RCLCPP_ERROR(node_->get_logger(), "parameter loading failure");
    rclcpp::shutdown();
    exit(1);
  }

  RCLCPP_INFO_ONCE(node_->get_logger(), "initialized");
}

//}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<NodeTest>(rclcpp::NodeOptions());
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}

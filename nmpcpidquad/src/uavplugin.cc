#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <thread>

template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

class DronePlugin : public gazebo::ModelPlugin {
public:
  DronePlugin() : ModelPlugin() {
    RCLCPP_INFO(rclcpp::get_logger("drone_plugin"), "Starting drone_plugin");
  }

  void Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf) override {
    this->model = _model;

    // Initialize ROS2 node
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }

    // Create ROS2 node
    node_ = rclcpp::Node::make_shared("drone_plugin_node");

    // Read parameters from SDF
    this->rate = _sdf->HasElement("updateRate") ? _sdf->Get<double>("updateRate") : 400;
    this->publish_tf = _sdf->HasElement("publishTf") ? _sdf->Get<bool>("publishTf") : true;
    this->rotor_thrust_coeff = _sdf->HasElement("rotorThrustCoeff") ? _sdf->Get<double>("rotorThrustCoeff") : 0.00025;
    this->rotor_torque_coeff = _sdf->HasElement("rotorTorqueCoeff") ? _sdf->Get<double>("rotorTorqueCoeff") : 0.000075;

    RCLCPP_INFO(node_->get_logger(), "ROS2 Model Plugin Loaded with parameters!");

    // Subscribe to propeller velocity command topic
    sub_ = node_->create_subscription<std_msgs::msg::Float64MultiArray>("/prop_vel", 10, std::bind(&DronePlugin::ActivateCallback, this, std::placeholders::_1));

    // Start ROS spinning in a separate thread
    ros_thread_ = std::thread([this]() { rclcpp::spin(node_); });

    // Initialize joints
    jointbackleft = _model->GetJoint("pbl");
    jointbackright = _model->GetJoint("pbr");
    jointfrontleft = _model->GetJoint("pfl");
    jointfrontright = _model->GetJoint("pfr");

    // Check if joints are successfully retrieved
    if (!jointbackleft || !jointbackright || !jointfrontleft || !jointfrontright) {
      RCLCPP_ERROR(node_->get_logger(), "Failed to get one or more joints.");
      return;
    }

    updateConnection = gazebo::event::Events::ConnectWorldUpdateBegin(std::bind(&DronePlugin::OnUpdate, this));
  }
  void OnUpdate() {
    if (propvel.size() != 4) {
      RCLCPP_WARN(node_->get_logger(), "Propeller velocities not properly initialized.");
      return;
    }

    // Set velocities to the joints
    jointfrontleft->SetVelocity(0, propvel[1]);
    jointfrontright->SetVelocity(0, propvel[0]);
    jointbackleft->SetVelocity(0, propvel[3]);
    jointbackright->SetVelocity(0, propvel[2]);

    // Calculate thrust and torque for each propeller
    double thrustfl = calculateThrust(propvel[1]);
    double thrustfr = calculateThrust(propvel[0]);
    double thrustbl = calculateThrust(propvel[3]);
    double thrustbr = calculateThrust(propvel[2]);

    double torquefl = calculateTorque(propvel[1]);
    double torquefr = calculateTorque(propvel[0]);
    double torquebl = calculateTorque(propvel[3]);
    double torquebr = calculateTorque(propvel[2]);

    // Apply thrust and torque to each link
    applyForcesAndTorques(thrustfl, thrustfr, thrustbl, thrustbr, torquefl, torquefr, torquebl, torquebr);

  }

  void ActivateCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
    if (msg->data.size() != 4) {
      RCLCPP_WARN(node_->get_logger(), "Received incorrect number of propeller velocities.");
      return;
    }
    propvel = msg->data;
  }

  double calculateThrust(double w) {
    return rotor_thrust_coeff * w * w;
  }

  double calculateTorque(double w) {
    return -sgn(w) * rotor_torque_coeff * w * w;
  }

private:
  rclcpp::Node::SharedPtr node_;  // ROS2 node handle
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_;
  std::thread ros_thread_;

  double rate;
  bool publish_tf;
  double rotor_thrust_coeff;
  double rotor_torque_coeff;

  gazebo::physics::ModelPtr model;
  gazebo::physics::JointPtr jointbackleft;
  gazebo::physics::JointPtr jointbackright;
  gazebo::physics::JointPtr jointfrontleft;
  gazebo::physics::JointPtr jointfrontright;

  gazebo::event::ConnectionPtr updateConnection;
  std::vector<double> propvel = std::vector<double>(4, 0.0);

  void applyForcesAndTorques(double thrustfl, double thrustfr, double thrustbl, double thrustbr,
                             double torquefl, double torquefr, double torquebl, double torquebr) {
    // Get links
    gazebo::physics::LinkPtr linkfl = model->GetLink("propfrontleft");
    gazebo::physics::LinkPtr linkfr = model->GetLink("propfrontright");
    gazebo::physics::LinkPtr linkbl = model->GetLink("propbackleft");
    gazebo::physics::LinkPtr linkbr = model->GetLink("propbackright");
    gazebo::physics::LinkPtr base = model->GetLink("base_link");

    // Apply forces
    if (linkfl) linkfl->AddLinkForce(ignition::math::Vector3d(0, 0, thrustfl));
    if (linkfr) linkfr->AddLinkForce(ignition::math::Vector3d(0, 0, thrustfr));
    if (linkbl) linkbl->AddLinkForce(ignition::math::Vector3d(0, 0, thrustbl));
    if (linkbr) linkbr->AddLinkForce(ignition::math::Vector3d(0, 0, thrustbr));

    // Aggregate torques and apply to base link
    if (base) {
      double total_torque = torquefl + torquefr + torquebl + torquebr;
      base->AddRelativeTorque(ignition::math::Vector3d(0, 0, total_torque));
    }
  }
};

GZ_REGISTER_MODEL_PLUGIN(DronePlugin)

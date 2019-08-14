#include <ros/ros.h>
#include <ros/network.h>
#include "../include/ur_assemble_ui/qnode.hpp"


namespace ur_assemble_ui {

QNode::QNode(int argc, char** argv )
: init_argc(argc), init_argv(argv) {
	actualQ_.resize(6);
	actualX_.resize(6);
	actualForce_.resize(6);
}

QNode::~QNode() {
    if(ros::isStarted()) {
      ros::shutdown(); // explicitly needed since we use ros::start();
      ros::waitForShutdown();
    }
	wait();
}

bool QNode::init() {
	ros::init(init_argc,init_argv,"ur_assemble_ui");
	if ( ! ros::master::check() ) {
		return false;
	}
	ros::start(); // explicitly needed since our nodehandle is going out of scope.
	ros::NodeHandle n("");

	robotStateSubscriber_ = n.subscribe("robot_states", 10, &QNode::robotStateCallback, this);
	targetJointPositionClient_ = n.serviceClient<ur_control_msgs::SetTargetJointPosition>("target_joint_position");
	targetPoseClient_ = n.serviceClient<ur_control_msgs::SetTargetPose>("target_pose");
    setImpedanceClient_ = n.serviceClient<ur_control_msgs::ImpedanceControl>("set_impedance");
    assemblePinClient_ = n.serviceClient<ur_control_msgs::AssemblePin>("assemble_pin");
	commandClient_ = n.serviceClient<ur_control_msgs::SetCommand>("set_command");

	ROS_INFO("qnode : init");

	start();
	return true;
}

void QNode::run() {
	ros::Rate loop_rate(1);
	while ( ros::ok() ) {
		ros::spinOnce();
		loop_rate.sleep();
	}
	Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}

bool QNode::setCommand(uint32_t command) {
	ur_control_msgs::SetCommand service;
	service.request.command = command;
	return commandClient_.call(service);
}

bool QNode::setTargetJointPosition(std::vector<double> jointPosition, double maxVelocity, double acceleration, bool isRelative) {
	ur_control_msgs::SetTargetJointPosition service;
	service.request.joint_position = jointPosition;
	service.request.max_velocity = maxVelocity;
	service.request.acceleration = acceleration;
	service.request.relative = isRelative;
	return targetJointPositionClient_.call(service);
}

bool QNode::setTargetPose(std::vector<double> pose, double maxVelocity, double acceleration, bool isRelative) {
	ur_control_msgs::SetTargetPose service;
	service.request.pose = pose;
	service.request.max_velocity = maxVelocity;
	service.request.acceleration = acceleration;
	service.request.relative = isRelative;
	return targetPoseClient_.call(service);
}

bool QNode::setImpedanceControl(bool enable, std::vector<double> stiffness, std::vector<double> nf, std::vector<double> zeta, std::vector<double> forceLimit) {
	ur_control_msgs::ImpedanceControl service;
	service.request.enable = enable;
	service.request.stiffness = stiffness;
	service.request.nf = nf;
	service.request.zeta = zeta;
	service.request.force_limit = forceLimit;
	return setImpedanceClient_.call(service);
}

bool QNode::setForceBias() {
	setCommand(CS_FORCE_BIAS);
}

bool QNode::setAssemblePin(const std::vector<double> pinPose, const double pinDepth, const std::vector<double> moveVelocity, const std::vector<double> contactVelocity, const std::vector<double> insertVelocity, const std::vector<double> targetForce, const std::vector<double> contactForce, const std::vector<double> forceLimit, const std::vector<double> stiffness, const std::vector<double> zeta) {
	ur_control_msgs::AssemblePin service;
	service.request.pin_pose = pinPose;
	service.request.pin_depth = pinDepth;
	service.request.move_velocity = moveVelocity;
	service.request.contact_velocity = contactVelocity;
	service.request.insert_velocity = insertVelocity;
	service.request.target_force = targetForce;
	service.request.force_limit = forceLimit;
	service.request.contact_force = contactForce;
	service.request.stiffness = stiffness;
	service.request.zeta = zeta;
	return assemblePinClient_.call(service);
}


void QNode::robotStateCallback(const ur_control_msgs::RobotState::ConstPtr &msg) {

	if (msg->actualQ.size() != 0) {
		actualQ_ = msg->actualQ;
	}

	if (msg->actualX.size() != 0) {
		actualX_ = msg->actualX;
	}

	if (msg->actualForce.size() != 0) {
		actualForce_ = msg->actualForce;
	}
}

}  // namespace ur_assemble_ui

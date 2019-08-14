#ifndef ur_assemble_ui_QNODE_HPP_
#define ur_assemble_ui_QNODE_HPP_

#include <ros/ros.h>
#include <string>
#include <QThread>

#include "ur_control_msgs/AssemblePin.h"
#include "ur_control_msgs/ImpedanceControl.h"
#include "ur_control_msgs/SetCommand.h"
#include "ur_control_msgs/SetTargetJointPosition.h"
#include "ur_control_msgs/SetTargetPose.h"
#include "ur_control_msgs/StopRobot.h"
#include "ur_control_msgs/RobotState.h"


// Control Command
const int COMMAND_DEFAULTS = 100;
const int ROBOT_ENABLE = 101;
const int ROBOT_DISABLE = 102;
const int ROBOT_STOP = 103;

const int JTS_BIAS = 104;
const int CD_ON = 105;
const int CD_OFF = 106;
const int FRICTION_OBSERVER = 107;
const int FRICTION_MODEL = 108;

const int JS_HG_ON = 110;
const int JS_HG_OFF = 111;
const int CS_HG_ON = 112;
const int CS_HG_OFF = 113;
const int CS_HG_X_ON = 114;
const int CS_HG_X_OFF = 115;
const int CS_HG_Y_ON = 116;
const int CS_HG_Y_OFF = 117;
const int CS_HG_Z_ON = 118;
const int CS_HG_Z_OFF = 119;

const int JS_FRICTION_OBSERVER = 107;
const int CS_IMPEDANCE_INITIATION = 316;

const int SET_DYN_PARA_GRIPPER = 121;
const int SET_DYN_PARA_POINTER = 122;
const int SET_DYN_PARA_PAYLOAD = 123;

const int JSCTRL_START = 200;
const int JS_TARGET_PATH = 201;

const int CSCTRL_START = 300;
const int CS_TARGET_PATH = 301;
const int CS_CIRCULAR_PATH = 317;

const int CS_IMPEDANCE_CTRL_ON = 310;
const int CS_IMPEDANCE_CTRL_OFF = 311;

const int CS_FORCE_CTRL_ON = 320;
const int CS_FORCE_CTRL_OFF = 321;
const int CS_FORCE_BIAS = 322;

const int DEMO1_START = 400;
const int DEMO1_END = 401;
const int DEMO2_START = 402;
const int DEMO2_END = 403;

const int GRIPPER_OPEN = 500;
const int GRIPPER_CLOSED = 501;

const int SET_TCP = 600;
const int SET_PAYLOAD = 601;
const int SET_SAFETY = 602;

const int COMMAND_ARCBLENDING = 800;
const int COMMAND_ARCBLENDING_ORIENTATION = 801;
const int COMMAND_ARCBLENDING_CHECK = 802;
const int COMMAND_ARCBLENDING_DIFFVEL = 803;

const int ASSEMBLE_TARGET_START = 900;


namespace ur_assemble_ui {

class QNode : public QThread {
    Q_OBJECT
public:
	QNode(int argc, char** argv );
	virtual ~QNode();

Q_SIGNALS:
    void rosShutdown();

public:
	bool init();
	void run();

	std::vector<double> getActualQ() { return actualQ_; }
	std::vector<double> getActualX() { return actualX_; }
	std::vector<double> getActualForce() { return actualForce_; }

	bool setCommand(uint32_t command);
    bool setTargetJointPosition(std::vector<double> jointPosition, double maxVelocity, double acceleration, bool isRelative = false);
    bool setTargetPose(std::vector<double> pose, double maxVelocity, double acceleration, bool isRelative = false);
	bool setImpedanceControl(bool enable, std::vector<double> stiffness, std::vector<double> nf, std::vector<double> zeta, std::vector<double> forceLimit);
	bool setForceBias();
	bool setAssemblePin(const std::vector<double> pinPose, const double pinDepth, const std::vector<double> moveVelocity, const std::vector<double> contactVelocity, const std::vector<double> insertVelocity, const std::vector<double> targetForce, const std::vector<double> contactForce, const std::vector<double> forceLimit, const std::vector<double> stiffness, const std::vector<double> zeta);

private:
	void robotStateCallback(const ur_control_msgs::RobotState::ConstPtr &msg);

private:
	int init_argc;
	char** init_argv;

	ros::Subscriber robotStateSubscriber_;
    ros::ServiceClient targetJointPositionClient_;
    ros::ServiceClient targetPoseClient_;
    ros::ServiceClient stopRobotClient_;
    ros::ServiceClient setImpedanceClient_;
    ros::ServiceClient assemblePinClient_;
	ros::ServiceClient commandClient_;

	std::vector<double> actualQ_;
	std::vector<double> actualX_;
	std::vector<double> actualForce_;
};

}  // namespace ur_assemble_ui

#endif /* ur_assemble_ui_QNODE_HPP_ */

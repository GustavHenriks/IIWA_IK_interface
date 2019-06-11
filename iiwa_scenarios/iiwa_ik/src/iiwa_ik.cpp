/*
 * Copyright (C) 2010 Learning Algorithms and Systems Laboratory, EPFL, Switzerland
 * Author: Eric Sauser
 * email:   eric.sauser@a3.epf.ch
 * website: lasa.epfl.ch
 *
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
 */

#include "iiwa_ik.h"

bool Garve_comp = false;
bool Mod_DS = false;
bool Mod_DS_3D = false;
bool SVM_grad = true;
bool lin_DS = true;
bool target_hand = 1;

iiwa_ik::iiwa_ik()
	: RobotInterface()
{
}
iiwa_ik::~iiwa_ik()
{
}

void iiwa_ik::chatterCallback_Desired_end(const geometry_msgs::Pose &msg)
{
	Desired_EndPos(0) = msg.position.x;
	Desired_EndPos(1) = msg.position.y;
	Desired_EndPos(2) = msg.position.z;
	rotation_temp.w() = msg.orientation.w;
	rotation_temp.x() = msg.orientation.x;
	rotation_temp.y() = msg.orientation.y;
	rotation_temp.z() = msg.orientation.z;
	rot_mat_temp = rotation_temp.toRotationMatrix();
	Desired_EndDirY(0) = rot_mat_temp(0, 1);
	Desired_EndDirZ(0) = rot_mat_temp(0, 2);
	Desired_EndDirY(1) = rot_mat_temp(1, 1);
	Desired_EndDirZ(1) = rot_mat_temp(1, 2);
	Desired_EndDirY(2) = rot_mat_temp(2, 1);
	Desired_EndDirZ(2) = rot_mat_temp(2, 2);
}

void iiwa_ik::chatterCallback_Desired_end_conv(const geometry_msgs::Pose &msg)
{
	Desired_EndPos_conv(0) = msg.position.x;
	Desired_EndPos_conv(1) = msg.position.y;
	Desired_EndPos_conv(2) = msg.position.z;
	Desired_End_orientation(0) = msg.orientation.x;
	Desired_End_orientation(1) = msg.orientation.y;
	Desired_End_orientation(2) = msg.orientation.z;
	Desired_End_orientation(3) = msg.orientation.w;
	Position_of_the_desired_converted_end = true;
}

void iiwa_ik::chatterCallback_end_pos_conv(const geometry_msgs::Pose &msg)
{
	EndPos_conv(0) = msg.position.x;
	EndPos_conv(1) = msg.position.y;
	EndPos_conv(2) = msg.position.z;
}

void iiwa_ik::chatterCallback_base(const geometry_msgs::PoseStamped &msg)
{
	base_pos(0) = -msg.pose.position.x;
	base_pos(1) = -msg.pose.position.y;
	base_pos(2) = msg.pose.position.z;
}

void iiwa_ik::chatterCallback_shoulder(const geometry_msgs::PoseStamped &msg)
{
	Shoulder_pos(0) = -msg.pose.position.x - base_pos(0);
	Shoulder_pos(1) = -msg.pose.position.y - base_pos(1) + 0.18;
	Shoulder_pos(2) = msg.pose.position.z - base_pos(2);
	// cout << "Shoulder" << Shoulder_pos << endl;
}

void iiwa_ik::chatterCallback_hand(const geometry_msgs::PoseStamped &msg)
{
	Hand_pos(0) = -msg.pose.position.x - base_pos(0);
	Hand_pos(1) = -msg.pose.position.y - base_pos(1);
	Hand_pos(2) = msg.pose.position.z - base_pos(2);
}

void iiwa_ik::chatterCallback_position(const sensor_msgs::JointState &msg)
{
	JointPos_handle(0) = msg.position[0];
	JointPos_handle(1) = msg.position[1];
	JointPos_handle(2) = msg.position[2];
	JointPos_handle(3) = msg.position[3];
	JointPos_handle(4) = msg.position[4];
	JointPos_handle(5) = msg.position[5];
	JointPos_handle(6) = msg.position[6];
	// JointPos_handle(0) = 0.2*msg.position[0]+0.8*JointPos_handle(0);
	// JointPos_handle(1) = 0.2*msg.position[1]+0.8*JointPos_handle(1);
	// JointPos_handle(2) = 0.2*msg.position[2]+0.8*JointPos_handle(2);
	// JointPos_handle(3) = 0.2*msg.position[3]+0.8*JointPos_handle(3);
	// JointPos_handle(4) = 0.2*msg.position[4]+0.8*JointPos_handle(4);
	// JointPos_handle(5) = 0.2*msg.position[5]+0.8*JointPos_handle(5);
	// JointPos_handle(6) = 0.2*msg.position[6]+0.8*JointPos_handle(6);

	JointPos = JointPos_handle;
	Position_of_the_robot_recieved = true;
}

void iiwa_ik::ds(Vector2d x, double r_value)
{
	theta = atan2(x(1), x(0));
	r = sqrt(pow(x(0), 2) + pow(x(1), 2));

	theta_dot = theta_value;
	r_dot = -1 * (r - r_value);

	x_dot = r_dot * cos(theta) - r * theta_dot * sin(theta);
	y_dot = r_dot * sin(theta) + r * theta_dot * cos(theta);
	// cout << "theta" << theta << endl;
	// cout << "r" << r << endl;
	// cout << "r_dot" << r_dot << endl;
	// cout << "x_dot" << x_dot << endl;
	v(0) = x_dot;
	v(1) = y_dot;
	new_ds = v;
}
void iiwa_ik::Send_Postion_To_Robot(VectorXd Position)
{

	kuka_fri_bridge::JointStateImpedance msg;
	msg.position.resize(KUKA_DOF);
	msg.stiffness.resize(KUKA_DOF);
	for (int i = 0; i < KUKA_DOF; i = i + 1)
	{
		msg.position[i] = Position(i);
		msg.stiffness[i] = 2000;
	}
	pub_command_robot_real.publish(msg);
}

void iiwa_ik::reset_the_bool()
{

	Position_of_the_robot_recieved = false;
	Position_of_the_desired_converted_end = false;
	svm_activate = true;
}
bool iiwa_ik::everythingisreceived()
{

	return Position_of_the_robot_recieved;
}
void iiwa_ik::Parameter_initialization()
{
	Desired_JointVel.resize(KUKA_DOF);
	Desired_JointVel.setZero();

	cJob.resize(KUKA_DOF);

	EndPos.setZero();
	EndDirY.setZero();
	EndDirZ.setZero();
	Desired_EndDirY.setZero();
	Desired_EndDirZ.setZero();
	Desired_EndPos.setZero();

	// Original Job
	// cJob(0) = 00.0 * PI / 180;
	// cJob(1) = -30 * PI / 180;
	// cJob(2) = -0 * PI / 180;
	// cJob(3) = -60 * PI / 180;
	// cJob(4) = 0;
	// cJob(5) = 90.0 * PI / 180;
	// cJob(6) = 0;

	// C and S Job
	// cJob(0) = 00.0 * PI / 180;
	// cJob(1) = 20 * PI / 180;
	// cJob(2) = 30 * PI / 180;
	// cJob(3) = -45 * PI / 180;
	// cJob(4) = 0;
	// cJob(5) = 45.0 * PI / 180;
	// cJob(6) = 0;

	// N Job
	// cJob(0) = -0.53;
	// cJob(1) = 0.97;
	// cJob(2) = 0.18;
	// cJob(3) = -1.26;
	// cJob(4) = 0.03;
	// cJob(5) = 0.58;
	// cJob(6) = 0.04;

	// 3D Job
	// -0.44084870128580195, 0.9745496791958407, 0.5582655230556134, -0.9795844916846757, -0.535278856848388, 0.12864130667891643, 0.11057550711136438]
	// -0.42624843991648076, 0.9512639706089566, 0.49822191974859337, -0.9943500155537912, 0.18947423897987525, 0.25085559339441793, -0.730038654338395
	// /[-0.6016262463909637, 1.0421462203663066, 0.6491631725074521, -1.2531045313923426, 0.1310644690379921, -0.09092107864263442, -0.8000234717686987]
	// [-0.5851651344990373, 1.1224597020329503, 0.803169270883952, -1.2736288347964264, -0.0387505597946015, -0.41393824414824004, -0.8208081923688973]
	// cJob(0) = -0.655;
	// cJob(1) = 1.222;
	// cJob(2) = 0.803;
	// cJob(3) = -1.174;
	// cJob(4) = 0.039;
	// cJob(5) = -0.414;
	// cJob(6) = -0.820;

	// SVM Job
	// cJob(0) = -15.0 * PI / 180;
	// cJob(1) = 5 * PI / 180;
	// cJob(2) = 0.0;
	// cJob(3) = -45.0 * PI / 180;
	// cJob(4) = 0.00;
	// cJob(5) = 90.0 * PI / 180;
	// cJob(6) = 0.0;

	cJob(0) = -1.0466773637532726;
	cJob(1) = 0.8984651908478877;
	cJob(2) = 1.66122111083375;
	cJob(3) = -1.671668386750558;
	cJob(4) = -0.9563753743412879;
	cJob(5) = 1.033237387257867;
	cJob(6) = 0.5312504435741207;
	char cwd[PATH_MAX];
	if (getcwd(cwd, sizeof(cwd)) != NULL)
	{
		printf("Current working dir: %s\n", cwd);
	}
	else
	{
		perror("getcwd() error");
	}

	if (Mod_DS)
	{
		cout << "initialize LPV" << endl;
		lpv.initialize(3, 2);
		cout << " end of initialization of LPV" << endl;
		lpv.initialize_A("../iiwa_scenarios/iiwa_ik/data/S/A2.txt");
		lpv.initialize_theta("../iiwa_scenarios/iiwa_ik/data/S/priors2.txt", "../iiwa_scenarios/iiwa_ik/data/S/mu2.txt", "../iiwa_scenarios/iiwa_ik/data/S/Sigma2.txt");
		DS_Target(0) = -0.21;
		DS_Target(1) = 0.75;
	}

	if (Mod_DS_3D)
	{
		cout << "initialize LPV" << endl;
		lpv.initialize(4, 3);
		cout << " end of initialization of LPV" << endl;
		lpv.initialize_A("../iiwa_scenarios/iiwa_ik/data/S3/A2.txt");
		lpv.initialize_theta("../iiwa_scenarios/iiwa_ik/data/S3/priors2.txt", "../iiwa_scenarios/iiwa_ik/data/S3/mu2.txt", "../iiwa_scenarios/iiwa_ik/data/S3/Sigma2.txt");
		DS_Target3(0) = -0.68;
		DS_Target3(1) = -0.05;
		DS_Target3(2) = 0.15;
	}

	if (SVM_grad) // svminit
	{
		cout << "Loading SVM model" << endl;
		SVM.loadModel(modelpath);
		Desired_End_orientation.resize(4);
		circle_normal << 0, 0, 1;
		circle_gain = 0.6; //0.1 0.2 (small)
		lin_gain = 0.0001; //0.001 0.0 (small)
		svm_gain = 0.7;	//0.5 0.15 (small)
		theta_value = 0.1;
	}
	reset_the_bool();
}
void iiwa_ik::initKinematics()
{

	mSKinematicChain = new sKinematics(KUKA_DOF, dt);

	mSKinematicChain->setDH(0, 0.0, 0.34, M_PI_2, 0.0, 1, DEG2RAD(-170.), DEG2RAD(170.), DEG2RAD(98.0));
	mSKinematicChain->setDH(1, 0.0, 0.00, -M_PI_2, 0.0, 1, DEG2RAD(-110.), DEG2RAD(110.), DEG2RAD(98.0));
	mSKinematicChain->setDH(2, 0.0, 0.40, -M_PI_2, 0.0, 1, DEG2RAD(-170.), DEG2RAD(170.), DEG2RAD(100.0));
	mSKinematicChain->setDH(3, 0.0, 0.00, M_PI_2, 0.0, 1, DEG2RAD(-110.), DEG2RAD(110.), DEG2RAD(120.0));
	mSKinematicChain->setDH(4, 0.0, 0.39, M_PI_2, 0.0, 1, DEG2RAD(-170.), DEG2RAD(170.), DEG2RAD(140.0));
	mSKinematicChain->setDH(5, 0.0, 0.00, -M_PI_2, 0.0, 1, DEG2RAD(-110.), DEG2RAD(110.), DEG2RAD(180.0));
	mSKinematicChain->setDH(6, 0.0, 0.126, 0.0, 0.0, 1, DEG2RAD(-175.), DEG2RAD(175.), DEG2RAD(180.0));
	//                                 0.126 is the distance between the fifth joint and the real end-effector of the robot
	T0.setZero();

	T0(0, 0) = 1;
	T0(1, 1) = 1;
	T0(2, 2) = 1;
	T0(3, 3) = 1;
	T0(0, 3) = 0;
	T0(1, 3) = 0;
	T0(2, 3) = 0;
	mSKinematicChain->setT0(T0);
	mSKinematicChain->readyForKinematics();

	MatrixXd W(KUKA_DOF, KUKA_DOF);
	W.setZero();

	// variable for ik
	Jacobian3.resize(3, KUKA_DOF);
	Jacobian3_Inve.resize(3, KUKA_DOF);
	JacobianDirY.resize(3, KUKA_DOF);
	JacobianDirZ.resize(3, KUKA_DOF);
	Jacobian9.resize(9, KUKA_DOF);

	mSKinematicChain->setJoints(JointPos.data());
	mSKinematicChain->getEndPos(EndPos);

	MathLib::Vector mJointVelLimitsUp;
	MathLib::Vector mJointVelLimitsDn;
	mJointVelLimitsUp.Resize(KUKA_DOF);
	mJointVelLimitsDn.Resize(KUKA_DOF);
}
void iiwa_ik::Topic_initialization()
{
	mRobot->SetControlMode(Robot::CTRLMODE_POSITION);
	ros::NodeHandle *n = mRobot->InitializeROS();

	sub_position_robot = n->subscribe("/real_r_arm_pos_controller/joint_states", 3, &iiwa_ik::chatterCallback_position, this);
	sub_desired_position_end = n->subscribe("/IIWA/Desired_E_Pos", 3, &iiwa_ik::chatterCallback_Desired_end, this);
	pub_command_robot_real = n->advertise<kuka_fri_bridge::JointStateImpedance>("/real_r_arm_controller/joint_imp_cmd", 3);

	pub_end_of_robot_measured = n->advertise<geometry_msgs::Pose>("/IIWA/Real_E_Pos", 3);

	pub_end_of_robot_converted = n->advertise<geometry_msgs::Pose>("/robot/end/desired", 3);
	pub_gamma = n->advertise<geometry_msgs::Pose>("/gamma/pose", 3);

	sub_desired_position_desired_end_converted = n->subscribe("/robot/end/desired_converted", 3, &iiwa_ik::chatterCallback_Desired_end_conv, this);
	sub_desired_position_end_converted = n->subscribe("/robot/end/measured_converted", 3, &iiwa_ik::chatterCallback_end_pos_conv, this);
	sub_hand = n->subscribe("/Hand/pose", 3, &iiwa_ik::chatterCallback_hand, this);
	sub_shoulder = n->subscribe("/Shoulder/pose", 3, &iiwa_ik::chatterCallback_shoulder, this);
	sub_base = n->subscribe("/Robot_base/pose", 3, &iiwa_ik::chatterCallback_base, this);

	pub_command = n->advertise<std_msgs::Int64>("/command", 3);
}

void iiwa_ik::prepare_sovlve_IK()
{

	mSKinematicChain->getJacobianPos(Jacobian3);

	mSKinematicChain->getJacobianPos(mJacobian3);				   // Get jacobian matrix which contributes to the position of the end-effector
	mSKinematicChain->getJacobianDirection(AXIS_Z, lJacobianDirZ); // Get jacobian matrix which contributes to the orientation of the end-effector around Z axis
	mSKinematicChain->getJacobianDirection(AXIS_Y, lJacobianDirY); // Get jacobian around which contributes to the orientation of the end-effector around Y axis

	for (int i = 0; i < 3; i++)
	{
		mJacobian9.SetRow(mJacobian3.GetRow(i), i);
		mJacobian9.SetRow((lJacobianDirZ.GetRow(i)) * 0.5, i + 3);
		mJacobian9.SetRow((lJacobianDirY.GetRow(i)) * 0.2, i + 6);
	}
}

RobotInterface::Status iiwa_ik::RobotInit()
{
	// This runs only one time, once the robot-toolkid module is started.

	JointPos.resize(KUKA_DOF);
	JointPos.setZero();
	Desired_JointPos.resize(KUKA_DOF);
	Desired_JointPos.setZero();
	JointPos_handle.resize(KUKA_DOF);
	JointPos_handle.setZero();

	// Inverse kinematic solver initialization.
	mEndEffectorId = mRobot->GetLinksCount() - 1;
	mKinematicChain.SetRobot(mRobot);
	mKinematicChain.Create(0, 0, mEndEffectorId);

	mIKSolver.SetSizes(KUKA_DOF);
	mIKSolver.AddSolverItem(IK_CONSTRAINTS);
	mIKSolver.SetVerbose(false);									// No comments
	mIKSolver.SetThresholds(0.0001, 0.00001);						// Singularities thresholds
	mIKSolver.Enable(true, 0);										// Enable first solver
	mIKSolver.SetDofsIndices(mKinematicChain.GetJointMapping(), 0); // Joint maps for first solver
	MathLib::Vector lJointWeight;
	lJointWeight.Resize(KUKA_DOF);

	// All the joints are going to contribute equally.
	lJointWeight(0) = 1;
	lJointWeight(1) = 1;
	lJointWeight(2) = 1;
	lJointWeight(3) = 1.0; //0.5
	lJointWeight(4) = 1.0;
	lJointWeight(5) = 1.0; //0.5
	lJointWeight(6) = 1.0;

	mIKSolver.SetDofsWeights(lJointWeight);

	mTargetVelocity9.Resize(IK_CONSTRAINTS);
	mJacobian9.Resize(9, KUKA_DOF);
	mJacobian3.Resize(3, KUKA_DOF);
	lJacobianDirZ.Resize(3, KUKA_DOF);
	lJacobianDirY.Resize(3, KUKA_DOF);
	mJointDesVel.Resize(KUKA_DOF);
	mJointDesVelFiltered.Resize(KUKA_DOF);
	mJointVelLimitsUp.Resize(KUKA_DOF);
	mJointVelLimitsDn.Resize(KUKA_DOF);

	Topic_initialization();
	Parameter_initialization();

	initKinematics();
	mPlanner = PLANNER_NONE;
	mCommand = COMMAND_NONE;

	flag_job = true;
	AddConsoleCommand("job");  // Move the robot to its initial position which is defined by cJob
	AddConsoleCommand("init"); // Initialize all the parameters, stops the robot and make it ready for polish!
	AddConsoleCommand("task"); // Excuting the task! reads the desired motion of the robot at the task level and convert them to the joint level motion while preparing the robot for contacting the surface.

	srand(time(NULL));

	Initia_time = ros::Time::now().toSec();
	return STATUS_OK;
}
RobotInterface::Status iiwa_ik::RobotFree()
{
	return STATUS_OK;
}
RobotInterface::Status iiwa_ik::RobotStart()
{

	while (!Position_of_the_robot_recieved) //COMMENT FOR TESTING
	{
		cout << "Waiting for the robot's position" << endl;
		ros::spinOnce();
	}

	// If joint values are filtered
	// i=0;
	// ros::Rate rate(100);
	// while (i<200) {
	// 	ros::spinOnce();
	// 	cout << JointPos << endl;
	// 	i++;
	// 	rate.sleep();
	// }

	Desired_JointPos = JointPos;

	mSKinematicChain->setJoints(JointPos.data());
	mSKinematicChain->getEndPos(EndPos);
	mSKinematicChain->getEndDirAxis(AXIS_X, EndDirX);
	mSKinematicChain->getEndDirAxis(AXIS_Y, EndDirY);
	mSKinematicChain->getEndDirAxis(AXIS_Z, EndDirZ);

	MOrientation.block(0, 0, 3, 1) = EndDirX;
	MOrientation.block(0, 1, 3, 1) = EndDirY;
	MOrientation.block(0, 2, 3, 1) = EndDirZ;
	Desired_EndPos = EndPos;
	Desired_EndDirZ = EndDirZ;
	Desired_EndDirY = EndDirY;

	return STATUS_OK;
}
RobotInterface::Status iiwa_ik::RobotStop()
{
	return STATUS_OK;
}
RobotInterface::Status iiwa_ik::RobotUpdate()
{

	ros::spinOnce();

	switch (mCommand)
	{
	case COMMAND_INITIAL:
		mPlanner = PLANNER_NONE;
		if (!flag_init[0])
		{
			cout << "Initialization" << endl;
			ros::spinOnce();
			msg_command.data = COMMAND_INITIAL;
			Parameter_initialization();
			initKinematics();
			ros::spinOnce();
			Desired_JointPos = JointPos;
			flag_init[0] = true;
			reset_the_bool();
		}
		if (everythingisreceived() && !flag_init[1])
		{
			ros::spinOnce();
			Desired_JointPos = JointPos;
			mSKinematicChain->setJoints(JointPos.data());
			mSKinematicChain->getEndPos(EndPos);
			mSKinematicChain->getEndDirAxis(AXIS_X, EndDirX);
			mSKinematicChain->getEndDirAxis(AXIS_Y, EndDirY);
			mSKinematicChain->getEndDirAxis(AXIS_Z, EndDirZ);

			MOrientation.block(0, 0, 3, 1) = EndDirX;
			MOrientation.block(0, 1, 3, 1) = EndDirY;
			MOrientation.block(0, 2, 3, 1) = EndDirZ;
			Desired_EndPos = EndPos;
			Desired_EndDirZ = EndDirZ;
			Desired_EndDirY = EndDirY;
			flag_init[1] = true;
			cout << "Initialization finished" << endl;
		}
		break;
	case COMMAND_JOB:
		mPlanner = PLANNER_JOINT;
		mCommand = COMMAND_NONE;
		msg_command.data = COMMAND_JOB;
		ros::spinOnce();
		Desired_JointPos = JointPos;
		break;
	case COMMAND_Polish:
		if (everythingisreceived())
		// if (!everythingisreceived()) // USED FOR TESTING
		{
			Desired_JointPos = JointPos;
			mSKinematicChain->setJoints(JointPos.data());
			mSKinematicChain->getEndPos(EndPos);
			Desired_EndPos = EndPos;
			mPlanner = PLANNER_CARTESIAN;
			mCommand = COMMAND_NONE;
			msg_command.data = COMMAND_Polish;
		}
		else
		{
			cout << "Position_of_the_robot_recieved " << Position_of_the_robot_recieved << endl;
		}
		break;
	}

	pub_command.publish(msg_command);

	return STATUS_OK;
}
RobotInterface::Status iiwa_ik::RobotUpdateCore()
{

	ros::spinOnce();

	mSKinematicChain->setJoints(JointPos.data());
	mSKinematicChain->getEndPos(EndPos);

	mSKinematicChain->getEndDirAxis(AXIS_X, EndDirX);
	mSKinematicChain->getEndDirAxis(AXIS_Y, EndDirY);
	mSKinematicChain->getEndDirAxis(AXIS_Z, EndDirZ);

	MOrientation.block(0, 0, 3, 1) = EndDirX;
	MOrientation.block(0, 1, 3, 1) = EndDirY;
	MOrientation.block(0, 2, 3, 1) = EndDirZ;

	Orientation = MOrientation;

	switch (mPlanner)
	{
	case PLANNER_CARTESIAN:

		prepare_sovlve_IK();

		if (Mod_DS)
		{
			Old_Pos(0) = EndPos(1) - DS_Target(0);
			Old_Pos(1) = EndPos(2) - DS_Target(1);
			new_A = lpv.Calculate_A(Old_Pos);
			New_Pos = Old_Pos + new_A * Old_Pos / 500;
			Desired_EndPos(1) = New_Pos(0) + DS_Target(0);
			Desired_EndPos(2) = New_Pos(1) + DS_Target(1);
			cout << New_Pos(0) + DS_Target(0) << endl;
			cout << New_Pos(1) + DS_Target(1) << endl;
		}

		if (Mod_DS_3D)
		{
			//   x: -0.735186192686
			//   y: 0.136760180311
			// z: 0.281867040758

			Old_Pos3(0) = EndPos(0) - DS_Target3(0);
			Old_Pos3(1) = EndPos(1) - DS_Target3(1) + 0.01;
			Old_Pos3(2) = EndPos(2) - DS_Target3(2) + 0.12;

			new_A = lpv.Calculate_A(Old_Pos3);
			New_Pos3 = Old_Pos3 + new_A * Old_Pos3 / 500;
			Desired_EndPos(0) = New_Pos3(0) + DS_Target3(0);
			Desired_EndPos(1) = New_Pos3(1) + DS_Target3(1) - 0.01;
			Desired_EndPos(2) = New_Pos3(2) + DS_Target3(2) - 0.12;
			// cout << New_Pos3(0) + DS_Target3(0) << endl;
			// cout << New_Pos3(1) + DS_Target3(1) << endl;
		}

		if (SVM_grad)
		{
			gamma_dist = SVM.calculateGamma(EndPos_conv); //Hand: - 0.12
			gamma_vec = SVM.calculateGammaDerivative(EndPos_conv);
			gamma_pose.position.x = gamma_vec(0);
			gamma_pose.position.y = gamma_vec(1);
			gamma_pose.position.z = gamma_vec(2);
			pub_gamma.publish(gamma_pose);
			// SVM_out = SVM.calculateGammaDerivative(EndPos_conv) / 1000;
			cout << "gamma_dist" << gamma_dist << endl;
			if (gamma_dist > 0.22 | svm_activate) //Lin 0.03+0.095+0.05
			{
				SVM_out = SVM.calculateGammaDerivative(EndPos_conv) / 500;
				Desired_EndPos_tmp = EndPos_conv - SVM_out;
				svm_activate = true;
				if (gamma_dist < 0.20)
				{
					svm_activate = false;
				}
			}
			else
			{

				svm_activate = false;
			}
			svm_activate_neg = false;
			if (gamma_dist < 0.15 | svm_activate_neg) //Lin 0.03+0.095+0.05
			{
				SVM_out = SVM.calculateGammaDerivative(EndPos_conv) / 200;
				Desired_EndPos_tmp = EndPos_conv + SVM_out;
				svm_activate_neg = true;
				if (gamma_dist > 0.17)
				{
					svm_activate_neg = false;
				}
			}
			else
			{
				svm_activate_neg = false;
			}
			Desired_EndPos_tmp_pose.position.x = Desired_EndPos_tmp(0);
			Desired_EndPos_tmp_pose.position.y = Desired_EndPos_tmp(1);
			Desired_EndPos_tmp_pose.position.z = Desired_EndPos_tmp(2);

			pub_end_of_robot_converted.publish(Desired_EndPos_tmp_pose);
			cout << "waiting for desired end converted " << endl;
			while (!Position_of_the_desired_converted_end)
			{
				ros::spinOnce(); 
			}
			Position_of_the_desired_converted_end = false;
			cout << "Curr " << Desired_EndPos[0] << endl;
			cout << "Next " << Desired_EndPos_conv[0] << endl;
			cout << "Gamma " << SVM.calculateGamma(EndPos_conv) << endl;
			if (gamma_dist > 0.22 | svm_activate == true | svm_activate_neg == true) //Lin 0.03+0.095+0.05
			{
				cout << "Too far from surface" << endl;
				cout << gamma_vec << endl;
				last_end(0) = EndPos(0);
				last_end(1) = EndPos(1);
				last_end(2) = EndPos(2);
				last_circle = Desired_EndPos;
				DS_vec = Desired_EndPos;
			}
			if (gamma_dist < 0.22 && gamma_dist > 0.15 && svm_activate == false && svm_activate_neg == false) //Lin 0.03+0.095+0.05 0.01+0.095+0.05
			{
				if (lin_DS) //Use for linear DS only
				{
					if (target_hand)
					{
						target = Hand_pos;
						tmp_vec<<0,0,0.00; //0.05 300, 
						lin_grad_vec = Hand_pos-Shoulder_pos+tmp_vec;
					}
					else
					{
						tmp_vec<<0,0,0.00; //0.05 300
						target = Shoulder_pos; 	
						lin_grad_vec = Shoulder_pos-Hand_pos+tmp_vec;
					}

					///////1D linear line/////////
					// lin_grad = (target(1) - EndPos(1)) / 5;
					// if (lin_grad > 0.002)
					// {

					// 	lin_grad = 0.002;
					// }
					// if (lin_grad < -0.001)
					// {
					// 	lin_grad = -0.001;
					// }

					// cout << "lin_grad" << lin_grad << endl;
					// Desired_EndPos(0) = Desired_EndPos(0);
					// Desired_EndPos(1) = EndPos(1) + lin_grad;

					// Desired_EndPos(2) = Desired_EndPos(2);

					/////////3D linear line///////
					// lin_grad_vec = (target-0.255/gamma_vec.norm()*gamma_vec - EndPos) / 5;
					if (lin_grad_vec.norm() > 0.01)
					{

						lin_grad_vec = (0.01 / lin_grad_vec.norm()) * lin_grad_vec;
					}

					cout << "lin_grad" << lin_grad_vec << endl;
					Desired_EndPos = EndPos + lin_grad_vec; 
					// Desired_EndPos = last_circle;
					/////////////////////////////

					cout << "EndPos" << EndPos << endl;
					cout << "Desired_EndPos " << Desired_EndPos << endl;
					cout << "target " << target<< endl;
					cout << "EndOrientation" << Desired_End_orientation << endl;

					rotation_temp.x() = Desired_End_orientation(0);
					rotation_temp.y() = Desired_End_orientation(1);
					rotation_temp.z() = Desired_End_orientation(2);
					rotation_temp.w() = Desired_End_orientation(3);
					rot_mat_temp = rotation_temp.toRotationMatrix();
					Desired_EndDirY(0) = rot_mat_temp(0, 1);
					Desired_EndDirZ(0) = rot_mat_temp(0, 2);
					Desired_EndDirY(1) = rot_mat_temp(1, 1);
					Desired_EndDirZ(1) = rot_mat_temp(1, 2);
					Desired_EndDirY(2) = rot_mat_temp(2, 1);
					Desired_EndDirZ(2) = rot_mat_temp(2, 2);

					if (abs(EndPos(1) - target(1)) < 0.15) //Hand +0.15
					{
						target_hand = (1 - target_hand);
					}
				}
				// if (lin_DS) // Attempt for circular DS
				// {
				// 	if (target_hand)
				// 	{
				// 		target = Hand_pos;
				// 	}
				// 	else
				// 	{
				// 		target = Shoulder_pos;
				// 	}
				// 	EndPos_conv_svm = EndPos_conv;
				// 	SVM_out2 = SVM.calculateGammaDerivative(EndPos_conv_svm) / 500;
				// 	if (abs(gamma_dist - 0.14) > 0.005)
				// 	{
				// 		svm_sign = (gamma_dist > 0.14) - (gamma_dist < 0.14);
				// 		do
				// 		{
				// 			Desired_EndPos_tmp = EndPos_conv_svm - SVM_out2 * svm_sign;
				// 			SVM_vec = Desired_EndPos_tmp - EndPos_conv;
				// 			EndPos_conv_svm = Desired_EndPos_tmp;
				// 		} while (abs(SVM.calculateGamma(Desired_EndPos_tmp) - 0.14) > 0.005);
				// 	}
				// 	else
				// 	{
				// 		SVM_vec << 0, 0, 0;
				// 	}
				// 	lin_grad3D << (Desired_EndPos(0) - EndPos(0)), (target(1) - EndPos(1)), (Desired_EndPos(2) - EndPos(2));
				// 	// lin_grad3D<<0,(target(1) - EndPos(1)),0;
				// 	// lin_grad3D << last_end(0)-EndPos(0), (target(1) - EndPos(1)), last_end(2)-EndPos(2);
				// 	q.setFromTwoVectors(circle_normal, SVM_out2);
				// 	// cout << "q1 " << q.x() << q.y() << q.z() << q.w() << endl;
				// 	circle_rot = q.normalized().toRotationMatrix();
				// 	// cout << "circle_rot" << circle_rot << endl;
				// 	q.setFromTwoVectors(SVM_out2, circle_normal);
				// 	// cout << "q2" << q2 << endl;
				// 	circle_rot_inv = q.normalized().toRotationMatrix();
				// 	// circle_rot_inv = circle_rot.inverse();
				// 	// cout << "circle_rot_in" << circle_rot_inv << endl;
				// 	// last_end = Desired_EndPos_lin;
				// 	// circle_tmp = EndPos - last_end;
				// 	circle_tmp = last_circle - DS_vec; //Used for small
				// 	// circle_tmp = last_circle-EndPos-lin_grad3D*0.001;
				// 	// circle_tmp = last_circle-EndPos+lin_grad3D*0.01;
				// 	cout << "last_circle " << last_circle << endl;
				// 	// cout << "Next circle " << EndPos + lin_grad3D * 0.01 << endl;
				// 	// cout << "EndPos " << EndPos << " last_end " << last_end << endl;
				// 	cout << "EndPos " << EndPos << "circle_tmp " << circle_tmp << endl;
				// 	circle_2d = circle_rot_inv * circle_tmp.matrix();
				// 	// circle_2d = circle_tmp;
				// 	cout << "circle_2d " << circle_2d << endl;
				// 	circle_2d_2(0) = circle_2d(0);
				// 	circle_2d_2(1) = circle_2d(1);
				// 	// cout << "circle_2d_2" << circle_2d_2 << endl;
				// 	ds(circle_2d_2, 0.04); // Outputs: new_ds, theta and r.
				// 	// cout << "new_ds" << new_ds << endl;
				// 	new_ds_3d << new_ds, 0;
				// 	cout << "new_ds_3d" << new_ds_3d << endl;
				// 	circle_grad = circle_rot * new_ds_3d.matrix();
				// 	// circle_grad = new_ds_3d;
				// 	// cout << "circle_grad" << circle_grad << endl;
				// 	// Desired_EndPos(0) = EndPos(0) + lin_grad3D(0) * lin_gain + circle_grad(0) * circle_gain + SVM_vec(0) * svm_gain;
				// 	// Desired_EndPos(0) = Desired_EndPos(0) + circle_grad(0) * circle_gain;
				// 	// Desired_EndPos(0) = EndPos(0) + circle_grad(0) * circle_gain; // Works for TD circle
				// 	// Desired_EndPos(0) = Desired_EndPos(0);
				// 	// Desired_EndPos(0) = Desired_EndPos(0) + circle_grad(0) * circle_gain + SVM_vec(0) * svm_gain;
				// 	// Desired_EndPos(1) = EndPos(1) + lin_grad3D(1) * lin_gain + circle_grad(1) * circle_gain + SVM_vec(1) * svm_gain;
				// 	// Desired_EndPos(1) = Desired_EndPos(1) + circle_grad(1) * circle_gain; // Works for TD circle
				// 	// Desired_EndPos(1) = EndPos(1) + circle_grad(1) * circle_gain;
				// 	// Desired_EndPos(1) = Desired_EndPos(1);  // Works for TD circle
				// 	// Desired_EndPos(1) = EndPos(1) + (target(1) - EndPos(1)) / 500;
				// 	// Desired_EndPos(2) = EndPos(2) + lin_grad3D(2) * lin_gain + circle_grad(2) * circle_gain + SVM_vec(2) * svm_gain;
				// 	// Desired_EndPos(2) = Desired_EndPos(2);
				// 	Desired_EndPos = EndPos + circle_grad * circle_gain + SVM_vec * svm_gain; //Creates TD circles
				// 	// Desired_EndPos = EndPos + circle_grad * circle_gain + lin_grad*lin_gain + SVM_vec * svm_gain;

				// 	cout << "Lin_ds " << lin_grad3D(0) * lin_gain << " " << lin_grad3D(1) * lin_gain << " " << lin_grad3D(2) * lin_gain << " " << endl;
				// 	cout << "Circle_ds " << circle_grad(0) * circle_gain << " " << circle_grad(1) * circle_gain << " " << circle_grad(2) * circle_gain << endl;
				// 	cout << "SVM " << SVM_vec(0) * svm_gain << " " << SVM_vec(1) * svm_gain << " " << SVM_vec(2) * svm_gain << endl;
				// 	last_circle(0) = EndPos(0) + circle_grad(0) * circle_gain;
				// 	last_circle(1) = EndPos(1) + circle_grad(1) * circle_gain;
				// 	last_circle(2) = EndPos(2) + circle_grad(2) * circle_gain;
				// 	// lambda = (x(2,end)-r.*sin(theta)) /v_circle(2);
				// 	//  if(lambda <0 ),lambda = 0;end
				// 	// if(lambda > 5*dt), lambda = 5*dt;end
				// 	// x_center(1,end+1) = x(1,end)-r.*cos(theta)- v_circle(1)*lambda;

				// 	// last_circle=Desired_EndPos + circle_grad*circle_gain;
				// 	// DS_vec=EndPos-circle_grad*circle_gain+lin_grad3D*lin_gain;
				// 	DS_vec = DS_vec + lin_grad3D * lin_gain; //Creates TD circles
				// 	cout << "DS " << DS_vec << endl;
				// 	// DS_vec=DS_vec;
				// 	// last_circle(0) = Desired_EndPos(0) + circle_grad(0) * circle_gain;
				// 	// last_circle(1) = Desired_EndPos(1) + circle_grad(1) * circle_gain;
				// 	// last_circle(2) = Desired_EndPos(2);

				// 	// last_circle = EndPos;
				// 	cout << "Desired_EndPos " << Desired_EndPos << endl;
				// 	cout << "target " << target << endl;
				// 	rotation_temp.x() = Desired_End_orientation(0);
				// 	rotation_temp.y() = Desired_End_orientation(1);
				// 	rotation_temp.z() = Desired_End_orientation(2);
				// 	rotation_temp.w() = Desired_End_orientation(3);
				// 	rot_mat_temp = rotation_temp.toRotationMatrix();
				// 	Desired_EndDirY(0) = rot_mat_temp(0, 1);
				// 	Desired_EndDirZ(0) = rot_mat_temp(0, 2);
				// 	Desired_EndDirY(1) = rot_mat_temp(1, 1);
				// 	Desired_EndDirZ(1) = rot_mat_temp(1, 2);
				// 	Desired_EndDirY(2) = rot_mat_temp(2, 1);
				// 	Desired_EndDirZ(2) = rot_mat_temp(2, 2);

				// 	if (abs(EndPos(1) - target(1)) < 0.1 | abs(DS_vec(1) - target(1)) < 0.1) //Hand: + 0.15
				// 	{
				// 		target_hand = (1 - target_hand);
				// 	}
				// }
				else
				{
					cout << "In the zone" << endl;
					Desired_EndPos = EndPos;
					Desired_EndDirZ = EndDirZ;
					Desired_EndDirY = EndDirY;
					// In_the_zone=true;
				}
			}
			else
			{
				Desired_EndPos = Desired_EndPos_conv;
				rotation_temp.x() = Desired_End_orientation(0);
				rotation_temp.y() = Desired_End_orientation(1);
				rotation_temp.z() = Desired_End_orientation(2);
				rotation_temp.w() = Desired_End_orientation(3);
				cout << "test" << endl;
				rot_mat_temp = rotation_temp.toRotationMatrix();
				Desired_EndDirY(0) = rot_mat_temp(0, 1);
				Desired_EndDirZ(0) = rot_mat_temp(0, 2);
				Desired_EndDirY(1) = rot_mat_temp(1, 1);
				Desired_EndDirZ(1) = rot_mat_temp(1, 2);
				Desired_EndDirY(2) = rot_mat_temp(2, 1);
				Desired_EndDirZ(2) = rot_mat_temp(2, 2);
				// Desired_EndDirZ = EndDirZ;
				// Desired_EndDirY = EndDirY;
				// In_the_zone=false
			}
		}

		if (Garve_comp)
		{
			mTargetVelocity9(0) = 0.0;
			mTargetVelocity9(1) = 0.0;
			mTargetVelocity9(2) = 0.0;
			mTargetVelocity9(3) = 0.0;
			mTargetVelocity9(4) = 0.0;
			mTargetVelocity9(5) = 0.0;
			mTargetVelocity9(6) = 0.0;
			mTargetVelocity9(7) = 0.0;
			mTargetVelocity9(8) = 0.0;
		}
		else
		{

			// Preparing the Inverse-Kinematic solver
			// cout << "in the loop" << endl;
			// cout << "Desired_EndDirZ" << Desired_EndDirZ << endl;
			// cout << "EndDirZ" << EndDirZ << endl;
			mTargetVelocity9(0) = (Desired_EndPos - EndPos)(0, 0) / (0.1 * dt);
			mTargetVelocity9(1) = (Desired_EndPos - EndPos)(1, 0) / (0.1 * dt);
			mTargetVelocity9(2) = (Desired_EndPos - EndPos)(2, 0) / (0.1 * dt);
			mTargetVelocity9(3) = (Desired_EndDirZ - EndDirZ)(0, 0) / (0.1 * Gain_Orientation);
			mTargetVelocity9(4) = (Desired_EndDirZ - EndDirZ)(1, 0) / (0.1 * Gain_Orientation);
			mTargetVelocity9(5) = (Desired_EndDirZ - EndDirZ)(2, 0) / (0.1 * Gain_Orientation);
			mTargetVelocity9(6) = (Desired_EndDirY - EndDirY)(0, 0) / (0.1 * Gain_Orientation);
			mTargetVelocity9(7) = (Desired_EndDirY - EndDirY)(1, 0) / (0.1 * Gain_Orientation);
			mTargetVelocity9(8) = (Desired_EndDirY - EndDirY)(2, 0) / (0.1 * Gain_Orientation);
		}
		// cout << "mTargetVelocity9 " << mTargetVelocity9 << endl;

		// Preparing the Inverse-Kinematic solver, setting up the joint limits
		for (int i = 0; i < KUKA_DOF; i++)
		{
			double deltaLow = JointPos(i) + 0.9 * mSKinematicChain->getMax(i);
			double deltaHigh = 0.9 * mSKinematicChain->getMax(i) - JointPos(i);
			mJointVelLimitsDn(i) = -Gain_velocity_limit * mSKinematicChain->getMaxVel(i);
			mJointVelLimitsUp(i) = Gain_velocity_limit * mSKinematicChain->getMaxVel(i);
			if (deltaLow < 0.0)
				mJointVelLimitsDn(i) *= 0.0;
			else if (deltaLow < DEG2RAD(5.0))
				mJointVelLimitsDn(i) *= deltaLow / DEG2RAD(5.0);
			if (deltaHigh < 0.0)
				mJointVelLimitsUp(i) *= 0.0;
			else if (deltaHigh < DEG2RAD(5.0))
				mJointVelLimitsUp(i) *= deltaHigh / DEG2RAD(5.0);
		}

		mIKSolver.SetLimits(mJointVelLimitsDn, mJointVelLimitsUp);
		// Preparing the Inverse-Kinematic solver
		mIKSolver.SetJacobian(mJacobian9);
		mIKSolver.SetTarget(mTargetVelocity9, 0);
		// Solve the Inverse-Kinematic problem
		mIKSolver.Solve();
		// Get the joint space results, It is at the velocity level!
		mJointDesVel = mIKSolver.GetOutput();

		for (int i = 0; i < KUKA_DOF; i++)
		{
			// Desired_JointVel(i) = mJointDesVel(i);

			// mJointDesVelFiltered(i) = 0.8 * mJointDesVelFiltered(i) + 0.2* mJointDesVel(i);
			input = mJointDesVel(i);
			// input=mJointDesVelFiltered(i);
			if (input > 0)
			{
				input = input - disturbance;
				input = (input < 0) ? 0 : input;
				// input = (input > threshold) ? threshold : input;
			}
			else if (input < 0)
			{
				input = input + disturbance;
				input = (input > 0) ? 0 : input;
				// input = (input < -threshold) ? -threshold : input;
			}
			Desired_JointVel(i) = input;
		}

		// Desired_JointPos = JointPos + Desired_JointVel * dt * 0.5;
		// cout << "JointVel " << Desired_JointVel << endl;
		Desired_JointPos = 0.85 * Desired_JointPos + 0.15 * (JointPos + Desired_JointVel * dt * 0.4);
		break;
	case PLANNER_JOINT:

		Desired_JointPos = JointPos + 0.01 * (1 / dt) * (cJob - JointPos) * dt;
		Desired_EndPos = EndPos;

		break;
	}

	msg_robot_end.position.x = EndPos(0);
	msg_robot_end.position.y = EndPos(1);
	msg_robot_end.position.z = EndPos(2);
	msg_robot_end.orientation.x = Orientation.x();
	msg_robot_end.orientation.y = Orientation.y();
	msg_robot_end.orientation.z = Orientation.z();
	msg_robot_end.orientation.w = Orientation.w();

	pub_end_of_robot_measured.publish(msg_robot_end);

	Send_Postion_To_Robot(Desired_JointPos);

	return STATUS_OK;
}
int iiwa_ik::RespondToConsoleCommand(const string cmd, const vector<string> &args)
{

	if (cmd == "init")
	{
		if (!flag_job)
		{
			mPlanner = PLANNER_NONE;
			mCommand = COMMAND_INITIAL;
			flag_init[2] = true;
			flag_init[0] = false;
			flag_init[1] = false;
		}
	}
	else if (cmd == "job")
	{
		mCommand = COMMAND_JOB;
		mPlanner = PLANNER_NONE;
		flag_job = false;
		flag_init[2] = false;
	}
	else if (cmd == "task")
	{
		if ((!flag_job) && (flag_init[2]))
		{
			cout << "task!" << endl;
			mCommand = COMMAND_Polish;
			mPlanner = PLANNER_NONE;
			flag_job = true;
			flag_init[2] = false;
		}
	}

	return 0;
}

extern "C"
{
	// These two "C" functions manage the creation and destruction of the class
	iiwa_ik *create() { return new iiwa_ik(); }
	void destroy(iiwa_ik *module) { delete module; }
}
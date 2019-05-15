#include "motion.h"
motion::motion()
{
}
void motion::chatterCallback_base(const geometry_msgs::PoseStamped &msg)
{
	base_pos(0) = -msg.pose.position.x;
	base_pos(1) = -msg.pose.position.y;
	base_pos(2) = msg.pose.position.z;
	Base_received = true;
}

void motion::chatterCallback_hand(const geometry_msgs::PoseStamped &msg)
{
	Hand_pos(0) = -msg.pose.position.x - base_pos(0);
	Hand_pos(1) = -msg.pose.position.y - base_pos(1);
	Hand_pos(2) = msg.pose.position.z - base_pos(2);
	Hand_received = true;
}

void motion::chatterCallback_end(const geometry_msgs::Pose &msg)
{
	end_pos(0) = 0.9*end_pos(0)+0.1*msg.position.x;
	end_pos(1) = 0.9*end_pos(1)+0.1*msg.position.y;
	end_pos(2) = 0.9*end_pos(2)+0.1*msg.position.z;
	end_orientation(0) = 0.9*end_pos(0)+0.1*msg.orientation.x;
	end_orientation(1) = 0.9*end_pos(0)+0.1*msg.orientation.y;
	end_orientation(2) = 0.9*end_pos(0)+0.1*msg.orientation.z;
	end_orientation(3) = 0.9*end_pos(0)+0.1*msg.orientation.w;
	End_received = true;
}

void motion::chatterCallback_position(const sensor_msgs::JointState &msg)
{
	JointPos_handle(0) = msg.position[0];
	JointPos_handle(1) = msg.position[1];
	JointPos_handle(2) = msg.position[2];
	JointPos_handle(3) = msg.position[3];
	JointPos_handle(4) = msg.position[4];
	JointPos_handle(5) = msg.position[5];
	JointPos_handle(6) = msg.position[6];
	joint_pose.position.x=JointPos_handle(0);
	joint_pose.position.y=JointPos_handle(1);
	joint_pose.position.z=JointPos_handle(2);
	joint_pose.orientation.x=JointPos_handle(3);
	joint_pose.orientation.y=JointPos_handle(4);
	joint_pose.orientation.z=JointPos_handle(5);
	joint_pose.orientation.w=JointPos_handle(6);
}

void motion::Topic_initialization()
{
	ros::NodeHandle n;
	basesub = n.subscribe("/Robot_base/pose", 3, &motion::chatterCallback_base, this);
	handsub = n.subscribe("/Hand/pose", 3, &motion::chatterCallback_hand, this);
	endsub = n.subscribe("/IIWA/Real_E_Pos", 3, &motion::chatterCallback_end, this);
	sub_position_robot = n.subscribe("/real_r_arm_controller/joint_imp_cmd", 3, &motion::chatterCallback_position, this);
	joint_pose_pub = n.advertise<geometry_msgs::Pose>("/robot/states", 3);
	desiredpub = n.advertise<geometry_msgs::Pose>("/IIWA/Desired_E_Pos", 3);
}

void motion::Init_parameters()
{
	Hand_received = false;
	Base_received = false;
	End_received = false;
	end_orientation.resize(4);
	JointPos_handle.resize(7);
	gain=0.0001;
}

int motion::main(int argc, char **argv)
{

	ros::init(argc, argv, "motion");
	// ros::Subscriber handsub = n.subscribe("/Hand/pose", 3, chatterCallback_base);
	// ros::Subscriber basesub = n.subscribe("/Hand/pose", 3, chatterCallback_hand);
	cout << "test" << endl;
	Init_parameters();
	Topic_initialization();

	while (Hand_received != true && Base_received != true && End_received != true)
	{
		ros::spinOnce();
		cout << Hand_received << Base_received << End_received << endl;
	}
	cout << (Hand_received != true && Base_received != true && End_received != true) << endl;
	cout << "test" << endl;
	cout << Hand_pos << endl;
	cout << base_pos << endl;
	cout << end_pos << endl;
	cout << end_orientation << endl;
	desired_pos = end_pos;
	// desired_pose.position.x = -0.619281291409;
	// desired_pose.position.y = -0.0315954633816;
	// desired_pose.position.z = 0.501841220954;
	// desired_pose.orientation.x = 0.0736833406431;
	// desired_pose.orientation.y = 0.966738460205;
	// desired_pose.orientation.z = -0.166608603561;
	// desired_pose.orientation.w = 0.179524616949;

	// desiredpub.publish(desired_pose);
	// cout << "end pos" << desired_pose.position.x << endl;
	ros::Rate rate(500);
	while (ros::ok())
	{
		cout << "Desired " << desired_pose.position.z << endl;
		desired_pose.position.x = end_pos(0) + (-0.619281291409-end_pos(0))*gain;
		desired_pose.position.y = end_pos(1) + (0.0797833063432-end_pos(1))*gain;
		desired_pose.position.z = end_pos(2) + (0.501841220954-end_pos(2))*gain;
		// desired_pose.orientation.x = end_orientation(0);
		// desired_pose.orientation.y = end_orientation(1);
		// desired_pose.orientation.z = end_orientation(2);
		// desired_pose.orientation.w = end_orientation(3);
		desired_pose.orientation.x = 0.0736833406431;
		desired_pose.orientation.y = 0.966738460205;
		desired_pose.orientation.z = -0.166608603561;
		desired_pose.orientation.w = 0.179524616949;
		desiredpub.publish(desired_pose);
		ros::spinOnce();
		rate.sleep();
		joint_pose_pub.publish(joint_pose);
	}

	return 0;
}
int main(int argc, char **argv)
{
	motion m;
	m.main(argc, argv);
}
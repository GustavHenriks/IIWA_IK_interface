#include "motion.h"

void chatterCallback_base(const geometry_msgs::PoseStamped &msg)
{
	base_pos(0) = -msg.pose.position.x;
	base_pos(1) = -msg.pose.position.y;
	base_pos(2) = msg.pose.position.z;
	Base_received = true;
}

void chatterCallback_hand(const geometry_msgs::PoseStamped &msg)
{
	Hand_pos(0) = -msg.pose.position.x - base_pos(0);
	Hand_pos(1) = -msg.pose.position.y - base_pos(1);
	Hand_pos(2) = msg.pose.position.z - base_pos(2);
	Hand_received = true;
}

void chatterCallback_end(const geometry_msgs::Pose &msg)
{
	end_pos(0) = msg.position.x;
	end_pos(1) = msg.position.y;
	end_pos(2) = msg.position.z;
	end_orientation(0) = msg.orientation.x;
	end_orientation(1) = msg.orientation.y;
	end_orientation(2) = msg.orientation.z;
	end_orientation(3) = msg.orientation.w;
	End_received = true;
}

void Topic_initialization()
{
	// ros::NodeHandle n;
	// ros::Subscriber handsub = n.subscribe("/Hand/pose", 3, chatterCallback_base);
	// ros::Subscriber basesub = n.subscribe("/Hand/pose", 3, chatterCallback_hand);
}

void Init_parameters()
{
	Hand_received = false;
	Base_received = false;
	End_received = false;
}

int main(int argc, char **argv)
{

	ros::init(argc, argv, "motion");
	// ros::Subscriber handsub = n.subscribe("/Hand/pose", 3, chatterCallback_base);
	// ros::Subscriber basesub = n.subscribe("/Hand/pose", 3, chatterCallback_hand);
	cout << "test" << endl;
	Init_parameters();
	ros::NodeHandle n;
	ros::Subscriber handsub = n.subscribe("/Robot_base/pose", 3, chatterCallback_base);
	ros::Subscriber basesub = n.subscribe("/Hand/pose", 3, chatterCallback_hand);
	ros::Subscriber endsub = n.subscribe("/IIWA/Real_E_Pos", 3, chatterCallback_end);
	ros::Publisher desiredpub = n.advertise<geometry_msgs::Pose>("/IIWA/Desired_E_Pos", 3);

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
	desired_pose.position.x = end_pos(0) - 0.4;
	desired_pose.position.y = end_pos(1);
	desired_pose.position.z = end_pos(2);
	desiredpub.publish(desired_pose);
	while (end_pos(0) < -0.4)
	{
		desired_pose.position.x = end_pos(0) + 0.1;
		desired_pose.position.y = end_pos(1);
		desired_pose.position.z = end_pos(2);
		desiredpub.publish(desired_pose);
		ros::spinOnce();
	}
	// ros::spin();
	// %EndTag(SUBSCRIBER)%

	/**
   * ros::spin() will enter a loop, pumping callbacks.  With this version, all
   * callbacks will be called from within this thread (the main one).  ros::spin()
   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
   */
	// %Tag(SPIN)%

	// %EndTag(SPIN)%

	return 0;
}
// %EndTag(FULLTEXT)%
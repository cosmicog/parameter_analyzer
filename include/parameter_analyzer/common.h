#ifndef COMMON_H
#define COMMON_H
 
#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <actionlib/client/simple_action_client.h>
#include <boost/graph/graph_concepts.hpp>
#include <dynamic_reconfigure/server.h>
#include <parameter_analyzer/analyzerConfig.h>
#include <urdf_parser/urdf_parser.h>
#include <tf/tf.h>

#define REACHED 3

#define GOING 1
#define COMING 0
#define ALLDONE 2

//////////////////////////////////////////////////////////////////////////////////

double A_POS_X, A_POS_Y, A_ORI_Z, A_ORI_W;
double G_POS_X, G_POS_Y, G_ORI_Z, G_ORI_W, G_ANG;
double PREV_G_POS_X, PREV_G_POS_Y, PREV_G_ORI_Z, PREV_G_ORI_W, PREV_G_ANG;
double POS_DIFF_X, POS_DIFF_Y, ORI_DIFF_Z, ORI_DIFF_W, ANG_DIFF;

double ERROR_PENALTY_X = .0;
double ERROR_PENALTY_Y = .0;
double ERROR_PENALTY_A = .0;
double ERROR_PENALTY = .0;

double CUR_ERR_X = .0;
double CUR_ERR_Y = .0;
double CUR_ERR_A = .0;

double JOURNEY_TIME;
double JOURNEY_LENGTH = .0;
double PATH_DIST_MEAN = .0;

double GOAL_POS_X;
double GOAL_POS_Y;

double GOAL_ORI_Z;
double GOAL_ORI_W;

double SEP_RADIUS;

double PARAM_END;

double INC = 0.0;
int TRY_COUNT;

int GOAL_STATUS = 0;
int PREV_STATUS = 0;
int GOAL_COUNT = 1;

int JOB_STATUS = GOING;

bool BIG_BANG = true;
bool ALL_DONE = false;
bool PATH_DIST_CALCULATED = false;

ros::Time GOAL_PUB_TIME;

geometry_msgs::PoseWithCovarianceStamped init_msg;
geometry_msgs::PoseStamped goal_msg;

ros::Publisher init_pub;
ros::Publisher goal_pub;

////////////////////////////////////////////////////////////////////////////////////////////////

void printAllInfo();
void updateParams();
void printParamInfo();
void updateErrorPenalty();
void groundTruthCB ( const nav_msgs::OdometryConstPtr &msg );
void amclPoseCB ( const geometry_msgs::PoseWithCovarianceStampedConstPtr &msg );
void goalCB( const actionlib_msgs::GoalStatusArrayConstPtr & msg );
void pathCB( const nav_msgs::PathConstPtr &msg );
void dynamicReconfigureCallback(parameter_analyzer::analyzerConfig &config, uint32_t level);

bool publishSimpleGoal( double px, double py, double oz, double ow );
bool isEqual( double x, double y, double allowed_abs_diff );
bool publishToInitialPose( double px, double py, double oz, double ow );

////////////////////////////////////////////////////////////////////////////////////////////////
 

bool isEqual(double x, double y, double allowed_abs_diff = 0.0001)
{
	double diff = x - y;
	if ( diff > (-1 * allowed_abs_diff) && diff < allowed_abs_diff ) return true;
	return false;
}

double getSeparationRadius()
{
    std::string robot_model_str="";
    if (!ros::param::get("robot_description", robot_model_str))
    {
		ROS_ERROR("Robot descripion couldn't be retrieved from param server.");
		return false;
    }

    boost::shared_ptr<urdf::ModelInterface> model(urdf::parseURDF(robot_model_str));
    boost::shared_ptr<const urdf::Joint> left_wheel_joint(model->getJoint("wheel_left_joint"));
    boost::shared_ptr<const urdf::Joint> right_wheel_joint(model->getJoint("wheel_left_joint"));

	// Get wheel separation
	if (!left_wheel_joint)
	{
	ROS_ERROR(" couldn't be retrieved from model description");
	return false;
	}

	if (!right_wheel_joint)
	{
	ROS_ERROR(" couldn't be retrieved from model description");
	return false;
	}

	return left_wheel_joint->parent_to_joint_origin_transform.position.y;
}

void updateErrorPenalty()
{	
	CUR_ERR_X = fabs(G_POS_X - A_POS_X);
	CUR_ERR_Y = fabs(G_POS_Y - A_POS_Y);
	CUR_ERR_A = fabs (tf::Quaternion(0.0, 0.0, G_ORI_Z, G_ORI_W).getAngle() - tf::Quaternion(0.0, 0.0, A_ORI_Z, A_ORI_W).getAngle());
	
	ERROR_PENALTY_X += CUR_ERR_X;
	ERROR_PENALTY_Y += CUR_ERR_Y;
	ERROR_PENALTY_A += CUR_ERR_A;
	
	ERROR_PENALTY += (ERROR_PENALTY_X + ERROR_PENALTY_Y + ERROR_PENALTY_A);
	
	std::cout << PATH_DIST_MEAN << "\t" << JOURNEY_LENGTH << "\t" <<JOURNEY_TIME << "\t" << CUR_ERR_X << "\t" << CUR_ERR_Y << "\t" << CUR_ERR_A << std::endl;
	//std::cout << "REACH: " << GOAL_COUNT << ", ERROR: " << ERROR_PENALTY << "" << std::endl;
	//ROS_WARN("REACH: %d, ERROR: %lf", GOAL_COUNT, ERROR_PENALTY);
	//ROS_WARN("CURRENT ERRORS ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::  X: %lf, Y: %lf, A: %lf", CUR_ERR_X, CUR_ERR_Y, CUR_ERR_A);
	
	GOAL_COUNT ++;
	if (GOAL_COUNT > TRY_COUNT)
	{
		GOAL_COUNT = 1;
		std::cout << "\nTOTAL ERROR FOR THIS LOOP:\n" << ERROR_PENALTY << std::endl << std::endl;
		ERROR_PENALTY = 0.0;
		ERROR_PENALTY_X = 0.0;
		ERROR_PENALTY_Y = 0.0;
		ERROR_PENALTY_A = 0.0;
		publishToInitialPose(G_POS_X, G_POS_Y, G_ORI_Z, G_ORI_W);
		updateParams();
	}
}

void goalCB(const actionlib_msgs::GoalStatusArrayConstPtr & msg)
{
	actionlib_msgs::GoalStatus val0;
	if (!msg->status_list.empty())
	{
		val0 = msg->status_list[0];
		GOAL_STATUS = val0.status;
		if (ALL_DONE) GOAL_STATUS = ALLDONE;
		if (GOAL_STATUS == REACHED && PREV_STATUS != REACHED)
		{
			JOURNEY_TIME = ros::Time::now().toSec() - GOAL_PUB_TIME.toSec();
			updateErrorPenalty();
			JOURNEY_LENGTH = 0.0;
			PATH_DIST_CALCULATED = false;
			PATH_DIST_MEAN = 0.0;
			if (JOB_STATUS == GOING)
			{
				JOB_STATUS = COMING;
				publishSimpleGoal(0.0, 0.0, 0.0, 1.0);
			}
			else if (JOB_STATUS == COMING)
			{
				JOB_STATUS = GOING;
				publishSimpleGoal(4.41599369, -0.5125627, -0.0117687, 0.9999307);
			}
			else
			{
				ROS_INFO("DID ROBOT QUIT HIS JOB?");
			}
		}
		PREV_STATUS = GOAL_STATUS;
	}
}

bool publishSimpleGoal(double px, double py, double oz, double ow)
{
	GOAL_PUB_TIME = ros::Time::now();
	goal_msg.header.stamp = GOAL_PUB_TIME;
	goal_msg.header.frame_id = "map";
	goal_msg.pose.position.x = px;  //4.41599369;
	goal_msg.pose.position.y = py;  //-0.5125627;
	goal_msg.pose.position.z = 0.0;
	goal_msg.pose.orientation.x = 0.0;
	goal_msg.pose.orientation.y = 0.0;
	goal_msg.pose.orientation.z = oz;  //-0.0117687;
	goal_msg.pose.orientation.w = ow;  //0.9999307;
	
	goal_pub.publish( goal_msg );
	
	if (goal_pub)
	{
		//ROS_INFO("to %s ->DONE", goal_pub.getTopic().c_str());
		return true;
	}
	else
	{		
		ROS_ERROR("Failed to publish message to %s topic", goal_pub.getTopic().c_str());
		return false;
	}
}

bool publishToInitialPose(double px, double py, double oz, double ow)
{
	init_msg.header.stamp = ros::Time::now();
	init_msg.header.frame_id = "map";
	init_msg.pose.pose.position.x = px;  //4.41599369;
	init_msg.pose.pose.position.y = py;  //-0.5125627;
	init_msg.pose.pose.position.z = 0.0;
	init_msg.pose.pose.orientation.x = 0.0;
	init_msg.pose.pose.orientation.y = 0.0;
	init_msg.pose.pose.orientation.z = oz;  //-0.0117687;
	init_msg.pose.pose.orientation.w = ow;  //0.9999307;
	init_msg.pose.covariance[0] = 0.25; // X
	init_msg.pose.covariance[7] = 0.25; // Y
	init_msg.pose.covariance[35] = 0.05; // Radius
	
	init_pub.publish( init_msg );
	
	if (init_pub)
	{
		//ROS_INFO_ONCE( "To %s -> DONE.", init_pub.getTopic().c_str() );
		return true;
	}
	else
	{		
		ROS_ERROR( "Failed to publish message to %s topic", init_pub.getTopic().c_str() );
		return false;
	}
}

void groundTruthCB ( const nav_msgs::OdometryConstPtr &msg ) // const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg
{
	
	PREV_G_POS_X = G_POS_X;
	PREV_G_POS_Y = G_POS_Y;
	PREV_G_ORI_Z = G_ORI_Z;
	PREV_G_ORI_W = G_ORI_W;
	PREV_G_ANG = G_ANG;
	
	G_POS_X = msg->pose.pose.position.x;
	G_POS_Y = msg->pose.pose.position.y;
	G_ORI_Z = msg->pose.pose.orientation.z;
	G_ORI_W = msg->pose.pose.orientation.w;
	G_ANG = tf::Quaternion(0.0, 0.0, G_ORI_Z, G_ORI_W).getAngle();
	
	
	POS_DIFF_X = fabs (PREV_G_POS_X - G_POS_X);
	POS_DIFF_Y = fabs (PREV_G_POS_Y - G_POS_Y);
	ORI_DIFF_Z = fabs (PREV_G_ORI_Z - G_ORI_Z);
	ORI_DIFF_W = fabs (PREV_G_ORI_W - G_ORI_W);
	ANG_DIFF = fabs (PREV_G_ANG - G_ANG);
	
	JOURNEY_LENGTH += 
	( 
		hypot(POS_DIFF_X, POS_DIFF_Y) + (SEP_RADIUS * ANG_DIFF)
	);
	
	//std::cout << "hypot: " << hypot(POS_DIFF_X, POS_DIFF_Y) << std::endl;
	//std::cout << "angle: " << (SEP_RADIUS * ANG_DIFF) << std::endl;
	
	//ROS_INFO("groundTruthCB executed.");
}

void pathCB(const nav_msgs::PathConstPtr &msg) 
{
	if (!PATH_DIST_CALCULATED)
	{
		PATH_DIST_CALCULATED = true;
		int count = msg->poses.size();
		
		double posx = msg->poses[0].pose.position.x;
		double posy = msg->poses[0].pose.position.y;
		
		double lposx;
		double lposy;
		
		for (int i = 1; i < count; i++)
		{
			lposx = posx;
			lposy = posy;
			
			posx = msg->poses[i].pose.position.x;
			posy = msg->poses[i].pose.position.y;
			
			PATH_DIST_MEAN += hypot( fabs(lposx - posx), fabs(lposy - posy) );
		}
	}
}

void amclPoseCB ( const geometry_msgs::PoseWithCovarianceStampedConstPtr &msg ) // const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg
{
	A_POS_X = msg->pose.pose.position.x;
	A_POS_Y = msg->pose.pose.position.y;
	
	A_ORI_Z = msg->pose.pose.orientation.z;
	A_ORI_W = msg->pose.pose.orientation.w;
	
	// ROS_INFO("amclPoseCB executed.");
}

void printAllInfo()
{	
	// TODO Write all information
	ROS_INFO("ALL DONE!");
}



#endif
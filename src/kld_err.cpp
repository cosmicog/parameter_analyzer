#include <common.h>

double KLD_ERR;

int main ( int argc, char** argv )
{
	ros::init ( argc, argv, "parameter_analyzer" );
	ros::NodeHandle nh;
	
	init_pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped> ( "initialpose", 100 );
	goal_pub = nh.advertise<geometry_msgs::PoseStamped> ( "move_base_simple/goal", 100 );
	
	ros::Subscriber truth_sub = nh.subscribe( "ground_truth_odom", 2, groundTruthCB);
	ros::Subscriber amcl_sub = nh.subscribe( "amcl_pose", 2, amclPoseCB);
	ros::Subscriber goal_sub = nh.subscribe( "move_base/status", 2, goalCB);
	ros::Subscriber path_sub = nh.subscribe( "move_base/GlobalPlanner/plan", 2, pathCB);
	
	dynamic_reconfigure::Server<parameter_analyzer::analyzerConfig> analyzerConfigServer;
	dynamic_reconfigure::Server<parameter_analyzer::analyzerConfig>::CallbackType cbTyp;
	
	cbTyp = boost::bind(&dynamicReconfigureCallback, _1, _2);
	analyzerConfigServer.setCallback( cbTyp );
	
	ros::param::get("parameter_analyzer/increment", INC);
	ros::param::get("parameter_analyzer/tide_count", TRY_COUNT);
	
	ros::param::get("parameter_analyzer/goal_pos_x", GOAL_POS_X);
	ros::param::get("parameter_analyzer/goal_pos_y", GOAL_POS_Y);
	
	ros::param::get("parameter_analyzer/goal_ori_z", GOAL_ORI_Z);
	ros::param::get("parameter_analyzer/goal_ori_w", GOAL_ORI_W);
	
	ros::param::get("parameter_analyzer/param_start", KLD_ERR);
	ros::param::get("parameter_analyzer/param_end", PARAM_END);
	
	ros::param::set("/amcl/kld_err", KLD_ERR);
	
	ros::Time current_time = ros::Time::now();
	ros::Rate r(10);
	
    while ( ros::ok() )
    {	
		if (ALL_DONE) 
		{
			printAllInfo();
			return 0;
		}
		else if (BIG_BANG)
		{
			ros::spinOnce();
			r.sleep();
			ROS_INFO("BIG-BANG OCCURED!");
			printParamInfo();
			BIG_BANG = false;
			publishSimpleGoal(GOAL_POS_X, GOAL_POS_Y, GOAL_ORI_Z, GOAL_ORI_W);
		}
		
		ros::spinOnce();
		r.sleep();
	}
	
return 0;	
}

void printParamInfo()
{
	ROS_INFO("-------------------------------");
	ros::param::get("/amcl/kld_err", KLD_ERR);
	ROS_INFO("-- kld_err: %lf", KLD_ERR);
	ROS_INFO("-------------------------------\n");
	std::cout << "PATH\tLENGTH\tTIME\tERR_X     \tERR_Y\t\tERR_A" << std::endl;
}

void updateParams()
{
	KLD_ERR += INC;
	if (INC > 0)
	{
		if (KLD_ERR > PARAM_END)
		{
			ALL_DONE = true;
			return;
		}
	}
	else
	{
		if (KLD_ERR < PARAM_END)
		{
			ALL_DONE = true;
			return;
		}
	}
	ros::param::set("/amcl/kld_err", KLD_ERR);
	printParamInfo();
}

void dynamicReconfigureCallback(parameter_analyzer::analyzerConfig &config, uint32_t level) 
{
	KLD_ERR = config.kld_err;
}





















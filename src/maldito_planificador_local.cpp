#include "ros/ros.h"
#include <iostream>
#include <vector>
#include <costmap_2d/costmap_2d_ros.h>
#include <global_planner/planner_core.h>
#include <dwa_local_planner/dwa_planner_ros.h>
#include <tf2_ros/transform_listener.h>
#include "tf2_ros/buffer.h"
#include "stero_mobile_init/STPT.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include <tf2/LinearMath/Quaternion.h>

using namespace std;










/* -----------------  GLOBAL VARIABLES  ----------------------------- */

// Planer globalny
tf2_ros::Buffer* globalBuffer;
tf2_ros::TransformListener* tf_global;
costmap_2d::Costmap2DROS* costmap_global;
global_planner::GlobalPlanner* global_planner_;

// Planer lokalny
tf2_ros::Buffer* localBuffer;
tf2_ros::TransformListener* tf_local;
costmap_2d::Costmap2DROS* costmap_local;
dwa_local_planner::DWAPlannerROS* local_planner;

// 
ros::Publisher pub_cmd_vel;
ros::ServiceServer stpt_srv;
//ros::Subscriber sub_odom;

bool inProgress = false;
bool doCancel = false;





/* -----------------  FUNCTION DECLARATIONS  ------------------------ */

/*
* callback do serwera serwisu pozycji zadanej robota
*
*/
bool stpt_service_callback(stero_mobile_init::STPT::Request  &req, stero_mobile_init::STPT::Response &res);

/*
* callback subscribera danych odometrii
*
*/
void odom_callback(const nav_msgs::Odometry::ConstPtr&  msg);

/*
* planowanie ruchu ze startu do setpoint
*
*/
int32_t planMove( geometry_msgs::PoseStamped &start, geometry_msgs::PoseStamped &stpt );








/* -----------------  MAIN  ----------------------------------------- */

/*
* main
*
*/
int main(int argc, char *argv[])
{
    ros::init(argc, argv, "stero/planificador");
    ros::NodeHandle nodeHandle;

    // Planer globalny
    globalBuffer = new tf2_ros::Buffer(ros::Duration(10),true); 
    tf_global = new tf2_ros::TransformListener(*globalBuffer); // bez transformera tf czuje się samotny :(
    costmap_global = new costmap_2d::Costmap2DROS("global_mapa_de_costos", *globalBuffer);
    costmap_global->start();
    global_planner_ = new global_planner::GlobalPlanner("global_planificador", costmap_global->getCostmap(), "map");

    // Planer lokalny
    localBuffer = new tf2_ros::Buffer(ros::Duration(10),true); 
    tf_local = new tf2_ros::TransformListener(*localBuffer);
    costmap_local = new costmap_2d::Costmap2DROS("local_mapa_de_costos", *localBuffer);
    local_planner = new dwa_local_planner::DWAPlannerROS();
    local_planner->initialize("local_planificador", localBuffer, costmap_local);

    // Serwer serwisu pozycji zadanej robota
    stpt_srv = nodeHandle.advertiseService("stero/go_to_stpt", stpt_service_callback);
    // Publisher poleceń prędkości robota
    pub_cmd_vel = nodeHandle.advertise<geometry_msgs::Twist>("mux_vel_nav/cmd_vel", 10); 
	// Subscriber danych z odometrii
	//sub_odom = nodeHandle.subscribe("/elektron/mobile_base_controller/odom", 10, odom_callback);
    

    cout<<"Maldito planificador local laboral\n\r";
	ros::spin();

    delete local_planner;
    delete costmap_local;
    delete tf_local;
    delete localBuffer;
    delete global_planner_;
    delete costmap_global;
    delete tf_global;
    delete globalBuffer;
    return 0;
}

bool stpt_service_callback(stero_mobile_init::STPT::Request  &req, stero_mobile_init::STPT::Response &res)
{
    tf2::Quaternion quat;
    geometry_msgs::PoseStamped start, stpt;
    cout<<"stpt service server callback\n\r";
    cout<<"STPT: x="<<req.pose.x<<"; y="<<req.pose.y<<"; theta="<<req.pose.theta<<";\n\r";

    // gdy w trakcie trwania obsługi pojawi się kolejne, anuluj pierwsze    
    doCancel = true;
    while(inProgress);
    inProgress = true;
    doCancel = false;

    stpt.header.frame_id = "map";
    stpt.header.stamp = ros::Time(0);
    stpt.pose.position.x = req.pose.x;
    stpt.pose.position.y = req.pose.y;
    stpt.pose.position.z = 0;
    quat.setRPY( 0, 0, req.pose.theta ); 
    stpt.pose.orientation.x = quat.getX();
    stpt.pose.orientation.y = quat.getY();
    stpt.pose.orientation.z = quat.getZ();
    stpt.pose.orientation.w = quat.getW();
    start.header.frame_id = "map";
    start.header.stamp = ros::Time(0);

    try
    {
        // pobranie pozycji robota
        if( !costmap_global->getRobotPose(start) )
        {
            res.result = -1;
            inProgress = false;
            return true;
        }

        cout<<"pose: x="<<start.pose.position.x<<"; y="<<start.pose.position.y<<"; theta=---"<<";\n\r";

        res.result = planMove(start, stpt);
    }
    catch(...)
    {
        cout << "Exception occurred";
    }

    inProgress = false;
    return true;
}

//UNUSED
void odom_callback(const nav_msgs::Odometry::ConstPtr&  msg)
{
    static int n=0;
    if(n>100)
    {
        cout<<"odom callback\n\r";
        n=0;
    }
    n++;
    
}

int32_t planMove( geometry_msgs::PoseStamped &start, geometry_msgs::PoseStamped &stpt )
{

    if(doCancel)
        return -10;

    return 0;
}
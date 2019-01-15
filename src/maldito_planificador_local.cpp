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

using namespace std;


/* -----------------  GLOBAL VARIABLES  ----------------------------- */



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
    tf2_ros::Buffer globalBuffer(ros::Duration(10),true); 
    tf2_ros::TransformListener tf_global(globalBuffer); // bez transformera tf czuje się samotny :(
    costmap_2d::Costmap2DROS costmap_global("global_mapa_de_costos", globalBuffer);
    costmap_global.start();
    global_planner::GlobalPlanner global_planner("global_planificador", costmap_global.getCostmap(), "map");

    // Planer lokalny
    tf2_ros::Buffer localBuffer(ros::Duration(10),true); 
    tf2_ros::TransformListener tf_local(localBuffer);
    costmap_2d::Costmap2DROS costmap_local("local_mapa_de_costos", localBuffer);
    dwa_local_planner::DWAPlannerROS local_planner;
    local_planner.initialize("local_planificador", &localBuffer, &costmap_local);

    // Serwer serwisu pozycji zadanej robota
    ros::ServiceServer stpt_srv = nodeHandle.advertiseService("stero/go_to_stpt", stpt_service_callback);
    // Publisher poleceń prędkości robota
    ros::Publisher pub_cmd_vel = nodeHandle.advertise<geometry_msgs::Twist>("mux_vel_nav/cmd_vel", 10); 
	// Subscriber danych z odometrii
	ros::Subscriber sub_odom = nodeHandle.subscribe("/elektron/mobile_base_controller/odom", 10, odom_callback);

    cout<<"Maldito planificador local laboral\n\r";
	ros::spin();
    return 0;
}

bool stpt_service_callback(stero_mobile_init::STPT::Request  &req, stero_mobile_init::STPT::Response &res)
{
    
    cout<<"stpt service server callback\n\r";
    cout<<"STPT: x="<<req.pose.x<<"; y="<<req.pose.y<<"; theta="<<req.pose.theta<<";\n\r";
    return true;
}

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
#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>

#define image_height 480
#define	image_width 640

#define PUBLISH_TOPIC "/cabbage1/cmd_vel"
//#define PUBLISH_TOPIC "/my_robo_two/diff_drive_controller/cmd_vel"

class cabbageController{
public:
    cabbageController();
    ~cabbageController();
    void centerCallback(const std_msgs::Float32::ConstPtr& msg);
    float center_center;//畝の中央の画素の位置の全部の行の平均。畝の中心線の重心。
    void cmd_vel_publisher();
//    void setMoveVector(float linear_x, int cnt);
//    void timerCallback(const ros::TimerEvent&);

private:
    ros::Publisher twist_pub;
    ros::Subscriber center_sub;
    ros::Timer timer;
    ros::NodeHandle nh;
};


cabbageController::cabbageController(){
    twist_pub = nh.advertise<geometry_msgs::Twist>(PUBLISH_TOPIC, 100);
//    twist_pub = nh.advertise<geometry_msgs::Twist>("/my_robo_two/diff_drive_controller/cmd_vel", 100);
    center_sub = nh.subscribe<std_msgs::Float32>("/cabbage/center", 1, &cabbageController::centerCallback, this);
//    timer = nh.createTimer(ros::Duration(0.1), &cabbageController::timerCallback, this);
}


cabbageController::~cabbageController(){
}


void cabbageController::centerCallback(const std_msgs::Float32::ConstPtr& msg){
    int center_width = 60; //px.畝の中心の領域とする幅。この間に入ってる時はロボットはまっすぐ進む

	center_center = msg->data;
    ROS_INFO("center:%f",center_center);//ここではmsg.dataではダメ。msg->dataでなくてはいけない。

    geometry_msgs::Twist twist;
    twist.linear.y = 0.0;
    twist.linear.z = 0.0;
    twist.angular.x = 0.0;
    twist.angular.y = 0.0;
	if(center_center < image_width * 0.5 - center_width * 0.5){//カメラ中心より左に畝のセンターがあるとき
	    twist.linear.x = 0.10;
		twist.angular.z = +0.5;
	}else if(center_center > image_width * 0.5 + center_width * 0.5){
	    twist.linear.x = 0.10;
		twist.angular.z = -0.5;
	}else{
	    twist.linear.x = 0.20;
		twist.angular.z = 0.0;
	}
    twist_pub.publish(twist);
}


/*
void cabbageController::cmd_vel_publisher(){
    geometry_msgs::Twist twist;
    twist.linear.y = 0.0;
    twist.linear.z = 0.0;
    twist.angular.x = 0.0;
    twist.angular.y = 0.0;

	if(center_center < image_width * 0.5 - 20){//カメラ中心より左に畝のセンターがあるとき
	    twist.linear.x = 0.0;
		twist.angular.z = -0.5;
	}else if(center_center > image_width * 0.5 + 20){
	    twist.linear.x = 0.0;
		twist.angular.z = 0.5;
	}else{
	    twist.linear.x = 1.0;
		twist.angular.z = 0.0;
	}

    twist.linear.x = 1.0;
    ROS_INFO("Hello world");
    twist_pub.publish(twist);
}
*/

/*
void cabbageController::timerCallback(const ros::TimerEvent&){
    setMoveVector(0.2, 10);
    setMoveVector(-0.2, 10);
}
*/ 
/*
void cabbageController::setMoveVector(float linear_x, int cnt){
    int i;
    geometry_msgs::Twist twist;
    ros::Rate loop_rate(10);

    for(i = 0;i < cnt; i++){
        twist.linear.x = linear_x;
        twist_pub.publish(twist);
        loop_rate.sleep();
    }
}
*/


int main(int argc, char **argv){
    ros::init(argc, argv, "robot_controller");

	cabbageController cabbageController;
//	cabbageController.cmd_vel_publisher();

    ros::spin();
    return 0;
}
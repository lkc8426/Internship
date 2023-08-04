#include <ros/ros.h>
#include <carla_msgs/CarlaEgoVehicleControl.h>
#include <cmath>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose2D.h>
#include <tf/tf.h>
#include <std_msgs/String.h>
#include <visualization_msgs/MarkerArray.h>
#include <fstream>
#include <vector>
#include <sstream>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/Float32.h>


class tracking{

    ros::NodeHandle nh;

    ros::Publisher vehicle_control_pub;
    ros::Publisher waypoint_marker_pub;
    ros::Publisher waypoint_point_pub;
    ros::Subscriber odometry_sub;
    ros::Subscriber speedometer_sub;
    ros::Publisher car_pub;

    std::vector<std::vector<double>> double_vec_pointer;
    carla_msgs::CarlaEgoVehicleControl vehicle_control_cmd_msg;
    visualization_msgs::MarkerArray waypoint_path;
    geometry_msgs::Pose2D pose2d;
    visualization_msgs::Marker point_marker;
    visualization_msgs::MarkerArray car_path;
    visualization_msgs::Marker marker3;


    double distance_between_vehicle_point; //d
    double wheelbase; //L
    double steering; // theta

    double point_x;
    double point_y;
    double slope;
    double yaw_;
    double temp_x;
    double temp_y;
    double temp_distance;
    int count = 0;

    double Kp;
    double Ki;
    double Kd;
    double target_velocity;
    double current_velocity;
    double control_velocity;

    int index = 0;
    int error_count;
    double min = 10000;

public:
    tracking();

    void PublishControl();
    void OdometryCallback(const nav_msgs::Odometry::ConstPtr& msg);
    void PublishWayPoint();
    std::vector<std::string> split(std::string str,char delimeter);
    void WaypointVisualize();
    double to_double(std::string s);
    void WayPointCreate();
    void ControlCommand();
    void SelectPoint();
    void PidControl(double velocity);
    void SpeedCallback(const std_msgs::Float32::ConstPtr& msg);
    void Error();
    void CarWayVisualize();
};

tracking::tracking(){

    vehicle_control_pub = nh.advertise<carla_msgs::CarlaEgoVehicleControl>("/carla/ego_vehicle/vehicle_control_cmd",100);
    odometry_sub = nh.subscribe("/carla/ego_vehicle/odometry",100, &tracking::OdometryCallback, this);
    speedometer_sub = nh.subscribe("/carla/ego_vehicle/speedometer",100, &tracking::SpeedCallback, this);
    car_pub = nh.advertise<visualization_msgs::Marker>("car", 10);
}

void tracking::WayPointCreate(){
    std::ifstream in("/home/kichang/catkin_ws/src/practice/waypoint_town05.txt");

    if(!in.is_open()){
        ROS_ERROR("No Such Files");
    }
    std::string s;

    while(in){
        getline(in,s); // in이라는 문자열을 한줄씩 s에 저장한다.
        std::istringstream ss(s);
        std::vector<double> waypoint;
        std::vector<std::string> xy = split(s,',');

        for(std::vector<std::string>::iterator itr = xy.begin(); itr != xy.end(); ++itr){
            waypoint.push_back(to_double(*itr));
        }
        double_vec_pointer.push_back(waypoint);
    }


    waypoint_marker_pub = nh.advertise<visualization_msgs::MarkerArray>("odometry",10);
    waypoint_point_pub = nh.advertise<visualization_msgs::Marker>("point", 10);
    
}

std::vector<std::string> tracking::split(std::string input,char delimeter){
    std::vector<std::string> point;
    std::stringstream ss(input);
    std::string temp;

    while(getline(ss,temp,delimeter)){
        point.push_back(temp);
    }
    return point;
}

double tracking::to_double(std::string s) {
    std::istringstream ss(s);
    double x;

    ss >> x;
    return x;
}

void tracking::PublishWayPoint(){
    
    waypoint_marker_pub.publish(waypoint_path);
}

void tracking::CarWayVisualize(){    

    marker3.header.frame_id = "map";  // 마커의 좌표계 설정
    marker3.header.stamp = ros::Time::now();
    marker3.ns = "my_path";
    marker3.id = 0;
    marker3.type = visualization_msgs::Marker::LINE_STRIP;
    marker3.action = visualization_msgs::Marker::ADD;
        
    marker3.scale.x = 0.3;

    marker3.color.r = 1.0;
    marker3.color.g = 0.0;
    marker3.color.b = 0.0;
    marker3.color.a = 1.0;

    geometry_msgs::Point car_point;

    
    if(pose2d.x == 0. && pose2d.y == 0.){
        return;
    }

    else{
        car_point.x = pose2d.x;
        car_point.y = pose2d.y;
        car_point.z = 0.05;
        marker3.points.push_back(car_point);
    }

    car_pub.publish(marker3);
    

}

void tracking::WaypointVisualize(){
    // 벡터의 각 값에 대해 시각화 마커를 발행댐
    visualization_msgs::Marker marker;
    std::vector<double> temp;

    marker.header.frame_id = "map";  // 마커의 좌표계 설정
    marker.header.stamp = ros::Time::now();
    marker.ns = "my_namespace";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.action = visualization_msgs::Marker::ADD;
        
    marker.scale.x = 0.3;

    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.color.a = 0.7;

    for(std::vector<std::vector<double>>::iterator itr = double_vec_pointer.begin(); itr != double_vec_pointer.end(); ++itr) {
        temp = *itr;

        geometry_msgs::Point point;
        point.x = temp[0];
        point.y = temp[1];
        point.z = 0;

        marker.points.push_back(point);
    }

    waypoint_path.markers.push_back(marker);
}

void tracking::OdometryCallback(const nav_msgs::Odometry::ConstPtr& msg2){
    pose2d.x = msg2->pose.pose.position.x;
    pose2d.y = msg2->pose.pose.position.y;

    tf::Quaternion q{
        msg2->pose.pose.orientation.x,
        msg2->pose.pose.orientation.y,
        msg2->pose.pose.orientation.z,
        msg2->pose.pose.orientation.w
    };
    
    tf::Matrix3x3 m(q);
    double roll,pitch,yaw;
    m.getRPY(roll,pitch,yaw);

    yaw_ = yaw;
}

void tracking::PublishControl(){
    vehicle_control_pub.publish(vehicle_control_cmd_msg);
}


void tracking::ControlCommand(){
    std::cout << "---------------------------------------------------------" << std::endl;
    slope = atan2((point_y-pose2d.y),(point_x - pose2d.x));
    distance_between_vehicle_point = sqrt(pow((point_x-pose2d.x),2) + pow((point_y-pose2d.y),2));
    wheelbase = 2.875;
    steering = atan2(2*wheelbase*sin(slope - yaw_), distance_between_vehicle_point);
    std::cout << "Now x : " << pose2d.x << " ";
    std::cout << "Now y : " << pose2d.y << std::endl;
    std::cout << "yaw :" << yaw_ << std::endl;
    std::cout << "distance_between_vehicle_point : " << distance_between_vehicle_point << std::endl;
    std::cout << "steering :" << steering << std::endl;
    std::cout << "current_velocity: " << current_velocity << std::endl;
    // std::cout << "count : " << count << std::endl;
    
    if(count == 632){
        vehicle_control_cmd_msg.throttle = 0;
        vehicle_control_cmd_msg.brake = true;
    }

    vehicle_control_cmd_msg.steer = -steering;

    PublishControl();
}

void tracking::SelectPoint(){
    
    std::vector<double> temp;
    std::vector<std::vector<double>>::iterator itr = double_vec_pointer.begin() + count;

    temp = *itr;

    point_x = temp.at(0);
    point_y = temp.at(1);

    ControlCommand();

    while(distance_between_vehicle_point < 5){
        if(count > 631){
            break;
        }
        
        count++;
        itr = double_vec_pointer.begin() + count;
        temp = *itr;
        temp_x = temp.at(0);
        temp_y = temp.at(1);
        temp_distance = sqrt(pow((temp_x-pose2d.x),2) + pow((temp_y-pose2d.y),2));
        distance_between_vehicle_point = temp_distance;
    }

    // Error();
    

    point_x = temp_x;
    point_y = temp_y;

    point_marker.header.frame_id = "map";  // 마커의 좌표계 설정
    point_marker.header.stamp = ros::Time::now();
    point_marker.ns = "waypoint_specific_point";
    point_marker.id = 0;
    point_marker.type = visualization_msgs::Marker::SPHERE;
    point_marker.action = visualization_msgs::Marker::ADD;

    point_marker.scale.x = 1.0;
    point_marker.scale.y = 1.0;
    point_marker.scale.z = 1.0;

    point_marker.color.r = 1.0;
    point_marker.color.g = 0.0;
    point_marker.color.b = 0.0;
    point_marker.color.a = 1.0;


    point_marker.pose.position.x = point_x;
    point_marker.pose.position.y = point_y;

    waypoint_point_pub.publish(point_marker);

    ControlCommand();

}

void tracking::SpeedCallback(const std_msgs::Float32::ConstPtr& msg){
    current_velocity = 3.6 * (msg->data);
    
    PidControl(current_velocity);
}


void tracking::PidControl(double velocity){
   target_velocity = 35;
   Kp = 0.01;
   Ki = 0.005;
   Kd = 0.007;

   double error = target_velocity - velocity;
   double prev_error = 0;
   double time = 0.1;
   double i_term = 0;

   double p_term = Kp * error;
   i_term = i_term + Ki * (error * time);
   double d_term = Kd * ((error - prev_error) / time);
   control_velocity = p_term + i_term + d_term;
   prev_error = error;

    if(control_velocity > 1){
        control_velocity = 1;
    }

   vehicle_control_cmd_msg.throttle = control_velocity;

}
/*
void tracking::Error(){
    std::vector<double> error_waypoint;
    double error_x;
    double error_y;
    double current_error;
    error_count = count;

    std::cout << "error_count : " << error_count << std::endl;
    for(int i = 0; i < error_count; i++){
        std::vector<std::vector<double>>::iterator itr = double_vec_pointer.begin()+i;
        error_waypoint = *itr;

        error_x = pow((error_waypoint.at(0) - pose2d.x),2);
        error_y = pow((error_waypoint.at(1) - pose2d.y),2);
        current_error = sqrt(error_x + error_y);

        if(current_error < min){
            min = current_error;
            index = i;
        }
    
    }

    std::cout << "current_error : " << current_error << std::endl;
    std::cout << "index : " << index << std::endl;
    std::cout << "error : " << min << std::endl;
    */
    /*
   int index = 0;
   double min_x = -37.7037086486816;
   double min_y = 84.4505920410156;
   min = 10000;
   int interval = 200;
   double coordinate_x;
   double coordinate_y;
   double error_distance;

   for(int i = index; i <= interval + index ;i++){
    coordinate_x = min_x + (point_x - min_x) /200 * i;
    coordinate_y = (point_y - min_y)/(point_x - min_x) * (coordinate_x- min_x) + min_y;
    std::cout << "coordinate_x : " << coordinate_x << " " << "coordinate_y : " << coordinate_y << std::endl;
    error_distance  = sqrt((coordinate_x - pose2d.x) * (coordinate_x - pose2d.x) + (coordinate_y - pose2d.y) * (coordinate_y - pose2d.y));

    if(error_distance < min){
        index = i;
        min = error_distance;
        min_x = coordinate_x;
        min_y = coordinate_y;
    }
    
    
   }
    std::cout << "error : " << min << std::endl;
   
}
*/
int main(int argc,char ** argv){

    ros::init(argc, argv, "trajectory");
    tracking tr;

    tr.WayPointCreate();
    tr.WaypointVisualize();
    ros::Rate loop_rate(10);
    
    while(ros::ok){
        tr.PublishWayPoint();
        tr.SelectPoint();
        tr.CarWayVisualize();

        ros::spinOnce();
        loop_rate.sleep();
    }

    
    return 0;
}
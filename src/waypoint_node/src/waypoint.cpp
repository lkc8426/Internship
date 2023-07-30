#include <ros/ros.h>
#include <std_msgs/String.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <fstream>
#include <vector>
#include <sstream>
#include <geometry_msgs/PoseStamped.h>


class WayPoint{

// 매개변수 선언
    std::vector<std::vector<double>> double_vec_pointer;
    ros::Publisher WayPoint_marker_pub;
    ros::Publisher WayPoint_marker_pub2;
    ros::NodeHandle nh;
    visualization_msgs::MarkerArray marker_array;
    visualization_msgs::Marker marker2;

//생성자+함수 선언
public:
    WayPoint();
    std::vector<std::string> split(std::string str,char delimeter);
    void Visualize();
    double to_double(std::string s);
    void Publish();
    void Visualize2();
};


//기본 생성자
/*--------------------------------------------------------------------------------------------------*/
WayPoint::WayPoint(){
    std::ifstream in("/home/kichang/catkin_ws/src/waypoint_node/waypoint_town05.txt");

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

    WayPoint_marker_pub2 = nh.advertise<visualization_msgs::Marker>("point",10);
    WayPoint_marker_pub = nh.advertise<visualization_msgs::MarkerArray>("odometry",10);
}


//문자열을 구분 문자를 이용해 분할하는 함수
/*--------------------------------------------------------------------------------------------------*/
std::vector<std::string> WayPoint::split(std::string input,char delimeter){
    std::vector<std::string> point;
    std::stringstream ss(input);
    std::string temp;

    while(getline(ss,temp,delimeter)){
        point.push_back(temp);
    }
    return point;
}
/*--------------------------------------------------------------------------------------------------*/


//문자열을 double로 변환하는 함수
/*--------------------------------------------------------------------------------------------------*/
double WayPoint::to_double(std::string s) {
    std::istringstream ss(s);
    double x;

    ss >> x;
    return x;
}
/*--------------------------------------------------------------------------------------------------*/

//publish 함수
/*--------------------------------------------------------------------------------------------------*/
void WayPoint::Publish(){
    WayPoint_marker_pub.publish(marker_array);
    WayPoint_marker_pub2.publish(marker2);
}


//rviz에 visualize하는 함수
/*--------------------------------------------------------------------------------------------------*/

void::WayPoint::Visualize(){
    // 벡터의 각 값에 대해 시각화 마커를 발행댐
    visualization_msgs::Marker marker;
    std::vector<double> temp;

    marker.header.frame_id = "map";  // 마커의 좌표계 설정
    marker.header.stamp = ros::Time::now();
    marker.ns = "waypoint";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.action = visualization_msgs::Marker::ADD;
    
    marker.scale.x = 0.3;

    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;

    for(std::vector<std::vector<double>>::iterator itr = double_vec_pointer.begin(); itr != double_vec_pointer.end(); ++itr) {
        temp = *itr;
        
        geometry_msgs::Point point;
        point.x = temp[0];
        point.y = temp[1];
        marker.points.push_back(point);

    }
    marker_array.markers.push_back(marker);
}

void::WayPoint::Visualize2(){

    marker2.header.frame_id = "map";  // 마커의 좌표계 설정
    marker2.header.stamp = ros::Time::now();
    marker2.ns = "waypoint_specific_point";
    marker2.id = 0;
    marker2.type = visualization_msgs::Marker::SPHERE;
    marker2.action = visualization_msgs::Marker::ADD;
    double b;

    marker2.scale.x = 1.0;
    marker2.scale.y = 1.0;
    marker2.scale.z = 1.0;

    marker2.color.r = 0.0;
    marker2.color.g = 1.0;
    marker2.color.b = 0.0;
    marker2.color.a = 1.0;

    std::vector<double> temp;

    for(std::vector<std::vector<double>>::iterator itr = double_vec_pointer.begin(); itr != double_vec_pointer.end(); ++itr) {
        temp = *itr;

        marker2.pose.position.x = temp.at(0);
        marker2.pose.position.y = temp.at(1);
        std::cout << "marker2.pose.position.x : " << marker2.pose.position.x   << std::endl;
        std::cout << "marker2.pose.position.y : " << marker2.pose.position.y   << std::endl;
        std::cout << "---------------------------------------------" << std::endl;

    }
    
}


//main 함수
/*--------------------------------------------------------------------------------------------------*/
int main(int argc, char ** argv){
    ros::init(argc, argv, "WayPointVisualize"); //노드명 초기화
    
    WayPoint wp;
    wp.Visualize();
    wp.Visualize2();

    ros::Rate loop_rate(10);

    while(ros::ok()){
        wp.Publish();

        ros::spinOnce();

        loop_rate.sleep();
    }

    return 0;
}
#include <ros/ros.h>
#include "geometry_msgs/PoseStamped.h"
#include <ar_track_alvar_msgs/AlvarMarkers.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include <Eigen/Dense>
#include <std_srvs/SetBool.h>

Eigen::Isometry3d marker_iso = Eigen::Isometry3d::Identity();

static std::list< std::string > gs_fileName_string; //global & static

void cleaningFile(std::string _file_name, std::string& _ret_file) {
    _ret_file += "/home/seunghyeonbang/scorpio_ws/src/ExperimentData/";

    _ret_file += _file_name;
    _ret_file += ".txt";

    std::list<std::string>::iterator iter = std::find(
        gs_fileName_string.begin(), gs_fileName_string.end(), _file_name);
    if (gs_fileName_string.end() == iter) {
        gs_fileName_string.push_back(_file_name);
        remove(_ret_file.c_str());
    }
}


void saveVector(const Eigen::VectorXd& vec_, std::string name_) {
    std::string file_name;
    cleaningFile(name_, file_name);

    std::ofstream savefile(file_name.c_str(), std::ios::app);
    for (int i(0); i < vec_.rows(); ++i) {
        savefile << vec_(i) << "\t";
    }
    savefile << "\n";
    savefile.flush();
}

void OnRosMsg(ar_track_alvar_msgs::AlvarMarkers p) {
    Eigen::Quaternion<double> tmp_quat;
    Eigen::VectorXd tmp_vec = Eigen::VectorXd::Zero(3);

    if (p.markers.size() > 0 ) {
        tmp_quat.x() = p.markers[0].pose.pose.orientation.x;
        tmp_quat.y() = p.markers[0].pose.pose.orientation.y;
        tmp_quat.z() = p.markers[0].pose.pose.orientation.z;
        tmp_quat.w() = p.markers[0].pose.pose.orientation.w;
        tmp_vec << p.markers[0].pose.pose.position.x,
                p.markers[0].pose.pose.position.y,
                p.markers[0].pose.pose.position.z;
        marker_iso.linear() = tmp_quat.toRotationMatrix();
        marker_iso.translation() = tmp_vec;       
    }


}

bool ar_tag_saver_callback(std_srvs::SetBool::Request & req, std_srvs::SetBool::Response& res){
    Eigen::VectorXd tmp_vec = Eigen::VectorXd::Zero(16);
    double tmp_val(0);

    for (int row = 0; row < 4; ++row) {
        for (int col = 0; col < 4; ++col) {
            if (row < 3 and col < 3) {
                tmp_val = marker_iso.linear()(row, col);
            } else if (row == 3 and col == 3){
                tmp_val = 1.;
            } else if (col == 3) {
                tmp_val = marker_iso.translation()(row);
            } else {
                tmp_val = 0.;
            }
            tmp_vec(4*row + col) = tmp_val;
        }
    }
    saveVector(tmp_vec, "ar_tag_se3");

    std::cout << "-------------------" << std::endl;
    std::cout << "Saving" << std::endl;
    std::cout << "-------------------" << std::endl;
    std::cout << "ar tag iso" << std::endl;
    std::cout << marker_iso.matrix() << std::endl;
    std::cout << "tmp_vec" << std::endl;
    std::cout << tmp_vec << std::endl;

    res.success = true;
    res.message= "Hi";
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "ar_subscribe");
    ros::NodeHandle n("");
    ros::Subscriber ar_sub = n.subscribe<ar_track_alvar_msgs::AlvarMarkers>("ar_pose_marker", 1000, &OnRosMsg);
    ros::ServiceServer service = n.advertiseService("ar_tag_saver", &ar_tag_saver_callback);
    ros::Rate loop_rate(10);
    ros::spin();

}

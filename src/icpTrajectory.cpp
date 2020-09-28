//This code is created based on A-LOAM by Weichen WEI: weichen.wei@monash.edu
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from this
//    software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.


#include <iostream>

#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/transform_datatypes.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/convert.h>
#include <math.h>
#include "std_msgs/String.h"
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/NavSatFix.h>
#include <gps_common/GPSFix.h>
#include <gps_common/conversions.h>
#include <pcl_ros/point_cloud.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Transform.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <ceres/ceres.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <chrono>
#include "loam_itm/rotation.h"

using namespace std;

class icp_iter_class
{

public:
  icp_iter_class()
  {

    std::remove("Hector.txt");
    std::remove("Tracker.txt");
    std::remove("Icp.txt");
    std::remove("TrackerOrig.txt");

    n_.param<int>("sampling_round", p_sampling_round, 0);
    n_.param<int>("sampling_num", p_sampling_num, 100);
    n_.param<int>("iter_cycle", p_iter_cycle, 5);
    n_.param<float>("gap_limit", p_gap_limit, 0.0f);
    n_.param<float>("angle_limit", p_angle_limit, 0.02f);
    n_.param<int>("residual_limit", p_residual_limit, 300);

    reference_location_init = 0;
    laser_count = 0;
    hactor_count = 0;
    laser_offset = 0;
    hactor_offset = 0;
    laser_stamp_min.fromSec(0.0);
    laser_stamp_max.fromSec(0.0);
    first_call_escaper = 0;
    orientation_aline_escaper = 0;
    gps_location_init = false;
    path_flag = 1;
    min_residual = 10.00;
    round_hector_points = 0;
    round_tracker_points = 0;

    sub_ = n_.subscribe("/aft_mapped_path", 50, &icp_iter_class::hector_path_Callback, this);
    sub_kitti = n_.subscribe("/path_gt", 50, &icp_iter_class::kitti_path_Callback, this);
    timer = n_.createTimer(ros::Duration(p_iter_cycle), &icp_iter_class::icp_iter, this);
    icp_path_pub = n_.advertise<pcl::PointCloud<pcl::PointXYZ>>("icp_path", 1);
    align_path_pub = n_.advertise<pcl::PointCloud<pcl::PointXYZ>>("align_path", 1);
    hector_path_pub = n_.advertise<pcl::PointCloud<pcl::PointXYZ>>("Hector_path", 1);
    tracker_path_pub = n_.advertise<pcl::PointCloud<pcl::PointXYZ>>("Tracker_path", 1);
    PCL_Trans_pub = n_.advertise<geometry_msgs::TransformStamped>("PCL_Trans", 1);
    gps_pub = n_.advertise<geometry_msgs::PoseStamped>("GPS_path", 1);
  }

  struct cost_function_define
  {
    cost_function_define(Eigen::Vector3d p1, Eigen::Vector3d p2) : _p1(p1), _p2(p2) {}
    template <typename T>
    bool operator()(const T *const cere_r, T *residual) const
    {
      T p_1[3];
      T p_2[3];
      p_1[0] = T(_p1.x());
      p_1[1] = T(_p1.y());
      p_1[2] = T(_p1.z());
      AngleAxisRotatePoint(cere_r, p_1, p_2);
      p_2[0] += cere_r[3];
      p_2[1] += cere_r[4];
      p_2[2] += cere_r[5];
      T p_3[3];
      p_3[0] = T(_p2.x());
      p_3[1] = T(_p2.y());
      p_3[2] = T(_p2.z());

      residual[0] = p_2[0] - p_3[0];
      residual[1] = p_2[1] - p_3[1];
      residual[2] = p_2[2] - p_3[2];
      return true;
    }
    Eigen::Vector3d _p1, _p2;
  };

  void icp_iter(const ros::TimerEvent &)
  {
    if (first_call_escaper < 2)
    {
      ROS_INFO("Skip 5 sec %d", first_call_escaper); 
      first_call_escaper++;
      ROS_INFO("pre-sampleling in %d rounds, every round with %d points, Iter every %d Seconds. ITM tigger if gap bigger than %f meters.", p_sampling_round, p_sampling_num, p_iter_cycle, p_gap_limit);
    }
    else
    {
      
      int dim = 3;
      
      int i = 0;
      int j = 0;
      double *M = (double *)calloc(3 * hactor_count, sizeof(double));
      double *T = (double *)calloc(3 * laser_count, sizeof(double));
      double *ceres_rot = new double[6];
      Eigen::Quaterniond R;
      Eigen::Vector3d t;
      Eigen::Vector3d mass_center(dim, 1);
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_Tracker(new pcl::PointCloud<pcl::PointXYZ>);
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_Hector(new pcl::PointCloud<pcl::PointXYZ>);
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ICP(new pcl::PointCloud<pcl::PointXYZ>);

      if (laser_count >= path_flag * p_sampling_num && path_flag <= p_sampling_round)
      {
        double residual = pclAssembler(cloud_Hector, cloud_Tracker, i, j, M, T, R, t, dim, mass_center, ceres_rot);
        
        min_residual = residual;
        ROS_INFO("###################### Alineing Reference Frame ###################### \n");
        path_rotation = R;
        path_translation = t;
        rotatePoint(T, j, R, t, dim, ceres_rot);
        ITMEncoder(cloud_ICP, T, j, dim);
        if (residual < min_residual)
        {
          align_path_pub.publish(cloud_ICP);
        }
        else
        {
          ROS_INFO("\n######Residual not valid, ICP has not converged.###### %f", residual);
        }
        
        
        
        path_flag++;
      }
      else if (laser_count >= path_flag * p_sampling_num && path_flag > p_sampling_round)
      {
        double residual = pclAssembler(cloud_Hector, cloud_Tracker, i, j, M, T, R, t, dim, mass_center, ceres_rot);
        
        rotatePoint(M, i, R, t, dim, ceres_rot);
        ITMEncoder(cloud_ICP, M, i, dim);
        if (residual < p_residual_limit)
        {
          ROS_INFO("###################### Sending Rotation Matrix ###################### \n");
          ROS_INFO("Residual is: %f", residual);
          
          
          
          ROS_INFO("Rotation Mass Center: X %f, Y %f, Z %f", mass_center.x(), mass_center.y(), mass_center.z());
          
          
          if ((abs(t.x()) + abs(t.y()) + abs(t.z())) > p_gap_limit)
          {
            icp_path_pub.publish(cloud_ICP);
            ITMpub(laser_stamp_min, laser_stamp_max, ceres_rot, mass_center, dim);
          }
          else
          {
            align_path_pub.publish(cloud_ICP);
            ROS_INFO("Displance or Rotation is too Small, No need to Correct");
          }
        }
        else
        {
          ROS_INFO("######Residual not valid, ICP has not converged.######");
          align_path_pub.publish(cloud_ICP);
        }
        path_flag++;
      }
      else
      {
        ROS_INFO("waiting for more Reference points");
      }
      free(M); 
      free(T); 
    }
  }

  double pclAssembler(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_Hector, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_Tracker, int &i, int &j, double *&M, double *&T, Eigen::Quaterniond &R, Eigen::Vector3d &t, int dim, Eigen::Vector3d &mass_center, double *ceres_rot)
  {
    pointCloudResize(cloud_Hector);
    pointCloudResize(cloud_Tracker);
    round_hector_points = hactor_count - hactor_offset;
    round_tracker_points = laser_count - laser_offset; 

    i = hectorEncoder(cloud_Hector, M, i, dim, mass_center);
    j = kittiEncoder(cloud_Tracker, T, j, dim);

    hector_path_pub.publish(cloud_Hector);
    tracker_path_pub.publish(cloud_Tracker);

    double residual = 0.0;
    if (path_flag <= p_sampling_round)
    {
      
    }
    else
    {
      ROS_INFO("Trajectory Alignment");
      residual = ceres_ICP(i, j, M, T, R, t, dim, residual, ceres_rot, mass_center);
      return residual;
    }
  }

  void pointCloudResize(pcl::PointCloud<pcl::PointXYZ>::Ptr targetPC)
  {
    targetPC->width = 50;
    targetPC->height = 50;
    targetPC->points.resize(targetPC->width * targetPC->height);
    pcl_conversions::toPCL(ros::Time::now(), targetPC->header.stamp);
    targetPC->header.frame_id = "/camera_init";
  }

  int hectorEncoder(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_Hector, double *&M, int i, int dim, Eigen::Vector3d &mass_center)
  {
    
    float m_c_x = 0.0;
    float m_c_y = 0.0;
    float m_c_z = 0.0;
    pcl_conversions::toPCL(ros::Time::now(), cloud_Hector->header.stamp);
    cloud_Hector->header.frame_id = "/camera_init";
    ROS_INFO("Processing hactor points from %d to %d, processed [%f] points", hactor_offset, hactor_count, round_hector_points);
    while (hactor_offset < Hactor_Path.poses.size())
    {
      M[i * dim + 0] = Hactor_Path.poses[hactor_offset].pose.position.x;
      M[i * dim + 1] = Hactor_Path.poses[hactor_offset].pose.position.y;
      M[i * dim + 2] = Hactor_Path.poses[hactor_offset].pose.position.z;
      cloud_Hector->points[i].x = Hactor_Path.poses[hactor_offset].pose.position.x;
      cloud_Hector->points[i].y = Hactor_Path.poses[hactor_offset].pose.position.y;
      cloud_Hector->points[i].z = Hactor_Path.poses[hactor_offset].pose.position.z;
 
      hactor_offset++;
      
      m_c_x += M[i * dim + 0];
      m_c_y += M[i * dim + 1];
      m_c_z += M[i * dim + 2];
      i++;
    }
    mass_center.x() = m_c_x / i;
    mass_center.y() = m_c_y / i;
    mass_center.z() = m_c_z / i;
    ROS_INFO("Hector points downsamping to %d points.", i);
    return i;
  }

  int kittiEncoder(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_Tracker, double *&T, int j, int dim)
  {
    double factor = round_tracker_points / round_hector_points;
    double cont = laser_offset;
    pcl_conversions::toPCL(ros::Time::now(), cloud_Tracker->header.stamp);
    cloud_Tracker->header.frame_id = "/camera_init";
    ROS_INFO("Processing Kitti_GPS points from %d to %d, processed [%f] points", laser_offset, laser_count, round_tracker_points);
    while (laser_offset < Laser_Path.poses.size())
    {
      T[j * dim + 0] = Laser_Path.poses[laser_offset].pose.position.x;
      T[j * dim + 1] = Laser_Path.poses[laser_offset].pose.position.y;
      T[j * dim + 2] = Laser_Path.poses[laser_offset].pose.position.z;
      cloud_Tracker->points[j].x = Laser_Path.poses[laser_offset].pose.position.x;
      cloud_Tracker->points[j].y = Laser_Path.poses[laser_offset].pose.position.y;
      cloud_Tracker->points[j].z = Laser_Path.poses[laser_offset].pose.position.z;

      cont += factor; 
      laser_offset = ceil(cont);
      
      j++;
    }

    ROS_INFO("Kitti_GPS points downsamping to %d points.", j);
    return j;
  }

  void ITMEncoder(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ICP, double *&M, int i, int dim)
  {
    pointCloudResize(cloud_ICP);
    pcl_conversions::toPCL(ros::Time::now(), cloud_ICP->header.stamp);
    cloud_ICP->header.frame_id = "camera_init";
    for (int c = 0; c < i; ++c)
    {
      
      cloud_ICP->points[c].x = M[dim * c + 0];
      cloud_ICP->points[c].y = M[dim * c + 1];
      cloud_ICP->points[c].z = M[dim * c + 2];
    }
  }

  void ITMpub(ros::Time laser_stamp_min, ros::Time laser_stamp_max, double *ceres_rot, Eigen::Vector3d mass_center, int dim)
  {
    double q_curr[4];
    geometry_msgs::Quaternion ICP_rot;
    geometry_msgs::Vector3 ICP_trans;
    AngleAxisToQuaternion(ceres_rot, q_curr);
    ICP_rot.w = q_curr[0];
    ICP_rot.x = q_curr[1];
    ICP_rot.y = q_curr[2];
    ICP_rot.z = q_curr[3];
    ICP_trans.x = ceres_rot[3];
    ICP_trans.y = ceres_rot[4];
    ICP_trans.z = ceres_rot[5];
    geometry_msgs::TransformStamped ICP_message;
    geometry_msgs::Transform new_frame_trans;
    new_frame_trans.translation = ICP_trans;
    new_frame_trans.rotation = ICP_rot;
    
    ICP_message.header.frame_id = "camera_init";
    ICP_message.transform = new_frame_trans;
    
    PCL_Trans_pub.publish(ICP_message); 
    ROS_INFO("Current Trans Time: %f to %f\n", laser_stamp_min.toSec(), laser_stamp_max.toSec());
  }

  void rotatePoint(double *&points_in, int &points_num, Eigen::Quaterniond Rq, Eigen::Vector3d t, int dim, double *ceres_rot)
  {
    for (int i = 0; i < points_num; ++i)
    { 
      double pointout[3] = {points_in[dim * i + 0], points_in[dim * i + 1], points_in[dim * i + 2]};
      AngleAxisRotatePoint(ceres_rot, pointout, pointout);
      
      points_in[dim * i + 0] = pointout[0] + ceres_rot[3];
      points_in[dim * i + 1] = pointout[1] + ceres_rot[4];
      points_in[dim * i + 2] = pointout[2] + ceres_rot[5];
    }
  }

  void hector_path_Callback(const nav_msgs::Path &msg)
  {
    Hactor_Path = msg;
    hactor_count = Hactor_Path.poses.size();
    
  }

  void kitti_path_Callback(const nav_msgs::Path &msg)
  {
    Laser_Path = msg;
    laser_count = Laser_Path.poses.size();
    
  }

  double ceres_ICP(int &i, int &j, double *&M, double *&T, Eigen::Quaterniond &R, Eigen::Vector3d &t, int dim, double residual, double *ceres_rot3, Eigen::Vector3d &mass_center)
  {
    double cere_r_t[6] = {0.0, 0, 0, 0, 0, 0};
    

    

    vector<Eigen::Vector3d> pts1, pts2;

    for (int i1 = 0; i1 < i; i1++)
    {
      Eigen::Vector3d cp1;
      cp1.x() = M[dim * i1 + 0];
      cp1.y() = M[dim * i1 + 1];
      cp1.z() = M[dim * i1 + 2];
      pts1.push_back(cp1);
    }
    for (int j1 = 0; j1 < j; j1++)
    {
      Eigen::Vector3d cp2;
      cp2.x() = T[dim * j1 + 0];
      cp2.y() = T[dim * j1 + 1];
      cp2.z() = T[dim * j1 + 2];
      pts2.push_back(cp2);
    }
    ceres::Problem problem;

    for (int conti = 0; conti < pts2.size(); conti++)
    {
      ceres::CostFunction *costfunction = new ceres::AutoDiffCostFunction<cost_function_define, 3, 6>(new cost_function_define(pts1[conti], pts2[conti])); 
      problem.AddResidualBlock(costfunction, NULL, cere_r_t);                                                                                              
    }

    ceres::Solver::Options option;
    option.linear_solver_type = ceres::DENSE_QR;
  
    option.max_num_iterations =100;
    
    ceres::Solver::Summary summary;
    
    ceres::Solve(option, &problem, &summary);
    
    ceres_rot3[0] = cere_r_t[0];
    ceres_rot3[1] = cere_r_t[1];
    ceres_rot3[2] = cere_r_t[2];
    ceres_rot3[3] = cere_r_t[3];
    ceres_rot3[4] = cere_r_t[4];
    ceres_rot3[5] = cere_r_t[5];
    double q_curr[4];
    AngleAxisToQuaternion(cere_r_t, q_curr);
    cout << "R = " << cere_r_t[0] << ", " << cere_r_t[1] << ", " << cere_r_t[2] << endl;
    cout << "Q = " << q_curr[0] << ", " << q_curr[1] << ", " << q_curr[2] << "," << q_curr[3] << endl;
    cout << "t = " << cere_r_t[3] << ", " << cere_r_t[4] << ", " << cere_r_t[5] << endl;
    cout << "Average Cost = " << summary.final_cost/p_sampling_num << endl;

    R.w() = q_curr[0];
    R.x() = q_curr[1];
    R.y() = q_curr[2];
    R.z() = q_curr[3];
    t.x() = cere_r_t[3];
    t.y() = cere_r_t[4];
    t.z() = cere_r_t[5];

    if(summary.IsSolutionUsable()==true){
      return summary.final_cost/p_sampling_num;
    }else{
      return 99999;
    }
  }

  
  ros::NodeHandle n_;
  ros::Subscriber sub_, sub_kitti;
  ros::Subscriber tracker_sub;
  ros::Subscriber gps_sub;
  ros::Subscriber pose_sub;
  ros::Timer timer;
  ros::Publisher icp_path_pub;
  ros::Publisher align_path_pub;
  ros::Publisher hector_path_pub;
  ros::Publisher tracker_path_pub;
  ros::Publisher PCL_Trans_pub;
  ros::Publisher gps_pub;
  nav_msgs::Path Hactor_Path;
  nav_msgs::Path Laser_Path;
  
  tf2_ros::TransformBroadcaster pcl_br;
  int reference_location_init;
  float reference_location_init_x;
  float reference_location_init_y;
  float reference_location_init_z;
  bool gps_location_init;
  float gps_location_init_x;
  float gps_location_init_y;
  float gps_location_init_z;
  int laser_count;
  int hactor_count;
  int laser_offset;
  int hactor_offset;
  ros::Time laser_stamp_min;
  ros::Time laser_stamp_max;
  int first_call_escaper;
  int orientation_aline_escaper;
  ros::Duration windowsXP_offset;
  int path_flag;
  double min_residual;
  Eigen::Quaterniond path_rotation;
  Eigen::Vector3d path_translation;
  int p_sampling_round;
  int p_sampling_num;
  int p_iter_cycle;
  float p_gap_limit;
  float p_angle_limit;
  int p_residual_limit;
  float round_hector_points;
  float round_tracker_points;
  float val_tm[3 * 3]; 
  float val_Rm[3 * 3];
};

int main(int argc, char **argv)
{

  ros::init(argc, argv, "path_icp");
  icp_iter_class my_pcl;
  ros::spin();

  return 0;
}

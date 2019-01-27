#include <iostream>
#include <vector>

#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <Eigen/Eigen>

#include <ceres/ceres.h>
#include <cmath>
using ceres::AutoDiffCostFunction;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solver;
using ceres::Solve;
using ceres::CauchyLoss;


class TrajectoryMatcher
{
public:
    TrajectoryMatcher();
    ~TrajectoryMatcher();

private:
    std::vector<geometry_msgs::Point> ndt_trajectory_;
    std::vector<geometry_msgs::Point> gnss_trajectory_;

    void poseCallback(const geometry_msgs::PoseStampedConstPtr &gnss_pose,
                      const geometry_msgs::PoseStampedConstPtr &ndt_pose );
    void timerCallback(const ros::TimerEvent& event);
    void visualize();
    ros::NodeHandle nh_;

    typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::PoseStamped, geometry_msgs::PoseStamped> ApproximateSyncPolicy;
    message_filters::Synchronizer<ApproximateSyncPolicy> *sync_;
    message_filters::Subscriber<geometry_msgs::PoseStamped> gnss_sub_;
    message_filters::Subscriber<geometry_msgs::PoseStamped> ndt_sub_;

    double trans_x_;
    double trans_y_;
    double trans_z_;
    double roll_;
    double pitch_;
    double yaw_;
    Eigen::Vector3d avg_ndt_;
    Eigen::Vector3d avg_gnss_;

    ros::Subscriber map_sub_;
    ros::Publisher marker_pub_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    sensor_msgs::PointCloud2 map_points_;
    Eigen::Matrix4f transform_;
    double resolution_;

    ros::Timer timer_;
    std::random_device rand_;
    std::mt19937 mt_;


};

// struct xResidual {
//     xResidual(double x1, double y1, double z1, double x2)
//         : x1_(x1), y1_(y1), z1_(z1), x2_(x2)
//     {
//     }
//
//     template <typename T>
//     bool operator()(const T* const trans_x_a,
//                     const T* const yaw_a,
//                     T* residual) const
//     {
//         T trans_x, roll,pitch,yaw;
//         trans_x = trans_x_a[0];
//         yaw=yaw_a[0];
//         residual[0] = T(x2_) - ( ( T(x1_) * cos(yaw) + T(y1_) * -sin(yaw) ) + trans_x);
//         return true;
//     }
// private:
//     const double x1_;
//     const double y1_;
//     const double z1_;
//     const double x2_;
// };
// struct yResidual {
//     yResidual(double x1, double y1, double z1, double y2)
//         : x1_(x1), y1_(y1), z1_(z1), y2_(y2)
//     {
//     }
//
//     template <typename T>
//     bool operator()(const T* const trans_y_a,
//                     const T* const yaw_a,
//                     T* residual) const
//     {
//         T trans_y, roll,pitch,yaw;
//         trans_y = trans_y_a[0];
//         yaw=yaw_a[0];
//         residual[0] = T(y2_) - (  T(x1_) * sin(yaw) + T(y1_) * cos(yaw)  + trans_y);
//         return true;
//     }
// private:
//     const double x1_;
//     const double y1_;
//     const double z1_;
//     const double y2_;
// };
//
//
// struct zResidual {
//     zResidual(double x1, double y1, double z1, double z2)
//         : x1_(x1), y1_(y1), z1_(z1), z2_(z2)
//     {
//     }
//
//     template <typename T>
//     bool operator()(const T* const trans_z_a,
//                     const T* const yaw_a,
//                     T* residual) const
//     {
//         T trans_z, roll,pitch,yaw;
//         trans_z = trans_z_a[0];
//         yaw=yaw_a[0];
//         residual[0] = pow(T(z2_) - trans_z,2);
//         return true;
//     }
// private:
//     const double x1_;
//     const double y1_;
//     const double z1_;
//     const double z2_;
// };


struct xResidual {
    xResidual(double x1, double y1, double z1, double x2)
        : x1_(x1), y1_(y1), z1_(z1), x2_(x2)
    {
    }

    template <typename T>
    bool operator()(const T* const trans_x_a,
                    const T* const roll_a,
                    const T* const pitch_a,
                    const T* const yaw_a,
                    T* residual) const
    {
        residual[0] = T(x2_) - ( ( T(x1_) * cos(yaw_a[0]) * cos(pitch_a[0]) )
                                 + T(y1_) * (cos(yaw_a[0]) * sin(pitch_a[0]) * sin(roll_a[0]) - sin(yaw_a[0]) * cos(roll_a[0]) )
                                 + T(z1_) * (cos(yaw_a[0]) * sin(pitch_a[0]) * cos(roll_a[0]) + sin(yaw_a[0]) * sin(roll_a[0]) )
                                 + trans_x_a[0]);
        return true;
    }
private:
    const double x1_;
    const double y1_;
    const double z1_;
    const double x2_;
};

struct yResidual {
    yResidual(double x1, double y1, double z1, double y2)
        : x1_(x1), y1_(y1), z1_(z1), y2_(y2)
    {
    }

    template <typename T>
    bool operator()(const T* const trans_y_a,
                    const T* const roll_a,
                    const T* const pitch_a,
                    const T* const yaw_a,
                    T* residual) const
    {
        residual[0] = T(y2_) - ( ( T(x1_) * sin(yaw_a[0]) * cos(pitch_a[0]) )
                                 + T(y1_) * (sin(yaw_a[0]) * sin(pitch_a[0]) * sin(roll_a[0]) + cos(yaw_a[0]) * cos(roll_a[0]) )
                                 + T(z1_) * (sin(yaw_a[0]) * sin(pitch_a[0]) * cos(roll_a[0]) - cos(yaw_a[0]) * sin(roll_a[0]) )
                                 + trans_y_a[0]);
        return true;
    }
private:
    const double x1_;
    const double y1_;
    const double z1_;
    const double y2_;
};

struct zResidual {
    zResidual(double x1, double y1, double z1, double z2)
        : x1_(x1), y1_(y1), z1_(z1), z2_(z2)
    {
    }

    template <typename T>
    bool operator()(const T* const trans_z_a,
                    const T* const roll_a,
                    const T* const pitch_a,
                    const T* const yaw_a,
                    T* residual) const
    {

        residual[0] = pow( T(z2_) - ( ( T(x1_) * -sin(pitch_a[0]) )
                                      + T(y1_) * (cos(pitch_a[0]) * sin(roll_a[0]) )
                                      + T(z1_) * (cos(pitch_a[0]) * cos(roll_a[0]) )
                                      + trans_z_a[0] ), 2);
        return true;
    }
private:
    const double x1_;
    const double y1_;
    const double z1_;
    const double z2_;
};


struct fineTransResidual {
    fineTransResidual(double x1,double x2)
        : x1_(x1), x2_(x2)
    {
    }

    template <typename T>
    bool operator()(const T* const trans_x_a,
                    T* residual) const
    {
        residual[0] = T(x2_) - trans_x_a[0];
        return true;
    }
private:
    const double x1_;
    const double x2_;
};
struct fineRotResidual {
    fineRotResidual(double x1,double y1, double x2, double y2)
        : x1_(x1),y1_(y1),x2_(x2),y2_(y2)
    {
    }

    template <typename T>
    bool operator()(const T* const theta_a,
                    T* residual) const
    {
        residual[0] = (T(x2_) - (T(x1_) * cos(theta_a[0]) - T(y1_) * sin(theta_a[0])) ) * (T(x2_) - (T(x1_) * cos(theta_a[0]) - T(y1_) * sin(theta_a[0])) )
                      + (T(y2_) - (T(x1_) * sin(theta_a[0]) + T(y1_) * cos(theta_a[0])) ) * (T(y2_) - (T(x1_) * sin(theta_a[0]) + T(y1_) * cos(theta_a[0])) );
        return true;
    }
private:
    const double x1_;
    const double y1_;
    const double x2_;
    const double y2_;
};

TrajectoryMatcher::TrajectoryMatcher()
    : tf_listener_(tf_buffer_), mt_(rand_())
{
    ndt_sub_.subscribe(nh_,"ndt_pose", 1);
    gnss_sub_.subscribe(nh_,"gnss_pose", 1);
    ros::NodeHandle pnh("~");

    marker_pub_ = nh_.advertise<visualization_msgs::Marker>( "visualization_marker", 0 );

    sync_ =
        new message_filters::Synchronizer<ApproximateSyncPolicy>(ApproximateSyncPolicy(100),
                                                                 gnss_sub_,
                                                                 ndt_sub_);
    sync_->registerCallback(boost::bind(&TrajectoryMatcher::poseCallback,this, _1, _2));
    // gnss_sub_ = nh_.subscribe("gnss_pose", 1, &TrajectoryMatcher::gnssCallback, this);

    timer_ = nh_.createTimer(ros::Duration(0.2), &TrajectoryMatcher::timerCallback, this);
    pnh.param<double>("resolution", resolution_, 1.0);


}

TrajectoryMatcher::~TrajectoryMatcher()
{
}

double calculateCost(std::vector<geometry_msgs::Point>  trajectory1, std::vector<geometry_msgs::Point>  trajectory2, Eigen::Vector3d translation, Eigen::Matrix3d rotation)
{

    double distance = 0;
    int index = 0;
    for (int index = 0; index < trajectory1.size(); index++)
    {
        geometry_msgs::Point pt1 = trajectory1.at(index);
        geometry_msgs::Point pt2 = trajectory2.at(index);

        Eigen::Vector3d eigen_pt(pt1.x, pt1.y, pt1.z );

        Eigen::Vector3d transformed = translation + rotation * eigen_pt;

        distance += pow(pt2.x - transformed[0], 2)
                    + pow(pt2.y - transformed[1], 2)
                    + pow(pt2.z - transformed[2], 2);
    }
    return distance;
}
double calculateCost2(std::vector<geometry_msgs::Point>  trajectory1, std::vector<geometry_msgs::Point>  trajectory2,
                      double trans_x,double trans_y, double trans_z,
                      double roll, double pitch, double yaw)
{

    double distance = 0;
    int index = 0;
    for (int index = 0; index < trajectory1.size(); index++)
    {
        geometry_msgs::Point pt1 = trajectory1.at(index);
        geometry_msgs::Point pt2 = trajectory2.at(index);

        double diff_x = pt2.x -(  pt1.x * cos(yaw) * cos(pitch)
                                  + pt1.y * (cos(yaw) * sin(pitch) * sin(roll) - sin(yaw) * cos(roll) )
                                  + pt1.z * (cos(yaw) * sin(pitch) * cos(roll) + sin(yaw) * sin(roll) )
                                  + trans_x);

        double diff_y = pt2.y - (  pt1.x * sin(yaw) * cos(pitch)
                                   + pt1.y * (sin(yaw) * sin(pitch) * sin(roll) + cos(yaw) * cos(roll) )
                                   + pt1.z * (sin(yaw) * sin(pitch) * cos(roll) - cos(yaw) * sin(roll) )
                                   + trans_y);
        double diff_z = pt2.z - (  pt1.x * -sin(pitch)
                                   + pt1.y * (cos(pitch) * sin(roll) )
                                   + pt1.z * (cos(pitch) * cos(roll) )
                                   + trans_z );
        // Eigen::Vector3d eigen_pt(pt1.x, pt1.y, pt1.z );
        //
        // Eigen::Vector3d transformed = translation + rotation * eigen_pt;

        distance += pow(diff_x, 2)
                    + pow(diff_y, 2)
                    + pow(diff_z, 2);
    }
    std::cout << cos(yaw) * cos(pitch) << " " << (cos(yaw) * sin(pitch) * sin(roll) - sin(yaw) * cos(roll) ) << " " << (cos(yaw) * sin(pitch) * cos(roll) + sin(yaw) * sin(roll) )<< std::endl;
    std::cout << sin(yaw) * cos(pitch) << " " << (sin(yaw) * sin(pitch) * sin(roll) + cos(yaw) * cos(roll) ) << " " << (sin(yaw) * sin(pitch) * cos(roll) - cos(yaw) * sin(roll) ) << std::endl;
    std::cout << -sin(pitch) << " " << (cos(pitch) * sin(roll) ) << " " << (cos(pitch) * cos(roll) ) << std::endl;

    return distance;
}

std::vector<geometry_msgs::Point> transform_trajectory(std::vector<geometry_msgs::Point> trajectory, Eigen::Vector3d translation, Eigen::Matrix3d rotation)
{
    for (auto &point : trajectory)
    {
        Eigen::Vector3d eigen_pt(point.x, point.y, point.z );

        Eigen::Vector3d transformed = translation + rotation * eigen_pt;

        point.x = transformed[0];
        point.y = transformed[1];
        point.z = transformed[2];
    }
    return trajectory;
}

void TrajectoryMatcher::timerCallback(const ros::TimerEvent &event)
{

    if(gnss_trajectory_.size() == 0)
    {
        return;
    }
    if(gnss_trajectory_.size() != ndt_trajectory_.size() )
    {
        ROS_FATAL_STREAM("size of trajectories are not equeal: "<< gnss_trajectory_.size() << " vs. " << ndt_trajectory_.size());
        exit(1);
    }
    Problem problem;
    double initial_trans_x = trans_x_; //0; // ndt_trajectory_.front().x - gnss_trajectory_.front().x;
    double initial_trans_y = trans_y_; //0; // ndt_trajectory_.front().y - gnss_trajectory_.front().y;
    double initial_trans_z = trans_z_; //0; // ndt_trajectory_.front().z - gnss_trajectory_.front().z;
    double initial_roll = roll_; // 0;
    double initial_pitch = pitch_; // 0;
    double initial_yaw = yaw_; //0.18 deg randomness;

    std::vector<geometry_msgs::Point> translated_gnss;
    std::vector<geometry_msgs::Point> translated_ndt;
    avg_ndt_[0]=avg_ndt_[1] = avg_ndt_[2] = 0;
    for(auto data : ndt_trajectory_)
    {
        avg_ndt_[0] += data.x / ndt_trajectory_.size();
        avg_ndt_[1] += data.y / ndt_trajectory_.size();
        avg_ndt_[2] += data.z / ndt_trajectory_.size();
    }
    for(auto data : ndt_trajectory_)
    {
        geometry_msgs::Point pt;
        pt.x = data.x - avg_ndt_[0];
        pt.y = data.y - avg_ndt_[1];
        pt.z = data.z - avg_ndt_[2];
        translated_ndt.push_back(pt);
    }

    avg_gnss_[0]=avg_gnss_[1] = avg_gnss_[2] = 0;
    for(auto data : gnss_trajectory_)
    {
        avg_gnss_[0] += data.x / gnss_trajectory_.size();
        avg_gnss_[1] += data.y / gnss_trajectory_.size();
        avg_gnss_[2] += data.z / gnss_trajectory_.size();
    }
    for(auto data : gnss_trajectory_)
    {
        geometry_msgs::Point pt;
        pt.x = data.x - avg_gnss_[0];
        pt.y = data.y - avg_gnss_[1];
        pt.z = data.z - avg_gnss_[2];
        translated_gnss.push_back(pt);
    }


    for (int i = 0; i < gnss_trajectory_.size(); i++)
    {
        geometry_msgs::Point gnss_data = translated_gnss.at(i);
        geometry_msgs::Point ndt_data = translated_ndt.at(i);

        problem.AddResidualBlock(
            new AutoDiffCostFunction<xResidual, 1, 1, 1, 1, 1>(
                new xResidual(ndt_data.x, ndt_data.y, ndt_data.z, gnss_data.x)),
            NULL,
            &trans_x_, &roll_, &pitch_, &yaw_);
        problem.AddResidualBlock(
            new AutoDiffCostFunction<yResidual, 1, 1, 1, 1, 1>(
                new yResidual(ndt_data.x, ndt_data.y, ndt_data.z, gnss_data.y)),
            NULL,
            &trans_y_, &roll_, &pitch_, &yaw_);
        problem.AddResidualBlock(
            new AutoDiffCostFunction<zResidual, 1, 1, 1, 1,1 >(
                new zResidual(ndt_data.x, ndt_data.y, ndt_data.z, gnss_data.z)),
            NULL,
            &trans_z_, &roll_, &pitch_, &yaw_);
        // problem.AddResidualBlock(
        //     new AutoDiffCostFunction<residual, 1, 1, 1, 1,1 >(
        //         new residual(ndt_data.x, ndt_data.y, ndt_data.z, gnss_data.x, gnss_data.y, gnss_data.z)),
        //     NULL,
        //     &trans_x_, &trans_y_, &trans_z_, &roll_, &pitch_, &yaw_);

    }
    problem.SetParameterLowerBound(&roll_, 0,-M_PI*2);
    problem.SetParameterLowerBound(&pitch_, 0,-M_PI*2);
    problem.SetParameterLowerBound(&yaw_, 0,-M_PI*2);
    problem.SetParameterUpperBound(&roll_, 0, M_PI*2);
    problem.SetParameterUpperBound(&pitch_, 0, M_PI*2);
    problem.SetParameterUpperBound(&yaw_, 0,M_PI*2);


    Solver::Options options;
    options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
    options.minimizer_progress_to_stdout = false;
    options.line_search_sufficient_function_decrease = 1e-32;
    options.function_tolerance = 1e-32;
    options.parameter_tolerance = 1e-32;
    Solver::Summary summary;
    Solve(options, &problem, &summary);

    ROS_INFO_STREAM( std::endl << summary.BriefReport()
                     // << summary.FullReport() << "\n";
                                           << "x     : " << initial_trans_x << " -> " << trans_x_ << std::endl
                                           << "y     : " << initial_trans_y << " -> " << trans_y_ << std::endl
                                           << "z     : " << initial_trans_z << " -> " << trans_z_ << std::endl
                                           << "roll  : " << fmod(initial_roll,M_PI * 2)    << " -> " << fmod(roll_,M_PI * 2) << std::endl
                                           << "pitch : " << fmod(initial_pitch, M_PI * 2)  << " -> " << fmod(pitch_,M_PI * 2) << std::endl
                                           << "yaw   : " << fmod(initial_yaw,M_PI * 2)     << " -> " << fmod(yaw_,M_PI * 2) << std::endl
                                         );



    // double cost = calculateCost(translated_ndt, translated_gnss, trans, rotation);
    // std::cout << "cost:" << cost << std::endl;

    if(roll_ > M_PI )
        roll_ -= M_PI * 2;
    if(roll_ < -M_PI )
        roll_ += M_PI * 2;
    if(pitch_ > M_PI )
        pitch_ -= M_PI * 2;
    if(pitch_ < -M_PI )
        pitch_ += M_PI * 2;
    if(yaw_ > M_PI )
        yaw_ -= M_PI * 2;
    if(yaw_ < -M_PI )
        yaw_ += M_PI * 2;

    visualize();

    //final_transform

    Eigen::Matrix3d rotation;

    rotation = Eigen::AngleAxisd(yaw_, Eigen::Vector3d::UnitZ())
    * Eigen::AngleAxisd(pitch_,  Eigen::Vector3d::UnitY())
    * Eigen::AngleAxisd(roll_, Eigen::Vector3d::UnitX());
    Eigen::Vector3d trans(trans_x_, trans_y_, trans_z_);

    std::cout << rotation << std::endl;
    Eigen::Vector3d final_translation =  avg_gnss_ + trans + rotation * -avg_ndt_;
    // std::cout << rotation * (-avg_ndt_) << std::endl;
    // std::cout << avg_gnss_ << std::endl << avg_ndt_ << std::endl << trans;
    ROS_INFO_STREAM(std::endl<<"FINAL TRANSFORM:" << std::fixed << std::endl
              << "x:" << final_translation[0] << std::endl
              << "y:" << final_translation[1] << std::endl
              << "z:" << final_translation[2] << std::endl
              << "roll:" << roll_ << std::endl
              << "pitch:" << pitch_ << std::endl
              << "yaw:" << yaw_ << std::endl << std::endl);

}



void TrajectoryMatcher::visualize()
{

    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    // marker.header.stamp = 0;
    marker.ns = "ndt";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 1.0;
    marker.color.a = 0.75; // Don't forget to set the alpha!
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    //only if using a MESH_RESOURCE marker type:

    // marker.action = visualization_msgs::Marker::DELETE;
    marker_pub_.publish( marker );

    for (auto ndt : ndt_trajectory_)
    {
        marker.points.push_back(ndt);
    }
    marker.action = visualization_msgs::Marker::ADD;
    marker_pub_.publish( marker );

    marker.points.clear();
    marker.ns = "gnss";
    marker.id = 1;
    marker.color.g = 0;
    marker.color.b = 1;
    for (auto gnss : gnss_trajectory_)
    {
        Eigen::Matrix3d rotation;
        rotation =   Eigen::AngleAxisd(yaw_, Eigen::Vector3d::UnitZ())
                   * Eigen::AngleAxisd(pitch_,  Eigen::Vector3d::UnitY())
                   * Eigen::AngleAxisd(roll_, Eigen::Vector3d::UnitX());
        Eigen::Vector3d trans(trans_x_, trans_y_, trans_z_);

        Eigen::Vector3d pt(gnss.x, gnss.y, gnss.z );

        // Eigen::Vector3d trans1(avg_.x, avg_.y, avg_.z);
        // trans1 = m * trans1;
        // Eigen::Vector3d result = m * pt + trans + trans1;

        // Eigen::Vector3d result = avg_ndt_ + rotation.inverse() * (pt - avg_gnss_ - trans);

        Eigen::Vector3d translate_tot = avg_ndt_ + rotation.inverse() * (-trans - avg_gnss_);
        Eigen::Vector3d result = translate_tot + rotation.inverse() * pt;


        geometry_msgs::Point transformed_point;
        transformed_point.x = result(0);
        transformed_point.y = result(1);
        transformed_point.z = result(2);
        // geometry_msgs::Point avg_;

        marker.points.push_back(transformed_point);
    }
    marker_pub_.publish( marker );

}


void TrajectoryMatcher::poseCallback(const geometry_msgs::PoseStampedConstPtr &gnss_pose,
                                     const geometry_msgs::PoseStampedConstPtr &ndt_pose )
{
    static bool first = true;
    static geometry_msgs::PoseStamped past;
    double distance = pow(past.pose.position.x - gnss_pose->pose.position.x, 2)
                      + pow(past.pose.position.y - gnss_pose->pose.position.y, 2)
                      + pow(past.pose.position.z - gnss_pose->pose.position.z, 2);

    if(first || distance > resolution_ * resolution_)
    {
        gnss_trajectory_.push_back(gnss_pose->pose.position);
        ndt_trajectory_.push_back(ndt_pose->pose.position);
        past = *gnss_pose;
        first = false;
    }
}

int main(int argc, char *argv[])
{
    ros::init(argc,argv,"trajectory_matcher");

    TrajectoryMatcher matcher;

    ros::spin();
    return 0;
}

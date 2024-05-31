#include "ros/ros.h"
#include <RBDyn/parsers/urdf.h>
#include "RBDyn/FK.h"
#include "RBDyn/FV.h"
#include <iostream>
#include "mc_rbdyn_urdf/urdf.h" 
#include "RBDyn/Jacobian.h"
#include "geometry_msgs/WrenchStamped.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/Float64MultiArray.h"

class ForceEstimator
{
public:
    ForceEstimator(){
        ros::NodeHandle nh("~");
        ros::AsyncSpinner spinner(0);
        spinner.start();
        std::string urdf_string;
        // search and wait for robot_description on param server
        while (urdf_string.empty()) {
            ROS_INFO("ForceEstimator is waiting for model");

            nh.getParam(robot_description, urdf_string);
            // std::cout << urdf_string << std::endl;

            usleep(100000);
        }
         try
        {
            this->init_rbdyn(urdf_string, end_effector);
        }
        catch (std::runtime_error e)
        {
            ROS_ERROR("Error when intializing RBDyn: %s", e.what());
        }
        ROS_INFO_STREAM_NAMED("ForceEstimator", "Received urdf from param server, parsing...");
        
        pub_ = nh.advertise<geometry_msgs::WrenchStamped>("/cartesian_wrench_test", 1);
        sub_joint_states_ = nh.subscribe("/iiwa/joint_states", 1, &ForceEstimator::joints_callback, this);
        sub_torques_ = nh.subscribe("/iiwa/CartesianImpedance_trajectory_controller/commanded_torques", 1, &ForceEstimator::joint_torque_callback, this);
        ros::waitForShutdown();
    }

    void init_rbdyn(const std::string &urdf_string, const std::string &end_effector)
    {
    // Convert URDF to RBDyn
        rbdyn_urdf_ = mc_rbdyn_urdf::rbdyn_from_urdf(urdf_string);
        rbd_indices_.clear();
        for (size_t i = 0; i < rbdyn_urdf_.mb.nrJoints(); i++)
        {
            if (rbdyn_urdf_.mb.joint(i).type() != rbd::Joint::Fixed)
                rbd_indices_.push_back(i);
        }

        for (size_t i = 0; i < rbdyn_urdf_.mb.nrBodies(); i++)
        {
            if (rbdyn_urdf_.mb.body(i).name() == end_effector)
            {
                _ef_index = i;
                // ROS_INFO("eef_index: %d", _ef_index);
                return;
            }
        }
        throw std::runtime_error("Index for end effector link " + end_effector + " not found in URDF. Aborting.");
    }
    
    void joints_callback(const sensor_msgs::JointState::ConstPtr& msg){

        Eigen::VectorXd q = Eigen::VectorXd::Zero(rbd_indices_.size());
        Eigen::VectorXd dq = Eigen::VectorXd::Zero(rbd_indices_.size());
        for (size_t i = 0; i < rbd_indices_.size(); i++)
        {
            q[i] = msg->position[i];
            dq[i] = msg->velocity[i];
        }
        // ROS_INFO("q: %f, %f, %f, %f, %f, %f, %f", q[0], q[1], q[2], q[3], q[4], q[5], q[6]);
        Eigen::MatrixXd J = jacobian(q, dq);
        // std::cout << "J" << std::endl; 
        // for (int i = 0; i < J.rows(); ++i) {
        //     for (int j = 0; j < J.cols(); ++j) {
        //     std::cout << J(i, j) << " ";
        //     }
        // std::cout << std::endl;  // Move to the next row
        // }

        // ******************************************************
        // (J^T)^+ = (J * J^T)^-1 * J^T
        //  NOT STABLE !!!
        // J_pinv = ((J*J.transpose()).inverse())*J.transpose();
        // ******************************************************

        // ******************************************************
        //  STABLE !!!
        // J_T_inv = ((J*J.transpose()).inverse())*J.transpose();
        J_T_inv = pseudo_inverse(J.transpose()); 
    }  

    Eigen::MatrixXd jacobian(const Eigen::VectorXd &q, const Eigen::VectorXd &dq)
    {
        mc_rbdyn_urdf::URDFParserResult rbdyn_urdf = rbdyn_urdf_;
        rbdyn_urdf.mbc.zero(rbdyn_urdf.mb);
        _update_urdf_state(rbdyn_urdf, q, dq);
        //compute jacobian
        rbd::Jacobian jac(rbdyn_urdf.mb, rbdyn_urdf.mb.body(_ef_index).name());
        rbd::forwardKinematics(rbdyn_urdf.mb, rbdyn_urdf.mbc);
        rbd::forwardVelocity(rbdyn_urdf.mb, rbdyn_urdf.mbc);
        return jac.jacobian(rbdyn_urdf.mb,rbdyn_urdf.mbc);

        // Eigen::MatrixXd jacMat = jac.bodyJacobian(rbdyn_urdf.mb, rbdyn_urdf.mbc);
        // Eigen::MatrixXd fullJacMat(6, rbdyn_urdf.mb.nrDof());
        // jac.fullJacobian(rbdyn_urdf.mb, jacMat, fullJacMat);
        // return fullJacMat;
    }

    void joint_torque_callback(const std_msgs::Float64MultiArray::ConstPtr& msg){

        Eigen::VectorXd torques = Eigen::VectorXd::Zero(rbd_indices_.size());
        for (size_t i = 0; i < rbd_indices_.size(); i++)
        {
            torques[i] = msg->data[i];
        }
        // ROS_INFO("torques: %f, %f, %f, %f, %f, %f, %f", torques[0], torques[1], torques[2], torques[3], torques[4], torques[5], torques[6]);
        if (J_T_inv.rows() == 0 || J_T_inv.cols() == 0) {
            return;
        }
        // ROS_INFO("Jacobian shape: %ld, %ld", J_T_inv.rows(), J_T_inv.cols());
        Eigen::VectorXd force = J_T_inv*torques;

        // ROS_INFO("force: %f, %f, %f, %f, %f, %f", force[0], force[1], force[2], force[3], force[4], force[5]);
        geometry_msgs::WrenchStamped force_msg;
        force_msg.header.frame_id = "iiwa_link_0";
        force_msg.header.stamp = ros::Time::now();
        force_msg.wrench.force.x = force[3];
        force_msg.wrench.force.y = force[4];
        force_msg.wrench.force.z = force[5];
        force_msg.wrench.torque.x = force[0];
        force_msg.wrench.torque.y = force[1];
        force_msg.wrench.torque.z = force[2];
        pub_.publish(force_msg);        
    }

private:
    ros::Subscriber sub_joint_states_;
    ros::Subscriber sub_torques_;
    ros::Publisher pub_;
    mc_rbdyn_urdf::URDFParserResult rbdyn_urdf_;
    std::vector<size_t> rbd_indices_;
    size_t _ef_index;
    std::string robot_description = "/robot_description";
    std::string end_effector = "iiwa_link_ee";
    Eigen::MatrixXd J_T_inv;


    void _update_urdf_state(mc_rbdyn_urdf::URDFParserResult &rbdyn_urdf, const Eigen::VectorXd &q, const Eigen::VectorXd &dq)
    {
        for (size_t i = 0; i < rbd_indices_.size(); i++)
        {
            size_t rbd_index = rbd_indices_[i];
            if (q.size() > i)
                rbdyn_urdf.mbc.q[rbd_index][0] = q[i];
            if (dq.size() > i)
                rbdyn_urdf.mbc.alpha[rbd_index][0] = dq[i];
        }
        
        
    }
    template <class MatT>
    Eigen::Matrix<typename MatT::Scalar, MatT::ColsAtCompileTime, MatT::RowsAtCompileTime> pseudo_inverse(const MatT& mat, typename MatT::Scalar tolerance = typename MatT::Scalar{1e-4}) // choose appropriately
    {
    typedef typename MatT::Scalar Scalar;
    auto svd = mat.jacobiSvd(Eigen::ComputeFullU | Eigen::ComputeFullV);
    const auto& singularValues = svd.singularValues();
    Eigen::Matrix<Scalar, MatT::ColsAtCompileTime, MatT::RowsAtCompileTime> singularValuesInv(mat.cols(), mat.rows());
    singularValuesInv.setZero();
    for (unsigned int i = 0; i < singularValues.size(); ++i) {
        if (singularValues(i) > tolerance) {
            singularValuesInv(i, i) = Scalar{1} / singularValues(i);
        }
        else {
            singularValuesInv(i, i) = Scalar{0};
        }
    }
    return svd.matrixV() * singularValuesInv * svd.matrixU().adjoint();
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "force_estimator");
    ForceEstimator fe;
    return 0;
}

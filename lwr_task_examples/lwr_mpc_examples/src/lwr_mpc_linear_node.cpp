#include "ros/ros.h"
#include "lwr_controllers/FF_FB_plan.h"
#include "geometry_msgs/PoseStamped.h"
#include "eigen_conversions/eigen_msg.h"
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/JointState.h>
#include <robot_motion_generation/IterativeLQSolver.h>
#include <PointMassLinear.h>
#include <tf/tf.h>

#include <kdl/jntspaceinertiamatrix.hpp>
#include <kdl/chaindynparam.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <urdf/model.h>


/**
 *  Linear Quadratic MPC example
 *
 */

tf::Pose  pose;
geometry_msgs::Twist x_dot;
KDL::JntArray joint_pos;
void pose_callback(const geometry_msgs::PoseConstPtr &msg){

    pose.setOrigin(tf::Vector3(msg->position.x,msg->position.y,msg->position.z));
    pose.setRotation(tf::Quaternion(msg->orientation.x,
                                    msg->orientation.y,
                                    msg->orientation.z,
                                    msg->orientation.w));
}

void joint_state_callback(const sensor_msgs::JointStateConstPtr &msg){
    joint_pos.resize(7);
    for(int i=0;i<7;i++)
        joint_pos(i) = msg->position[i];
}

void velocity_callback(const geometry_msgs::TwistConstPtr &msg){
    x_dot.linear.x = msg->linear.x;
    x_dot.linear.y = msg->linear.y;
    x_dot.linear.z = msg->linear.z;
    x_dot.angular.x = msg->angular.x;
    x_dot.angular.y = msg->angular.y;
    x_dot.angular.z = msg->angular.z;
}

bool initialize_robot_model(ros::NodeHandle nh,KDL::Tree &tree, KDL::Chain& chain){


    // get URDF and name of root and tip from the parameter server
    std::string robot_description, root_name, tip_name;

    std::string name_space = "/lwr";

    if (!ros::param::search(name_space,"robot_description", robot_description))
    {
        ROS_ERROR_STREAM("KinematicChainControllerBase: No robot description (URDF) found on parameter server ("<<nh.getNamespace()<<"/robot_description)");

    }

    if (!nh.getParam( name_space + "/root_name", root_name))
    {
        ROS_ERROR_STREAM("KinematicChainControllerBase: No root name found on parameter server ("<<nh.getNamespace()<<"/root_name)");
   //     return false;
    }

    if (!nh.getParam(name_space + "/tip_name", tip_name))
    {
        ROS_ERROR_STREAM("KinematicChainControllerBase: No tip name found on parameter server ("<<nh.getNamespace()<<"/tip_name)");
     //   return false;
    }
    // Construct an URDF model from the xml string
    std::string xml_string;

    if (nh.hasParam(robot_description))
        nh.getParam(robot_description.c_str(), xml_string);
    else
    {
        ROS_ERROR("Parameter %s not set, shutting down node...", robot_description.c_str());
        nh.shutdown();
       // return false;
    }

    if (xml_string.size() == 0)
    {
        ROS_ERROR("Unable to load robot model from parameter %s",robot_description.c_str());
        nh.shutdown();
        //return false;
    }
    boost::shared_ptr<KDL::ChainDynParam> dyn_param_;

    // Get urdf model out of robot_description
    urdf::Model model;
    if (!model.initString(xml_string))
    {
        ROS_ERROR("Failed to parse urdf file");
        nh.shutdown();
        //return false;
    }
    ROS_INFO("Successfully parsed urdf file");

    //KDL::Tree tree;
    if (!kdl_parser::treeFromUrdfModel(model, tree))
    {
        ROS_ERROR("Failed to construct kdl tree");
        nh.shutdown();
        //return false;
    }



    // Populate the KDL chain
    if(!tree.getChain(root_name, tip_name, chain))
    {
        ROS_ERROR_STREAM("Failed to get KDL chain from tree: ");
        ROS_ERROR_STREAM("  "<<root_name<<" --> "<<tip_name);
        ROS_ERROR_STREAM("  Tree has "<<tree.getNrOfJoints()<<" joints");
        ROS_ERROR_STREAM("  Tree has "<<tree.getNrOfSegments()<<" segments");
        ROS_ERROR_STREAM("  The segments are:");

        KDL::SegmentMap segment_map = tree.getSegments();
        KDL::SegmentMap::iterator it;

        for( it=segment_map.begin(); it != segment_map.end(); it++ )
          ROS_ERROR_STREAM( "    "<<(*it).first);

        return false;
    }

    ROS_INFO("tip_name:  %s",tip_name.c_str());
    ROS_INFO("root_name: %s",root_name.c_str());
    ROS_INFO("Number of segments: %d", chain.getNrOfSegments());
    ROS_INFO("Number of joints in chain: %d", chain.getNrOfJoints());
    for(std::size_t i = 0; i < chain.getNrOfSegments(); i++){
        ROS_INFO_STREAM("segment("<<i<<"): " << chain.getSegment(i).getName());
    }



//    // Parsing joint limits from urdf model along kdl chain
//    boost::shared_ptr<const urdf::Link> link_ = model.getLink(tip_name);
//    boost::shared_ptr<const urdf::Joint> joint_;
//    joint_limits_.min.resize(chain.getNrOfJoints());
//    joint_limits_.max.resize(chain.getNrOfJoints());
//    joint_limits_.center.resize(chain.getNrOfJoints());
//    int index;

//   for (std::size_t i = 0; i < chain.getNrOfJoints() && link_; i++)
//    {
//        joint_ = model.getJoint(link_->parent_joint->name);
//        ROS_INFO("Getting limits for joint: %s", joint_->name.c_str());
//        index = chain.getNrOfJoints() - i - 1;

//        if(joint_->limits){
//            joint_limits_.min(index) = joint_->limits->lower;
//            joint_limits_.max(index) = joint_->limits->upper;
//            joint_limits_.center(index) = (joint_limits_.min(index) + joint_limits_.max(index))/2;
//        }else{
//            joint_limits_.min(index) = 0;
//            joint_limits_.max(index) = 0;
//            joint_limits_.center(index) = 0;
//            ROS_INFO("joint_->limits is NULL %s",joint_->name.c_str());
//        }

//        link_ = model.getLink(link_->getParent()->name);
//    }
//    ROS_INFO("Getting joint handles");
//    // Get joint handles for all of the joints in the chain
//    int count=0;
//    for(std::vector<KDL::Segment>::const_iterator it = chain.segments.begin(); it != chain.segments.end(); ++it)
//    {

//        ROS_INFO("%s type: %s", it->getJoint().getName().c_str(),it->getJoint().getTypeName().c_str() );
//        if(it->getJoint().getTypeName() != "None" && count < 7){
//            joint_handles_.push_back(robot->getHandle(it->getJoint().getName()));
//        }
//        count++;
//    }

//    ROS_INFO("Number of joints in handle = %lu", joint_handles_.size() );
//    ROS_INFO_STREAM("kdl_chain.getNrOfJoints: " << chain.getNrOfJoints());


return true;
}


int main(int argc, char** argv)
{

    ros::init(argc, argv, "lwr_mpc_linear_node");
    ros::NodeHandle nh("lwr_mpc_linear_node");

    /* Node parameters */
    double control_rate = 100;
    ros::Rate loop_rate(control_rate);


    /* Publishers and subscribers */
    ros::Publisher ff_fb_pub  = nh.advertise<lwr_controllers::FF_FB_plan>("/lwr/joint_controllers/command_ff_fb_plan", 1);
    ros::Subscriber  pose_sub     = nh.subscribe("/lwr/ee_pose",5,pose_callback);
    ros::Subscriber  vel_sub     = nh.subscribe("/lwr/ee_vel",5,velocity_callback);
    ros::Subscriber joint_state_sub = nh.subscribe("/lwr/joint_states", 5, joint_state_callback);

    /* Iterative solver parameters */
    unsigned int time_horizon = 50;
    unsigned int  x_dim = 6, u_dim = 3; // 3d position + velocity, 3d force at the end effector
    double sampling_time = 1/100.0;
    // IterativeLQSolverParams( minLineSearchParameter, initialLineSearchParameter, minRelativeCostImprovement, maxNumberIterations )
    motion::IterativeLQSolverParams sparams(0.00000001, 1, 0.0001, 10);
    ros::Time solution_time;

    /* Allocate/Initialize message variables */
    lwr_controllers::FF_FB_plan             ff_fb_plan;
    ff_fb_plan.times.resize(time_horizon);
    ff_fb_plan.ff.resize(time_horizon);
    ff_fb_plan.fb.resize(time_horizon);
    ff_fb_plan.xd.resize(time_horizon);
    ff_fb_plan.xd_dot.resize(time_horizon);
    motion::Matrix fb_matrix(6,12);
    fb_matrix.setZero();


    KDL::Vector gravity = KDL::Vector::Zero();
    gravity(2) = -9.81;

    KDL::Tree tree;
    KDL::Chain chain;
    initialize_robot_model(nh, tree, chain);
    KDL::ChainDynParam dyn_param(chain, gravity);


    /**             PROBLEM DEFINITION:
     *
     *  - Robot dynamics (3D cartesian admittance): M x_r_ddot + D x_r_dot = u
     *
     *          Robot state (2nd order) -> x_rob =  [x_r^T x_r_dot^T]^T;    --> Linear dynamics    x_rob_dot = A_rob x_rob + B_rob u;
     *
     *  - Goal dynamics: x_d_dot = A_d(x_d_center - x_rob)
     *
     *  - Cost:  e^T Q_track_f e + sum_0^{time_horizon} e^T Q_track e + u^T R u
     *
     *               where e = (x_d- x_rob)
     *
     *    -- For convenience, we solve the problem with an augmented state x = [x_rob^T e^T]^T --
     *
     * - Initial state: x_initial = [x_rob^T  0^T]^T
     *
    **/

    /*** DYNAMICS ***/
    // Robot dynamic parameters M x_r_ddot + D x_r_dot = u
    motion::Matrix mass(x_dim/2, x_dim/2), damping(x_dim/2,x_dim/2), massInverse(x_dim/2, x_dim/2);
    mass = motion::Matrix::Identity(x_dim/2, x_dim/2)* 10;
    damping = motion::Matrix::Identity(x_dim/2, x_dim/2)* 0.1;
    massInverse = mass.inverse();

    // Robot dynamics x_rob_dot = A_rob x_rob + B_rob u;
    motion::Matrix A_rob(x_dim,x_dim), B_rob(x_dim,u_dim), A_d(x_dim,x_dim);
    A_rob.setZero();
    A_rob.topRightCorner(x_dim/2, x_dim/2).setIdentity();
    A_rob.bottomRightCorner(x_dim/2, x_dim/2) << - damping * massInverse;
    B_rob << motion::Matrix::Zero(x_dim/2, x_dim/2) , massInverse;

    // Goal dynamics  x_d_dot = A_d(x_d_center - x_rob)
    A_d.setZero();
    //A_d.topLeftCorner(x_dim/2,x_dim/2) << -0.0915, 4.9308, 0, -3.0534, -0.2027, 0, 0, 0, 1;
    A_d.topLeftCorner(x_dim/2,x_dim/2) << 10, 0, 0, 0, 10, 0, 0, 0, 10;
    motion::Vector x_d_center(x_dim);
    x_d_center << -0.3,0.3,0.7,0,0,0;


    /*** COST ***/
    // Cost parameters
    motion::Matrix Q_track(x_dim, x_dim), Q_track_f(x_dim, x_dim), R(u_dim, u_dim);
    Q_track = motion::Matrix::Identity(x_dim, x_dim) * 5000;
    Q_track_f = Q_track;
    R = motion::Matrix::Identity(u_dim, u_dim) * 1;


    /*** INITIAL STATE ***/
    // Note that the error part of the augmented state x = [x_rob^T e^T]^T at the beginning is always 0
    motion::Vector x_initial(x_dim*2);
    x_initial.setZero();


    /***PROBLEM AND SOLVER***/
    PointMassLinear pointMassLinearProblem(time_horizon, A_rob, B_rob, A_d, x_d_center, Q_track, Q_track_f, R); // Define problem
    motion::LQSolution lqsol(time_horizon, x_dim*2, u_dim);     // Initial nominal trajectory (0 trajectory)
    motion::IterativeLQSolver iterative_solver(time_horizon, x_dim*2, u_dim, sampling_time,
                                               pointMassLinearProblem, sparams, x_initial, lqsol);     // Define iterative solver

    ROS_INFO("starting MPC loop...");

    while(ros::ok()) {
        KDL::JntSpaceInertiaMatrix js_inertia;
        js_inertia.resize(7);
        dyn_param.JntToMass(joint_pos, js_inertia);
        std::cout<<js_inertia.data<<std::endl;
        //std::cout<<"klas"<<std::endl;

        // Update initial state
        x_initial.head(x_dim) <<  pose.getOrigin().x() , pose.getOrigin().y() , pose.getOrigin().z() ,
                x_dot.linear.x , x_dot.linear.y , x_dot.linear.z;
        x_initial.tail(x_dim).setZero();
        iterative_solver.setInitialState(x_initial);

        // Get time
        solution_time = ros::Time::now();
        // Solve problem
        iterative_solver.solve(lqsol);

        // Populate ff_fb_plan message
        for (size_t i=0 ; i < time_horizon-1 ; i++) {
            // Time stamp
            ff_fb_plan.times[i].data = solution_time + ros::Duration((double)(i*sampling_time));
            ff_fb_plan.ff[i].force.x = lqsol.ff[i](0);
            ff_fb_plan.ff[i].force.y = lqsol.ff[i](1);
            ff_fb_plan.ff[i].force.z = lqsol.ff[i](2);
            ff_fb_plan.ff[i].torque.x = 0;
            ff_fb_plan.ff[i].torque.x = 0;
            ff_fb_plan.ff[i].torque.x = 0;
            ff_fb_plan.xd[i].position.x = lqsol.x[i](0);
            ff_fb_plan.xd[i].position.y = lqsol.x[i](1);
            ff_fb_plan.xd[i].position.z = lqsol.x[i](2);
            ff_fb_plan.xd_dot[i].linear.x = lqsol.x[i](3);
            ff_fb_plan.xd_dot[i].linear.y = lqsol.x[i](4);
            ff_fb_plan.xd_dot[i].linear.z = lqsol.x[i](5);
            fb_matrix.block<3,3>(0,0) << lqsol.K[i].block<3,3>(0,0);
            fb_matrix.block<3,3>(0,6) << lqsol.K[i].block<3,3>(0,6);
            tf::matrixEigenToMsg(fb_matrix, ff_fb_plan.fb[i]);
        }

        ff_fb_pub.publish(ff_fb_plan);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}

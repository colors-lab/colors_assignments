#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseArray.h"

#include "kdl/chain.hpp"
#include "kdl/chainfksolver.hpp"
#include "kdl/chainfksolverpos_recursive.hpp"
#include "kdl/chainiksolverpos_lma.hpp"
#include "kdl/chainiksolvervel_pinv_givens.hpp"
#include "kdl/chainiksolverpos_nr_jl.hpp"
#include "kdl/frames_io.hpp"

#include <vector>

#define UR10_DOF 6

geometry_msgs::Pose calculatedPosition;

void endEffectorPositionCallback(const geometry_msgs::PoseStampedConstPtr& endEffectorPositionMessage) {
    std::cout << "End Effector Position :" << "[ "<< endEffectorPositionMessage->pose.position.x << " , " << endEffectorPositionMessage->pose.position.y << " , " << endEffectorPositionMessage->pose.position.z << " ]" << std::endl;
}

KDL::Chain initChainUR10() {
    KDL::Chain chain;
    
    chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),KDL::Frame(KDL::Rotation::RPY(0,-KDL::PI/2,0), KDL::Vector(	-0.09265011549,	0,	0.0833499655128	))));
    chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),KDL::Frame(KDL::Rotation::RPY(0,0,0), KDL::Vector(	0.612088978291,	0,	0.0196119993925	))));
    chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),KDL::Frame(KDL::Rotation::RPY(0,0,0), KDL::Vector(	0.572199583054,	0,	-0.00660470128059	))));
    chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),KDL::Frame(KDL::Rotation::RPY(0,KDL::PI/2,0), KDL::Vector(	0.0572912693024,	0,0.0584142804146	))));
    chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),KDL::Frame(KDL::Rotation::RPY(0,-KDL::PI/2,0), KDL::Vector(	-0.0573025941849,	0,0.0583996772766))));
    chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),KDL::Frame(KDL::Rotation::RPY(0,0,0), KDL::Vector(	0,	0,	0.110513627529	))));

    return chain;
}

int main(int argc, char **argv) {
    ros::init(argc,argv,"UR10_Main");

    ros::NodeHandle node;

    ros::Publisher joints[6];
    joints[0] = node.advertise<std_msgs::Float32>("/UR10/jointAngles/joint001",1000);
    joints[1] = node.advertise<std_msgs::Float32>("/UR10/jointAngles/joint002",1000);
    joints[2] = node.advertise<std_msgs::Float32>("/UR10/jointAngles/joint003",1000);
    joints[3] = node.advertise<std_msgs::Float32>("/UR10/jointAngles/joint004",1000);
    joints[4] = node.advertise<std_msgs::Float32>("/UR10/jointAngles/joint005",1000);
    joints[5] = node.advertise<std_msgs::Float32>("/UR10/jointAngles/joint006",1000);

    for(int index=0;index<UR10_DOF;index++){
        std_msgs::Float32 jointMessage;
        jointMessage.data = 0;
        joints[index].publish(jointMessage);
    }

    ros::Rate loop(10);
    ros::Subscriber endEffectorPositionSubscriber = node.subscribe<geometry_msgs::PoseStamped>("/UR10/positions/endEffectorPosition",1000,endEffectorPositionCallback);
    KDL::Chain chain = initChainUR10();
    unsigned int nj = chain.getNrOfJoints();
    while(ros::ok()) {
        KDL::ChainFkSolverPos_recursive fksolver = KDL::ChainFkSolverPos_recursive(chain);
        unsigned int nj = chain.getNrOfJoints();
        KDL::JntArray jointpositions = KDL::JntArray(nj);

        for(unsigned int i=0;i<6;i++){
            jointpositions(i)=(double)10 * KDL::PI/180; // You can assign angles to go like this.
        }

        for(unsigned int i=0;i<6;i++){        // Sending joint angles to robot.
            std_msgs::Float32 jointMessage;
            jointMessage.data = 10*KDL::PI/180;
            joints[i].publish(jointMessage);
        }

        // Create the frame that will contain the results
        KDL::Frame cartpos;

        // Calculate forward position kinematics
        bool kinematics_status;
        kinematics_status = fksolver.JntToCart(jointpositions,cartpos);
        if(kinematics_status>=0){
            calculatedPosition.position.x=(double)cartpos.p.x();
            calculatedPosition.position.y=(double)cartpos.p.y();
            calculatedPosition.position.z=(double)cartpos.p.z();
        }
        // You can now follow end effector position and real position in terminal 
        
        /*
        You can check KDL::ChainIkSolverPos_LMA for inverse kinematic solver.
        You can use this matrix for ChainIkSolverPos_LMA if you'll use that.
        Eigen::Matrix<double,6,1> L;
        L(0)=1;L(1)=1;L(2)=1;
        L(3)=0.01;L(4)=0.01;L(5)=0.01;
        
        
        goalEndEffectorPosition = KDL::Frame(
            KDL::Rotation::RPY(0,0,0),
            KDL::Vector(0,0,0)
        );
        */

        ros::spinOnce();
        loop.sleep();
        
        
    }

    return 0;
}

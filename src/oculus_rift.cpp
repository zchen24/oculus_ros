// Zihan Chen
// 2014-11-23

#include <iostream>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>
#include "oculus_rift.h"

namespace oculus_ros
{


OculusRift::OculusRift(ros::NodeHandle& node):
    mHmd(NULL),
    mParentFrame("world"),
    mOculusFrame("oculus")
{
    ROS_INFO("Oculus Rift Created");

    // ros setup
    ros::NodeHandle privateNode("~");
    privateNode.getParam("parent_frame", mParentFrame);
    privateNode.getParam("oculus_frame", mOculusFrame);

//    mPubHmd = node.advertise<>("oculus/hmd_info",10);
    mPubPose = node.advertise<geometry_msgs::PoseStamped>("oculus/head_pose_stamped", 10);
}


OculusRift::~OculusRift()
{
//    ROS_INFO("Oculus Rift Cleanup");
    std::cerr << "Oculus Rift Cleanup" << std::endl;
    ovrHmd_Destroy(mHmd);
    ovr_Shutdown();
}


bool OculusRift::Init()
{
    ovr_Initialize();
    ROS_INFO_STREAM("Detected " << ovrHmd_Detect() << " Oculus Rifts");

    mHmd = ovrHmd_Create(0);
    if (mHmd == NULL) {
        ROS_WARN("No Oculus Device Detected, Creating Virtual Hmd");
        mHmd = ovrHmd_CreateDebug(ovrHmd_DK2);
    }

    // Show info
    ROS_INFO(mHmd->ProductName);
    ROS_INFO_STREAM("TrackingCaps = 0x" << std::hex << mHmd->TrackingCaps);

    InitTracking();
}


void OculusRift::UpdateSensors()
{
    ros::Time now = ros::Time::now();

    // Query current tracking state
    ovrTrackingState ts = ovrHmd_GetTrackingState(mHmd, ovr_GetTimeInSeconds());
    if (ts.StatusFlags & (ovrStatus_OrientationTracked | ovrStatus_PositionTracked))
    {
        ovrPosef pose = ts.HeadPose.ThePose;

        // publish msg
        geometry_msgs::PoseStamped poseMsg;
        poseMsg.header.stamp = now;
        poseMsg.pose.position.x = pose.Position.x;
        poseMsg.pose.position.y = pose.Position.y;
        poseMsg.pose.position.z = pose.Position.z;
        poseMsg.pose.orientation.x = pose.Orientation.x;
        poseMsg.pose.orientation.y = pose.Orientation.y;
        poseMsg.pose.orientation.z = pose.Orientation.z;
        poseMsg.pose.orientation.w = pose.Orientation.w;
        mPubPose.publish(poseMsg);

        // tf
//        tf::tran
        tf::Transform transform;
        transform.setOrigin(tf::Vector3(pose.Position.x,
                                        pose.Position.y,
                                        pose.Position.z));
        transform.setRotation(tf::Quaternion(pose.Orientation.x,
                                             pose.Orientation.y,
                                             pose.Orientation.z,
                                             pose.Orientation.w));

        static tf::TransformBroadcaster br;
        br.sendTransform(tf::StampedTransform(transform,
                                              now,
                                              mParentFrame,
                                              mOculusFrame));
    }
}

bool OculusRift::InitTracking(void)
{
    // Configure Tracking
    unsigned int supportedTrackingCaps, requiredTrackingCaps;
    supportedTrackingCaps =
            ovrTrackingCap_Orientation |
            ovrTrackingCap_MagYawCorrection |
            ovrTrackingCap_Position;
//    requiredTrackingCaps = supportedTrackingCaps;
    requiredTrackingCaps = 0;
    ovrBool ret = ovrHmd_ConfigureTracking(mHmd, supportedTrackingCaps, requiredTrackingCaps);
    if (ret) {
        ROS_INFO("Oculus Tracking Initialized");
        return true;
    }
    else {
        ROS_WARN("Failed to Configure Oculus");
        return false;
    }
}

}

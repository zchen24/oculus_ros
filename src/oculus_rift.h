// Zihan Chen
// 2014-11-23

#ifndef OCULUS_RIFT_H
#define OCULUS_RIFT_H

#include <ros/ros.h>

#include <OVR.h>
#include <OculusSDK/Src/OVR_Profile.h>
#include <OculusSDK/Src/OVR_CAPI_Keys.h>


namespace oculus_ros
{
class OculusRift
{
public:
    OculusRift(ros::NodeHandle& node);
    virtual ~OculusRift();

    bool Init();
    void UpdateSensors();

    const ovrHmdDesc* GetHmd(){
        return mHmd;
    }

protected:
    bool InitTracking(void);

    const ovrHmdDesc* mHmd;
    std::string mParentFrame;
    std::string mOculusFrame;

    ros::Publisher mPubHmd;
    ros::Publisher mPubPose;
};

}


#endif // OCULUS_RIFT_H

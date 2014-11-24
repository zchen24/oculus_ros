// Zihan Chen
// 2014-11-23

#ifndef OCULUS_SHADER_H
#define OCULUS_SHADER_H

#include <opencv2/opencv.hpp>
#include <GL/gl.h>
#include <OVR.h>
#include <sensor_msgs/Image.h>
#include <ros/ros.h>
#include <image_transport/image_transport.h>

namespace oculus_ros
{

class OculusShader
{
public:
    static OculusShader* Instance(ros::NodeHandle &node);

    // Callback Function
    static void CallbackKeyFuncWrapper(unsigned char key, int x, int y);
    static void CallbackRenderWrapper(void);
    static void CallbackOnIdleWrapper(void);
    static void CallbackTimer(int value);

    void RosCallbackImageLeft(const sensor_msgs::ImageConstPtr& msg);
    void RosCallbackImageRight(const sensor_msgs::ImageConstPtr& msg);

private:
    // private
    OculusShader(ros::NodeHandle& node);
    OculusShader(const OculusShader&){}
    OculusShader& operator =(const OculusShader&){}
    virtual ~OculusShader();

    // Render Function
    void CallbackRender(void);
    void CallbackOnIdle(void);
    void CallbackKeyFunc(unsigned char key, int x, int y);


    GLuint makeBuffer(GLenum target,
                             const void *buffer_data,
                             GLsizei buffer_size);
    GLuint makeTexture(const char *filename);
    GLuint makeShader(GLenum type, const char *filename);
    GLuint makeProgram(GLuint vertex_shader, GLuint fragment_shader);
    void* fileContents(const char *filename, GLint *length);


    // Instance
    static OculusShader* mInstance;
    cv::VideoCapture mCap;
    cv::Mat mVideoImage;

    const ovrHmdDesc* mHmd;

    GLuint mVertexBuffer, mElementBuffer;
    GLuint mTexture;
    GLuint mVertexShader, mFragmentShader;
    GLuint mProgram;

    // Shader related stuff
    GLuint mOculusVertexBuffer[2];
    GLuint mOculusTex0[2];
    GLuint mOculusTex1[2];
    GLuint mOculusTex2[2];
    GLuint mOculusVignetteBuffer[2];
    GLuint mOculusElementBuffer[2];
    GLsizei mOculusElementCount[2];
    GLfloat mEyeScale[2][2];
    GLfloat mEyeOffset[2][2];

    // "pointer" to GPU stuff
    struct {
        GLint fadeFactorLoc;
        GLint textureLoc;
        GLint eyeToSourceUVScaleLoc;
        GLint eyeToSourceUVOffsetLoc;
    } mUniforms;

    struct {
        GLint positionLoc;
        GLint vignetteLoc;
        GLint texcoord0Loc;
        GLint texcoord1Loc;
        GLint texcoord2Loc;
    } mAttributes;

    GLfloat mFadeFactor;

    image_transport::Subscriber  subLeft;
    image_transport::Subscriber  subRight;
};

}

#endif  // OCULUS_SHADER_H

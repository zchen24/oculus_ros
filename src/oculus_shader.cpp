// Zihan Chen
// 2014-11-23


#include <iostream>
#include <GL/glew.h>
#include <GL/glut.h>
#include <ros/package.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include "oculus_shader.h"

namespace oculus_ros
{

OculusShader* OculusShader::mInstance = NULL;

OculusShader* OculusShader::Instance(ros::NodeHandle& node)
{
    if (!mInstance) {
        mInstance = new OculusShader(node);
    }
    return mInstance;
}


OculusShader::OculusShader(ros::NodeHandle &node)
{
    // ------ Open CV webcam ----------
//    mCap.open(0);
//    if (!mCap.isOpened()) {
//        ROS_ERROR("Failed to open webcam");
//    }

    image_transport::ImageTransport it(node);
    std::string leftTopic = "/image_raw";
    std::string rightTopic = "/image_raw";
    subLeft = it.subscribe(leftTopic, 1,
                           &OculusShader::RosCallbackImageLeft, this);
    subRight = it.subscribe(rightTopic, 1,
                            &OculusShader::RosCallbackImageRight, this);

//    subLeft = node.subscribe(leftTopic, 1,
//                             &OculusShader::RosCallbackImageLeft, this);


    // make shader
    std::string pkgPath = ros::package::getPath("oculus_ros");
    std::string vertexShaderFname, fragShaderFname;
    vertexShaderFname = pkgPath + "/shader/oculus.v.glsl";
    fragShaderFname = pkgPath + "/shader/oculus.f.glsl";

    std::cerr << vertexShaderFname << std::endl;
    std::cerr << fragShaderFname << std::endl;

    mVertexShader = makeShader(GL_VERTEX_SHADER, vertexShaderFname.c_str());
    mFragmentShader = makeShader(GL_FRAGMENT_SHADER, fragShaderFname.c_str());

    if (mVertexShader == 0 | mFragmentShader == 0)
        std::cerr << "shaders failed to load" << std::endl;

    mProgram = makeProgram(mVertexShader, mFragmentShader);
    if (mProgram == 0)
        std::cerr << "Failed to create program" << std::endl;


    // ----------------- Oculus --------------------
    ovr_Initialize();
    mHmd = ovrHmd_Create(0);
    if (mHmd == NULL) {
        ROS_WARN("No Oculus Device Detected, Creating Virtual Hmd");
        mHmd = ovrHmd_CreateDebug(ovrHmd_DK2);
    }

    //------- Basically Update Mesh Stuff ----------

    ovrEyeRenderDesc mEyeRenderDesc[2];
    ovrFovPort mEyeFov[2];
    ovrRecti mEyeRenderViewport[2];
    ovrSizei mRenderSize;
    ovrVector2f mUVScaleOffset[2][2];


    // Initialize ovrEyeRenderDesc struct.
    mRenderSize.w = 800;
    mRenderSize.h = 450;

    mEyeFov[0] = mHmd->DefaultEyeFov[0];
    mEyeFov[1] = mHmd->DefaultEyeFov[1];

    mEyeRenderViewport[0].Pos.x = 0;
    mEyeRenderViewport[0].Pos.y = 0;
    mEyeRenderViewport[0].Size.w = mRenderSize.w/2;
    mEyeRenderViewport[0].Size.h = mRenderSize.h;
    mEyeRenderViewport[1].Pos.x = (mRenderSize.w + 1)/2;
    mEyeRenderViewport[1].Pos.y = 0;
    mEyeRenderViewport[1].Size = mEyeRenderViewport[0].Size;

    mEyeRenderDesc[0] = ovrHmd_GetRenderDesc(mHmd, ovrEye_Left, mEyeFov[0]);
    mEyeRenderDesc[1] = ovrHmd_GetRenderDesc(mHmd, ovrEye_Right, mEyeFov[1]);


    for (size_t eyeNum = 0; eyeNum < 2; eyeNum++)
    {
        ovrDistortionMesh meshData;
        unsigned int distortionCaps = 0;
        //        distortionCaps |= ovrDistortionCap_Vignette;
        //        distortionCaps |= ovrDistortionCap_Chromatic;

        // create hmd
        ovrHmd_CreateDistortionMesh(mHmd,
                                    mEyeRenderDesc[eyeNum].Eye,
                                    mEyeRenderDesc[eyeNum].Fov,
                                    distortionCaps,
                                    &meshData);

        // allocate & generate distortion mesh vertices
        ovrHmd_GetRenderScaleAndOffset(mEyeRenderDesc[eyeNum].Fov,
                                       mRenderSize,
                                       mEyeRenderViewport[eyeNum],
                                       mUVScaleOffset[eyeNum]);

        mEyeScale[eyeNum][0] = mUVScaleOffset[eyeNum][0].x;
        mEyeScale[eyeNum][1] = mUVScaleOffset[eyeNum][0].y;
        mEyeOffset[eyeNum][0] = mUVScaleOffset[eyeNum][1].x;
        mEyeOffset[eyeNum][1] = mUVScaleOffset[eyeNum][1].y;

        // save data here
        //        GLfloat Position[2][meshData.VertexCount];
        GLfloat Position[meshData.VertexCount][2];
        GLfloat Texcoord0[meshData.VertexCount][2];
        GLfloat Texcoord1[meshData.VertexCount][2];
        GLfloat Texcoord2[meshData.VertexCount][2];

        GLfloat VignetteFactor[meshData.VertexCount];
        ovrDistortionVertex* ov = meshData.pVertexData;
        for (size_t i = 0; i < meshData.VertexCount; i++)
        {
            Position[i][0] = ov->ScreenPosNDC.x;
            Position[i][1] = ov->ScreenPosNDC.y;

            Texcoord0[i][0] = ov->TanEyeAnglesR.x;
            Texcoord0[i][1] = ov->TanEyeAnglesR.y;

            Texcoord1[i][0] = ov->TanEyeAnglesG.x;
            Texcoord1[i][1] = ov->TanEyeAnglesG.y;

            Texcoord2[i][0] = ov->TanEyeAnglesB.x;
            Texcoord2[i][1] = ov->TanEyeAnglesB.y;

            //            VignetteFactor[i] = ov->VignetteFactor;
            VignetteFactor[i] = 0.8;

            //            unsigned char factor = ov->VignetteFactor * 259.99;
            //            std::cout << "vignette = " << (int)(unsigned char)(ov->VignetteFactor * 259.99) << std::endl;

            ov++;
        }

        // CHANGE SHIT HERE

        // vertex buffer
        glGenBuffers(1, &mOculusVertexBuffer[eyeNum]);
        glBindBuffer(GL_ARRAY_BUFFER, mOculusVertexBuffer[eyeNum]);
        glBufferData(GL_ARRAY_BUFFER,
                     sizeof(float) * 2 * meshData.VertexCount,
                     Position,
                     GL_STATIC_DRAW);

        // texcoord0
        glGenBuffers(1, &mOculusTex0[eyeNum]);
        glBindBuffer(GL_ARRAY_BUFFER, mOculusTex0[eyeNum]);
        glBufferData(GL_ARRAY_BUFFER,
                     sizeof(float) * 2 * meshData.VertexCount,
                     Texcoord0,
                     GL_STATIC_DRAW);

        // texcoord1
        glGenBuffers(1, &mOculusTex1[eyeNum]);
        glBindBuffer(GL_ARRAY_BUFFER, mOculusTex1[eyeNum]);
        glBufferData(GL_ARRAY_BUFFER,
                     sizeof(float) * 2 * meshData.VertexCount,
                     Texcoord1,
                     GL_STATIC_DRAW);

        // texcoord2
        glGenBuffers(1, &mOculusTex2[eyeNum]);
        glBindBuffer(GL_ARRAY_BUFFER, mOculusTex2[eyeNum]);
        glBufferData(GL_ARRAY_BUFFER,
                     sizeof(float) * 2 * meshData.VertexCount,
                     Texcoord2,
                     GL_STATIC_DRAW);


        // vignette buffer
        glGenBuffers(1, &mOculusVignetteBuffer[eyeNum]);
        glBindBuffer(GL_ARRAY_BUFFER, mOculusVignetteBuffer[eyeNum]);
        glBufferData(GL_ARRAY_BUFFER,
                     sizeof(GLfloat) * meshData.VertexCount,
                     VignetteFactor,
                     GL_STATIC_DRAW);

        // index element buffer
        glGenBuffers(1, &mOculusElementBuffer[eyeNum]);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, mOculusElementBuffer[eyeNum]);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER,
                     sizeof(unsigned short) * meshData.IndexCount,
                     meshData.pIndexData,
                     GL_STATIC_DRAW);
        mOculusElementCount[eyeNum] = meshData.IndexCount;

        ovrHmd_DestroyDistortionMesh(&meshData);
    }

    // ----------------------------------------------


    std::string texFname = pkgPath + "/shader/hello.png";
    mTexture = makeTexture(texFname.c_str());
    if (mTexture == 0) {
        std::cerr << "failed to make texture" << std::endl;
    }

    // get params loc
    glUseProgram(mProgram);
    mUniforms.eyeToSourceUVScaleLoc =
            glGetUniformLocation(mProgram, "EyeToSourceUVScale");
    mUniforms.eyeToSourceUVOffsetLoc =
            glGetUniformLocation(mProgram, "EyeToSourceUVOffset");
    mUniforms.fadeFactorLoc =
            glGetUniformLocation(mProgram, "fadeFactor");
    mUniforms.textureLoc =
            glGetUniformLocation(mProgram, "texture");

    mAttributes.positionLoc =
            glGetAttribLocation(mProgram, "Position");
    mAttributes.vignetteLoc =
            glGetAttribLocation(mProgram, "Vignette");
    mAttributes.texcoord0Loc =
            glGetAttribLocation(mProgram, "Texcoord0");
    mAttributes.texcoord1Loc =
            glGetAttribLocation(mProgram, "Texcoord1");
    mAttributes.texcoord2Loc =
            glGetAttribLocation(mProgram, "Texcoord2");

    mVideoImage = cv::Mat(800, 450, CV_8UC3);
}

OculusShader::~OculusShader()
{
    ovrHmd_Destroy(mHmd);
    ovr_Shutdown();
}


void OculusShader::CallbackRenderWrapper(void)
{
    mInstance->CallbackRender();
    //    std::cout << "render" << std::endl;
}

void OculusShader::CallbackOnIdleWrapper()
{
    mInstance->CallbackOnIdle();
}

void OculusShader::CallbackKeyFuncWrapper(unsigned char key, int x, int y)
{
    mInstance->CallbackKeyFunc(key, x, y);
    //    std::cout << "keyboard" << std::endl;
}


void OculusShader::CallbackTimer(int value)
{
//    ros::spinOnce();
//    std::cout << "ROS Ok() =  " << ros::ok()
//              << "  value = " << value << std::endl;
}


// -------- ROS Callback -------------
void OculusShader::RosCallbackImageLeft(const sensor_msgs::ImageConstPtr &msg)
{
    ROS_INFO("Left Image Callback");

    cv_bridge::CvImageConstPtr cvImg = cv_bridge::toCvShare(msg);

    unsigned int width, height;
    width = cvImg->image.cols;
    height = cvImg->image.rows;

    mVideoImage.adjustROI(0, 0, 0, -width);
    cvImg->image.copyTo(mVideoImage);
    mVideoImage.adjustROI(0, 0, 0, width);
}


void OculusShader::RosCallbackImageRight(const sensor_msgs::ImageConstPtr &msg)
{
    ROS_INFO("Right Image Callback");

    cv_bridge::CvImageConstPtr cvImg = cv_bridge::toCvShare(msg);

    unsigned int width, height;
    width = cvImg->image.cols;
    height = cvImg->image.rows;

    mVideoImage.adjustROI(0, 0, -width, 0);
    cvImg->image.copyTo(mVideoImage);
    mVideoImage.adjustROI(0, 0, width, 0);
}


// ---------------------------------------------
void OculusShader::CallbackRender()
{
    ovrFrameTiming frameTiming = ovrHmd_BeginFrameTiming(mHmd, 0);

    glUseProgram(mProgram);

    // fade factor
    glUniform1f(mUniforms.fadeFactorLoc, mFadeFactor);

    // for textures
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, mTexture);
    glUniform1i(mUniforms.textureLoc, 0);


    ovrEyeType eyeRenderOrder[2];
    eyeRenderOrder[0] = ovrEye_Left;
    eyeRenderOrder[1] = ovrEye_Right;


    for (size_t eyeNum = 0; eyeNum < 2; eyeNum++)
    {
        // ----- Eye Constants -------
        // get eyeReleated params from oculus rift

        // Update uniforms values
        glUniform2fv(mUniforms.eyeToSourceUVScaleLoc,
                     1, &mEyeScale[eyeNum][0]);
        glUniform2fv(mUniforms.eyeToSourceUVOffsetLoc,
                     1, &mEyeOffset[eyeNum][0]);

        //        std::cout << "Scale = " << mEyeScale[eyeNum][0] << " " << mEyeScale[eyeNum][1] << std::endl;
        //        std::cout << "Offse = " << mEyeOffset[eyeNum][0] << " " << mEyeOffset[eyeNum][1] << std::endl;

        // vertex position
        glBindBuffer(GL_ARRAY_BUFFER, mOculusVertexBuffer[eyeNum]);
        glVertexAttribPointer(
                    mAttributes.positionLoc,          /* attribute */
                    2,                                /* size */
                    GL_FLOAT,                         /* type */
                    GL_FALSE,                         /* normalized? */
                    sizeof(GLfloat)*2,                /* stride */
                    (void*)0                          /* array buffer offset */
                    );
        glEnableVertexAttribArray(mAttributes.positionLoc);

        // vignette
        glBindBuffer(GL_ARRAY_BUFFER, mOculusVignetteBuffer[eyeNum]);
        glVertexAttribPointer(
                    mAttributes.vignetteLoc,    // attribute
                    1,
                    GL_FLOAT,
                    GL_FALSE,
                    sizeof(GLfloat),
                    (void*)0
                    );
        glEnableVertexAttribArray(mAttributes.vignetteLoc);

        // texcoord0
        glBindBuffer(GL_ARRAY_BUFFER, mOculusTex0[eyeNum]);
        glVertexAttribPointer(
                    mAttributes.texcoord0Loc,    // attribute
                    2,
                    GL_FLOAT,
                    GL_FALSE,
                    sizeof(GLfloat)*2,
                    (void*)0
                    );
        glEnableVertexAttribArray(mAttributes.texcoord0Loc);

        glBindBuffer(GL_ARRAY_BUFFER, mOculusTex1[eyeNum]);
        glVertexAttribPointer(
                    mAttributes.texcoord1Loc,    // attribute
                    2,
                    GL_FLOAT,
                    GL_FALSE,
                    sizeof(GLfloat)*2,
                    (void*)0
                    );
        glEnableVertexAttribArray(mAttributes.texcoord1Loc);

        glBindBuffer(GL_ARRAY_BUFFER, mOculusTex2[eyeNum]);
        glVertexAttribPointer(
                    mAttributes.texcoord2Loc,    // attribute
                    2,
                    GL_FLOAT,
                    GL_FALSE,
                    sizeof(GLfloat)*2,
                    (void*)0
                    );
        glEnableVertexAttribArray(mAttributes.texcoord2Loc);

        // Draw
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, mOculusElementBuffer[eyeNum]);
        glDrawElements(
                    GL_TRIANGLE_STRIP,   /* mode */
                    mOculusElementCount[eyeNum], /* count */
                    GL_UNSIGNED_SHORT,   /* type */
                    (void*)0             /* element array buffer offset */
                    );

        glDisableVertexAttribArray(mAttributes.positionLoc);
        glDisableVertexAttribArray(mAttributes.vignetteLoc);
        glDisableVertexAttribArray(mAttributes.texcoord0Loc);
        glDisableVertexAttribArray(mAttributes.texcoord1Loc);
        glDisableVertexAttribArray(mAttributes.texcoord2Loc);
    }

    glutSwapBuffers();
    ovrHmd_EndFrameTiming(mHmd);
}


void OculusShader::CallbackOnIdle()
{
    int milliseconds = glutGet(GLUT_ELAPSED_TIME);
    mFadeFactor = sinf((float)milliseconds * 0.001f) * 0.5f + 0.5f;

#if 0
    // grep image and update texture from webcam;
    cv::Mat frame;
    mCap >> frame;
    //    cv::flip(frame, frame, 0);  // flip around x


    // Code Snippet to put two images side by side
    cv::Mat fullFrame(frame.rows, frame.cols * 2, CV_8UC3);
    fullFrame.adjustROI(0, 0, 0, -frame.cols);
    frame.copyTo(fullFrame);
    fullFrame.adjustROI(0, 0, -frame.cols, frame.cols);
    frame.copyTo(fullFrame);
    fullFrame.adjustROI(0, 0, frame.cols, 0);
#endif

    ros::spinOnce();

    // opencv version
    int width, height;
    width = mVideoImage.cols;
    height = mVideoImage.rows;
    glTexImage2D(
                GL_TEXTURE_2D, 0,           /* target, level */
                GL_RGB8,                    /* internal format */
                width, height, 0,           /* width, height, border */
                GL_BGR, GL_UNSIGNED_BYTE,   /* external format, type */
                mVideoImage.data            /* pixels */
                );

    glutPostRedisplay();
}


void OculusShader::CallbackKeyFunc(unsigned char key, int x, int y)
{
}


/*
 * Functions for creating OpenGL objects:
 */
GLuint OculusShader::makeBuffer(GLenum target,
                                const void *buffer_data,
                                GLsizei buffer_size)
{
    GLuint buffer;
    glGenBuffers(1, &buffer);
    glBindBuffer(target, buffer);
    glBufferData(target, buffer_size, buffer_data, GL_STATIC_DRAW);
    return buffer;
}


GLuint OculusShader::makeTexture(const char *filename)
{
    // opencv version
    int width, height;
    GLuint texture;
    cv::Mat image = cv::imread(filename);
    cv::flip(image, image, 0);  // flip around x
    width = image.cols;
    height = image.rows;

    glGenTextures(1, &texture);
    glBindTexture(GL_TEXTURE_2D, texture);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S,     GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T,     GL_CLAMP_TO_EDGE);
    glTexImage2D(
                GL_TEXTURE_2D, 0,           /* target, level */
                GL_RGB8,                    /* internal format */
                width, height, 0,           /* width, height, border */
                GL_BGR, GL_UNSIGNED_BYTE,   /* external format, type */
                image.data                  /* pixels */
                );

    return texture;
}


GLuint OculusShader::makeShader(GLenum type, const char *filename)
{
    GLint length;
    GLchar *source = (GLchar*)fileContents(filename, &length);
    GLuint shader;
    GLint shader_ok;

    if (!source)
        return 0;

    shader = glCreateShader(type);
    glShaderSource(shader, 1, (const GLchar**)&source, &length);
    free(source);
    glCompileShader(shader);

    glGetShaderiv(shader, GL_COMPILE_STATUS, &shader_ok);
    if (!shader_ok) {
        fprintf(stderr, "Failed to compile %s:\n", filename);

        ROS_ERROR_STREAM("Failed to compile " << filename);
        glDeleteShader(shader);
        return 0;
    }
    return shader;
}

GLuint OculusShader::makeProgram(GLuint vertex_shader, GLuint fragment_shader)
{
    GLint program_ok;

    GLuint program = glCreateProgram();

    glAttachShader(program, vertex_shader);
    glAttachShader(program, fragment_shader);
    glLinkProgram(program);

    glGetProgramiv(program, GL_LINK_STATUS, &program_ok);
    if (!program_ok) {
        fprintf(stderr, "Failed to link shader program:\n");
        //        show_info_log(program, glGetProgramiv, glGetProgramInfoLog);
        glDeleteProgram(program);
        return 0;
    }
    return program;
}

/*
 * Boring, non-OpenGL-related utility functions
 */

void* OculusShader::fileContents(const char *filename, GLint *length)
{
    FILE *f = fopen(filename, "r");
    void *buffer;

    if (!f) {
        fprintf(stderr, "Unable to open %s for reading\n", filename);
        return NULL;
    }

    fseek(f, 0, SEEK_END);
    *length = ftell(f);
    fseek(f, 0, SEEK_SET);

    buffer = malloc(*length+1);
    *length = fread(buffer, 1, *length, f);
    fclose(f);
    ((char*)buffer)[*length] = '\0';

    return buffer;
}



}  // namespace oculus_ros



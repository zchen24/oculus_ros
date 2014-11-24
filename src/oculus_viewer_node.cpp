#include <iostream>

#include <GL/glew.h>
#include <GL/glut.h>

#include <ros/ros.h>

#include "oculus_shader.h"


int main(int argc, char** argv)
{
    using namespace oculus_ros;
    ros::init(argc, argv, "oculus_viewer_node");
    ros::NodeHandle node;

    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE);
    glutInitWindowSize(800, 450);
    glutCreateWindow(ros::this_node::getName().c_str());
    std::cout << "OpenGL: " << glGetString(GL_VERSION) << std::endl;

    glewInit();
    if (!GLEW_VERSION_2_0) {
        fprintf(stderr, "OpenGL 2.0 not available\n");
        return 1;
    }

    // NOTE: put this after glewInit() !!!
    //   uses glew function internally
    OculusShader* instance = OculusShader::Instance(node);

    glutDisplayFunc(&oculus_ros::OculusShader::CallbackRenderWrapper);
    glutIdleFunc(&oculus_ros::OculusShader::CallbackOnIdleWrapper);
    glutKeyboardFunc(&oculus_ros::OculusShader::CallbackKeyFuncWrapper);
    glutTimerFunc(10, &oculus_ros::OculusShader::CallbackTimer, 99);

    glutMainLoop();

    std::cout << "quitting " << std::endl;
    return 0;
}

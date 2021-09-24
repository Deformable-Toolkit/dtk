/**
 * @file dtkGUIOpenGLbyGLAD.cpp
 * @author TOMsworkspace (2683322180@qq.com)
 * @brief dtk GUI by opengl with glad and glfw.
 * @version 1.0
 * @date 2021-09-24
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#include <glad/glad.h>
#include <GLFW/glfw3.h>

#include <iostream>

// GLFW function declarations
void framebuffer_size_callback(GLFWwindow* window, int width, int height);
void key_callback(GLFWwindow* window, int key, int scancode, int action, int mode);

void scene_init();
void ProcessInput(float deltaTime);
void Update(float delatTime);
void Render();

// The Width of the screen
const unsigned int SCREEN_WIDTH = 800;
// The height of the screen
const unsigned int SCREEN_HEIGHT = 600;

int main(int argc, char *argv[])
{
    glfwInit();
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
#ifdef __APPLE__
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
#endif
    glfwWindowHint(GLFW_RESIZABLE, false);

    GLFWwindow* window = glfwCreateWindow(SCREEN_WIDTH, SCREEN_HEIGHT, "Breakout", nullptr, nullptr);
    glfwMakeContextCurrent(window);

    // glad: load all OpenGL function pointers
    // ---------------------------------------
    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
    {
        std::cout << "Failed to initialize GLAD" << std::endl;
        return -1;
    }

    glfwSetKeyCallback(window, key_callback);
    glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);

    // OpenGL configuration
    // --------------------
    glViewport(0, 0, SCREEN_WIDTH, SCREEN_HEIGHT);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    // initialize scene
    // ---------------
    scene_init();

    // deltaTime variables
    // -------------------
    float deltaTime = 0.0f;
    float lastFrame = 0.0f;

    while (!glfwWindowShouldClose(window))
    {
        // calculate delta time
        // --------------------
        float currentFrame = glfwGetTime();
        deltaTime = currentFrame - lastFrame;
        lastFrame = currentFrame;
        glfwPollEvents();

        // manage user input
        // -----------------
        ProcessInput(deltaTime);

        // update game state
        // -----------------
        Update(deltaTime);

        // render
        // ------
        glClearColor(0x11 * 1.0 / 0xff, 0x2f * 1.0 / 0xff, 0x41 * 1.0 / 0xff, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);
        Render();

        glfwSwapBuffers(window);
    }

    glfwTerminate();
    return 0;
}

void scene_init(){
    //TODO: init your scene here

}

void ProcessInput(float deltaTime){
    //TODO: process or pass input to scene here.
}

void Update(float delatTime){
    //TODO: update scene here.
}

void Render(){
    //TODO: render scene here.
}


void key_callback(GLFWwindow* window, int key, int scancode, int action, int mode)
{
    // when a user presses the escape key, we set the WindowShouldClose property to true, closing the application
    if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS)
        glfwSetWindowShouldClose(window, true);
        
    if (key >= 0 && key < 1024)
    {
        if (action == GLFW_PRESS)
            Breakout.Keys[key] = true;
        else if (action == GLFW_RELEASE)
        {
            Breakout.Keys[key] = false;
            Breakout.KeysProcessed[key] = false;
        }
    }
    
}

void framebuffer_size_callback(GLFWwindow* window, int width, int height)
{
    // make sure the viewport matches the new window dimensions; note that width and 
    // height will be significantly larger than specified on retina displays.
    glViewport(0, 0, width, height);
}


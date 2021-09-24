/**
 * @file dtkGUIOpenGLbyFreeGLUT.cpp
 * @author TOMsworkspace (2683322180@qq.com)
 * @brief dtk GUI by opengl with FreeGLUT.
 * @version 1.0
 * @date 2021-09-13
 * 
 * @copyright Copyright (c) 2021
 */

#include "GL/freeglut.h"
#include <chrono>
#include <cmath>
#include <iostream>

void scene_init();
void ProcessInput(float deltaTime);
void Update(float delatTime);
void Render();

static void draw_text(int x, int y, const char *format, ...);
void display();
void reshape(int width, int height);
void mouse(int button, int state, int x, int y);
void keyboard(unsigned char key, int x, int y);
void motion(int x, int y);
void special(int key, int x, int y);

static auto last_clock = std::chrono::high_resolution_clock::now();

// The Width of the screen
const unsigned int SCREEN_WIDTH = 800;
// The height of the screen
const unsigned int SCREEN_HEIGHT = 600;

int main(int argc, char *argv[]) {
    glutInit(&argc, argv);
    glutInitWindowSize(800, 600);
    glutInitWindowPosition(50, 50);
    glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE);
    glutCreateWindow("Physics Engine -- dtk");

    scene_init();
    glutDisplayFunc(&display);
    glutReshapeFunc(&reshape);
    glutMouseFunc(&mouse);
    glutMotionFunc(&motion);
    glutSpecialFunc(&special);
    glutKeyboardFunc(&keyboard);
    glutIdleFunc(&idle);
    glutSetOption(GLUT_ACTION_ON_WINDOW_CLOSE, GLUT_ACTION_CONTINUE_EXECUTION);
    glutMainLoop();
    return 0;
}

void scene_init(){
    //TODO: init your scene here
}

void ProcessInput(){
    //TODO: process or pass input to scene here.
}

void Update(float delatTime){
    //TODO: update scene here.
}

void Render(){
    //TODO: render scene here.
}

static void draw_text(int x, int y, const char *format, ...) {
    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    glLoadIdentity();
    int w = glutGet(GLUT_WINDOW_WIDTH);
    int h = glutGet(GLUT_WINDOW_HEIGHT);
    gluOrtho2D(0, w, h, 0);
    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glLoadIdentity();

    glColor3f(0.9f, 0.9f, 0.9f);
    glRasterPos2i(x, y);

    char buffer[256];
    va_list args;
    va_start(args, format);
    int len = vsprintf(buffer, format, args);
    va_end(args);
    for (int i = 0; i < len; ++i) {
        glutBitmapCharacter(GLUT_BITMAP_9_BY_15, buffer[i]);
    }

    glPopMatrix();
    glMatrixMode(GL_PROJECTION);
    glPopMatrix();
}

void display() {
    glClearColor(0x11 * 1.0 / 0xff, 0x2f * 1.0 / 0xff, 0x41 * 1.0 / 0xff, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    glTranslatef(-0.5f, -0.55f, -1.0f);

    auto now = std::chrono::high_resolution_clock::now();
    auto dt = std::chrono::duration_cast<std::chrono::duration<double>>(now - last_clock).count();
    last_clock = now;

    int h = glutGet(GLUT_WINDOW_HEIGHT);
    int w = glutGet(GLUT_WINDOW_WIDTH);
    
    Update(dt);
    Render();

    glutSwapBuffers();
}

void reshape(int width, int height) {
    glViewport(0, 0, width, height);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(45.0, width / (float) height, 0.1, 100.0);
}

void mouse(int button, int state, int x, int y) {
    //TODO: mouse pressed/released event.
    ProcessInput();
}

void keyboard(unsigned char key, int x, int y) {
    //TODO: keyboard event.
    ProcessInput();
}

void motion(int x, int y) {
    //TODO: mouse moved event.
    ProcessInput()
}

void special(int key, int x, int y) {

}

void idle() {
    display();
}
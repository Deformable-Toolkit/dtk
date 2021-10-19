/**
 * @file main.cpp
 * @author TOMsworkspace (2683322180@qq.com)
 * @brief 
 * @version 1.0
 * @date 2021-09-13
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#include "GL/freeglut.h"
#include <chrono>
#include <cmath>
#include <iostream>

#include "dtkFemSimulation.h"

static auto last_clock = std::chrono::high_resolution_clock::now();

// The Width of the screen
const unsigned int SCREEN_WIDTH = 800;
// The height of the screen
const unsigned int SCREEN_HEIGHT = 600;

dtkFemSimulation Breakout(SCREEN_WIDTH, SCREEN_HEIGHT);

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

static int pcount = 1;
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

    draw_text(5, 15, "dtk @Deformation FEM simulation");

    draw_text(5, 30, "Method: Semi-implict Euler");
    //draw_text(5, 45, "Time step: %.6f", 1.0 / 32);
    draw_text(5,45, "Poisson's ratio: %.2f", 0.3f);
    draw_text(5, 60, "Young's modulus: %.2f", 1000.0f);
    draw_text(5, 75, "Energy : %.2f", Breakout.getEnergy());

    //draw_text(5, 40, "Push [1-5] to switch scene");
    //draw_text(w - 150, h - 20, "refer: apollonia");

    if (Breakout.State == SCENE_PAUSE)
        draw_text(5, h - 20, "dt: %.2f ms (%.2F FPS) PAUSED", dt * 1000, 1.0 / dt);
    else
        draw_text(5, h - 20, "dt: %.2f ms (%.2F FPS)", dt * 1000, 1.0 / dt);

    //if(pcount++ < 2)
    
    Breakout.Update(dt);
    Breakout.Render();

    glutSwapBuffers();
}

void reshape(int width, int height) {
    glViewport(0, 0, width, height);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(45.0, width / (float) height, 0.1, 100.0);
}

void mouse(int button, int state, int x, int y) {
}

void keyboard(unsigned char key, int x, int y) {
    switch (key) {
        case ' ':
            if(Breakout.State == SCENE_PAUSE){
                Breakout.State = SCENE_ACTIVE;
                break;
            }
            if(Breakout.State == SCENE_ACTIVE){
                Breakout.State = SCENE_PAUSE;
                break;
            }
        case 27:
            glutLeaveMainLoop();
            break;
        default:
            break;
    }
}

void motion(int x, int y) {

}

void special(int key, int x, int y) {

}

void idle() {
    display();
}


int main(int argc, char *argv[]) {
    glutInit(&argc, argv);
    glutInitWindowSize(800, 600);
    glutInitWindowPosition(50, 50);
    glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE);
    glutCreateWindow("Physics Engine -- dtk");

    Breakout.Init();
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
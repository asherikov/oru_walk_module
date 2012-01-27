/**
 * @file
 * @author Dimitar Dimitrov
 */


// Linux
#ifdef LINUX
#include <GL/gl.h>
#include <GL/glu.h>
#endif

// MAC
#ifdef DARWIN
#include <OpenGL/gl.h>
#include <OpenGL/glu.h>
#endif

// SDL related includes
#include "SDL/SDL.h"
#include "SDL/SDL_mouse.h"
#include "SDL/SDL_keyboard.h"
#include "SDL/SDL_keysym.h"


// ----------------------
// visualization
// ----------------------
void draw_grid(int);
void draw_CF(void);


SDL_Event event;
SDL_Surface *screen;

int isRunning;
int screenWidth=800;
int screenHeight=600;

// initial view
float zoom = 10.0;
float rotx = -65.0;
float roty = 0.0;
float rotz = 90.0;
float transx = 0.0;
float transy = 0.0;
// ----------------------



/** \brief SDL pre-draw function.

    \return void
*/
void pre_draw()
{
    glViewport(0,0,(GLsizei) screen->w,(GLsizei) screen->h);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(60.0,(GLfloat)screen->w/(GLfloat)screen->h,0.1,1500.0);

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    glClearColor(0.0,0.0,0.4,0.0); // blue background
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // -----------------------------------------------------------
    // Add LIGHT stuff
    // -----------------------------------------------------------
    glDisable(GL_CULL_FACE);
    glEnable(GL_DEPTH_TEST);   // Allow depth buffering
    glDisable(GL_BLEND);
    //glEnable(GL_LIGHTING);

    glEnable(GL_COLOR_MATERIAL);
    glEnable(GL_LIGHT0);

    // -----------------------------------------------------------
    // Rotations + Translations mouse control
    // -----------------------------------------------------------

    // positive x is to the right
    // positive y is up
    // positive z is away from the screen (towards the viewer)
    //
    // initially draw all objects at [0,0,-zoom] (zoom with the middle button)
    glTranslatef(0.0, 0.0, -zoom);

    // translate with the right mouse button
    glTranslatef(transx, 0.0, 0.0);
    glTranslatef(0.0, -transy, 0.0);

    // rotate with the left mouse button (left shift & left mouse button rotates around Z)
    glRotatef(rotx,1,0,0);
    glRotatef(roty,0,1,0);
    glRotatef(rotz,0,0,1);
    // -----------------------------------------------------------
    draw_grid(15);

    glPushMatrix();
    //glTranslatef(-3.0f,-3.0f,0.0f);
    glScalef(0.1, 0.1, 0.1);
    draw_CF(); // +x : blue, +y : green, +z : red,
    glPopMatrix();

}



/** \brief SDL events handling.

    \return void
*/
void events()
{
    // Process any events that have occured
    while(SDL_PollEvent(&event))
    {
        switch(event.type)
        {
        case SDL_QUIT:
            isRunning=0;
            break;
        case SDL_MOUSEBUTTONDOWN:

            // zoom with the scroll

            if (event.button.button == 4)
                zoom += 1.0f;

            if (event.button.button == 5)
                zoom -= 1.0f;

            break;
        case SDL_KEYDOWN:
            if(event.key.keysym.sym == SDLK_ESCAPE)
            {
                isRunning=0;
                break;
            }

            break;
        case SDL_VIDEORESIZE:
            screen = SDL_SetVideoMode(event.resize.w, event.resize.h, 16, SDL_OPENGL | SDL_RESIZABLE);

            break;
        }
    }
}



/** \brief SDL related initializations.

    \return void
*/
void initSDL()
{
    /* Initialize SDL and create window */
    if(SDL_Init(SDL_INIT_VIDEO) < 0)
    {
        printf("Failed to initialize SDL. Error '%s'\n",SDL_GetError());
        exit(-1);
    }
    atexit(SDL_Quit);

    screen = SDL_SetVideoMode(screenWidth, screenHeight, 32, SDL_SWSURFACE | SDL_OPENGL | SDL_RESIZABLE);
    if (!screen)
    {
        fprintf(stderr, "Couldn't set video mode: %s\n", SDL_GetError());
        exit(-1);
    }

    SDL_WM_SetCaption("A cube", "test");

    SDL_GL_SetAttribute( SDL_GL_RED_SIZE, 5 );
    SDL_GL_SetAttribute( SDL_GL_GREEN_SIZE, 5 );
    SDL_GL_SetAttribute( SDL_GL_BLUE_SIZE, 5 );
    SDL_GL_SetAttribute( SDL_GL_DEPTH_SIZE, 16 );
    SDL_GL_SetAttribute( SDL_GL_DOUBLEBUFFER, 1 );
}



/** \brief Mouse and keyboard interaction handling.

    \return void
*/
void tick()
{

    int shift_pressed = 0;

    //Uint8 *keystate = SDL_GetKeyState(NULL);

    int mouseX,mouseY;
    static int oldMouseX=0, oldMouseY=0;
    Uint8 mouseButtonState=SDL_GetMouseState(&mouseX,&mouseY);

    // mouse control
    // -------------------
    if(SDL_GetModState() & KMOD_LSHIFT)
        shift_pressed = 1;

    // left button & left shift
    if ((shift_pressed) & (mouseButtonState==1))
        rotz += (mouseX - oldMouseX) / 10.0;
    else if(mouseButtonState==1) // left button
    {
        roty += (mouseX - oldMouseX) / 10.0;
        rotx += (mouseY - oldMouseY) / 10.0;
    }

    if(mouseButtonState==2) // middle button
        zoom += (mouseY - oldMouseY) /20;

    if(mouseButtonState==4) // right button
    {
        transx += (mouseX - oldMouseX) / 400.0;
        transy += (mouseY - oldMouseY) / 400.0;
    }

    oldMouseX = mouseX;
    oldMouseY = mouseY;
}



/** \brief Draws a right-handed coordinate frame in OpenGL.

    \return void
*/
void draw_CF(void)
{
    // draw coordinate frame

    glLineWidth(2.0);
    glBegin(GL_LINES);
    glColor3f(0.0, 0.0, 255.0);
    glVertex3f(0,0,0);
    glVertex3f(5,0,0);
    glColor3f(0.0, 255.0, 0.0);
    glVertex3f(0,0,0);
    glVertex3f(0,5,0);
    glColor3f(255.0, 0.0, 0.0);
    glVertex3f(0,0,0);
    glVertex3f(0,0,5);
    glEnd();

    glLineWidth(1.0);
}



// The end-effectors for the feet are not in the center of the support polygons.
// I assume
//  ------------------------------------------------
//  |                    |                         |
//  |                    |0.025                    |
//  |                    |                         |
//  |      0.03          |          0.09           |
//  |------------------- 0 ------------------------|
//  |                    |                         |
//  |                    |                         |
//  |                    |0.025                    |
//  |                    |                         |
//  ------------------------------------------------
//
void drawPolygon()
{
    glBegin(GL_QUADS);             // Draw A Quad
    glVertex3f(-0.3f, 0.25f, 0.0f); // Top Left
    glVertex3f( 0.9f, 0.25f, 0.0f); // Top Right
    glVertex3f( 0.9f,-0.25f, 0.0f); // Bottom Right
    glVertex3f(-0.3f,-0.25f, 0.0f); // Bottom Left

    /* origianl
    glVertex3f(-1.0f, 1.0f, 0.0f); // Top Left
    glVertex3f( 1.0f, 1.0f, 0.0f); // Top Right
    glVertex3f( 1.0f,-1.0f, 0.0f); // Bottom Right
    glVertex3f(-1.0f,-1.0f, 0.0f); // Bottom Left
    */

    glEnd();                       // Done Drawing The Quad
}




/** \brief Draws a cube in OpenGL.

    \return void
*/
void drawCube()
{

    glBegin(GL_QUADS);			// Start Drawing Quads
    // Front Face
    glNormal3f( 0.0f, 0.0f, 1.0f);		// Normal Facing Forward
    glTexCoord2f(0.0f, 0.0f);
    glVertex3f(-0.5f, -0.5f,  0.5f);	// Bottom Left Of The Texture and Quad
    glTexCoord2f(0.5f, 0.0f);
    glVertex3f( 0.5f, -0.5f,  0.5f);	// Bottom Right Of The Texture and Quad
    glTexCoord2f(0.5f, 0.5f);
    glVertex3f( 0.5f,  0.5f,  0.5f);	// Top Right Of The Texture and Quad
    glTexCoord2f(0.0f, 0.5f);
    glVertex3f(-0.5f,  0.5f,  0.5f);	// Top Left Of The Texture and Quad
    // Back Face
    glNormal3f( 0.0f, 0.0f,-1.0f);		// Normal Facing Away
    glTexCoord2f(0.5f, 0.0f);
    glVertex3f(-0.5f, -0.5f, -0.5f);	// Bottom Right Of The Texture and Quad
    glTexCoord2f(0.5f, 0.5f);
    glVertex3f(-0.5f,  0.5f, -0.5f);	// Top Right Of The Texture and Quad
    glTexCoord2f(0.0f, 0.5f);
    glVertex3f( 0.5f,  0.5f, -0.5f);	// Top Left Of The Texture and Quad
    glTexCoord2f(0.0f, 0.0f);
    glVertex3f( 0.5f, -0.5f, -0.5f);	// Bottom Left Of The Texture and Quad
    // Top Face
    glNormal3f( 0.0f, 1.0f, 0.0f);		// Normal Facing Up
    glTexCoord2f(0.0f, 0.5f);
    glVertex3f(-0.5f,  0.5f, -0.5f);	// Top Left Of The Texture and Quad
    glTexCoord2f(0.0f, 0.0f);
    glVertex3f(-0.5f,  0.5f,  0.5f);	// Bottom Left Of The Texture and Quad
    glTexCoord2f(0.5f, 0.0f);
    glVertex3f( 0.5f,  0.5f,  0.5f);	// Bottom Right Of The Texture and Quad
    glTexCoord2f(0.5f, 0.5f);
    glVertex3f( 0.5f,  0.5f, -0.5f);	// Top Right Of The Texture and Quad
    // Bottom Face
    glNormal3f( 0.0f,-1.0f, 0.0f);		// Normal Facing Down
    glTexCoord2f(0.5f, 0.5f);
    glVertex3f(-0.5f, -0.5f, -0.5f);	// Top Right Of The Texture and Quad
    glTexCoord2f(0.0f, 0.5f);
    glVertex3f( 0.5f, -0.5f, -0.5f);	// Top Left Of The Texture and Quad
    glTexCoord2f(0.0f, 0.0f);
    glVertex3f( 0.5f, -0.5f,  0.5f);	// Bottom Left Of The Texture and Quad
    glTexCoord2f(0.5f, 0.0f);
    glVertex3f(-0.5f, -0.5f,  0.5f);	// Bottom Right Of The Texture and Quad
    // Right face
    glNormal3f( 1.0f, 0.0f, 0.0f);		// Normal Facing Right
    glTexCoord2f(0.5f, 0.0f);
    glVertex3f( 0.5f, -0.5f, -0.5f);	// Bottom Right Of The Texture and Quad
    glTexCoord2f(0.5f, 0.5f);
    glVertex3f( 0.5f,  0.5f, -0.5f);	// Top Right Of The Texture and Quad
    glTexCoord2f(0.0f, 0.5f);
    glVertex3f( 0.5f,  0.5f,  0.5f);	// Top Left Of The Texture and Quad
    glTexCoord2f(0.0f, 0.0f);
    glVertex3f( 0.5f, -0.5f,  0.5f);	// Bottom Left Of The Texture and Quad
    // Left Face
    glNormal3f(-1.0f, 0.0f, 0.0f);		// Normal Facing Left
    glTexCoord2f(0.0f, 0.0f);
    glVertex3f(-0.5f, -0.5f, -0.5f);	// Bottom Left Of The Texture and Quad
    glTexCoord2f(0.5f, 0.0f);
    glVertex3f(-0.5f, -0.5f,  0.5f);	// Bottom Right Of The Texture and Quad
    glTexCoord2f(0.5f, 0.5f);
    glVertex3f(-0.5f,  0.5f,  0.5f);	// Top Right Of The Texture and Quad
    glTexCoord2f(0.0f, 0.5f);
    glVertex3f(-0.5f,  0.5f, -0.5f);	// Top Left Of The Texture and Quad
    glEnd();					// Done Drawing Quads

}



/** \brief Draws a grid in OpenGL.

    \param[in] grid_size Size of the grid in number of squares

    \return void
*/
void draw_grid(int grid_size)
{
    // draw grid

    //glEnable (GL_LINE_SMOOTH);     /* Enable Antialiased lines */
    glEnable (GL_LINE_STIPPLE);
    glLineStipple (6, 0xAAAA);
    glColor3f (0.3f, 0.3f, 0.3f);

    glBegin(GL_LINES);
    for(int i=-grid_size; i<=grid_size; i+=2 )
    {
        glVertex3f(i,-grid_size,0);
        glVertex3f(i,grid_size,0);

        glVertex3f(grid_size,i,0);
        glVertex3f(-grid_size,i,0);
    }
    glEnd();

    glDisable(GL_LINE_STIPPLE);
    glColor3f (0.4f, 0.4f, 0.4f);

    glBegin(GL_LINES);
    for(int i=-grid_size; i<=grid_size; i+=10 )
    {
        glVertex3f(i,-grid_size,0);
        glVertex3f(i,grid_size,0);

        glVertex3f(grid_size,i,0);
        glVertex3f(-grid_size,i,0);
    }
    glEnd();

}



/** \brief 4x4 homogeneous matrix (double T[16]) to 4x4 homogeneous matrix (GLfloat T[16])

    \return void
 */
void HomogGL(double *T, GLfloat *M, double scale)
{
    for (int i=0; i<4*4; i++)
        M[i] = T[i];

    M[12] *= scale;
    M[13] *= scale;
    M[14] *= scale;

}




/** \brief Creates a 4x4 homogeneous matrix of GLfloat (and possibly scales the position part)

    \return void
 */
void HomogGL(double x, double y , double z, double alpha, double beta, double gamma, GLfloat *M, double scale)
{
    posture T;
    T.init(x, y, z, alpha, beta, gamma);

    for (int i=0; i<POSTURE_MATRIX_SIZE; i++)
        M[i] = T.data[i];

    M[12] *= scale;
    M[13] *= scale;
    M[14] *= scale;
}



// structure of array A
// i[4*4]      - joint i (i=1..24)
// (24+i)[4*4] - end-effector i (i=i..6)
// 31[3]       - CoM
void draw_nao(int case_flag, double *q)
{
    // if case_flag = 1 LLeg is in support
    // if case_flag = 0 RLeg is in support

    double scale1 = 10; // scaling factor
    double scale2 = 0.1;

    double A[483];
    GLfloat glM[4*4];

    int kLeg;

    if (case_flag)
    {
        kLeg = 25;
        LLeg2Joints(q,A);
    }
    else
    {
        kLeg = 26;
        RLeg2Joints(q,A);
    }

    for (int k=0; k<30; k++)
    {
        if (k == kLeg) // draw frames
        {
            glPushMatrix();
            HomogGL(A+k*16, glM, scale1);
            glMultMatrixf(glM);
            glScalef(scale2, scale2, scale2);
            draw_CF();
            glColor3f(0.0f,0.0f,1.0f);
            glScalef(1.0, 1.0, 1.0);
            glPopMatrix();
        }

        // draw cubes
        if (k == 24 || k == 25 || k == 26 || k == 27 || k == 28) // without the head
        {
            glPushMatrix();
            HomogGL(A+k*16, glM, scale1);
            glMultMatrixf(glM);
            glScalef(scale2, scale2, scale2);
            glColor3f(1.0f,1.0f,1.0f);
            drawCube();
            glScalef(1.0, 1.0, 1.0);
            glPopMatrix();
        }
        else
        {
            glPushMatrix();
            HomogGL(A+k*16, glM, scale1);
            glMultMatrixf(glM);
            glScalef(scale2, scale2, scale2);
            glColor3f(0.0f,0.0f,1.0f);
            drawCube();
            glScalef(1.0, 1.0, 1.0);
            glPopMatrix();
        }

    }

    // CoM
    glPushMatrix();
    glTranslatef(A[30*16+0]*scale1, A[30*16+1]*scale1, A[30*16+2]*scale1);
    glScalef(scale2, scale2, scale2);
    glColor3f(1.0f,0.0f,0.0f);
    drawCube();
    glScalef(1.0, 1.0, 1.0);
    glPopMatrix();

    // draw connections Joints
    glLineWidth(2.0);
    glBegin(GL_LINES);

    glColor3f(0.0, 255.0, 0.0);

    // -------------------------------------------------------------------------------------

    glVertex3f(A[24*16+12]*scale1, A[24*16+13]*scale1, A[24*16+14]*scale1); // torso
    glVertex3f(A[0*16+12]*scale1, A[0*16+13]*scale1, A[0*16+14]*scale1);    // left hip

    glVertex3f(A[24*16+12]*scale1, A[24*16+13]*scale1, A[24*16+14]*scale1); // torso
    glVertex3f(A[6*16+12]*scale1, A[6*16+13]*scale1, A[6*16+14]*scale1);    // right hip

    glVertex3f(A[0*16+12]*scale1, A[0*16+13]*scale1, A[0*16+14]*scale1);    // left hip
    glVertex3f(A[6*16+12]*scale1, A[6*16+13]*scale1, A[6*16+14]*scale1);    // right hip

    glVertex3f(A[24*16+12]*scale1, A[24*16+13]*scale1, A[24*16+14]*scale1); // torso
    glVertex3f(A[12*16+12]*scale1, A[12*16+13]*scale1, A[12*16+14]*scale1); // left sholder

    glVertex3f(A[24*16+12]*scale1, A[24*16+13]*scale1, A[24*16+14]*scale1); // torso
    glVertex3f(A[17*16+12]*scale1, A[17*16+13]*scale1, A[17*16+14]*scale1); // right sholder

    // -------------------------------------------------------------------------------------

    glVertex3f(A[0*16+12]*scale1, A[0*16+13]*scale1, A[0*16+14]*scale1);    // left hip
    glVertex3f(A[3*16+12]*scale1, A[3*16+13]*scale1, A[3*16+14]*scale1);    // left knee

    glVertex3f(A[3*16+12]*scale1, A[3*16+13]*scale1, A[3*16+14]*scale1);    // left knee
    glVertex3f(A[5*16+12]*scale1, A[5*16+13]*scale1, A[5*16+14]*scale1);    // left ankle

    glVertex3f(A[5*16+12]*scale1, A[5*16+13]*scale1, A[5*16+14]*scale1);    // left ankle
    glVertex3f(A[25*16+12]*scale1, A[25*16+13]*scale1, A[25*16+14]*scale1); // left foot

    glVertex3f(A[6*16+12]*scale1, A[6*16+13]*scale1, A[6*16+14]*scale1);    // right hip
    glVertex3f(A[9*16+12]*scale1, A[9*16+13]*scale1, A[9*16+14]*scale1);    // right knee

    glVertex3f(A[9*16+12]*scale1, A[9*16+13]*scale1, A[9*16+14]*scale1);    // right knee
    glVertex3f(A[11*16+12]*scale1, A[11*16+13]*scale1, A[11*16+14]*scale1); // right ankle

    glVertex3f(A[11*16+12]*scale1, A[11*16+13]*scale1, A[11*16+14]*scale1);    // right ankle
    glVertex3f(A[26*16+12]*scale1, A[26*16+13]*scale1, A[26*16+14]*scale1); // right foot

    // -------------------------------------------------------------------------------------

    glVertex3f(A[12*16+12]*scale1, A[12*16+13]*scale1, A[12*16+14]*scale1); // left sholder
    glVertex3f(A[15*16+12]*scale1, A[15*16+13]*scale1, A[15*16+14]*scale1); // left elbow1

    glVertex3f(A[15*16+12]*scale1, A[15*16+13]*scale1, A[15*16+14]*scale1); // left elbow1
    glVertex3f(A[16*16+12]*scale1, A[16*16+13]*scale1, A[16*16+14]*scale1); // left elbow2

    glVertex3f(A[16*16+12]*scale1, A[16*16+13]*scale1, A[16*16+14]*scale1); // left elbow2
    glVertex3f(A[27*16+12]*scale1, A[27*16+13]*scale1, A[27*16+14]*scale1); // left hand

    glVertex3f(A[17*16+12]*scale1, A[17*16+13]*scale1, A[17*16+14]*scale1); // right sholder
    glVertex3f(A[20*16+12]*scale1, A[20*16+13]*scale1, A[20*16+14]*scale1); // right elbow1

    glVertex3f(A[20*16+12]*scale1, A[20*16+13]*scale1, A[20*16+14]*scale1); // right elbow1
    glVertex3f(A[21*16+12]*scale1, A[21*16+13]*scale1, A[21*16+14]*scale1); // right elbow2

    glVertex3f(A[21*16+12]*scale1, A[21*16+13]*scale1, A[21*16+14]*scale1); // right elbow2
    glVertex3f(A[28*16+12]*scale1, A[28*16+13]*scale1, A[28*16+14]*scale1); // right hand

    // -------------------------------------------------------------------------------------

    glVertex3f(A[12*16+12]*scale1, A[12*16+13]*scale1, A[12*16+14]*scale1); // left sholder
    glVertex3f(A[29*16+12]*scale1, A[29*16+13]*scale1, A[29*16+14]*scale1); // head

    glVertex3f(A[17*16+12]*scale1, A[17*16+13]*scale1, A[17*16+14]*scale1); // right sholder
    glVertex3f(A[29*16+12]*scale1, A[29*16+13]*scale1, A[29*16+14]*scale1); // head

    // -------------------------------------------------------------------------------------

    glEnd();
    glLineWidth(1.0);

    glPushMatrix();
    HomogGL(A+25*16, glM, scale1);
    glMultMatrixf(glM);
    glColor3f(1.0f,0.0f,0.0f);
    drawPolygon();
    glScalef(1.0, 1.0, 1.0);
    glPopMatrix();

    glPushMatrix();
    HomogGL(A+26*16, glM, scale1);
    glMultMatrixf(glM);
    glColor3f(1.0f,0.0f,0.0f);
    drawPolygon();
    glScalef(1.0, 1.0, 1.0);
    glPopMatrix();

}



// draw footsteps to execute
void draw_footsteps(
        std::vector<double> & x_coord,
        std::vector<double> & y_coord,
        std::vector<double> & angle_rot)
{
    GLfloat glM[4*4];
    double scale1 = 10;
    double scale3 = 0.03;

    for (unsigned int i=0; i < x_coord.size(); i++)
    {
        glPolygonMode( GL_FRONT_AND_BACK, GL_LINE );
        glPushMatrix();
        HomogGL(x_coord[i], y_coord[i], 0, 0, 0, angle_rot[i], glM, scale1);
        glMultMatrixf(glM);
        glColor3f(0.3f,0.3f,0.3f);
        drawPolygon();
        glScalef(1.0, 1.0, 1.0);
        glPopMatrix();
        glPolygonMode( GL_FRONT_AND_BACK, GL_FILL );

        // Draw reference point of the footsteps
        glPushMatrix();
        glTranslatef(x_coord[i]*scale1, y_coord[i]*scale1, 0);
        glScalef(scale3, scale3, scale3);
        glColor3f(1.0f,1.0f,1.0f);
        drawCube();
        glScalef(1.0, 1.0, 1.0);
        glPopMatrix();
    }
}



void drawSDL (
        int delay,
        std::vector<double> & x_coord, 
        std::vector<double> & y_coord, 
        std::vector<double> & angle_rot,
        igmSupportFoot support_foot,
        double * q)
{
    // slow down the execution of the program
    for (int i = 0; i <= delay; i++)
    {
        tick ();
        pre_draw();
        events ();
    }
    draw_footsteps(x_coord, y_coord, angle_rot);
    draw_nao(support_foot, q);
    SDL_GL_SwapBuffers();
}

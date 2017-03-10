
#include <iostream>
#include <GL/glu.h>
#include <GL/glx.h>
#include <GL/glut.h>

#include <X11/Xlib.h>
#include <X11/Xutil.h>
#include <X11/Xresource.h>

#include <libmove3d/include/Graphic-pkg.h>
#include <libmove3d/p3d/proto/p3d_matrix_proto.h>
#include <libmove3d/include/move3d-gui.h>

#include <sstream>

typedef GLXContext (*glXCreateContextAttribsARBProc)(Display*, GLXFBConfig, GLXContext, Bool, const int*);
typedef Bool (*glXMakeContextCurrentARBProc)(Display*, GLXDrawable, GLXDrawable, GLXContext);
static glXCreateContextAttribsARBProc glXCreateContextAttribsARB = NULL;
static glXMakeContextCurrentARBProc   glXMakeContextCurrentARB   = NULL;

const bool save_image_to_file=false;
void renderScene();

// compute camera parameters (taken from move3d)
void gl_calc_param(g3d_cam_param& p)
{
    //cout << "qt_ui_calc_param" <<  endl;
    p3d_vector4 up;

    calc_cam_param(G3D_WIN, p.Xc, p.Xw);

    if (G3D_WIN)
    {
        p3d_matvec4Mult(*G3D_WIN->cam_frame, G3D_WIN->vs.up, up);
    }
    else
    {
        up[0] = 0;
        up[1] = 0;
        up[2] = 1;
    }

    p.up[0] = up[0];
    p.up[1] = up[1];
    p.up[2] = up[2];
}

//save the image to a PPM file in /tmp
void save_image(unsigned char image[],int width,int height){
    FILE *file;
    static uint seq=0;
    std::ostringstream oss;
    oss<<"/tmp/image_"<<seq<<".ppm";
    file=fopen(oss.str().c_str(),"w");
    std::cout<<"saving " << oss.str()<<std::endl;
    if(++seq>20) seq=0;
    fprintf(file, "P6\n");
    fprintf(file, "# creator: move3d_facts\n");
    fprintf(file, "%d %d\n", width, height);
    fprintf(file, "255\n");
    fwrite(image, sizeof(unsigned char), 3*width*height, file);
    fclose(file);
}

//function to draw once the move3d env
void updateGL(){
    glPushMatrix();

    p3d_vector4 Xc, Xw;
    p3d_vector4 up;

    //	computeNewVectors(Xc,Xw,up);
    g3d_cam_param p;
    gl_calc_param(p);

    Xc[0] = p.Xc[0];
    Xc[1] = p.Xc[1];
    Xc[2] = p.Xc[2];

    Xw[0] = p.Xw[0];
    Xw[1] = p.Xw[1];
    Xw[2] = p.Xw[2];

    up[0] = p.up[0];
    up[1] = p.up[1];
    up[2] = p.up[2];

    gluLookAt(Xc[0], Xc[1], Xc[2], Xw[0], Xw[1], Xw[2], up[0], up[1], up[2]);

    G3D_WIN->vs.cameraPosition[0]= Xc[0];
    G3D_WIN->vs.cameraPosition[1]= Xc[1];
    G3D_WIN->vs.cameraPosition[2]= Xc[2];

    //cout << "g3d_draw in ::GLWidget(" << m_id << ")" << endl;

    if(G3D_WIN->vs.viewport[2] != 0 && G3D_WIN->vs.viewport[3] != 0){
        glViewport(G3D_WIN->vs.viewport[0],G3D_WIN->vs.viewport[1],G3D_WIN->vs.viewport[2],G3D_WIN->vs.viewport[3]);
    }else{
        glGetIntegerv(GL_VIEWPORT,G3D_WIN->vs.viewport);
    }

    g3d_draw(0);

    glPopMatrix();

    if(save_image_to_file){
        glEnable(GL_TEXTURE_2D);

        glReadBuffer(GL_BACK);
        GLint viewport[4];
        glGetIntegerv(GL_VIEWPORT,viewport);
        int width=viewport[2],height=viewport[3];
        unsigned char image[3*width*height];
        glReadPixels(0, 0, width, height, GL_RGB, GL_UNSIGNED_BYTE, image);
        glPixelStorei(GL_PACK_ALIGNMENT, 1);
        int count(0);
        for(int i=0;i<width*height;i++){
            if(image[3*i+1] >= 255*0.8)
                count++;
        }
        std::cout<<((float)count)/(width*height)<<std::endl;

        save_image(image,width,height);
    }

    glutSwapBuffers();
    glutPostRedisplay();
}


//setup a windowless context for rendering
int gl_setup_headless(int win_width, int win_height){
    glXCreateContextAttribsARB = (glXCreateContextAttribsARBProc) glXGetProcAddressARB( (const GLubyte *) "glXCreateContextAttribsARB" );
    glXMakeContextCurrentARB   = (glXMakeContextCurrentARBProc)   glXGetProcAddressARB( (const GLubyte *) "glXMakeContextCurrent"      );

    const char *displayName = NULL;
    Display* display = XOpenDisplay( displayName );
    if(!display)
        return 1;

    static int visualAttribs[0];
    int numberOfFramebufferConfigurations = 0;
    GLXFBConfig* fbConfigs = glXChooseFBConfig( display, DefaultScreen(display), visualAttribs, &numberOfFramebufferConfigurations );
    if(!fbConfigs){
        return 2;
    }

    int context_attribs[] = {
        GLX_CONTEXT_MAJOR_VERSION_ARB, 2,
        GLX_CONTEXT_MINOR_VERSION_ARB, 1,
        GLX_CONTEXT_FLAGS_ARB, GLX_CONTEXT_DEBUG_BIT_ARB,
        GLX_CONTEXT_PROFILE_MASK_ARB, GLX_CONTEXT_CORE_PROFILE_BIT_ARB,
        None
    };

    GLXContext openGLContext = glXCreateContextAttribsARB( display, fbConfigs[0], 0, True, context_attribs);
    if(!openGLContext){
        return 3;
    }

    int pbufferAttribs[] = {
        GLX_PBUFFER_WIDTH,  win_width,
        GLX_PBUFFER_HEIGHT, win_height,
        None
    };
    GLXPbuffer pbuffer = glXCreatePbuffer( display, fbConfigs[0], pbufferAttribs );

    // clean up:
    XFree( fbConfigs );
    XSync( display, False );

     if ( !glXMakeContextCurrent( display, pbuffer, pbuffer, openGLContext ) )
     {
         return 4;
     }

     return 0;

}

//setup a window display for the rendering (use GLUT)
int gl_setup_window(int win_width, int win_height){
    // init GLUT and create Window
    //glutInit(&argc, argv);
    int argc=1;
    char name[]="move3d_facts";
    glutInit(&argc, (char**)&name);
    glutInitDisplayMode(GLUT_DEPTH | GLUT_DOUBLE | GLUT_RGBA);
    glutInitWindowPosition(100, 100);
    glutInitWindowSize(win_width, win_height);
    glutCreateWindow("move3d_facts");

    // register callbacks
    //glutDisplayFunc(renderScene);

    // enter GLUT event processing cycle
    //glutMainLoop();


}

//setup the rendering env
int gl_setup(int win_width, int win_height)
{
    return gl_setup_window(win_width,win_height);
    //return gl_setup_headless(win_width,win_height);
}

/*
 *  collision_detection.c
 *
 *  Author(s): Tomas Sima
 *
 */

// ============================================================================
//	Includes
// ============================================================================

#include <vector>
#include <future>
#include <stdio.h>
#ifdef _WIN32
#  define snprintf _snprintf
#endif
#include <stdlib.h>
#include <GL/glut.h>
#include <AR/config.h>
#include <AR/video.h>
#include <AR/param.h>
#include <AR/ar.h>
#include <AR/gsub_lite.h>
#include <iostream>
#include <sstream>
#include <cmath>
#include <iostream>
#include <string>
#include "collision_detection.h"
#include "curl.h"
#include "imageloader.h"


// ============================================================================
//	Constants
// ============================================================================

#define VIEW_SCALEFACTOR		1.0         // Units received from ARToolKit tracking will be multiplied by this factor before being used in OpenGL drawing.
#define VIEW_DISTANCE_MIN		40.0        // Objects closer to the camera than this will not be displayed. OpenGL units.
#define VIEW_DISTANCE_MAX		10000.0     // Objects further away from the camera than this will not be displayed. OpenGL units.

// ============================================================================
//	Global variables
// ============================================================================

// Preferences.
static int windowed = TRUE;                     // Use windowed (TRUE) or fullscreen mode (FALSE) on launch.
static int windowWidth = 1280;					// Initial window width, also updated during program execution.
static int windowHeight = 960;                  // Initial window height, also updated during program execution.
static int windowDepth = 32;					// Fullscreen mode bit depth.
static int windowRefresh = 0;					// Fullscreen mode refresh rate. Set to 0 to use default rate.

// Image acquisition
static ARUint8		*gARTImage = NULL;
static int          gARTImageSavePlease = FALSE;

// Marker detection.
static ARHandle		*gARHandle = NULL;
static ARPattHandle	*gARPattHandle = NULL;
static long			gCallCountMarkerDetect = 0;

// Transformation matrix retrieval.
static AR3DHandle	*gAR3DHandle = NULL;
static ARdouble		gPatt_width     = 80.0;	// Per-marker, but we are using only 1 marker.
static ARdouble		gPatt_trans[3][4];		// Per-marker, but we are using only 1 marker.
static ARdouble		gPatt_trans2[3][4];		// Per-marker, but we are using only 1 marker.
static int			gPatt_found = FALSE;	// Per-marker, but we are using only 1 marker.
static int			gPatt_id = 1;				// Per-marker, but we are using only 1 marker.
static int			gPatt_id2 = 2;				// Per-marker, but we are using only 1 marker.

// paths to sources
static	char patt_name[]  = "Data/hiro.patt";
static	char patt_name2[]  = "Data/kanji.patt";
static char url[] = "https://augment-reality.herokuapp.com/data.txt";
std::future<std::string> params = std::async(curl,url);

// Drawing.
static ARParamLT *gCparamLT = NULL;
static ARGL_CONTEXT_SETTINGS_REF gArglSettings = NULL;
static int gShowHelp = 1;
static int gShowMode = 1;
static int gDrawRotate = FALSE;
static float gDrawRotateAngle = 10;			// For use in drawing.

// menu, changing properties of objects
static float gboxsize = 1.0;			// for resizing collision box
typedef enum {cBOX, cSPHERE} collision_type;
typedef enum {BOX, SPHERE} object_type;
static collision_type collision_box = cBOX;
static object_type object_model = BOX;
double dist = 0;
double xdist = 0;
double ydist = 0;
double zdist = 0;

// ============================================================================
//	Functions
// ============================================================================

GLuint loadTexture(Image* image) {

	GLuint textureId;

	glGenTextures(1, &textureId); //Make room for our texture

	glBindTexture(GL_TEXTURE_2D, textureId); //Tell OpenGL which texture to edit

	//Map the image to the texture

	glTexImage2D(GL_TEXTURE_2D,                //Always GL_TEXTURE_2D

				 0,                            //0 for now

				 GL_RGB,                       //Format OpenGL uses for image

				 image->width, image->height,  //Width and height

				 0,                            //The border of the image

				 GL_RGB, //GL_RGB, because pixels are stored in RGB format

				 GL_UNSIGNED_BYTE, //GL_UNSIGNED_BYTE, because pixels are stored

			//as unsigned numbers

				 image->pixels);               //The actual pixel data

	return textureId; //Returns the id of the texture

}

// Something to look at, draw a rotating colour cube.
static void DrawCube(void)
{
    // Colour cube data.
	int i;
	float fSize = 40.0f;
	const GLfloat cube_vertices [8][3] = {
        /* +z */ {0.866, 0.866, 0.866}, {0.866, -0.866, 0.866}, {-0.866, -0.866, 0.866}, {-0.866, 0.866, 0.866},
        /* -z */ {0.866, 0.866, -0.866}, {0.866, -0.866, -0.866}, {-0.866, -0.866, -0.866}, {-0.866, 0.866, -0.866} };
		const GLubyte cube_vertex_colors [8][4] = {
			{255, 255, 255, 255}, {255, 255, 0, 255}, {0, 255, 0, 255}, {0, 255, 255, 255},
			{255, 0, 255, 255}, {255, 0, 0, 255}, {0, 0, 0, 255}, {0, 0, 255, 255} };
    const GLubyte cube_faces [6][4] = { /* ccw-winding */
        /* +z */ {3, 2, 1, 0}, /* -y */ {2, 3, 7, 6}, /* +y */ {0, 1, 5, 4},
        /* -x */ {3, 0, 4, 7}, /* +x */ {1, 2, 6, 5}, /* -z */ {4, 5, 6, 7} };

	const GLubyte black [4] = {255, 255, 255, 255};
    glPushMatrix(); // Save world coordinate system.
    glScalef(fSize, fSize, fSize);
    glTranslatef(0.0f, 0.0f, 0.5f); // Place base of cube on marker surface.
//    glDisable(GL_LIGHTING);
//    glDisable(GL_TEXTURE_2D);
//    glDisable(GL_BLEND);
    glColorPointer(4, GL_UNSIGNED_BYTE, 0, cube_vertex_colors);
    glVertexPointer(3, GL_FLOAT, 0, cube_vertices);
    glEnableClientState(GL_VERTEX_ARRAY);
//    glEnableClientState(GL_COLOR_ARRAY);

    if (object_model == SPHERE){

		GLuint _textureId; //The id of the textur
		GLUquadric *quad;
		Image* image = loadBMP("earth.bmp");
		glEnable(GL_TEXTURE_2D);

		glBindTexture(GL_TEXTURE_2D, _textureId);
		_textureId = loadTexture(image);
		quad = gluNewQuadric();
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

		gluQuadricTexture(quad,1);
		gluSphere(quad, 1,10,10);

	} else{
    glEnableClientState(GL_COLOR_ARRAY);
    	for (i = 0; i < 6; i++) {
    		glDrawElements(GL_TRIANGLE_FAN, 4, GL_UNSIGNED_BYTE, &(cube_faces[i][0]));
    	}
    	for (i = 0; i < 6; i++) {
    		glDrawElements(GL_LINE_LOOP, 4, GL_UNSIGNED_BYTE, &(cube_faces[i][0]));
    	}
    	glVertexPointer(3, GL_FLOAT, 0, cube_vertices);
		glDisableClientState(GL_COLOR_ARRAY);
    }

    glScalef(gboxsize, gboxsize, gboxsize);
    if (collision_box == cSPHERE)
    {
//        std::cout << "dist: " << dist << " gboxsize " << gboxsize << std::endl;
		if(dist/2 < gboxsize*40*1.00)
		{
			glColor4d(100,0,0,1);
		}
    	glutWireSphere(1, 20, 20);
		glColor4d(1,1,1,1);
    }
    if (collision_box == cBOX)
    {
        std::cout << "dist: " << dist << " gboxsize " << gboxsize << std::endl;
        std::cout << "xdist: " << xdist;
        std::cout << "ydist: " << ydist;
        std::cout << "zdist: " << zdist;
        std::cout << std::endl;
        if(gboxsize*70 > xdist &&
          gboxsize*70 > ydist &&
          gboxsize*70 > zdist)
		{
			glColor4d(100,0,0,1);
		}
    	for (i = 0; i < 6; i++) {
    		glDrawElements(GL_LINE_LOOP, 4, GL_UNSIGNED_BYTE, &(cube_faces[i][0]));
    	}
		glColor4d(1,1,1,1);
    }
    glPopMatrix();    // Restore world coordinate system.
}

static void DrawCubeUpdate(float timeDelta)
{
	if (gDrawRotate) {
		gDrawRotateAngle += timeDelta * 45.0f; // Rotate cube at 45 degrees per second.
		if (gDrawRotateAngle > 360.0f) gDrawRotateAngle -= 360.0f;
	}
}

static int setupCamera(const char *cparam_name, char *vconf, ARParamLT **cparamLT_p, ARHandle **arhandle, AR3DHandle **ar3dhandle)
{
	ARParam			cparam;
	int				xsize, ysize;
	AR_PIXEL_FORMAT pixFormat;

    // Open the video path.
	if (arVideoOpen(vconf) < 0) {
		ARLOGe("setupCamera(): Unable to open connection to camera.\n");
		return (FALSE);
	}

    // Find the size of the window.
	if (arVideoGetSize(&xsize, &ysize) < 0) {
		ARLOGe("setupCamera(): Unable to determine camera frame size.\n");
		arVideoClose();
		return (FALSE);
	}
	ARLOGi("Camera image size (x,y) = (%d,%d)\n", xsize, ysize);

	// Get the format in which the camera is returning pixels.
	pixFormat = arVideoGetPixelFormat();
	if (pixFormat == AR_PIXEL_FORMAT_INVALID) {
		ARLOGe("setupCamera(): Camera is using unsupported pixel format.\n");
		arVideoClose();
		return (FALSE);
	}

	// Load the camera parameters, resize for the window and init.
	if (arParamLoad(cparam_name, 1, &cparam) < 0) {
		ARLOGe("setupCamera(): Error loading parameter file %s for camera.\n", cparam_name);
		arVideoClose();
		return (FALSE);
	}
	if (cparam.xsize != xsize || cparam.ysize != ysize) {
		ARLOGw("*** Camera Parameter resized from %d, %d. ***\n", cparam.xsize, cparam.ysize);
		arParamChangeSize(&cparam, xsize, ysize, &cparam);
	}
#ifdef DEBUG
	ARLOG("*** Camera Parameter ***\n");
	arParamDisp(&cparam);
#endif
	if ((*cparamLT_p = arParamLTCreate(&cparam, AR_PARAM_LT_DEFAULT_OFFSET)) == NULL) {
		ARLOGe("setupCamera(): Error: arParamLTCreate.\n");
		return (FALSE);
	}

	if ((*arhandle = arCreateHandle(*cparamLT_p)) == NULL) {
		ARLOGe("setupCamera(): Error: arCreateHandle.\n");
		return (FALSE);
	}
	if (arSetPixelFormat(*arhandle, pixFormat) < 0) {
		ARLOGe("setupCamera(): Error: arSetPixelFormat.\n");
		return (FALSE);
	}
	if (arSetDebugMode(*arhandle, AR_DEBUG_DISABLE) < 0) {
		ARLOGe("setupCamera(): Error: arSetDebugMode.\n");
		return (FALSE);
	}
	if ((*ar3dhandle = ar3DCreateHandle(&cparam)) == NULL) {
		ARLOGe("setupCamera(): Error: ar3DCreateHandle.\n");
		return (FALSE);
	}

	if (arVideoCapStart() != 0) {
		ARLOGe("setupCamera(): Unable to begin camera data capture.\n");
		return (FALSE);
	}

	return (TRUE);
}

static int setupMarker(ARHandle *arhandle, ARPattHandle **pattHandle_p)
{
	if ((*pattHandle_p = arPattCreateHandle2(16,10)) == NULL) {
		ARLOGe("setupMarker(): Error: arPattCreateHandle.\n");
		return (FALSE);
	}

	// Loading only 1 pattern in this example.
	if ((gPatt_id = arPattLoad(*pattHandle_p, patt_name)) < 0) {
		ARLOGe("setupMarker(): Error loading pattern file %s.\n", patt_name);
		arPattDeleteHandle(*pattHandle_p);
		return (FALSE);
	}

	if ((gPatt_id2 = arPattLoad(*pattHandle_p, patt_name2)) < 0) {
		ARLOGe("setupMarker(): Error loading pattern file %s.\n", patt_name);
		arPattDeleteHandle(*pattHandle_p);
		return (FALSE);
	}

	arPattAttach(arhandle, *pattHandle_p);

	return (TRUE);
}

static void cleanup(void)
{
	arglCleanup(gArglSettings);
	gArglSettings = NULL;
	arPattDetach(gARHandle);
	arPattDeleteHandle(gARPattHandle);
	arVideoCapStop();
	ar3DDeleteHandle(&gAR3DHandle);
	arDeleteHandle(gARHandle);
	arParamLTFree(&gCparamLT);
	arVideoClose();
}

static void Keyboard(unsigned char key, int x, int y)
{
	int mode, threshChange = 0;
	AR_LABELING_THRESH_MODE modea;

	switch (key) {
		case 0x1B:						// Quit.
		case 'Q':
		case 'q':
		cleanup();
		exit(0);
		break;
		case ' ':
		gDrawRotate = !gDrawRotate;
		break;
		case 'X':
		case 'x':
		arGetImageProcMode(gARHandle, &mode);
		switch (mode) {
			case AR_IMAGE_PROC_FRAME_IMAGE:  mode = AR_IMAGE_PROC_FIELD_IMAGE; break;
			case AR_IMAGE_PROC_FIELD_IMAGE:
			default: mode = AR_IMAGE_PROC_FRAME_IMAGE; break;
		}
		arSetImageProcMode(gARHandle, mode);
		break;
		case 'C':
		case 'c':
		ARLOGe("*** Camera - %f (frame/sec)\n", (double)gCallCountMarkerDetect/arUtilTimer());
		gCallCountMarkerDetect = 0;
		arUtilTimerReset();
		break;
		case 'a':
		case 'A':
		arGetLabelingThreshMode(gARHandle, &modea);
		switch (modea) {
			case AR_LABELING_THRESH_MODE_MANUAL:        modea = AR_LABELING_THRESH_MODE_AUTO_MEDIAN; break;
			case AR_LABELING_THRESH_MODE_AUTO_MEDIAN:   modea = AR_LABELING_THRESH_MODE_AUTO_OTSU; break;
			case AR_LABELING_THRESH_MODE_AUTO_OTSU:     modea = AR_LABELING_THRESH_MODE_AUTO_ADAPTIVE; break;
			case AR_LABELING_THRESH_MODE_AUTO_ADAPTIVE: modea = AR_LABELING_THRESH_MODE_AUTO_BRACKETING; break;
			case AR_LABELING_THRESH_MODE_AUTO_BRACKETING:
			default: modea = AR_LABELING_THRESH_MODE_MANUAL; break;
		}
		case 'e':
		case 'E':
		switch (collision_box) {
			case cBOX:        collision_box = cSPHERE; break;
			case cSPHERE:   collision_box = cBOX; break;
		}
		arSetLabelingThreshMode(gARHandle, modea);
		break;
		case '-':
		threshChange = -5;
		break;
		case '+':
		case '=':
		threshChange = +5;
		break;
		case 'D':
		case 'd':
		arGetDebugMode(gARHandle, &mode);
		arSetDebugMode(gARHandle, !mode);
		break;
		case 's':
		case 'S':
		if (!gARTImageSavePlease) gARTImageSavePlease = TRUE;
		break;
		case '?':
		case '/':
		gShowHelp++;
		if (gShowHelp > 1) gShowHelp = 0;
		break;
		case 'm':
		case 'M':
		gShowMode = !gShowMode;
		break;
		case 'o':
		case 'O':
		gboxsize += 0.1;
		break;
		case 'k':
		case 'K':
		gboxsize -= 0.1;
		break;
		default:
		break;
	}
	if (threshChange) {
		int threshhold;
		arGetLabelingThresh(gARHandle, &threshhold);
		threshhold += threshChange;
		if (threshhold < 0) threshhold = 0;
		if (threshhold > 255) threshhold = 255;
		arSetLabelingThresh(gARHandle, threshhold);
	}

}

void split(const std::string &s, char delim, std::vector<std::string> &elems) {
	std::stringstream ss;
	ss.str(s);
	std::string item;
	while (std::getline(ss, item, delim)) {
		elems.push_back(item);
	}
}

std::vector<std::string> split(const std::string &s, char delim) {
	std::vector<std::string> elems;
	split(s, delim, elems);
	return elems;
}

static void mainLoop(void)
{
	static int imageNumber = 0;
	static int ms_prev;
	static int ms_prevcurl;
	static int ms,mscurl;
	static float s_elapsed,s_elapsedcurl;
	ARUint8 *image;
	ARdouble err;
	int j,k;

	// Find out how long since mainLoop() last ran.
	ms = glutGet(GLUT_ELAPSED_TIME);
	mscurl = ms;
	s_elapsed = (float)(ms - ms_prev) * 0.001f;
	if (s_elapsed < 0.05f) return; // Don't update more often than 100 Hz.
	ms_prev = ms;

	// Update drawing.
	DrawCubeUpdate(s_elapsed);

	// Grab a video frame.
	if ((image = arVideoGetImage()) != NULL) {
		gARTImage = image;	// Save the fetched image.

		if (gARTImageSavePlease) {
			char imageNumberText[15];
			sprintf(imageNumberText, "image-%04d.jpg", imageNumber++);
			if (arVideoSaveImageJPEG(gARHandle->xsize, gARHandle->ysize, gARHandle->arPixelFormat, gARTImage, imageNumberText, 75, 0) < 0) {
				ARLOGe("Error saving video image.\n");
			}
			gARTImageSavePlease = FALSE;
		}

		gCallCountMarkerDetect++; // Increment ARToolKit FPS counter.

		gPatt_found = FALSE;
		// Detect the markers in the video frame.
		if (arDetectMarker(gARHandle, gARTImage) < 0) {
			exit(-1);
		}


		// visible marker matching our preferred pattern.
		int hiro = 0;
		int kanji = 0;
		k = -1;
		// printf("marker_num: %d \n",gARHandle->marker_num);
		for (j = 0; j < gARHandle->marker_num; j++) {
			if (gARHandle->markerInfo[j].id == gPatt_id) {
				hiro = 1;
				// printf("hiro \n");
			}
			if (gARHandle->markerInfo[j].id == gPatt_id2) {
				kanji = 1;
				// printf("kanj\n")
			}
		}

		if (kanji == 1 && hiro == 1) {  // I show objects only when both are visible
			s_elapsedcurl = (float)(mscurl - ms_prevcurl) * 0.001f;
			if (s_elapsedcurl > 2) {
				ms_prevcurl = mscurl;

				std::string fut = params.get();
				params = std::async(std::launch::async,curl,url);


				std::vector<std::string> prms = split(fut,':');
                    switch (prms[0][0]) {
                        case '2':   collision_box = cSPHERE; break;
                        case '1':   collision_box = cBOX; break;
                    }
                    switch (prms[1][0]) {
                        case '2':   object_model = SPHERE; break;
                        case '1':   object_model = BOX; break;
                    }
                    float sizee = strtof(prms[2].c_str(),NULL);
                    gboxsize = (float)sizee/(float)30 ;
                }
                // Get the transformation between the marker and the real camera into gPatt_trans.
                err = arGetTransMatSquare(gAR3DHandle, &(gARHandle->markerInfo[0]), gPatt_width, gPatt_trans);
                err = arGetTransMatSquare(gAR3DHandle, &(gARHandle->markerInfo[1]), gPatt_width, gPatt_trans2);
                gPatt_found = TRUE;

				double x1 = gPatt_trans[0][3];
				double x2 = gPatt_trans2[0][3];
				double y1 = gPatt_trans[1][3];
				double y2 = gPatt_trans2[1][3];
				double z1 = gPatt_trans[2][3];
				double z2 = gPatt_trans2[2][3];
    		    dist = pow(pow(x1-x2,2) + pow(y1-y2,2) + pow(z1-z2,2), 0.5);
				xdist= std::abs(x1 - x2);
                ydist= std::abs(y1 - y2);
                zdist= std::abs(z1 - z2);
//        		std::cout << "distance: " << dist << std::endl;
		}
	} else {
			// printf("nothing \n");
		gPatt_found = FALSE;
	}

	glutPostRedisplay();
}


static void Visibility(int visible)
{
	if (visible == GLUT_VISIBLE) {
		glutIdleFunc(mainLoop);
	} else {
		glutIdleFunc(NULL);
	}
}

static void Reshape(int w, int h)
{
	windowWidth = w;
	windowHeight = h;

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glViewport(0, 0, (GLsizei) w, (GLsizei) h);
}

static void Display(void)
{
	ARdouble p[16];
	ARdouble m[16];
	ARdouble m2[16];

	// Select correct buffer for this context.
	glDrawBuffer(GL_BACK);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); // Clear the buffers for new frame.

	arglPixelBufferDataUpload(gArglSettings, gARTImage);
	arglDispImage(gArglSettings);
	gARTImage = NULL; // Invalidate image data.

	// Projection transformation.
	arglCameraFrustumRH(&(gCparamLT->param), VIEW_DISTANCE_MIN, VIEW_DISTANCE_MAX, p);
	glMatrixMode(GL_PROJECTION);
#ifdef ARDOUBLE_IS_FLOAT
	glLoadMatrixf(p);
#else
	glLoadMatrixd(p);
#endif
	glMatrixMode(GL_MODELVIEW);

	glEnable(GL_DEPTH_TEST);

	// Viewing transformation.
	glLoadIdentity();
	// Lighting and geometry that moves with the camera should go here.
	// (I.e. must be specified before viewing transformations.)
	//none

	if (gPatt_found) {


		// Calculate the camera position relative to the marker.
		// Replace VIEW_SCALEFACTOR with 1.0 to make one drawing unit equal to 1.0 ARToolKit units (usually millimeters).
		arglCameraViewRH((const ARdouble (*)[4])gPatt_trans, m, VIEW_SCALEFACTOR);
#ifdef ARDOUBLE_IS_FLOAT
		glLoadMatrixf(m);
#else
		glLoadMatrixd(m);
#endif
		DrawCube();

		arglCameraViewRH((const ARdouble (*)[4])gPatt_trans2, m2, VIEW_SCALEFACTOR);
        std::cout << m2[0] << std::endl;
#ifdef ARDOUBLE_IS_FLOAT
		glLoadMatrixf(m2);
#else
		glLoadMatrixd(m2);
#endif
		DrawCube();

	} // gPatt_found

	// Any 2D overlays go here.
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glOrtho(0, (GLdouble)windowWidth, 0, (GLdouble)windowHeight, -1.0, 1.0);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	glDisable(GL_LIGHTING);
	glDisable(GL_DEPTH_TEST);

    //
    // Draw help text and mode.
    //
	if (gShowMode) {
		printMode();
	}
	if (gShowHelp) {
		if (gShowHelp == 1) {
			printHelpKeys();
		}
	}

	glutSwapBuffers();
}

int main(int argc, char** argv)
{
	char glutGamemode[32];
	char cparam_name[] = "Data/camera_para.dat";
	char vconf[] = "";

	glutInit(&argc, argv);

	if (!setupCamera(cparam_name, vconf, &gCparamLT, &gARHandle, &gAR3DHandle)) {
		ARLOGe("main(): Unable to set up AR camera.\n");
		exit(-1);
	}

	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH);
	if (!windowed) {
		if (windowRefresh) sprintf(glutGamemode, "%ix%i:%i@%i", windowWidth, windowHeight, windowDepth, windowRefresh);
		else sprintf(glutGamemode, "%ix%i:%i", windowWidth, windowHeight, windowDepth);
		glutGameModeString(glutGamemode);
		glutEnterGameMode();
	} else {
		glutInitWindowSize(windowWidth, windowHeight);
		glutCreateWindow(argv[0]);
	}

	if ((gArglSettings = arglSetupForCurrentContext(&(gCparamLT->param), arVideoGetPixelFormat())) == NULL) {
		ARLOGe("main(): arglSetupForCurrentContext() returned error.\n");
		cleanup();
		exit(-1);
	}
	arglSetupDebugMode(gArglSettings, gARHandle);
	arUtilTimerReset();
	if (!setupMarker(gARHandle, &gARPattHandle)) {
		ARLOGe("main(): Unable to set up AR marker.\n");
		cleanup();
		exit(-1);
	}

	arSetLabelingThreshMode(gARHandle,AR_LABELING_THRESH_MODE_AUTO_OTSU);
	glutDisplayFunc(Display);
	glutReshapeFunc(Reshape);
	glutVisibilityFunc(Visibility);
	glutKeyboardFunc(Keyboard);

	glutMainLoop();

	return (0);
}

static void print(const char *text, const float x, const float y, int calculateXFromRightEdge, int calculateYFromTopEdge)
{
	int i, len;
	GLfloat x0, y0;

	if (!text) return;

	if (calculateXFromRightEdge) {
		x0 = windowWidth - x - (float)glutBitmapLength(GLUT_BITMAP_HELVETICA_10, (const unsigned char *)text);
	} else {
		x0 = x;
	}
	if (calculateYFromTopEdge) {
		y0 = windowHeight - y - 10.0f;
	} else {
		y0 = y;
	}
	glRasterPos2f(x0, y0);

	len = (int)strlen(text);
	for (i = 0; i < len; i++) glutBitmapCharacter(GLUT_BITMAP_HELVETICA_10, text[i]);
}

static void drawBackground(const float width, const float height, const float x, const float y)
{
	GLfloat vertices[4][2];

	vertices[0][0] = x; vertices[0][1] = y;
	vertices[1][0] = width + x; vertices[1][1] = y;
	vertices[2][0] = width + x; vertices[2][1] = height + y;
	vertices[3][0] = x; vertices[3][1] = height + y;
	glLoadIdentity();
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glEnable(GL_BLEND);
	glVertexPointer(2, GL_FLOAT, 0, vertices);
	glEnableClientState(GL_VERTEX_ARRAY);
    glColor4f(0.0f, 0.0f, 0.0f, 0.5f);	// 50% transparent black.
    glDrawArrays(GL_TRIANGLE_FAN, 0, 4);
    glColor4f(1.0f, 1.0f, 1.0f, 1.0f); // Opaque white.
    //glLineWidth(1.0f);
    //glDrawArrays(GL_LINE_LOOP, 0, 4);
    glDisableClientState(GL_VERTEX_ARRAY);
    glDisable(GL_BLEND);
}

static void printHelpKeys()
{
	int i;
	GLfloat  w, bw, bh;
	const char *helpText[] = {
		"Keys:\n",
		" ? or /        Show/hide this help.",
		" q or [esc]    Quit program.",
		" d             Activate / deactivate debug mode.",
		" m             Toggle display of mode info.",
		" a             Toggle between available threshold modes.",
		" - and +       Switch to manual threshold mode, and adjust threshhold up/down by 5.",
		" x             Change image processing mode.",
		" c             Calulcate frame rate.",
	};
#define helpTextLineCount (sizeof(helpText)/sizeof(char *))

	bw = 0.0f;
	for (i = 0; i < helpTextLineCount; i++) {
		w = (float)glutBitmapLength(GLUT_BITMAP_HELVETICA_10, (unsigned char *)helpText[i]);
		if (w > bw) bw = w;
	}
    bh = helpTextLineCount * 10.0f /* character height */+ (helpTextLineCount - 1) * 2.0f /* line spacing */;
	drawBackground(bw, bh, 2.0f, 2.0f);

	for (i = 0; i < helpTextLineCount; i++) print(helpText[i], 2.0f, (helpTextLineCount - 1 - i)*12.0f + 2.0f, 0, 0);;
}

static void printMode()
{
	int len, thresh, line, mode, xsize, ysize;
	AR_LABELING_THRESH_MODE threshMode;
	ARdouble tempF;
	char text[256], *text_p;

	glColor3ub(255, 255, 255);
	line = 1;

    // Image size and processing mode.
	arVideoGetSize(&xsize, &ysize);
	arGetImageProcMode(gARHandle, &mode);
	if (mode == AR_IMAGE_PROC_FRAME_IMAGE) text_p = (char *) "full frame";
	else text_p = (char *)"even field only";
	snprintf(text, sizeof(text), "Processing %dx%d video frames %s", xsize, ysize, text_p);
	print(text, 2.0f,  (line - 1)*12.0f + 2.0f, 0, 1);
	line++;

    // Threshold mode, and threshold, if applicable.
	arGetLabelingThreshMode(gARHandle, &threshMode);
	switch (threshMode) {
		case AR_LABELING_THRESH_MODE_MANUAL: text_p = (char *)"MANUAL"; break;
		case AR_LABELING_THRESH_MODE_AUTO_MEDIAN: text_p = (char *)"AUTO_MEDIAN"; break;
		case AR_LABELING_THRESH_MODE_AUTO_OTSU: text_p = (char *)"AUTO_OTSU"; break;
		case AR_LABELING_THRESH_MODE_AUTO_ADAPTIVE: text_p = (char *)"AUTO_ADAPTIVE"; break;
		case AR_LABELING_THRESH_MODE_AUTO_BRACKETING: text_p = (char *)"AUTO_BRACKETING"; break;
		default: text_p = (char *)"UNKNOWN"; break;
	}
	snprintf(text, sizeof(text), "Threshold mode: %s", text_p);
	if (threshMode != AR_LABELING_THRESH_MODE_AUTO_ADAPTIVE) {
		arGetLabelingThresh(gARHandle, &thresh);
		len = (int)strlen(text);
		snprintf(text + len, sizeof(text) - len, ", thresh=%d", thresh);
	}
	print(text, 2.0f,  (line - 1)*12.0f + 2.0f, 0, 1);
	line++;

    // Border size, image processing mode, pattern detection mode.
	arGetBorderSize(gARHandle, &tempF);
	snprintf(text, sizeof(text), "Border: %0.1f%%", tempF*100.0);
	arGetPatternDetectionMode(gARHandle, &mode);
	switch (mode) {
		case AR_TEMPLATE_MATCHING_COLOR: text_p = (char *)"Colour template (pattern)"; break;
		case AR_TEMPLATE_MATCHING_MONO: text_p = (char *)"Mono template (pattern)"; break;
		case AR_MATRIX_CODE_DETECTION: text_p = (char *)"Matrix (barcode)"; break;
		case AR_TEMPLATE_MATCHING_COLOR_AND_MATRIX: text_p = (char *)"Colour template + Matrix (2 pass, pattern + barcode)"; break;
		case AR_TEMPLATE_MATCHING_MONO_AND_MATRIX: text_p = (char *)"Mono template + Matrix (2 pass, pattern + barcode "; break;
		default: text_p = (char *)"UNKNOWN"; break;
	}
	len = (int)strlen(text);
	snprintf(text + len, sizeof(text) - len, ", Pattern detection mode: %s", text_p);
	print(text, 2.0f,  (line - 1)*12.0f + 2.0f, 0, 1);
	line++;

    // Window size.
	snprintf(text, sizeof(text), "Drawing into %dx%d window", windowWidth, windowHeight);
	print(text, 2.0f,  (line - 1)*12.0f + 2.0f, 0, 1);
	line++;

}

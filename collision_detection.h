//
// Created by asket on 12/26/16.
//

#ifndef AR_COLLISION_DETECTION_H_H
#define AR_COLLISION_DETECTION_H_H


static void cleanup(void);

static void Display(void);

static void DrawCube(void);

static void DrawCubeUpdate(float timeDelta);

static void Keyboard(unsigned char key, int x, int y);

int main(int argc, char **argv);

static void mainLoop(void);

static void Reshape(int w, int h);

static int
setupCamera(const char *cparam_name, char *vconf, ARParamLT **cparamLT_p, ARHandle **arhandle, AR3DHandle **ar3dhandle);

static int setupMarker(ARHandle *arhandle, ARPattHandle **pattHandle_p);

void split(const std::string &s, char delim, std::vector <std::string> &elems);

static void Visibility(int visible);

static size_t WriteCallback(void *contents, size_t size, size_t nmemb, void *userp);

static void
print(const char *text, const float x, const float y, int calculateXFromRightEdge, int calculateYFromTopEdge);

static void drawBackground(const float width, const float height, const float x, const float y);

static void printHelpKeys();

static void printMode();

#endif //AR_COLLISION_DETECTION_H_H

#include "ofApp.h"
#include "ofMain.h"

//========================================================================
int main()
{
    ofGLWindowSettings settings;
    settings.setSize(1024, 1024);
    settings.windowMode = OF_WINDOW; //can also be OF_FULLSCREEN

    const auto window = ofCreateWindow(settings);

    ofRunApp(window, make_shared<ofApp>());
    ofRunMainLoop();
}

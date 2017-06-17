#pragma once

#include "ofMain.h"
#include "ofxOpenCv.h"
#include "MSAPhysics3D.h"

class ofApp : public ofBaseApp{

	public:
		void setup();
		void update();
		void draw();

		void keyPressed(int key);
		void keyReleased(int key);
		void mouseMoved(int x, int y );
		void mouseDragged(int x, int y, int button);
		void mousePressed(int x, int y, int button);
		void mouseReleased(int x, int y, int button);
		void mouseEntered(int x, int y);
		void mouseExited(int x, int y);
		void windowResized(int w, int h);
		void dragEvent(ofDragInfo dragInfo);
		void gotMessage(ofMessage msg);
        
private:
    int width, height;
    ofImage image;
    ofxCvColorImage cvImage;
    ofxCvGrayscaleImage grayImage;
    ofxCvContourFinder contourFinder;
    msa::physics::World3D_ptr         world;
    vector<msa::physics::Particle3D_ptr> moveNodes;
    vector<ofPoint> points;
    vector< vector<int> > connected;
    vector<ofNode> nodes;
    
    enum STATE {
        IMAGE, LINE, ALIVE
    } state;

};

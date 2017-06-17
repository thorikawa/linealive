#include "ofApp.h"

#define NUMP 1000
#define MIN_LEN 5
#define MAX_CONNECTED_LEN 20
#define BLACK_THRESHOLD 150
#define SCORE_THRESHOLD 0.8
#define FPS 60
#define GRAVITY 1
#define UNITS(s) (s * ofGetWidth() / 1280.0)
#define SPRING_MIN_STRENGTH 0.5
#define SPRING_MAX_STRENGTH 1.0

using namespace msa::physics;

//--------------------------------------------------------------
void ofApp::setup(){
    ofBackground(206, 211, 210);
    ofSetVerticalSync(true);
    ofSetFrameRate(FPS);
    
    width = ofGetWidth();
    height = ofGetHeight();

    image.load("IMG_1902.JPG");
    cvImage.setFromPixels(image.getPixels());
    grayImage = cvImage;
    grayImage.threshold(BLACK_THRESHOLD);
    grayImage.erode();
//    grayImage.erode();
//    grayImage.dilate();
//    grayImage.dilate();
    
    contourFinder.findContours(grayImage, 1, 1000, false, true);
    
    world = World3D::create();
    world->setGravity(ofVec3f(0, 0, 0));
//    world->enableCollision();
    world->disableCollision();
    world->setWorldSize(ofVec3f(-width/2, -height, -width/2), ofVec3f(width/2, height, width/2));
    world->setDrag(0.97f);
    world->setDrag(1);
    world->setSectorCount(1);
    
    ofPixels pix = grayImage.getPixels();
    int count = 0;
    int sx, sy;
    int failCount = 0;
    while (count < NUMP && failCount < 50) {
        // found random 1 black point
        sx = (int)(ofRandom(grayImage.width));
        sy = (int)(ofRandom(grayImage.height));
        ofPoint p = ofPoint(sx, sy);
        ofColor c = pix.getColor(sx, sy);
        if (c.r > BLACK_THRESHOLD) {
            continue;
        }
        
        bool isTooClose = false;
        for (int i = 0; i < points.size(); i++) {
            ofPoint q = points[i];
            float len = (p - q).length();
            if (len < MIN_LEN) {
                isTooClose = true;
                break;
            }
        }
        if (isTooClose) {
            failCount++;
            continue;
        }
        failCount = 0;
        
//        printf("push point: %d %d\n", sx, sy);
        points.push_back(p);
        count++;
    }
    printf("%lu point pushed\n", points.size());
    
    int n_links = 0;
    connected.resize(points.size());
    for (int i = 0; i < points.size(); i++) {
        auto p = points[i];
        Particle3D_ptr particle = world->makeParticle(ofVec3f(p.x, p.y, 0));
        particle->setMass(1)->setBounce(0.2)->setRadius(4);
//        if (ofRandom(1.0) < 0.1) {
//            particle->makeFixed();
//        } else {
            particle->makeFree();
//        }
        
        for (int j = i + 1; j < points.size(); j++) {
            if (i == j) continue;
            if (connected[i].size() > 0 && connected[i][0] == j) continue;
            ofPoint p1 = points[i];
            ofPoint p2 = points[j];
            float xdist = abs(p1.x - p2.x);
            float ydist = abs(p1.y - p2.y);
            int maxdist = max(xdist, ydist);
            if (maxdist > MAX_CONNECTED_LEN) {
                continue;
            }

            int total = 0;
            int online = 0;

            if (xdist >= ydist && xdist > 0) {
                float tan = (float)(p2.y - p1.y) / (float)(p2.x - p1.x);
                int step = (p2.x >= p1.x) ? 1 : -1;
                
                for (int x = p1.x; x != p2.x; x += step) {
                    total++;
                    int y = (x - p1.x) * tan + p1.y;
                    ofColor c = pix.getColor(x, y);
                    if (c.r < 100) {
                        online++;
                    }
                }
            } else if (ydist >= xdist && ydist > 0) {
                float tan = (float)(p2.x - p1.x) / (float)(p2.y - p1.y);
                int step = (p2.y >= p1.y) ? 1 : -1;
                
                for (int y = p1.y; y != p2.y; y += step) {
                    total++;
                    int x = (y - p1.y) * tan + p1.x;
                    ofColor c = pix.getColor(x, y);
                    if (c.r < BLACK_THRESHOLD) {
                        online++;
                    }
                }
            } else {
                continue;
            }
            float okratio = 0;
            if (total > 0) {
                okratio = (float)online / (float)total;
            } else {
                printf("error total is zero: %d, %d\n", i, j);
                okratio = 0.5;
            }
            if (okratio >= SCORE_THRESHOLD) {
                connected[i].push_back(j);
                connected[j].push_back(i);
                n_links++;
            }
        }
    }
    printf("%d links created\n", n_links);
    
//    for (int i = 0; i < points.size(); i++) {
//        printf("%d %lu\n", i, connected[i].size());
//    }
    
    for (int i = 0; i < points.size(); i++) {
        auto p1 = world->getParticle(i);
        for (int k = 0; k < connected[i].size(); k++) {
            int j = connected[i][k];
            auto p2 = world->getParticle(j);
            auto pos1 = p1->getPosition();
            auto pos2 = p2->getPosition();
            float len = (pos1 - pos2).length();
            world->makeSpring(p1, p2, 0.3, len);
//            world->makeAttraction(p1, p2, 0.005/len);
        }
    }
    
    for (int j = 0; j<10; j++) {
        int i = ofRandom(world->numberOfParticles());
        Particle3D_ptr p = world->getParticle(i);
//        p->addVelocity(ofVec3f(ofRandom(-1, 1), ofRandom(-1, 1), ofRandom(0, 0)));
    }
    
    for (int i = 0; i < 20; i++) {
        auto mouseNode = world->getParticle(ofRandom(world->numberOfParticles()));
        moveNodes.push_back(mouseNode);
    }
    
    state = IMAGE;
}

//--------------------------------------------------------------
void ofApp::update(){

    uint64_t num = ofGetFrameNum();
    float dt = (float)num / FPS;

    switch(state) {
        case (ALIVE):
            for (int i = 0; i < moveNodes.size(); i++) {
                auto p = moveNodes[i];
                int STEP = 2.0f * (cos(dt + i) + 1.0f);
                float xOffset = (((float)mouseX / (float)width) - 0.5f) * 1;
                float yOffset = (((float)mouseY / (float)height) - 0.5f) * 1;
                printf("%f, %f\n", xOffset, yOffset);
                p->moveBy(ofVec3f(ofRandom(STEP) + xOffset, ofRandom(STEP) + yOffset,  0));
            }
            break;
    }
    world->update();
            
}

//--------------------------------------------------------------
void ofApp::draw(){
//    ofSetColor(0, 0, 0);
//    ofFill();
    
    switch(state) {
        case (IMAGE):
            image.draw(0, 0);
            break;
        case(LINE):
        case(ALIVE):
            for(int i=0; i < world->numberOfParticles(); i++) {
                ofSetColor(255, 0, 0);
                Particle3D_ptr p = world->getParticle(i);
                auto pos = p->getPosition();
                ofDrawCircle(pos.x, pos.y, 1);
                
                ofSetColor(0, 0, 0);
                auto links = connected[i];
                for (int j = 0; j < links.size(); j++) {
                    int k = links[j];
                    auto p2 = world->getParticle(k);
                    auto pos2 = p2->getPosition();
                    ofDrawLine(pos.x, pos.y, pos2.x, pos2.y);
                }
            }

            break;
    }
    
//    for(int i=0; i<world->numberOfSprings(); i++) {
//        Spring3D_ptr spring     = world->getSpring(i);
//    }
    
    
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key){
    switch(key) {
            case '0':
            state = IMAGE;
            break;
            case '1':
            state = LINE;
            break;
            case '2':
            state = ALIVE;
            for (int i = 0; i < points.size(); i++) {
                auto p = points[i];
                Particle3D_ptr particle = world->getParticle(i);
                particle->moveTo(ofVec3f(p.x, p.y, 0));
            }
            break;
    }
}

//--------------------------------------------------------------
void ofApp::keyReleased(int key){

}

//--------------------------------------------------------------
void ofApp::mouseMoved(int x, int y ){
}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mouseEntered(int x, int y){

}

//--------------------------------------------------------------
void ofApp::mouseExited(int x, int y){

}

//--------------------------------------------------------------
void ofApp::windowResized(int w, int h){

}

//--------------------------------------------------------------
void ofApp::gotMessage(ofMessage msg){

}

//--------------------------------------------------------------
void ofApp::dragEvent(ofDragInfo dragInfo){ 

}

#pragma once

#include "ofMain.h"
#include "ofxCv.h"
#include "ofxToast.h"
#include "ofxGui.h"

class testApp : public ofBaseApp{

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
		void windowResized(int w, int h);
		void dragEvent(ofDragInfo dragInfo);
		void gotMessage(ofMessage msg);
		
    void capture();
    void exchangeCamera();
    // try to load existing settings.
    void tryLoad();
    void calcRemap();
    
    // gui event listener
    void onBMSettingChanged(int &i);
    

    // offset = 1 if I have i-sight, so that there are 3 cameras.
    int offset;
    int deviceID; // leftCam's deviceID
    ofVideoGrabber leftCam, rightCam;
    ofxCv::Calibration leftCalib, rightCalib;
    bool boardFound, monoCalibrated, stereoCalibrated;
    cv::Mat rotation, translation;
    ofImage undistortedLeft, undistortedRight;
    ofImage rectifiedLeft, rectifiedRight;
    cv::Mat leftMapX, leftMapY, rightMapX, rightMapY;
    cv::Mat disparityRaw;
    ofImage disparity;
    
    ofxToast toast;
    
    // block matcher
    cv::StereoBM bm;
    
    // gui
    ofxPanel gui;
    ofxIntSlider disparitySlider, windowSizeSlider, minDisparitySlider;
};

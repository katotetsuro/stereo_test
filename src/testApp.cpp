#include "testApp.h"

using namespace ofxCv;

const int camWidth = 640, camHeight = 480;

//--------------------------------------------------------------
void testApp::setup(){
    ofSetFrameRate(30);
//    ofSetWindowShape(1920, 1080);
    ofSetLogLevel(OF_LOG_VERBOSE);
    offset = 1;
    deviceID = 0;
    leftCam.setDeviceID(deviceID + offset);
    leftCam.initGrabber(camWidth, camHeight);
    rightCam.setDeviceID((deviceID+1)%2 + offset);
    rightCam.initGrabber(camWidth, camHeight);
    
    boardFound = monoCalibrated = stereoCalibrated = false;
    
    leftCalib.setPatternSize(4, 11);
    leftCalib.setPatternType(ASYMMETRIC_CIRCLES_GRID);
    leftCalib.setSquareSize(16.5);
    rightCalib.setPatternSize(4, 11);
    rightCalib.setPatternType(ASYMMETRIC_CIRCLES_GRID);
    rightCalib.setSquareSize(16.5);
    
    bm.init(StereoBM::BASIC_PRESET, 16*8, 11);
    
    // gui
    gui.setup();
    gui.add(minDisparitySlider.setup("min disparity", 0, -100, 100));
    gui.add(disparitySlider.setup("disparity(x16)", 8, 1, 100));
    gui.add(windowSizeSlider.setup("windowSize(2*v+1)", 5, 2, 50));
    minDisparitySlider.addListener(this, &testApp::onBMSettingChanged);
    disparitySlider.addListener(this, &testApp::onBMSettingChanged);
    windowSizeSlider.addListener(this, &testApp::onBMSettingChanged);
    
    // gyaku
    exchangeCamera();
}

//--------------------------------------------------------------
void testApp::update(){
    leftCam.update();
    rightCam.update();
    
    if (stereoCalibrated) {
        if (!undistortedLeft.isAllocated()) {
            imitate(undistortedLeft, leftCam);
        }
        if (!undistortedRight.isAllocated()) {
            imitate(undistortedRight, rightCam);
        }
        leftCalib.undistort(toCv(leftCam), toCv(undistortedLeft));
        rightCalib.undistort(toCv(rightCam), toCv(undistortedRight));
        
        undistortedLeft.update();
        undistortedRight.update();
        
        if (!rectifiedLeft.isAllocated()) {
            imitate(rectifiedLeft, leftCam);
        }
        if (!rectifiedRight.isAllocated()) {
            imitate(rectifiedRight, rightCam);
        }
        
        cv::Mat m = toCv(rectifiedLeft);
        cv::remap(toCv(leftCam), m, leftMapX, leftMapY, INTER_LINEAR);
        rectifiedLeft.update();
        
        m = toCv(rectifiedRight);
        cv::remap(toCv(rightCam), m, rightMapX, rightMapY, INTER_LINEAR);
        rectifiedRight.update();
        
        cv::Mat leftGray, rightGray;
        convertColor(rectifiedLeft, leftGray, CV_RGB2GRAY);
        convertColor(rectifiedRight, rightGray, CV_RGB2GRAY);
        
        disparityRaw.create(camHeight, camWidth, CV_32FC1);
        bm(leftGray, rightGray, disparityRaw, CV_32FC1);
        
//        disparityRaw = disparityRaw.colRange(100+camWidth/3, camWidth/3*2).rowRange(camHeight/3, camHeight/3*2);
        double min, max;
        minMaxLoc(disparityRaw, &min, &max);
        
        if (!disparity.isAllocated()) {
            disparity.allocate(disparityRaw.cols, disparityRaw.rows, OF_IMAGE_GRAYSCALE);
        }
        m = toCv(disparity);
        disparityRaw.convertTo(m, CV_8UC1, 255.0/(max-min), -min/(max-min));
        disparity.update();
    }
    
}

//--------------------------------------------------------------
void testApp::draw(){
    leftCam.draw(0,0);
    leftCalib.customDraw();
    
    ofPushMatrix();
    ofTranslate(camWidth, 0);
    rightCam.draw(0, 0);
    rightCalib.customDraw();
    ofPopMatrix();
    
    if (stereoCalibrated) {
        undistortedLeft.draw(0, camHeight);
        undistortedRight.draw(camWidth, camHeight);
        rectifiedLeft.draw(0, camHeight*2);
        rectifiedRight.draw(camWidth, camHeight*2);
        
        ofPushMatrix();
        ofTranslate(camWidth*2, 0);
        disparity.draw(0, 0);
        ofPopMatrix();
    }
    
    gui.draw();
}

//--------------------------------------------------------------
void testApp::keyPressed(int key){
    switch (key) {
        case ' ':
            capture();
            break;
        case 'c':
            exchangeCamera();
            break;
            
        case 's':
            if (leftCalib.isReady() && rightCalib.isReady()) {
                leftCalib.save("left.yml");
                rightCalib.save("right.yml");
                
                if (stereoCalibrated) {
                    saveMat(rotation, "rotation.yml");
                    saveMat(translation, "translation.yml");
                }
                toast.addText("Saved");
            }
            break;
            
        case 't':
            ofSaveImage(leftCam.getPixelsRef(), "left.jpg");
            ofSaveImage(rightCam.getPixelsRef(), "right.jpg");
            toast.addText("frame saved.");
            break;
            
        case 'l':
            tryLoad();
            
        default:
            break;
    }
}

void testApp::capture() {
    vector<cv::Point2f> tmp;
    boardFound = leftCalib.findBoard(toCv(leftCam), tmp) && rightCalib.findBoard(toCv(rightCam), tmp);
    ofLogNotice() << (boardFound ? "corners are found." : "corners are not found.");
    
    if (boardFound) {
        leftCalib.add(toCv(leftCam));
        rightCalib.add(toCv(rightCam));
        
        if (leftCalib.size() > 10) {
            monoCalibrated = leftCalib.calibrate() && rightCalib.calibrate();
            toast.addText("each cameras are calibrated");
        }
    }
    
    if (leftCalib.isReady() && rightCalib.isReady()) {
        ofLogNotice() << "try to stereo calibrate";
        stereoCalibrated = leftCalib.getTransformation(rightCalib, rotation, translation);
    }
    
    if (stereoCalibrated) {
        toast.addText("Stereo Calibrated");
        calcRemap();
    } else if (boardFound) {
        toast.addText("board Found");
    }
}

void testApp::exchangeCamera() {
    leftCam.close();
    rightCam.close();
    deviceID = deviceID ? 0 : 1;
    leftCam.setDeviceID(deviceID + offset);
    leftCam.initGrabber(camWidth, camHeight);
    rightCam.setDeviceID((deviceID+1)%2 + offset);
    rightCam.initGrabber(camWidth, camHeight);
}

void testApp::tryLoad() {
    leftCalib.load("left.yml");
    rightCalib.load("right.yml");
    loadMat(translation, "translation.yml");
    loadMat(rotation, "rotation.yml");
    stereoCalibrated = true;
    
    calcRemap();
}

void testApp::calcRemap() {
    Intrinsics leftIntrinsic = leftCalib.getDistortedIntrinsics();
    cv::Mat R1, R2, P1, P2, Q;
    stereoRectify(leftIntrinsic.getCameraMatrix(), leftCalib.getDistCoeffs(), rightCalib.getUndistortedIntrinsics().getCameraMatrix(), rightCalib.getDistCoeffs(), leftIntrinsic.getImageSize(), rotation, translation, R1, R2, P1, P2, Q);
    
    initUndistortRectifyMap(leftIntrinsic.getCameraMatrix(), leftCalib.getDistCoeffs(), R1, P1, leftIntrinsic.getImageSize(), CV_32FC1 /*CV_16SC2*/, leftMapX, leftMapY);
    initUndistortRectifyMap(rightCalib.getDistortedIntrinsics().getCameraMatrix(), leftCalib.getDistCoeffs(), R2, P2, rightCalib.getDistortedIntrinsics().getImageSize(), CV_32FC1 /*CV_16SC2*/, rightMapX, rightMapY);
    
    ofxCv::saveMat(R1, "leftgeometryrotation.yml");
    ofxCv::saveMat(P1, "leftnewcameramatrix.yml");
    ofxCv::saveMat(leftMapX, "leftmapx.yml");
    ofxCv::saveMat(leftMapY, "leftmapy.yml");
}

void testApp::onBMSettingChanged(int &i) {
    ofLogNotice() << "changing block matching parameters";
    bm.init(StereoBM::BASIC_PRESET, 16*disparitySlider, 2*windowSizeSlider+1);
    bm.state->minDisparity = minDisparitySlider;
}

//--------------------------------------------------------------
void testApp::keyReleased(int key){

}

//--------------------------------------------------------------
void testApp::mouseMoved(int x, int y ){

}

//--------------------------------------------------------------
void testApp::mouseDragged(int x, int y, int button){

}

//--------------------------------------------------------------
void testApp::mousePressed(int x, int y, int button){

}

//--------------------------------------------------------------
void testApp::mouseReleased(int x, int y, int button){

}

//--------------------------------------------------------------
void testApp::windowResized(int w, int h){

}

//--------------------------------------------------------------
void testApp::gotMessage(ofMessage msg){

}

//--------------------------------------------------------------
void testApp::dragEvent(ofDragInfo dragInfo){ 

}

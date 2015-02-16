
//BalancingBallApp
#include "cinder/app/AppNative.h"
#include "cinder/gl/gl.h"
#include "cinder/Capture.h"
#include "cinder/gl/Texture.h"
#include "CinderOpenCV.h"
#include "cinder/params/Params.h"
#include "cinder/Serial.h"
using namespace ci;
using namespace ci::app;
using namespace std;

class BalancingBallApp: public AppNative {
public:
	void setup();
	void mouseDown( MouseEvent event );
	void update();
	void draw();
    
    bool hasCamera;
    CaptureRef			mCapture;
    Surface8u        mSurface;
    gl::TextureRef		mTexture;
    gl::Texture		mTextureResult;
    Serial serial;
    int					hMin;
    int					hMax;
    int					sMin;
    int					sMax;
    int					vMin;
    int					vMax;
    int					divStep;
    
    
    void resetAngele();
    void anglePlus();
    void angleMin();
    params::InterfaceGlRef	mParams;
    cv::SimpleBlobDetector * blob_detector;
    vector<cv::KeyPoint> keypoints;
    bool  isOnline;
    
    Vec2f speedVec;
    Vec2f prevPoint;
    Vec2f currentPoint;
    double prevTrackTime;
    int targetStepPrev;
    float rotation;
    
    void compute();
    
    float kp, ki, kd;
    
    float lastTime;
    float Input, Output, Setpoint;
    float tarX;
    float errSum, lastErr;
    
    
    float rotationStep;
    int maxSpeed;
    float KpStep;
    float maxAccel;
    
    float tarRot;
    void writeIntData(int command, unsigned int data);
    
    
};

void BalancingBallApp::setup()
{
    rotationStep=0;
    maxSpeed =2000;
    KpStep=20;
    maxAccel=40;
    
    kp = 5;
    ki =0.500;
    kd =5;
    Input=0;
    errSum = 0;
    lastErr = 0;
    Setpoint =0;
    tarX = 640/2;
    
    tarRot =0;
    
    
    rotation =0;
    targetStepPrev =0;
    setFrameRate(60);
    cv::SimpleBlobDetector::Params params;
    params.filterByArea = true;
    params.filterByCircularity = false;
    params.filterByConvexity = false;
    params.filterByInertia = false;
    params.filterByColor = false;
    
    params.minArea = 500.0f;
    params.maxArea = 5000.0f;
    
    blob_detector = new cv::SimpleBlobDetector(params);
    blob_detector->create("SimpleBlobDetector");
    
    setWindowSize(1200, 800);
    hMin =0;
    hMax =16;
    sMin =110;
    sMax =255;
    
    vMin =180;
    vMax =255;
    mParams = params::InterfaceGl::create( getWindow(), "Appparameters", toPixels( Vec2i( 300, 400 ) ) );
    mParams->setOptions("", "position='650 10'");
    mParams->addParam( "divStep", &divStep, "min=0 max=100 step=1" );
    
	mParams->addParam( "Hmin", &hMin, "min=0 max=180 " );
    mParams->addParam( "Hmax", &hMax, "min=0 max=180 " );
    mParams->addParam( "Smin", &sMin, "min=0 max=255 " );
    mParams->addParam( "Smax", &sMax, "min=0 max=255 " );
    mParams->addParam( "Vmin", &vMin, "min=0 max=255 " );
    mParams->addParam( "Vmax", &vMax, "min=0 max=255 " );
    
    mParams->addSeparator();
    mParams->addParam( "rotation", &rotation).min( -90.0f ).max( 90.0f ).updateFn( [this] { writeIntData(1,(rotation+90)*100); } );
    mParams->addParam( "maxSpeed", &maxSpeed).min( 100.0f ).max( 4000.0f ).updateFn( [this] { writeIntData(2,maxSpeed) ;} );
    mParams->addParam( "KpMotor", &KpStep).step(0.05).min( 1.0f ).max( 300.0f ).updateFn( [this] { writeIntData(4,KpStep*10); } );
    mParams->addParam( "maxAccel", &maxAccel).step(0.05).min( 1.0f ).max( 1000.0f ).updateFn( [this] { writeIntData(5,maxAccel*10); } );
    mParams->addSeparator();
    
    mParams->addParam("kp", &kp);
    mParams->addParam("ki", &ki);
    mParams->addParam("kd", &kd);
    mParams->addParam("tarX", &tarX);
    
    hasCamera =false;
    for( auto device = Capture::getDevices().begin(); device != Capture::getDevices().end(); ++device ) {
        console() << "Device: " << (*device)->getName() << " "<< std::endl;
        if((*device)->getName().find("HD Pro Webcam C920") !=string::npos)
            //     if((*device)->getName().find("FaceTime HD Camera") !=string::npos)
        {
            try {
                mCapture = Capture::create( 640, 480,(*device) );
                mCapture->start();
                hasCamera =true;
                
                
            }
            catch( ... ) {
                console() << "Failed to initialize capture" << std::endl;
            }
            
        }
        
    }
    
    try {
		Serial::Device dev = Serial::findDeviceByNameContains("tty.usbmodem1451");
		serial = Serial( dev, 115200);
        console() << "Serial Connected" << std::endl;
        
        
        
        
        isOnline =true;
	}
	catch( ... ) {
		console() << "There was an error initializing the serial device!" << std::endl;
		//exit( -1 );
        isOnline =false;
        
        const vector<Serial::Device> &devices( Serial::getDevices() );
        for( vector<Serial::Device>::const_iterator deviceIt = devices.begin(); deviceIt != devices.end(); ++deviceIt ) {
            console() << "Device for MAIN?: " << deviceIt->getName() << endl;
        }
	}
    
}

void BalancingBallApp::mouseDown( MouseEvent event )
{
    
    tarX  = 321 -150 +rand()%300;//+ sin(getElapsedSeconds())*150
    
}
void BalancingBallApp::compute()
{
    /*How long since we last calculated*/
    double now = getElapsedSeconds()*1000;
    double timeChange = (double)(now - lastTime);
    
    /*Compute all the working error variables*/
    float error = Setpoint - Input;
    errSum += (error / timeChange);
    if (ki ==0)errSum = 0;
    
    float dErr = (error - lastErr) / timeChange;
    if (kd == 0)dErr = 0;
    /*Compute PID Output*/
    Output = kp * error + ki * errSum + kd * dErr;
    
    /*Remember some variables for next time*/
    lastErr = error;
    lastTime = now;
}

void BalancingBallApp::update()
{
    
    tarX = 321 + sin(getElapsedSeconds())*150;
    
    if( mCapture && mCapture->checkNewFrame() ) {
        
        mSurface =mCapture->getSurface();
        cv::Mat   src_brg;
        cv::Mat   src_gray;
        cv::Mat    imgThresholded;
        cv::Mat inputRaw( toOcv( mSurface) );
        
        cv::Rect myRoi(0, 200, 640, 80);
        cv::Mat croppedImage;
        cv::Mat(inputRaw, myRoi).copyTo(croppedImage);
        mTexture = gl::Texture::create( fromOcv(croppedImage)  );
        
        keypoints.clear();
        int iLowH =hMin;
        int iHighH =hMax;
        
        int iLowS = sMin;
        int iHighS = sMax;
        
        int iLowV =vMin;
        int iHighV = vMax;
        
        
        cv::Mat imgHSV;
        
        cvtColor(croppedImage, imgHSV, cv::COLOR_BGR2HSV);
        
        
        
        inRange(imgHSV, cv::Scalar(iLowH, iLowS, iLowV), cv::Scalar(iHighH, iHighS, iHighV), imgThresholded);
        
        blob_detector->detect(imgThresholded, keypoints);
        mTextureResult = gl::Texture( fromOcv(imgThresholded) );
        if(keypoints.size() ==1)
        {
            currentPoint.x=keypoints[0].pt.x;
            currentPoint.y=keypoints[0].pt.y;
            double currentTrackTime = getElapsedSeconds();
            double timeElapsed =currentTrackTime-prevTrackTime;
            
            speedVec.x  = (currentPoint.x-  prevPoint.x)*60 ;
            console()<<speedVec.x<<" "<< timeElapsed<<endl;
            prevTrackTime=currentTrackTime;
            prevPoint = currentPoint;
            
            Input =speedVec.x;
            
            float disError = tarX -currentPoint.x;
            
            float tarVel = disError*4;
            
            //tarVel;
            Setpoint = tarVel;
            compute();
            rotation = (Output / 1000);
            
         
            
        }
        if(keypoints.size() ==1)
        {
            rotation*=2;
            if(rotation>30)rotation =30;
            if(rotation<-30)rotation =-30;
            
            tarRot= rotation;
            
            writeIntData(1,(rotation+90)*100);
            
            
        }
        
    }
    
}
void BalancingBallApp::writeIntData(int command, unsigned int data)
{
    if( !isOnline )return;
    console()<<"write"<<command<<" "<<data<<endl;
    int i0 =data/10000;
    data-= i0*10000;
    int i1 =data/100;
    int i2 =data -i1*100;
    serial.writeByte(command);
    serial.writeByte(i0);
    serial.writeByte(i1);
    serial.writeByte(i2);
    serial.writeByte(0xFF);
    
}

void BalancingBallApp::draw()
{
	// clear out the window with black
	gl::clear( Color( 0, 0, 0 ) );
    gl::pushMatrices();
    gl::scale(2,2);
    if(mTexture){
        gl::enableAlphaBlending();
        gl::color(1, 1, 1,1);
        gl::draw(mTexture);
        gl::draw(mTextureResult,Vec2f(0,80));
        gl::drawLine(Vec2f(320,0), Vec2f(320,80));
        
        
        gl::color(1, 1, 0,1);
        for( size_t i = 0; i < keypoints.size(); i++ )
        {
            
            
            
            
            gl::drawSolidCircle(currentPoint, 10);
        }
        gl::pushMatrices();
        gl::color(0, 1, 1,1);
        gl::translate(Vec2f(640/2,200));
        gl::rotate(Vec3f(0,0,rotation));
        
        gl::drawLine(  Vec2f(-200,0), Vec2f(200,0)   );
        
        gl::popMatrices();
        
        
        gl::color(1, 1, 1,1);
        gl::drawLine(  Vec2f(currentPoint.x,200), Vec2f(currentPoint.x +speedVec.x ,200)   );
        
        gl::color(1, 0, 0,1);
        gl::drawLine(  Vec2f(currentPoint.x,220), Vec2f(currentPoint.x +rotation,220)   );
        
        gl::color(1, 0, 1,1);
        gl::drawLine(  Vec2f(currentPoint.x,0), Vec2f(currentPoint.x,250)   );
        gl::color(1, 0, 0,1);
        gl::drawLine(  Vec2f(tarX,0), Vec2f(tarX,250)   );
        
    }
    gl::popMatrices();
    mParams->draw();
}

CINDER_APP_NATIVE( BalancingBallApp, RendererGl )
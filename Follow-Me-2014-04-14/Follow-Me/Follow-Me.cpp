// Follow-Me.cpp : définit le point d'entrée pour l'application console.
//

#include "stdafx.h"
#include "Kinect.h"
#include "ImageProcess.h"
#include "OneEuroFilter.h"

#include <random>
#include <time.h>

static int frame=0;
static Point mousePoint;
Mat backgroundDepth=Mat::zeros(480,640,CV_16UC1);
ImageProcess imageProcess;

double cutFrequence=0.3;
double betaValue=0.0007;
OneEuroFilter* oneEuroFilter=new OneEuroFilter(cutFrequence,betaValue);

std::default_random_engine generator;
std::normal_distribution<double> distribution(0.0,5.0);

void mouseEvent(int evt, int x, int y, int flags, void* param) 
{                    
	if(evt==EVENT_MOUSEMOVE)
	{
		mousePoint.x=x;
		mousePoint.y=y;
	}

	//if(evt== EVENT_LBUTTONDOWN )
	//{
	//	cutFrequence+=0.1;
	//	oneEuroFilter->setMinCutOff(cutFrequence);
	//	std::cout<<"cutOff "<<cutFrequence<<std::endl;
	//}

	//if(evt==EVENT_RBUTTONDOWN)
	//{
	//	cutFrequence-=0.1;
	//	oneEuroFilter->setMinCutOff(cutFrequence);
	//	if(cutFrequence<0)
	//		cutFrequence=0;
	//	std::cout<<"cutOff "<<cutFrequence<<std::endl;
	//}
	if(evt==EVENT_LBUTTONDBLCLK)
	{
		betaValue+=0.0001;
		oneEuroFilter->setBeta(betaValue);
		std::cout<<"beta "<<betaValue<<std::endl;
	}
	if(evt==EVENT_RBUTTONDBLCLK)
	{
		betaValue-=0.0001;
		
		if(betaValue<0)
			betaValue=0;

		oneEuroFilter->setBeta(betaValue);
		std::cout<<"beta "<<betaValue<<std::endl;
	}
}

void catchScreen(int& frame,Kinect kinect)
{
	//Get the background depth image by computing the average of 60 frames
	int backCount=60;
	if(frame<backCount)
		backgroundDepth+=kinect.getCorrectedImage();
	if(frame==backCount)
		backgroundDepth=backgroundDepth/frame;
	if(frame>backCount)
	{
		cv::imwrite("backgroundDepth.png",backgroundDepth);
		Mat normalizeBackGround;
		backgroundDepth.convertTo(normalizeBackGround,CV_8UC1,255.0/kinect.getMaxDepth());
		imshow("background",normalizeBackGround);
	}
	frame++;
}

void getScreenDepth(vector<Point> screenCorners)
{
	//Get the average background depth image
	backgroundDepth=cv::imread("backgroundDepth.png",CV_LOAD_IMAGE_ANYCOLOR|CV_LOAD_IMAGE_ANYDEPTH);

	Mat screenDepth=Mat::zeros(480,640,CV_16UC1);
	unsigned short* screenP=(unsigned short*)screenDepth.data;
	unsigned short* backP=(unsigned short*)backgroundDepth.data;
	//Save only the screen depth image
	for(int i=0;i<640;++i)
	{
		for(int j=0;j<480;++j)
		{
			Point point(i,j);
			if(cv::pointPolygonTest(screenCorners,point,false)>=0)
				*(screenP+j*640+i)=*(backP+j*640+i);
		}
	}

	Mat normalizeScreenground;
	screenDepth.convertTo(normalizeScreenground,CV_8UC1,255.0/10000);
	imshow("screenDepth",normalizeScreenground);

	cv::imwrite("screenDepth.png",screenDepth);
}

int _tmain(int argc, _TCHAR* argv[])
{
	vector<Point> screenCorners(4);
	screenCorners[0]=Point(67,45);
	screenCorners[1]=Point(555,45);
	screenCorners[2]=Point(569,320);
	screenCorners[3]=Point(58,327);

	
	
	//Initial kinect
	Kinect kinect;

	/*bool connection=imageProcess.connect();*/
	while(true/*&&connection==true*/)
	{
		//Get images of each frame
		kinect.refreshFrame();

		int maxDepth;
		maxDepth=kinect.getMaxDepth();

		Mat colorImage=kinect.getColorImage();
		Mat depthImage=kinect.getNormalizedImage();

		//catchScreen(frame,kinect);
		//getScreenDepth(screenCorners);
		
		for(int i=0;i<screenCorners.size();++i)
		{
			circle(colorImage,screenCorners[i],2,Scalar(0,255,0));
		}
		imshow("colorImage",colorImage);
		imshow("depthImage",depthImage);

		//Mat screenDepth=cv::imread("backgroundDepth.png",CV_LOAD_IMAGE_ANYCOLOR|CV_LOAD_IMAGE_ANYDEPTH);
		//Mat screenDepthNormalize;
		//cv::convertScaleAbs(screenDepth,screenDepthNormalize);
		//imshow("screenDepth",screenDepthNormalize);

		//Camera camera;
		//vector<Point3f> screenCorners3D(0);
		//for(int i=0;i<screenCorners.size();++i)
		//{
		//	unsigned short depth=kinect.getCorrectedImage().at<unsigned short>(screenCorners[i].y,screenCorners[i].x);
		//	Matx31f pos3D=camera.depth2D2World3D(screenCorners[i].x,screenCorners[i].y,depth,2);
		//	Point3f point3D(pos3D(0),pos3D(1),pos3D(2));
		//	screenCorners3D.push_back(point3D);
		//}

		imageProcess.setColorImage(colorImage);
		imageProcess.setDepthImage(kinect.getCorrectedImage());
		imageProcess.extractHand();

		//////////OneEuroFilter
	/*	Mat mouseImage=Mat::zeros(480,640,CV_8UC3);
	
		setMouseCallback("mouseImage",mouseEvent,NULL);

		double numberX=distribution(generator);
		double numberY=distribution(generator);
		
		Point noisePoint=mousePoint+Point(numberX,numberY);
		circle(mouseImage,noisePoint,10,Scalar(0,255,255),2);

		
		int64 tick=cv::getTickCount();
		Point filterPoint=oneEuroFilter->filter(noisePoint,tick);
		circle(mouseImage,filterPoint,5,Scalar(0,255,0),2);

		imshow("mouseImage",mouseImage);*/
        
		


		

		

		if(waitKey(1)=='q')
		{
			break;
		}
	}
	return 0;
}


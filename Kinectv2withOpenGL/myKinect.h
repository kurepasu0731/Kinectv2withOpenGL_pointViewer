#pragma once

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string>
#include <stdint.h>
#include <memory>
#include <fstream>
#include <iomanip>

#include <opencv2/opencv.hpp>
#include <kinect.h>
#include "ComPtr.h"

using namespace cv;
using namespace std;

#define ERROR_CHECK( ret )  \
	if ( (ret) != S_OK ) {    \
	std::stringstream ss;	\
	ss << "failed " #ret " " << std::hex << ret << std::endl;			\
	throw std::runtime_error( ss.str().c_str() );			\
	}

namespace kinect{
	int colorWidth;
	int colorHeight;
	int depthWidth;
	int depthHeight;
	int infraredWidth;
	int infraredHeight;

	IKinectSensor* kinect = nullptr;
	IColorFrameReader* colorFrameReader = nullptr;
	unsigned int colorBytesPerPixel = 4;
	vector<BYTE> colorBuffer;
	unsigned int bufferSize; 
	cv::Mat rawBuffer; //多分depthの生データ
	IDepthFrameReader* depthFrameReader = nullptr;
	vector<UINT16> depthBuffer;
	IInfraredFrameReader* infraredFrameReader = nullptr;
	vector<UINT16> infraredBuffer;

	int minDepth;
	int maxDepth;
	UINT16 minDepthReliableDistance;
	UINT16 maxDepthReliableDistance;

	ICoordinateMapper *coordinateMapper;
	vector<ColorSpacePoint> colorPoints;
	vector<DepthSpacePoint> depthPoints;
	vector<CameraSpacePoint> cameraPoints;


	Mat colorImage;
	Mat colorImage_half;
	Mat depthImage;
	Mat infraredImage;
	Mat coordinatedImage; //MapDepthFrameToColorSpace
	Mat coordinatedImage2; //MapColorFrameToDepthSpace

	std::string colorWinName = "color window";
	std::string depthWinName = "depth window";
	std::string infraredWinName = "infrared window";
	std::string coordinatedWinName = "MapDepthFrameToColorSpace coordinated window";
	std::string coordinatedWinName2 = "MapColorFrameToDepthSpace coordinated window";

	int image_idx = 0;
	std::string outdir = "./capture";

	void init();
	void initColorFrame();
	void initDepthFrame();
	void initIRFrame();
	void updateColorFrame();
	void updateDepthFrame();
	void updateIRFrame();
	void coordinateColorDepth();
	bool isValidColorFrameRange(float x, float y);
	bool isValidDepthFrameRange(float x, float y);
	bool isValidDepthRange(int index);
	void draw();
	void saveDepth_PNG();
	void saveColor();
	void saveIR();

	void init(){
		try{
			ERROR_CHECK(GetDefaultKinectSensor(&kinect));
			ERROR_CHECK(kinect->Open());
			BOOLEAN isOpen = false;
			ERROR_CHECK(kinect->get_IsOpen(&isOpen));
			if(!isOpen) throw runtime_error("Kinect cannot open.");

			kinect->get_CoordinateMapper(&coordinateMapper);

			initColorFrame();
			initDepthFrame();
			initIRFrame();

			namedWindow(colorWinName);
			namedWindow(depthWinName);
			//namedWindow(infraredWinName);
			//namedWindow(coordinatedWinName);
			//namedWindow(coordinatedWinName2);

		}catch(exception& ex){
			cout << ex.what() << endl;
		}
	}

	void initIRFrame(){
		ComPtr<IInfraredFrameSource> infraredFrameSource;
		ERROR_CHECK( kinect->get_InfraredFrameSource( &infraredFrameSource ) );
		ERROR_CHECK( infraredFrameSource->OpenReader( &infraredFrameReader ) );
 
		// 赤外線画像のサイズを取得する
		ComPtr<IFrameDescription> infraredFrameDescription;
		ERROR_CHECK( infraredFrameSource->get_FrameDescription( &infraredFrameDescription ) );
		ERROR_CHECK( infraredFrameDescription->get_Width( &infraredWidth ) );
		ERROR_CHECK( infraredFrameDescription->get_Height( &infraredHeight ) );
 
		// バッファーを作成する
		infraredBuffer.resize( infraredWidth * infraredHeight );
	}

	void initColorFrame(){
		ComPtr<IColorFrameSource> colorFrameSource;
		ERROR_CHECK(kinect->get_ColorFrameSource(&colorFrameSource));
		ERROR_CHECK(colorFrameSource->OpenReader(&colorFrameReader));

		ComPtr<IFrameDescription> colorFrameDescription;
		ERROR_CHECK( colorFrameSource->CreateFrameDescription( 
			ColorImageFormat::ColorImageFormat_Bgra, &colorFrameDescription ) );
		ERROR_CHECK( colorFrameDescription->get_Width( &colorWidth ) );
		ERROR_CHECK( colorFrameDescription->get_Height( &colorHeight ) );
		ERROR_CHECK( colorFrameDescription->get_BytesPerPixel( &colorBytesPerPixel ) );

		colorBuffer.resize(colorWidth * colorHeight * colorBytesPerPixel);
	}

	void initDepthFrame(){
		ComPtr<IDepthFrameSource> depthFrameSource;
		ERROR_CHECK( kinect->get_DepthFrameSource( &depthFrameSource ) );
		ERROR_CHECK( depthFrameSource->OpenReader( &depthFrameReader ) );

		ComPtr<IFrameDescription> depthFrameDescription;
		ERROR_CHECK( depthFrameSource->get_FrameDescription( &depthFrameDescription ) );
		ERROR_CHECK( depthFrameDescription->get_Width( &depthWidth ) );
		ERROR_CHECK( depthFrameDescription->get_Height( &depthHeight ) );

		ERROR_CHECK( depthFrameSource->get_DepthMinReliableDistance( &minDepthReliableDistance ) );
		ERROR_CHECK( depthFrameSource->get_DepthMaxReliableDistance( &maxDepthReliableDistance ) );
		minDepth = minDepthReliableDistance;
		maxDepth = maxDepthReliableDistance;

		depthBuffer.resize(depthWidth * depthHeight);

		bufferSize = depthWidth * depthHeight * sizeof( unsigned short );
	}

	void updateIRFrame(){
		// フレームを取得する
		ComPtr<IInfraredFrame> infraredFrame;
		auto ret = infraredFrameReader->AcquireLatestFrame( &infraredFrame );
		if ( ret == S_OK ){
			// BGRAの形式でデータを取得する
			ERROR_CHECK( infraredFrame->CopyFrameDataToArray( infraredBuffer.size(), &infraredBuffer[0] ) );

			cv::Mat re( infraredHeight, infraredWidth, CV_16UC1, (unsigned short*)&infraredBuffer[0] );
			re.convertTo(infraredImage, CV_8UC1, 255.0f / 65535, 0.0f);
		}
	}


	void updateColorFrame(){
		ComPtr<IColorFrame> colorFrame;
		auto ret = colorFrameReader->AcquireLatestFrame( &colorFrame );
		if ( ret != S_OK ){
			return;
		}

		ERROR_CHECK( colorFrame->CopyConvertedFrameDataToArray(
			colorBuffer.size(), &colorBuffer[0], ColorImageFormat::ColorImageFormat_Bgra ) );

		 Mat re(colorHeight, colorWidth, CV_8UC4, &colorBuffer[0]);
		 cvtColor(re, colorImage, CV_RGBA2RGB);

	}

void saveColor()
{
	std::ostringstream s;
	string filename_jpg, filename_png, filename;

	s.fill('0');
	s << std::right << std::setw(4) << image_idx;
	filename = s.str() + "-c1.png";
	filename_jpg = "colorCamera" + s.str() + ".jpg";
	filename_png = "colorCamera" + s.str() + ".png";

	//JPG
	//cv::imwrite(outdir + "/" + filename_jpg, colorImage);
	cv::imwrite(outdir + "/" + filename, colorImage);
	//PNG
	//cv::imwrite(outdir + "/" + filename_png, colorImage);

}

void saveDepth_PNG()
{
	std::ostringstream s;
	string filename;

	s.fill('0');
	s << std::right << std::setw(4) << image_idx;
	filename = s.str() + "-d1.png";

	cv::imwrite(outdir + "/" + filename, rawBuffer);

}

void saveIR()
{
	std::ostringstream s;
	string filename;

	s.fill('0');
	s << std::right << std::setw(4) << image_idx;
	//filename = s.str() + "-r1.jpg";
	filename = "irCamera" + s.str() + ".jpg";

	//JPG
	cv::imwrite(outdir + "/" + filename, infraredImage);
}

void updateDepthFrame(){
	ComPtr<IDepthFrame> depthFrame;
	auto ret = depthFrameReader->AcquireLatestFrame( &depthFrame );
	if ( ret != S_OK ){
		return;
	}

	ERROR_CHECK( depthFrame->CopyFrameDataToArray( depthBuffer.size(), &depthBuffer[0] ) );

	Mat raw(depthHeight, depthWidth, CV_16UC1, (unsigned short*)&depthBuffer[0]);
	raw.convertTo(depthImage, CV_8UC1, 255.0f / 8000, 0.0f); //表示するために8bitに変換する
	rawBuffer = raw.clone(); //8bitに変換しない

}

void coordinateColorDepth(){
	colorPoints.resize(depthBuffer.size());
	ERROR_CHECK(coordinateMapper->MapDepthFrameToColorSpace(depthBuffer.size(), &depthBuffer[0], colorPoints.size(), &colorPoints[0]));

	Mat re(depthHeight, depthWidth, CV_8UC4);

	for(int i = 0; i < depthBuffer.size(); i++){
		int colorX = (int)colorPoints[i].X;
		int colorY = (int)colorPoints[i].Y;

		int colorIndex = (colorY * colorWidth + colorX) * colorBytesPerPixel;
		int depthIndex = i * colorBytesPerPixel;

		if(isValidColorFrameRange(colorX, colorY) && isValidDepthRange(i)){
			re.data[depthIndex + 0] = colorBuffer[colorIndex + 0];
			re.data[depthIndex + 1] = colorBuffer[colorIndex + 1];
			re.data[depthIndex + 2] = colorBuffer[colorIndex + 2];
		}else{
			re.data[depthIndex + 0] = 0;
			re.data[depthIndex + 1] = 0;
			re.data[depthIndex + 2] = 0;
		}
	}
	coordinatedImage = re.clone();
}

void coordinateDepthColor(){
	depthPoints.resize(colorWidth * colorHeight);
	ERROR_CHECK(coordinateMapper->MapColorFrameToDepthSpace(depthBuffer.size(), &depthBuffer[0], depthPoints.size(), &depthPoints[0]));

	Mat re(colorHeight, colorWidth, CV_8UC4);

	for(int i = 0; i < re.total(); i++){
		int depthX = (int)depthPoints[i].X;
		int depthY = (int)depthPoints[i].Y;

		int depthIndex = (depthY * depthWidth) + depthX;
		int colorIndex = i * colorBytesPerPixel;

		if(isValidDepthFrameRange(depthX, depthY) && isValidDepthRange(depthIndex)){
			re.data[colorIndex + 0] = colorBuffer[colorIndex + 0];
			re.data[colorIndex + 1] = colorBuffer[colorIndex + 1];
			re.data[colorIndex + 2] = colorBuffer[colorIndex + 2];
		}else{
			re.data[colorIndex + 0] = 255;
			re.data[colorIndex + 1] = 255;
			re.data[colorIndex + 2] = 255;
		}
	}
	coordinatedImage2 = re.clone();
}

void coordinateCameraSpace()
{
	cameraPoints.resize(depthWidth * depthHeight);
	ERROR_CHECK(coordinateMapper->MapDepthFrameToCameraSpace(depthBuffer.size(), &depthBuffer[0], cameraPoints.size(), &cameraPoints[0]));
}

void draw(){
	if(colorImage.data != nullptr) {
		Size half(colorWidth/2, colorHeight/2);
		colorImage_half = colorImage.clone();
		resize(colorImage_half, colorImage_half,half);
		imshow(colorWinName, colorImage_half);
	}
	if(depthImage.data != nullptr) imshow(depthWinName, depthImage);
	//if(infraredImage.data != nullptr) imshow(infraredWinName, infraredImage);
	if(coordinatedImage.data != nullptr) imshow(coordinatedWinName, coordinatedImage);
	//if(coordinatedImage2.data != nullptr){
	//	Size half(colorWidth/2, colorHeight/2);
	//	resize(coordinatedImage2, coordinatedImage2,half);
	//	imshow(coordinatedWinName2, coordinatedImage2);
	//}
}

bool isValidColorFrameRange(float x, float y){
	return ((0 <= x) && (x < colorWidth)) && ((0 <= y) && (y < colorHeight));
}

bool isValidDepthFrameRange(float x, float y){
	return ((0 <= x) && (x < depthWidth)) && ((0 <= y) && (y < depthHeight));
}

bool isValidDepthRange( int index )
{
	return (minDepth <= depthBuffer[index]) && (depthBuffer[index] <= maxDepth);
}

}
//構成マネージャのアクティブソリューションプラットフォームはx64で！(freeglutのdllやlibをx64用にビルドしたため)
#include <gl/freeglut.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string>

#include <opencv2/opencv.hpp>
#include "myKinect.h"
#include <iostream>


float theta = 135; //135
float fai = 0;
float r = 1.0f;

int width = 512;
int height = 424;

//xmlからの対応点復元先
vector<Point3d> loadresult;


//Depthの移動平均法による精度向上
const int nFrame = 100;      // 移動平均を行うフレーム数 30なら30FPSなので１秒分  100フレーム位が良好
const double Kd = 1 / (double)nFrame; // 移動平均を除算を使わず乗算で求めるための係数
const int Dx = 512;          // 画面イメージの水平方向画素数
const int Dy = 424;          // 画面イメージの垂直方向画素数
const int dByte = 4;         // XRGB形式なので４バイト
const int dPixels = Dx * Dy; // 1フレーム分の画素数
int ptr = 0;                 // 移動平均を行う為のデータ格納ポインタ


UINT16 *nDepthBuffer = new UINT16[dPixels * nFrame]; // nFrame分のデプスバッファ [dPixels * nFrame]
long *Sum = new long[dPixels]; // nFrame分の移動加算値を格納する為のバッファ[dPixels]

//平均結果のDepth
unsigned short* aveDepthData = new unsigned short[dPixels];

//平均結果を用いたCameraSpacePoint
CameraSpacePoint *cameraSpacePoints_ave = NULL;


void FIFOFilter()
{
    int j = dPixels * ptr;
    for (int i = 0; i < dPixels; i++)
    {
		Sum[i] += (long)kinect::depthBuffer[i] - (long)nDepthBuffer[j + i]; // 移動加算値Sum[i]の変化成分のみ修正
		nDepthBuffer[j + i] = kinect::depthBuffer[i]; // 新規データDataIn[i]をバッファに格納
		//cout << "Sum[" << i << "]: " << Sum[i] << endl;

    }
    ptr++;
    if (ptr == nFrame) { ptr = 0; } //【バッファポインタ更新】
}

void capture()
{
	kinect::updateColorFrame();
	kinect::updateDepthFrame();
	kinect::coordinateColorDepth();
	kinect::coordinateCameraSpace();
}

void keyboard(unsigned char key, int x, int y)
{
	switch(key)
	{
	  case 'j':
		theta = theta - 0.1f;
		break;
	  case 'k':
		theta = theta + 0.1f;
		break;
	  case 'h':
		fai = fai + 0.1f;
		break;
	  case 'l':
		fai = fai - 0.1f;
		break;
	  case 'i':
		  r = r * 1.1f;
		  break;
	  case 'm':
		  r = r * 0.9f;
		  break;
	  case 'q':
		  kinect::kinect->Close();
		  kinect::kinect->Release();
		exit(0);
		break;
	}
}

void draw_correspondPoints()
{
	//対応点群の表示
	glPointSize(3.0f);
	glBegin(GL_POINTS);
	for(int i = 0; i < loadresult.size(); i+=3)
	{
		float x = loadresult[i].x;
		float y = loadresult[i].y;
		float z = loadresult[i].z;
		glColor3f(255, 0, 0);
		glVertex3f(x, y, z);
	}
	glEnd();
}

void display()
{
	//画面のキャプチャ
	capture();

	kinect::draw();

	glEnable(GL_DEPTH_TEST);

	//画面が細かいので、適当なピッチに粗くする
	int pitch = 1;
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glLoadIdentity(); //変換行列の初期化
	float cx = 0.0f;
	float cy = 0.0f;
	float cz = 1.0f;
	float x = r * cos(fai) * sin(theta) + cx;
	float y = r * sin(fai) + cy;
	float z = r * cos(fai) * cos(theta) + cz;
	gluLookAt(x, y, z, cx, cy, cz, 0.0f, 1.0f, 0.0f);

	uchar *color = kinect::coordinatedImage.data;
	int channels = kinect::coordinatedImage.channels();

	//height - 1, width - 1まで
	for(int i = 0; i < height - pitch; i += pitch){
		for(int j= 0; j < width - pitch; j+= pitch){
			int index = i * width + j;
			float z0 = kinect::cameraPoints[index].Z;
			float z1 = kinect::cameraPoints[index + pitch].Z;
			float z2 = kinect::cameraPoints[index + width*pitch + pitch].Z;
			float z3 = kinect::cameraPoints[index + width*pitch].Z;
			if(z0 > 0 && z1 > 0 && z2 > 0 && z3 > 0)
			{
				float x0 = kinect::cameraPoints[index].X;
				float x1 = kinect::cameraPoints[index + pitch].X;
				float x2 = kinect::cameraPoints[index + width*pitch + pitch].X;
				float x3 = kinect::cameraPoints[index + width*pitch].X;
				float y0 = kinect::cameraPoints[index].Y;
				float y1 = kinect::cameraPoints[index + pitch].Y;
				float y2 = kinect::cameraPoints[index + width*pitch + pitch].Y;
				float y3 = kinect::cameraPoints[index + width*pitch].Y;
				glBegin(GL_POLYGON);
				//glColor3f(*(kinect::coordinatedImage.data + (index * 4) + 0), *(kinect::coordinatedImage.data + (index * 4) + 1), *(kinect::coordinatedImage.data + (index * 4) + 2));
				//glColor3ubv(kinect::coordinatedImage.data + (index * 4));
				glColor3ubv((color + (index * channels)));
				glVertex3f(x0, y0, z0);
				glVertex3f(x1, y1, z1);
				glVertex3f(x2, y2, z2);
				glVertex3f(x3, y3, z3);
				glEnd();
			}
		}
	}

	draw_correspondPoints();

	glutSwapBuffers();
}

void display_points()
{
	//画面のキャプチャ
	capture();

	kinect::draw();

	//移動平均
	FIFOFilter();
	//除算->データ格納
	for(int i = 0; i < dPixels; i++)
	{
		aveDepthData[i] = (unsigned short)(Sum[i]*Kd); //移動加算値Sum[i]から移動平均値kを求める
	}

	//CameraSpacePointへ変換
	ERROR_CHECK(kinect::coordinateMapper->MapDepthFrameToCameraSpace(
		width * height, (UINT16*)aveDepthData, 
	      width * height, cameraSpacePoints_ave),"MapDepthFrameToCameraSpace_ave");



	//**描画処理**//
	glEnable(GL_DEPTH_TEST);

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glLoadIdentity(); //変換行列の初期化
	float cx = 0.0f;
	float cy = 0.0f;
	float cz = 1.0f;
	float x = r * cos(fai) * sin(theta) + cx;
	float y = r * sin(fai) + cy;
	float z = r * cos(fai) * cos(theta) + cz;
	gluLookAt(x, y, z, cx, cy, cz, 0.0f, 1.0f, 0.0f);

	uchar *color = kinect::coordinatedImage.data;
	int channels = kinect::coordinatedImage.channels();

	glPointSize(1.0f);
	glBegin(GL_POINTS);
	for(int y = 0; y < height; y++){
		for(int x = 0; x < width; x++){
			int index = y * width + x;

			//float px = kinect::cameraPoints[index].X;
			//float py = kinect::cameraPoints[index].Y;
			//float pz = kinect::cameraPoints[index].Z;

			float px = cameraSpacePoints_ave[index].X;
			float py = cameraSpacePoints_ave[index].Y;
			float pz = cameraSpacePoints_ave[index].Z;

			glColor3ubv((color + (index * channels)));
			glVertex3f(px, py, pz);
		}
	}
	glEnd();

	draw_correspondPoints();

	glutSwapBuffers();
}

//idle状態になったら、再描画を強制する
//これにより、無限ループの表示が可能になる
void idle()
{
	glutPostRedisplay();
}

void loadFile(const string& filename)
{
	FileStorage fs("../g_worldPointInlierSet.xml", FileStorage::READ);
	FileNode node(fs.fs, NULL);

	read(node["points"], loadresult);

	cout << "file loaded." << endl;
}

int main(int argc, char **argv){
	printf("size of RGBQUAD %d\n", sizeof(RGBQUAD));
	printf("size of CameraSpacePoint %d\n", sizeof(CameraSpacePoint));
	//Kinectの初期化
	kinect::init();

	//移動平均用配列の初期化
	 for (int i = 0; i < dPixels; i++) { Sum[i] = 0; } //移動加算バッファを0クリア
	 for (int i = 0; i < dPixels * nFrame; i++) { nDepthBuffer[i] = 0; } //0クリア
	 cameraSpacePoints_ave = new CameraSpacePoint[width * height]; //CameraSpacePoint配列初期化


	//xmlファイルのロード
	loadFile("../g_worldPointInlierSet.xml");

	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_DEPTH); //ダブルバッファ、デブスバッファ用意
	glEnable(GL_DEPTH_TEST);

	glutCreateWindow("Kinect");

	glMatrixMode(GL_PROJECTION); //透視投影変換
	glLoadIdentity(); //変換行列初期化
	gluPerspective(45.0f, (float)width/height, 0.5f, 4.0f); //クリッピング領域指定

	glMatrixMode(GL_MODELVIEW);
	//ハンドラ関数の設定
	//glutDisplayFunc(display);
	glutDisplayFunc(display_points);
	glutKeyboardFunc(keyboard);
	glutIdleFunc(idle);
	glutMainLoop();
}

//�\���}�l�[�W���̃A�N�e�B�u�\�����[�V�����v���b�g�t�H�[����x64�ŁI(freeglut��dll��lib��x64�p�Ƀr���h��������)
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

//xml����̑Ή��_������
vector<Point3d> loadresult;


//Depth�̈ړ����ϖ@�ɂ�鐸�x����
const int nFrame = 100;      // �ړ����ς��s���t���[���� 30�Ȃ�30FPS�Ȃ̂łP�b��  100�t���[���ʂ��ǍD
const double Kd = 1 / (double)nFrame; // �ړ����ς����Z���g�킸��Z�ŋ��߂邽�߂̌W��
const int Dx = 512;          // ��ʃC���[�W�̐���������f��
const int Dy = 424;          // ��ʃC���[�W�̐���������f��
const int dByte = 4;         // XRGB�`���Ȃ̂łS�o�C�g
const int dPixels = Dx * Dy; // 1�t���[�����̉�f��
int ptr = 0;                 // �ړ����ς��s���ׂ̃f�[�^�i�[�|�C���^


UINT16 *nDepthBuffer = new UINT16[dPixels * nFrame]; // nFrame���̃f�v�X�o�b�t�@ [dPixels * nFrame]
long *Sum = new long[dPixels]; // nFrame���̈ړ����Z�l���i�[����ׂ̃o�b�t�@[dPixels]

//���ό��ʂ�Depth
unsigned short* aveDepthData = new unsigned short[dPixels];

//���ό��ʂ�p����CameraSpacePoint
CameraSpacePoint *cameraSpacePoints_ave = NULL;


void FIFOFilter()
{
    int j = dPixels * ptr;
    for (int i = 0; i < dPixels; i++)
    {
		Sum[i] += (long)kinect::depthBuffer[i] - (long)nDepthBuffer[j + i]; // �ړ����Z�lSum[i]�̕ω������̂ݏC��
		nDepthBuffer[j + i] = kinect::depthBuffer[i]; // �V�K�f�[�^DataIn[i]���o�b�t�@�Ɋi�[
		//cout << "Sum[" << i << "]: " << Sum[i] << endl;

    }
    ptr++;
    if (ptr == nFrame) { ptr = 0; } //�y�o�b�t�@�|�C���^�X�V�z
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
	//�Ή��_�Q�̕\��
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
	//��ʂ̃L���v�`��
	capture();

	kinect::draw();

	glEnable(GL_DEPTH_TEST);

	//��ʂ��ׂ����̂ŁA�K���ȃs�b�`�ɑe������
	int pitch = 1;
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glLoadIdentity(); //�ϊ��s��̏�����
	float cx = 0.0f;
	float cy = 0.0f;
	float cz = 1.0f;
	float x = r * cos(fai) * sin(theta) + cx;
	float y = r * sin(fai) + cy;
	float z = r * cos(fai) * cos(theta) + cz;
	gluLookAt(x, y, z, cx, cy, cz, 0.0f, 1.0f, 0.0f);

	uchar *color = kinect::coordinatedImage.data;
	int channels = kinect::coordinatedImage.channels();

	//height - 1, width - 1�܂�
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
	//��ʂ̃L���v�`��
	capture();

	kinect::draw();

	//�ړ�����
	FIFOFilter();
	//���Z->�f�[�^�i�[
	for(int i = 0; i < dPixels; i++)
	{
		aveDepthData[i] = (unsigned short)(Sum[i]*Kd); //�ړ����Z�lSum[i]����ړ����ϒlk�����߂�
	}

	//CameraSpacePoint�֕ϊ�
	ERROR_CHECK(kinect::coordinateMapper->MapDepthFrameToCameraSpace(
		width * height, (UINT16*)aveDepthData, 
	      width * height, cameraSpacePoints_ave),"MapDepthFrameToCameraSpace_ave");



	//**�`�揈��**//
	glEnable(GL_DEPTH_TEST);

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glLoadIdentity(); //�ϊ��s��̏�����
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

//idle��ԂɂȂ�����A�ĕ`�����������
//����ɂ��A�������[�v�̕\�����\�ɂȂ�
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
	//Kinect�̏�����
	kinect::init();

	//�ړ����ϗp�z��̏�����
	 for (int i = 0; i < dPixels; i++) { Sum[i] = 0; } //�ړ����Z�o�b�t�@��0�N���A
	 for (int i = 0; i < dPixels * nFrame; i++) { nDepthBuffer[i] = 0; } //0�N���A
	 cameraSpacePoints_ave = new CameraSpacePoint[width * height]; //CameraSpacePoint�z�񏉊���


	//xml�t�@�C���̃��[�h
	loadFile("../g_worldPointInlierSet.xml");

	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_DEPTH); //�_�u���o�b�t�@�A�f�u�X�o�b�t�@�p��
	glEnable(GL_DEPTH_TEST);

	glutCreateWindow("Kinect");

	glMatrixMode(GL_PROJECTION); //�������e�ϊ�
	glLoadIdentity(); //�ϊ��s�񏉊���
	gluPerspective(45.0f, (float)width/height, 0.5f, 4.0f); //�N���b�s���O�̈�w��

	glMatrixMode(GL_MODELVIEW);
	//�n���h���֐��̐ݒ�
	//glutDisplayFunc(display);
	glutDisplayFunc(display_points);
	glutKeyboardFunc(keyboard);
	glutIdleFunc(idle);
	glutMainLoop();
}

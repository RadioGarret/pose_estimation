#ifndef OPENPOSE_MYHEADERS_EKF_H
#define OPENPOSE_MYHEADERS_EKF_H
#define _CRT_SECURE_NO_WARNINGS


#include <math.h>
//#include <openpose/myheader/JUndistort.h>
#include "JUndistort.h"
//#include <opencv2/opencv.hpp>

//#include <Eigen/Dense>
#define PI 3.14159265


// ���� ����� ���� ���

#include <fstream>
#include <string>
#include <sstream>

//using namespace cv;
using namespace std;
//using namespace Eigen;



class EKF
{
public:
	
	// ī�޶� ��ġ�� ���ֱ�!!
	// camera fixed constants
	double CAM_TX = -0.0;     // 
	double CAM_TY = -0.59;   // camera heigt from bottm
	double CAM_TZ = -2.01;  // distance from camera to person
	double FX = 1436.35689;
	double FY = 1435.47675;
	double SKEW = 0.0;
	double CX = 951.08945;
	double CY = 549.32842;

	//// set ekf matrix gain
	//int R_sigma = 3; // 3 sigma
	//double R_const[14] = { 1.915, 1.968, 1.85, 0.115, 0.044, 1.918, 1.927, 1.959, 0.045, 1.911, 0.015, 0.061, 0.015, 0.061 };      // R matrix's gain
	//double Q_const[15] = { 0.03, 0.03, 0.03, 0.08, 0.08, 0.08, 0.08, 0.08, 0.08, 0.08, 0.08, 0.08, 0.03, 0.03, 0.03 };  // Q matrix's gain
	////double Q_const[15] = { 0.03, 0.03, 0.03, 0.001, 0.001,0.001,0.001,0.001,0.001,0.001,0.001,0.001, 0.03, 0.03, 0.03 };
	////double P_const[15] = { 1.0, 1.0, 1.0, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 1.0, 1.0, 1.0 };								 // P matrix's gain
	//double P_const[15] = { 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0 };

public:
	EKF();
	~EKF();
	EKF(double cameraParams[]);
	void Create(double cameraParams[]);

	std::string Execute(std::vector<coordinate>& vJoints);

	

public:
	JUnDistort undist;

	///*------------------------------------------------------*/


	// 0609 ����ȣ �߰�
	int GetKineticEnergy(std::vector<coordinate> vJoints);

private:
	std::vector<coordinate> pre_vJoints;
	bool isFirst_kineticEnergy = true;


protected:

};
//--------------------------------------------------------------------------
// ���� �����, �Ľ̰��� Ŭ����
class FileIO
{
	// #include iostream, fstream, string, sstream, ����� �߰��Ǿ�� ����� �� �ֽ��ϴ�.
public:
	FileIO();
	~FileIO();
	std::string Double2String(double dData);
	std::string StrConcate(std::string str1, std::string str2, std::string separator);
	std::string StrConcate(double d1, double d2, std::string separator);
	bool FileWriter(std::string filePath, std::string writeContent);

	double* Strline2doubleArr(std::string strLine);
private:

protected:

};


#endif
#ifndef OPENPOSE_MYHEADERS_EKF_H
#define OPENPOSE_MYHEADERS_EKF_H

//#define JDEBUG

#include <math.h>
#include "JUndistort.h"
#include <Eigen/Dense>
#define PI 3.14159265


// ���� ����� ���� ���
#define _CRT_SECURE_NO_WARNINGS
#include <fstream>
#include <string>
#include <sstream>

//using namespace cv;
using namespace std;
using namespace Eigen;

class EKF
{
public:
	// index to understand by LETTER
	enum Joint {
		MID_HIP,// assigned 0
		R_HIP,		// assigned 1
		R_KNEE,		// assigned 2
		R_ANKLE,	// assigned 3
		L_HIP,		// assigned 4
		L_KNEE,		// assigned 5
		L_ANKLE	// assigned 6
					
	};

	enum Coordinate3d {
		X,			// assigned 0
		Y,			// assigned 1
		Z			// assigned 2
	};

public:
	EKF();
	~EKF();
	EKF(double cameraParams[]);
	void Create(double cameraParams[]);

	std::string Execute(std::vector<coordinate>& vJoints);



public:
	JUnDistort undist;
	

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
	std::string Int2String(int nData);
	std::string Double2String(double dData);
	std::string StrConcate(std::string str1, std::string str2, std::string separator);
	std::string StrConcate(double d1, double d2, std::string separator);
	bool FileWriter(std::string filePath, std::string writeContent);

	double* Strline2doubleArr(std::string strLine);
private:

protected:

};

// ī�޶� ��ġ�� �޶����� _dThreshold ���� �޶��� ���̴�.
class Joker
{
#define  MAX_FRAME		20

public:
	Joker();
	Joker(double cameraParams[]);
	~Joker();
	void Create(double cameraParams[]);
	void SetThreshold(double dThres);
	int GetPlayerDirection(std::vector<coordinate> vShoulderJoints);
	double GetshoulderLength(std::vector<coordinate> vShoulderJoints);
	void SetRoi();
	void NormalizeShoulderLength(std::vector<coordinate> vShoulderJoints);
	int Execute(std::vector<coordinate> vShoulderJoints);
private:
	double _dThetaFiltered = 0; // GetPlayerDirection ���� ����ڰ� �ٶ󺸴� ������ ���������ͷ� ó���Ѱ�
	
	double GetUserBodyAngle(double dThetaFiltered);

	JUnDistort undist;
	enum PLAYER_DIRECTION {
		RIGHT = 1,
		LEFT = 2,
		FORWARD = 3,
		BACKWARD = 4
	};
	double save_dShoulderLength[MAX_FRAME];		//	�ʱ⿡ MAX_FRAME ��ŭ ������̸� ���� ��հ��� �������� �����ϴ� ��
	double dShoulderLength_normalized;	// �ʱ� ������� ��� ���̸� MAX_FRAME ��ŭ ��� ����� �� ��
	double _dThreshold;		// ������� thres, ����ڰ� �ٶ󺸴� ���� ���� ����.
	double _dShoulderLength;
	int nFrameCnt = 0;
protected:

};

#endif
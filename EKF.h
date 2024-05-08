#ifndef OPENPOSE_MYHEADERS_EKF_H
#define OPENPOSE_MYHEADERS_EKF_H

//#define JDEBUG

#include <math.h>
#include "JUndistort.h"
#include <Eigen/Dense>
#define PI 3.14159265


// 파일 입출력 관련 헤더
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
	

	// 0609 전성호 추가
	int GetKineticEnergy(std::vector<coordinate> vJoints);

private:
	std::vector<coordinate> pre_vJoints;
	bool isFirst_kineticEnergy = true;


protected:

};
//--------------------------------------------------------------------------
// 파일 입출력, 파싱관련 클래스
class FileIO
{
	// #include iostream, fstream, string, sstream, 헤더가 추가되어야 사용할 수 있습니다.
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

// 카메라 위치가 달라지면 _dThreshold 값이 달라질 것이다.
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
	double _dThetaFiltered = 0; // GetPlayerDirection 에서 사용자가 바라보는 방향을 저주파필터로 처리한것
	
	double GetUserBodyAngle(double dThetaFiltered);

	JUnDistort undist;
	enum PLAYER_DIRECTION {
		RIGHT = 1,
		LEFT = 2,
		FORWARD = 3,
		BACKWARD = 4
	};
	double save_dShoulderLength[MAX_FRAME];		//	초기에 MAX_FRAME 만큼 어깨길이를 보고 평균값을 내기위해 저장하는 곳
	double dShoulderLength_normalized;	// 초기 사용자의 어깨 길이를 MAX_FRAME 만큼 재고서 평균을 낸 값
	double _dThreshold;		// 어깨길이 thres, 사용자가 바라보는 방향 관련 변수.
	double _dShoulderLength;
	int nFrameCnt = 0;
protected:

};

#endif
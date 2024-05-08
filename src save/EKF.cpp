//#include <openpose/myheader/EKF.h>
#include "EKF.h"


///*-----------important notice-------------*/
//   //_X_(0, 0) = a;   _X_(1, 0) = b;   _X_(2, 0) = c;
//   //_X_(3, 0) = w_Lx;   _X_(4, 0) = w_Ly;    _X_(5, 0) = w_Lz;   _X_(6, 0) = w_L;
//   //_X_(7, 0) = w_Rx;   _X_(8, 0) = w_Ry;   _X_(9, 0) = w_Rz;   _X_(10, 0) = w_R; _X_(11, 0) = w_psi;
//   //_X_(12, 0) = tx;   _X_(13, 0) = ty;   _X_(14, 0) = tz;
///*------------------------------------------------*/
//
EKF::EKF()
{
	//time = 0;
}
EKF::~EKF()
{

}

EKF::EKF(double cameraParams[])
{
	undist = JUnDistort(cameraParams[0], cameraParams[1], cameraParams[2], cameraParams[3], cameraParams[4],
		cameraParams[5], cameraParams[6], cameraParams[7], cameraParams[8], cameraParams[9]);
}

void EKF::Create(double cameraParams[])
{
	undist = JUnDistort(cameraParams[0], cameraParams[1], cameraParams[2], cameraParams[3], cameraParams[4],
		cameraParams[5], cameraParams[6], cameraParams[7], cameraParams[8], cameraParams[9]);

}

// 칼만필터 실행부분
// 카메라 왜곡보정도 들어가야 한다.
std::string EKF::Execute(std::vector<coordinate>& vJoints)
{
	////// 이 곳에 왜곡보정 관련 식을 써주세요 //////
	undist.UndistortJoints(vJoints); // 이 소스만으로 vJoints 에 보정된 픽셀들이 들어간다.
	///////////////////////////////////////////////왜곡보정 관련 END


	////// 이 곳에 칼만필터 관련 식을 써주세요 //////
	//limitSqrtParameter();
	//for (int i = 0; i < 1; i++) // 전체 loop 1번 돌 때 칼만 5번 돌리기
	//{
	//	loopProcedure(vJoints);
	//	// P = APA + Q
	//	//predictEKF();

	//	//// h(Xk) 구하기
	//	//updateQuarternionMatrix();
	//	//updateTransformationMatrix();           // R_new = R_delta * R_old
	//	//findModelCoordinatefromWorld();     // Model 좌표계 ==> world좌표계
	//	//cameraProjection();                             // world 좌표계 ==> 픽셀좌표계

	//	//updateTransformationMatrix();
	//	//findModelCoordinatefromWorld();
	//	//cameraProjection();
	//	//// H 매트릭스(자코비안) 구하기
	//	//H_cal(a, b, c, w_Lx, w_Ly, w_Lz, w_L, w_Rx, w_Ry, w_Rz, w_R, w_psi, tx, ty, tz);

	//	//// 
	//	//updateEKF(vJoints);
	//	//saveData();
	//	//calculateAngle();

	//}

	//if (time % 1 == 0)
	//{
	//	ekf_cnt++;
	//	time = 0;
	//	for (int i = 0; i < 7; i++)
	//	{
	//		printf("[%d]번째 관절좌표(x,y) : %lf %lf\n", i, vJoints[i].j, vJoints[i].i);
	//	}
	//	//check
	//	std::cout << "----------------------------" << std::endl;
	//	//std::cout << "WPSI : " << W_PSI * 180 / PI << std::endl;
	//	std::cout << "m_img : " << m_img << std::endl;
	//	std::cout << "X " << std::endl;
	//	std::cout << _X_ << std::endl;
	//	std::cout << "----------------------------" << std::endl << std::endl;
	//	printf("[%d] 번째 출력\n\n", ekf_cnt);



	//	std::cout << "----------------------------" << std::endl;
	//}
	//time++;

	//int nDirection;
	///////////////////////////////////////////////칼만필터 관련 END



	////// 이 곳에 운동에너지 관련 식을 써주세요 //////
	//int nKineticEnergy = GetKineticEnergy(vJoints);
	//printf("kinetic energy : %d\n", nKineticEnergy);
	//int nStride;
	///////////////////////////////////////////////운동에너지 관련 END


	////// 이 곳에 string data 저장 관련 식을 써주세요 //////
	// nDirection, nStride 합쳐서 data 에 넣기
	std::string data = "43";
	///////////////////////////////////////////////string data 저장 관련 END    
	return data;
}


////////////////////////////////////////////////
// nStride가 아니라 운동에너지의 총합이 return된다.
////////////////////////////////////////////////

int EKF::GetKineticEnergy(std::vector<coordinate> vJoints)
{
	if (isFirst_kineticEnergy)
	{
		// pre_vJoints 가 저장된게 없으니 저장만 하는 과정을 거친다.
		isFirst_kineticEnergy = false;
		pre_vJoints = vJoints; // vJoints 저장
		return 0; // 당연히 운동에너지는 0
	}
	else
	{
		int nSum = 0;
		for (int i = 0; i < 6; i++)
		{
			int tmp_x = (int)(_SQR(vJoints[i].j - pre_vJoints[i].j)); // joints 의 x좌표 움직임의 제곱 계산
			int tmp_y = (int)(_SQR(vJoints[i].i - pre_vJoints[i].i)); // joints의 y 좌표 움직임의 제곱 계산
			nSum += (tmp_x + tmp_y);
		} // for문 거치고 나오면 joints들의 모든 움직임의 제곱이 계산 되어 nSum에 저장된다.

		pre_vJoints = vJoints; // vJoints 필요한곳에 다 사용했으니 저장
		return nSum;
	}
}

//----------------------------------------------------------------------------------------------------------
// 파일 입출력 관련

FileIO::FileIO()
{

}
FileIO::~FileIO()
{

}
std::string FileIO::Double2String(double dData)
{
	std::stringstream ssDouble;
	ssDouble << dData;
	return ssDouble.str();
}
std::string FileIO::StrConcate(std::string str1, std::string str2, std::string separator)
{
	return str1 + separator + str2;
}
std::string FileIO::StrConcate(double d1, double d2, std::string separator)
{
	return Double2String(d1) + separator + Double2String(d2);
}
bool FileIO::FileWriter(std::string filePath, std::string writeContent)
{
	std::ofstream writer;
	writer.open(filePath, std::ios_base::out | std::ios_base::app);
	writeContent = writeContent + "\n";
	int n = writeContent.length();
	char buff[256] = { 0 };
	for (int i = 0; i < n; i++)
	{
		buff[i] = writeContent[i];
	}
	buff[n] = '\0';
	writer.write(buff, strlen(buff));
	writer.close();
}
double* FileIO::Strline2doubleArr(std::string strLine)
{
	//strtok을 사용하기 위해 char형 배열에 복사.
	int n = strLine.length();
	char buff[256] = { 0 };
	for (int i = 0; i < n; i++)
	{
		buff[i] = strLine[i];
	}
	buff[n] = '\0';

	// parsing
	char* tmp = strtok(buff, ":");
	char* str[2];
	str[0] = tmp;
	tmp = strtok(NULL, "");
	str[1] = tmp;

	// str --> double
	double dData[2];
	dData[0] = std::stod(str[0]);
	dData[1] = std::stod(str[1]);

	return dData;
}





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

// Į������ ����κ�
// ī�޶� �ְ���� ���� �Ѵ�.
std::string EKF::Execute(std::vector<coordinate>& vJoints)
{
	////// �� ���� �ְ�� ���� ���� ���ּ��� //////
	undist.UndistortJoints(vJoints); // �� �ҽ������� vJoints �� ������ �ȼ����� ����.
	///////////////////////////////////////////////�ְ�� ���� END


	////// �� ���� Į������ ���� ���� ���ּ��� //////
	//limitSqrtParameter();
	//for (int i = 0; i < 1; i++) // ��ü loop 1�� �� �� Į�� 5�� ������
	//{
	//	loopProcedure(vJoints);
	//	// P = APA + Q
	//	//predictEKF();

	//	//// h(Xk) ���ϱ�
	//	//updateQuarternionMatrix();
	//	//updateTransformationMatrix();           // R_new = R_delta * R_old
	//	//findModelCoordinatefromWorld();     // Model ��ǥ�� ==> world��ǥ��
	//	//cameraProjection();                             // world ��ǥ�� ==> �ȼ���ǥ��

	//	//updateTransformationMatrix();
	//	//findModelCoordinatefromWorld();
	//	//cameraProjection();
	//	//// H ��Ʈ����(���ں��) ���ϱ�
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
	//		printf("[%d]��° ������ǥ(x,y) : %lf %lf\n", i, vJoints[i].j, vJoints[i].i);
	//	}
	//	//check
	//	std::cout << "----------------------------" << std::endl;
	//	//std::cout << "WPSI : " << W_PSI * 180 / PI << std::endl;
	//	std::cout << "m_img : " << m_img << std::endl;
	//	std::cout << "X " << std::endl;
	//	std::cout << _X_ << std::endl;
	//	std::cout << "----------------------------" << std::endl << std::endl;
	//	printf("[%d] ��° ���\n\n", ekf_cnt);



	//	std::cout << "----------------------------" << std::endl;
	//}
	//time++;

	//int nDirection;
	///////////////////////////////////////////////Į������ ���� END



	////// �� ���� ������� ���� ���� ���ּ��� //////
	//int nKineticEnergy = GetKineticEnergy(vJoints);
	//printf("kinetic energy : %d\n", nKineticEnergy);
	//int nStride;
	///////////////////////////////////////////////������� ���� END


	////// �� ���� string data ���� ���� ���� ���ּ��� //////
	// nDirection, nStride ���ļ� data �� �ֱ�
	std::string data = "43";
	///////////////////////////////////////////////string data ���� ���� END    
	return data;
}


////////////////////////////////////////////////
// nStride�� �ƴ϶� ��������� ������ return�ȴ�.
////////////////////////////////////////////////

int EKF::GetKineticEnergy(std::vector<coordinate> vJoints)
{
	if (isFirst_kineticEnergy)
	{
		// pre_vJoints �� ����Ȱ� ������ ���常 �ϴ� ������ ��ģ��.
		isFirst_kineticEnergy = false;
		pre_vJoints = vJoints; // vJoints ����
		return 0; // �翬�� ��������� 0
	}
	else
	{
		int nSum = 0;
		for (int i = 0; i < 6; i++)
		{
			int tmp_x = (int)(_SQR(vJoints[i].j - pre_vJoints[i].j)); // joints �� x��ǥ �������� ���� ���
			int tmp_y = (int)(_SQR(vJoints[i].i - pre_vJoints[i].i)); // joints�� y ��ǥ �������� ���� ���
			nSum += (tmp_x + tmp_y);
		} // for�� ��ġ�� ������ joints���� ��� �������� ������ ��� �Ǿ� nSum�� ����ȴ�.

		pre_vJoints = vJoints; // vJoints �ʿ��Ѱ��� �� ��������� ����
		return nSum;
	}
}

//----------------------------------------------------------------------------------------------------------
// ���� ����� ����

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
	//strtok�� ����ϱ� ���� char�� �迭�� ����.
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





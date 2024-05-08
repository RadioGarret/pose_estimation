#include "EKF.h"
//
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
        for (int i = 0; i < vJoints.size(); i++)
        {
            int tmp_x = (int)(_SQR(vJoints[i].j - pre_vJoints[i].j)); // joints �� x��ǥ �������� ���� ���
            int tmp_y = (int)(_SQR(vJoints[i].i - pre_vJoints[i].i)); // joints�� y ��ǥ �������� ���� ���
            nSum += (tmp_x + tmp_y);
        } // for�� ��ġ�� ������ joints���� ��� �������� ������ ��� �Ǿ� nSum�� ����ȴ�.

#ifdef JDEBUG
		if (nSum > 10000) // ���� 10000�������� �־ ������
		{
			cout << "��������� 1���� �Ѿ ���� ��ǥ�� ���� ��ǥ ���" << endl;
			for (int i = 0; i < vJoints.size(); i++)
			{
				cout << i << "��° (x,y)��ǥ ���(pre_vJoints) :" << pre_vJoints[i].j << pre_vJoints[i].i << endl;
				cout << i << "��° (x,y)��ǥ ���(vJoints) :" << vJoints[i].j << vJoints[i].i << endl;
			}
		}

#endif
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
std::string FileIO::Int2String(int nData)
{
    std::stringstream ssint;
    ssint << nData;
    return ssint.str();
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

//----------------------------------------------------------------------------------------------------
// ����� ��Ŀ

Joker::Joker()
{
    undist = JUnDistort(1436.35689, 1435.47675, 951.08945, 549.32842, 0, 0.05382, -0.16741, -0.00558, -0.00092, 0);
}
// set camera parameter
Joker::Joker(double cameraParams[])
{
    // (1436.35689, 1435.47675, 951.08945, 549.32842, 0, 0.05382, -0.16741, -0.00558, -0.00092, 0);
    undist = JUnDistort(cameraParams[0], cameraParams[1], cameraParams[2], cameraParams[3], cameraParams[4],
        cameraParams[5], cameraParams[6], cameraParams[7], cameraParams[8], cameraParams[9]);
}
void Joker::Create(double cameraParams[])
{
    undist = JUnDistort(cameraParams[0], cameraParams[1], cameraParams[2], cameraParams[3], cameraParams[4],
        cameraParams[5], cameraParams[6], cameraParams[7], cameraParams[8], cameraParams[9]);
}
Joker::~Joker()
{

}
void Joker::SetThreshold(double dThres)
{
    _dThreshold = dThres;
}
#define JDEBUG
// vShoulderJoints[0] :  right
// vShoulderJoints[1] : left
int Joker::GetPlayerDirection(std::vector<coordinate> vShoulderJoints)
{
	const double dShoulderLength = GetshoulderLength(vShoulderJoints);
	const double dVerticalDiff = vShoulderJoints[0].i - vShoulderJoints[1].i; // right.y - left.y
	const double dHorizontalDiff = vShoulderJoints[0].j - vShoulderJoints[1].j; // right.x - left.x

	const double RAD2DEG = 57.2957795131;
	const double dTheta = atan2(dVerticalDiff, dHorizontalDiff) * RAD2DEG;	// ���� tick ���� ������ ��� ����
#ifdef JDEBUG
    //printf("dShoulderLength : %lf\n", dShoulderLength);
	const double alpha = 0.9;	// ������� ������ ���� ����ġ�� ������ ���� �ξ� ���� �� �ȹٲ�� �Ѵ�.
	_dThetaFiltered = alpha * _dThetaFiltered + (1 - alpha) * dTheta;
	cout << "(in GetPlayerDirection) dThetaFiltered :" << _dThetaFiltered << endl;
#else
	const double alpha = 0.55;
	_dThetaFiltered = alpha * _dThetaFiltered + (1 - alpha) * dTheta;
	double dUserBodyAngle = GetUserBodyAngle(_dThetaFiltered);
	cout << "dUserBodyAngle : " << dUserBodyAngle << endl;
#endif

    // left, right
    if (dShoulderLength < _dThreshold)
    {
        if (dVerticalDiff < 0)
        {
            printf("����\n");
            return  LEFT; // 2
        }
            
        else
        {
            printf("������\n");
            return RIGHT; // 1
        }
            
    }
    // forward, backward
    else
    {
        if (dHorizontalDiff < 0)
        {
            printf("����\n");
            return BACKWARD; // 4
        }
        else
        {
            printf("����\n");
            return FORWARD; // 3
        }
            
    }
}

double Joker::GetUserBodyAngle(double dThetaFiltered)
{
	double dUserBodyAngle = 0.0;
	double dRandom = rand();
	// 0~1 ������ ������ �Ǽ� ����
	while (dRandom > 1)
		dRandom /= 10;

	/*if (a < dThetaFiltered && dThetaFiltered < b)
	{
		dUserBodyAngle = ��ħ��;
		return dUserBodyAngle + dRandom;
	}
	else if (a < dThetaFiltered && dThetaFiltered < b)
	{
		dUserBodyAngle = ��ħ��;
		return dUserBodyAngle + dRandom;
	}*/

}
double Joker::GetshoulderLength(std::vector<coordinate> vShoulderJoints)
{
    return sqrt(_SQR(vShoulderJoints[0].j - vShoulderJoints[1].j) + _SQR(vShoulderJoints[0].i - vShoulderJoints[1].i)); // root( (x1-x2)^2 + (y1-y2)^2 )
}
void Joker::SetRoi()
{
    // not yet
}
void Joker::NormalizeShoulderLength(std::vector<coordinate> vShoulderJoints)
{
    save_dShoulderLength[nFrameCnt] = GetshoulderLength(vShoulderJoints);
#ifdef JDEBUG
    printf("����ȭ�۾���...\n");
#endif
    
    if (nFrameCnt == MAX_FRAME - 1)
    {
        double dSum = 0;
        for (int i = 0; i < MAX_FRAME; i++)
        {
            dSum += save_dShoulderLength[i];
        }
        dShoulderLength_normalized = dSum / MAX_FRAME;

        //threshold�� dShoulderLength_normalized�� �ݺ��� ũ�� �����ؾ� �Ѵ�.
        _dThreshold = dShoulderLength_normalized * 0.65;

#ifdef JDEBUG
        printf("%lf\n", dShoulderLength_normalized);
#endif
    }
}

int Joker::Execute(std::vector<coordinate> vShoulderJoints)
{
    undist.UndistortJoints(vShoulderJoints);

    if (nFrameCnt < MAX_FRAME)
    {
        NormalizeShoulderLength(vShoulderJoints);
        nFrameCnt++;
        return 0; // do not send direction to client
    }
    else
    {
        int direction = GetPlayerDirection(vShoulderJoints);
#ifdef JDEBUG
        printf("player direction is : %d\n", direction);
#endif
        return direction; // send direction to client
    }
}
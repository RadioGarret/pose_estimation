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

// 칼만필터 실행부분
// 카메라 왜곡보정도 들어가야 한다.
std::string EKF::Execute(std::vector<coordinate>& vJoints)
{
    ////// 이 곳에 왜곡보정 관련 식을 써주세요 //////
    undist.UndistortJoints(vJoints); // 이 소스만으로 vJoints 에 보정된 픽셀들이 들어간다.
    ///////////////////////////////////////////////왜곡보정 관련 END
    



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
        for (int i = 0; i < vJoints.size(); i++)
        {
            int tmp_x = (int)(_SQR(vJoints[i].j - pre_vJoints[i].j)); // joints 의 x좌표 움직임의 제곱 계산
            int tmp_y = (int)(_SQR(vJoints[i].i - pre_vJoints[i].i)); // joints의 y 좌표 움직임의 제곱 계산
            nSum += (tmp_x + tmp_y);
        } // for문 거치고 나오면 joints들의 모든 움직임의 제곱이 계산 되어 nSum에 저장된다.

#ifdef JDEBUG
		if (nSum > 10000) // 가끔 10000넘을때가 있어서 디버깅용
		{
			cout << "운동에너지가 1만이 넘어서 이전 좌표와 현재 좌표 출력" << endl;
			for (int i = 0; i < vJoints.size(); i++)
			{
				cout << i << "번째 (x,y)좌표 출력(pre_vJoints) :" << pre_vJoints[i].j << pre_vJoints[i].i << endl;
				cout << i << "번째 (x,y)좌표 출력(vJoints) :" << vJoints[i].j << vJoints[i].i << endl;
			}
		}

#endif
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

//----------------------------------------------------------------------------------------------------
// 대망의 조커

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
	const double dTheta = atan2(dVerticalDiff, dHorizontalDiff) * RAD2DEG;	// 현재 tick 에서 측정한 어깨 각도
#ifdef JDEBUG
    //printf("dShoulderLength : %lf\n", dShoulderLength);
	const double alpha = 0.9;	// 현재까지 측정한 값에 가중치를 굉장히 많이 두어 값이 잘 안바뀌도록 한다.
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
            printf("왼쪽\n");
            return  LEFT; // 2
        }
            
        else
        {
            printf("오른쪽\n");
            return RIGHT; // 1
        }
            
    }
    // forward, backward
    else
    {
        if (dHorizontalDiff < 0)
        {
            printf("뒤쪽\n");
            return BACKWARD; // 4
        }
        else
        {
            printf("앞쪽\n");
            return FORWARD; // 3
        }
            
    }
}

double Joker::GetUserBodyAngle(double dThetaFiltered)
{
	double dUserBodyAngle = 0.0;
	double dRandom = rand();
	// 0~1 사이의 임의의 실수 생성
	while (dRandom > 1)
		dRandom /= 10;

	/*if (a < dThetaFiltered && dThetaFiltered < b)
	{
		dUserBodyAngle = 나침반;
		return dUserBodyAngle + dRandom;
	}
	else if (a < dThetaFiltered && dThetaFiltered < b)
	{
		dUserBodyAngle = 나침반;
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
    printf("정규화작업중...\n");
#endif
    
    if (nFrameCnt == MAX_FRAME - 1)
    {
        double dSum = 0;
        for (int i = 0; i < MAX_FRAME; i++)
        {
            dSum += save_dShoulderLength[i];
        }
        dShoulderLength_normalized = dSum / MAX_FRAME;

        //threshold를 dShoulderLength_normalized의 반보다 크게 설정해야 한다.
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
#include "EKF.h"
#include <iostream>
#include <math.h>
#include <Eigen/Dense>

using namespace Eigen;

//#include "EKF.h"



//MatrixXd X = MatrixXd(2, 1);

//MatrixXd T = MatrixXd(4, 4);

enum stateNum {
	XK_A,         // assigned 0
	XK_B,         // assigned 1
	XK_C,      // assigned 2
	XK_RX,   // assigned 3
	XK_RY,   // assigned 4
	XK_RZ,   // assigned 5
	XK_R,      // assigned 6
	XK_LX,   // assigned 7
	XK_LY,   // assigned 8
	XK_LZ,   // assigned 9
	XK_L,      // assigned 10
	XK_WPSI,      // assigned 11
	XK_TX,      // assigned 12
	XK_TY,      // assigned 13
	XK_TZ      // assigned 14
};

enum Joint {
	MID_HIP,      // assigned 0
	R_HIP,      // assigned 1
	R_KNEE,      // assigned 2
	R_ANKLE,   // assigned 3
	L_HIP,      // assigned 4
	L_KNEE,      // assigned 5
	L_ANKLE      // assigned 6
};

enum Coordinate3d {
	X,         // assigned 0
	Y,         // assigned 1
	Z         // assigned 2
};

MatrixXd roundCoordroundXk_Cam = MatrixXd(21, 15);
MatrixXd rot_P_C = MatrixXd(3, 3);
MatrixXd rot_C_W = MatrixXd(3, 3);

MatrixXd rot_W_M_delta = MatrixXd(3, 3);
MatrixXd rot_W_M_old = MatrixXd(3, 3);
 MatrixXd rot_W_M_new = MatrixXd(3, 3);

MatrixXd rot_M_R0_delta = MatrixXd(3, 3);
MatrixXd rot_M_R0_old = MatrixXd(3, 3);
 MatrixXd rot_M_R0_new = MatrixXd(3, 3);
MatrixXd rot_R0_R1_delta = MatrixXd(3, 3);
MatrixXd rot_R0_R1_old = MatrixXd(3, 3);
 MatrixXd rot_R0_R1_new = MatrixXd(3, 3);

MatrixXd rot_M_L0_delta = MatrixXd(3, 3);
MatrixXd rot_M_L0_old = MatrixXd(3, 3);
 MatrixXd rot_M_L0_new = MatrixXd(3, 3);
MatrixXd rot_L0_L1_delta = MatrixXd(3, 3);
MatrixXd rot_L0_L1_old = MatrixXd(3, 3);
MatrixXd rot_L0_L1_new = MatrixXd(3, 3);

// Translation matrix
MatrixXd trans_C_W = MatrixXd(3, 1);
MatrixXd trans_W_M = MatrixXd(3, 1);

MatrixXd trans_M_R0 = MatrixXd(3, 1);
MatrixXd trans_R0_R1 = MatrixXd(3, 1);
MatrixXd trans_R1_R2 = MatrixXd(3, 1);

MatrixXd trans_M_L0 = MatrixXd(3, 1);
MatrixXd trans_L0_L1 = MatrixXd(3, 1);
MatrixXd trans_L1_L2 = MatrixXd(3, 1);



// round delta rotation matrix
MatrixXd rot_W_Mdelta_roundWpsi = MatrixXd(3, 3);
MatrixXd rot_M_R0delta_roundWRX = MatrixXd(3, 3);
MatrixXd rot_M_R0delta_roundWRY = MatrixXd(3, 3);
MatrixXd rot_M_R0delta_roundWRZ = MatrixXd(3, 3);
MatrixXd rot_R0_R1delta_roundWR = MatrixXd(3, 3);
MatrixXd rot_M_L0delta_roundWLX = MatrixXd(3, 3);
MatrixXd rot_M_L0delta_roundWLY = MatrixXd(3, 3);
MatrixXd rot_M_L0delta_roundWLZ = MatrixXd(3, 3);
MatrixXd rot_L0_L1delta_roundWL = MatrixXd(3, 3);

// round camera coodinate round Xk
// 라운드 
MatrixXd RoundCoordRoundXk_Cam;// = MatrixXd(21, 15);

// camera coordinate ( Xc, Yc, Zc )
MatrixXd Coord_CamXcYcZc;// = MatrixXd(3, 7);

// jacobian matrix
//MatrixXd Hmatrix = MatrixXd(14, 15);
//MatrixXd hx = MatrixXd(14, 1);      // hx matrix
MatrixXd Xk;// = MatrixXd(15, 1);      // state variable
MatrixXd ekfP;// = MatrixXd::Identity(15, 15);
MatrixXd ekfQ;// = MatrixXd(15, 15);
MatrixXd ekfR;// = MatrixXd(14, 14);
MatrixXd ekfK;// = MatrixXd(15, 15);
MatrixXd ekfZ;// = MatrixXd(14, 1);

// 잠시 선언해둠!!!~~
const double FX = 948.63725;;////1436.35689;
const double FY = 946.75161; //1435.47675;////;
const double CX = 622.28201; //951.08945;////;
const double CY = 386.01434; //549.32842;// //;

//const double FX = 1436.35689;
//const double FY = 1435.47675;
//const double CX = 951.08945;
//const double CY = 549.32842;
//const double SKEW = 0;
//const double CAM_TX = 0.0;
//const double CAM_TY = 0.59;
//const double CAM_TZ = 2.01;

void GetRoundCameraCoordinate2roundXk();


// 완료됨.
// 델타 로테이션을 업데이트해주는 부분
// 매번 사용할 필요가 있음.
void updateRotation_delta(double w_Rx, double w_Ry, double w_Rz, double w_R, double w_Lx, double w_Ly, double w_Lz, double w_L, double w_psi)
{
	rot_W_M_delta << 
		1.0 - pow(w_psi, 2) / 2.0, -sqrt(1.0 - pow(w_psi, 2) / 4.0) * w_psi, 0.0,
		sqrt(1.0 - pow(w_psi, 2) / 4.0)* w_psi, 1.0 - pow(w_psi, 2) / 2.0, 0.0,
		0.0, 0.0, 1.0;

	rot_M_R0_delta(0, 0) = 1.0 - pow(w_Ry, 2) / 2.0 - pow(w_Rz, 2) / 2.0;
	rot_M_R0_delta(0, 1) = w_Rx * w_Ry / 2.0 - sqrt(1.0 - (pow(w_Rx, 2) + pow(w_Ry, 2) + pow(w_Rz, 2)) / 4.0) * w_Rz;
	rot_M_R0_delta(0, 2) = w_Rx * w_Rz / 2.0 + sqrt(1.0 - (pow(w_Rx, 2) + pow(w_Ry, 2) + pow(w_Rz, 2)) / 4.0) * w_Ry;
	rot_M_R0_delta(1, 0) = w_Rx * w_Ry / 2.0 + sqrt(1.0 - (pow(w_Rx, 2) + pow(w_Ry, 2) + pow(w_Rz, 2)) / 4.0) * w_Rz;
	rot_M_R0_delta(1, 1) = 1.0 - pow(w_Rx, 2) / 2.0 - pow(w_Rz, 2) / 2.0;
	rot_M_R0_delta(1, 2) = w_Ry * w_Rz / 2.0 - sqrt(1.0 - (pow(w_Rx, 2) + pow(w_Ry, 2) + pow(w_Rz, 2)) / 4.0) * w_Rx;
	rot_M_R0_delta(2, 0) = w_Rx * w_Rz / 2.0 - sqrt(1.0 - (pow(w_Rx, 2) + pow(w_Ry, 2) + pow(w_Rz, 2)) / 4.0) * w_Ry;
	rot_M_R0_delta(2, 1) = w_Ry * w_Rz / 2.0 + sqrt(1.0 - (pow(w_Rx, 2) + pow(w_Ry, 2) + pow(w_Rz, 2)) / 4.0) * w_Rx;
	rot_M_R0_delta(2, 2) = 1.0 - pow(w_Rx, 2) / 2.0 - pow(w_Ry, 2) / 2.0;

	rot_R0_R1_delta << 1.0 - pow(w_R, 2) / 2.0, -sqrt(1.0 - pow(w_R, 2) / 4.0) * w_R, 0.0,
		sqrt(1.0 - pow(w_R, 2) / 4.0)* w_R, 1.0 - pow(w_R, 2) / 2.0, 0.0,
		0.0, 0.0, 1.0;

	rot_M_L0_delta(0, 0) = 1.0 - pow(w_Ly, 2) / 2.0 - pow(w_Lz, 2) / 2.0;
	rot_M_L0_delta(0, 1) = w_Lx * w_Ly / 2.0 - sqrt(1.0 - (pow(w_Lx, 2) + pow(w_Ly, 2) + pow(w_Lz, 2)) / 4.0) * w_Lz;
	rot_M_L0_delta(0, 2) = w_Lx * w_Lz / 2.0 + sqrt(1.0 - (pow(w_Lx, 2) + pow(w_Ly, 2) + pow(w_Lz, 2)) / 4.0) * w_Ly;
	rot_M_L0_delta(1, 0) = w_Lx * w_Ly / 2.0 + sqrt(1.0 - (pow(w_Lx, 2) + pow(w_Ly, 2) + pow(w_Lz, 2)) / 4.0) * w_Lz;
	rot_M_L0_delta(1, 1) = 1.0 - pow(w_Lx, 2) / 2.0 - pow(w_Lz, 2) / 2.0;
	rot_M_L0_delta(1, 2) = w_Ly * w_Lz / 2.0 - sqrt(1.0 - (pow(w_Lx, 2) + pow(w_Ly, 2) + pow(w_Lz, 2)) / 4.0) * w_Lx;
	rot_M_L0_delta(2, 0) = w_Lx * w_Lz / 2.0 - sqrt(1.0 - (pow(w_Lx, 2) + pow(w_Ly, 2) + pow(w_Lz, 2)) / 4.0) * w_Ly;
	rot_M_L0_delta(2, 1) = w_Ly * w_Lz / 2.0 + sqrt(1.0 - (pow(w_Lx, 2) + pow(w_Ly, 2) + pow(w_Lz, 2)) / 4.0) * w_Lx;
	rot_M_L0_delta(2, 2) = 1.0 - pow(w_Lx, 2) / 2.0 - pow(w_Ly, 2) / 2.0;

	rot_L0_L1_delta << 1.0 - pow(w_L, 2) / 2.0, -sqrt(1.0 - pow(w_L, 2) / 4.0) * w_L, 0.0,
		sqrt(1.0 - pow(w_L, 2) / 4.0)* w_L, 1.0 - pow(w_L, 2) / 2.0, 0.0,
		0.0, 0.0, 1.0;
}

void InitQ()
{
	const int sizeQ = 15;
	const double Q_sigma[15] = { 0.05, 0.05, 0.05, 0.08, 0.08, 0.08, 0.08, 0.08, 0.08, 0.08, 0.08, 0.08, 0.1, 0.1, 0.1 };  // Q matrix's gain
	for (int i = 0; i < sizeQ; i++)
	{
		ekfQ(i, i) = pow(Q_sigma[i], 2);
	}
}
void InitR()
{
	const int sizeR = 14;
	// 원래 민재소스에서 제대로 맞게 바꿨다.
	const double R_sigma[14] = { 3.0, 3.0, 3.0, 3.0, 3.0, 3.0
	, 3.0 , 3.0 , 3.0 , 3.0 , 3.0 , 3.0 , 3.0 , 3.0 };      // R matrix's gain
	//const double R_sigma[14] = { 0.31916666, 0.328, 0.308333, 0.01916, 0.00733,
	//	0.319666, 0.32116, 0.3265, 0.0075, 0.3185, 0.0025, 0.010166, 0.0025, 0.01016 };      // 화질바꾸기 전
	for (int i = 0; i < sizeR; i++)
	{
		ekfR(i, i) = pow(R_sigma[i], 2);
	}
}
void InitXk()
{
	const int sizeX = 15;
	//const double X0[15] = { 0.10,0.4,0.41,0,0,0,0,0,0,0,0,0,0,0,0 };
	const double X0[15] = { 0.0994045,0.436625,0.390972,0,0,0,0,0,0,0,0,0,
		0,
		0,
		0};

	for (int i = 0; i < sizeX; i++)
	{
		Xk(i, 0) = X0[i];
	}
}
void InitPk()
{
	const int sizeP = 15;
	const double P_params[15] = { 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0 };
	for (int i = 0; i < sizeP; i++)
	{
		ekfP(i, i) = P_params[i];
	}
}
void initMatrixData()
{
	RoundCoordRoundXk_Cam = MatrixXd::Zero(21, 15);
	Coord_CamXcYcZc = MatrixXd::Zero(3, 7);
	Xk = MatrixXd::Zero(15, 1);
	ekfQ = MatrixXd::Zero(15, 15);
	ekfR = MatrixXd::Zero(14, 14);
	ekfP = MatrixXd::Zero(15, 15);

}

void InitRotation()
{
	rot_L0_L1_old = rot_R0_R1_old = rot_W_M_old = MatrixXd::Identity(3, 3);
	rot_M_L0_old << 
				0, 0, -1,
				0, -1, 0,
				-1, 0, 0;
	rot_M_R0_old << 
				0, 0, -1,
				0, -1, 0,
				-1, 0, 0;

	rot_P_C << 
				FX, 0, CX,
				0, FY, CY,
				0, 0, 1;


	rot_C_W << 0.999157, -0.040790, -0.004632,
		-0.022715, -0.455350, -0.890023,
		0.034195, 0.889378, -0.455892;

	/*rot_C_W <<
		0.993658, 0.100378, -0.0506822,
		-0.00583499, -0.404083, -0.914704,
		-0.112296, 0.909198, -0.400935;*/



	/*rot_C_W <<
		-0.00682663, -0.999977, -0.000147549,
		-0.0432067, 0.000147549, 0.999066,
		-0.999043, 0.00682663, -0.0432067;*/

}


void InitTranslation()
{
	trans_C_W << -0.0778537,
		0.362103100,
		2.315815174;

}


// 완성
void InitEKF()
{
	initMatrixData();
	InitQ();
	InitR();
	InitXk();
	InitPk();
	InitRotation();
	InitTranslation();
/*
	cout << "RoundCoordRoundXk_Cam \n" << RoundCoordRoundXk_Cam << endl;
	cout << "Coord_CamXcYcZc \n" << Coord_CamXcYcZc << endl;
	cout << "Xk \n" << Xk << endl;
	cout << "ekfQ \n" << ekfQ << endl;
	cout << "ekfR \n" << ekfR << endl;*/
}

// 완료되었나
MatrixXd GetJacobianMatrix()
{
	
	MatrixXd Hmatrix = MatrixXd(14, 15);
	//printf("7\n");
	// calculate round matrix
	GetRoundCameraCoordinate2roundXk();
	//printf("8\n");
	// Zc^2 으로 나누는 부분
	// every joint
	//for (int i = 0; i < Hmatrix.rows(); i = +2) {
	//	double tmpXc = Coord_CamXcYcZc(X, i / 2);
	//	double tmpYc = Coord_CamXcYcZc(Y, i / 2);
	//	double tmpZc = Coord_CamXcYcZc(Z, i / 2);

	//	//printf("10\n");
	//	// every state variable( Xk)
	//	for (int j = 0; j < Hmatrix.cols(); j++) {
	//		
	//		double tmpRoundXc = RoundCoordRoundXk_Cam(i / 2 + X, j);
	//		double tmpRoundYc = RoundCoordRoundXk_Cam(i / 2 + Y, j);
	//		double tmpRoundZc = RoundCoordRoundXk_Cam(i / 2 + Z, j);

	//		// round Xc / Zc round Xk
	//		Hmatrix(i, j) = FX * (tmpRoundXc * tmpZc - tmpXc * tmpRoundZc) / pow(tmpZc, 2);
	//		// round Yc / Zc round Xk
	//		Hmatrix(i + 1, j) = FY * (tmpRoundYc * tmpZc - tmpYc * tmpRoundZc) / pow(tmpZc, 2);
	//	}
	//}
	//printf("9\n");
	

	for (int i = 0; i < 7; i++) {
		for (int j = 0; j < Hmatrix.cols(); j++) {
			Hmatrix.block(2 * i, j, 2, 1) = rot_P_C.block(0, 0, 2, 2) * (RoundCoordRoundXk_Cam.block(3 * i, j, 2, 1) / Coord_CamXcYcZc(2, i) - Coord_CamXcYcZc.block(0, i, 2, 1) * RoundCoordRoundXk_Cam(3 * i + 2, j) / pow(Coord_CamXcYcZc(2, i), 2));
		}
	}

	return Hmatrix;
}

// hx구하거나 H구하는것중에 먼저 사용되는 애 에서 사용되면 되겠네.
// [Xc, Yc, Zc] 3 x 7 행렬 구하기
void findCoordinateFromCamera()
{
	Coord_CamXcYcZc.col(MID_HIP) = rot_C_W * trans_W_M + trans_C_W;
	Coord_CamXcYcZc.col(R_HIP) = rot_C_W * (rot_W_M_new * trans_M_R0 + trans_W_M) + trans_C_W;
	Coord_CamXcYcZc.col(R_KNEE) = rot_C_W * (rot_W_M_new * (rot_M_R0_new * trans_R0_R1 + trans_M_R0) + trans_W_M) + trans_C_W;
	Coord_CamXcYcZc.col(R_ANKLE) = rot_C_W * (rot_W_M_new * (rot_M_R0_new * (rot_R0_R1_new * (trans_R1_R2)+trans_R0_R1) + trans_M_R0) + trans_W_M) + trans_C_W;
	Coord_CamXcYcZc.col(L_HIP) = rot_C_W * (rot_W_M_new * trans_M_L0 + trans_W_M) + trans_C_W;
	Coord_CamXcYcZc.col(L_KNEE) = rot_C_W * (rot_W_M_new * (rot_M_L0_new * trans_L0_L1 + trans_M_L0) + trans_W_M) + trans_C_W;
	Coord_CamXcYcZc.col(L_ANKLE) = rot_C_W * (rot_W_M_new * (rot_M_L0_new * (rot_L0_L1_new * (trans_L1_L2)+trans_L0_L1) + trans_M_L0) + trans_W_M) + trans_C_W;
}

// 완성
// 다른 함수에서 R_?_?_new 를 쓸텐데 쓰기 쉽게 미리 업데이트해주는 함수
void updateRotation_new()
{
	rot_W_M_new = rot_W_M_delta * rot_W_M_old;
	rot_M_R0_new = rot_M_R0_delta * rot_M_R0_old;
	rot_R0_R1_new = rot_R0_R1_delta * rot_R0_R1_old;
	rot_M_L0_new = rot_M_L0_delta * rot_M_L0_old;
	rot_L0_L1_new = rot_L0_L1_delta * rot_L0_L1_old;
}

// 완성
// hx, H 다 구한후에 old 해주어야 한다.
void updateRotation_old()
{
	rot_W_M_old = rot_W_M_new;
	rot_M_R0_old = rot_M_R0_new;
	rot_R0_R1_old = rot_R0_R1_new;
	rot_M_L0_old = rot_M_L0_new;
	rot_L0_L1_old = rot_L0_L1_new;
}

// translation 행렬을 loop마다 업데이트 해주어야한다.
void updateTranslation()
{
	// 3 x  1 행렬
	trans_W_M << Xk(XK_TX, 0), Xk(XK_TY, 0), Xk(XK_TZ, 0) + Xk(XK_B, 0) + Xk(XK_C, 0) + 0.08;
	trans_M_R0 << Xk(XK_A, 0), 0, 0;
	trans_R0_R1 << Xk(XK_B, 0), 0, 0;
	trans_R1_R2 << Xk(XK_C, 0), 0, 0;
	trans_M_L0 << -Xk(XK_A, 0), 0, 0;
	trans_L0_L1 << Xk(XK_B, 0), 0, 0;
	trans_L1_L2 << Xk(XK_C, 0), 0, 0;
}
// 완성
// 편미분 관련한 로테이션은 다 해줘야한다.
// R_old 도 업데이트 해주어야 한다. 여기서는 안해준다.
void updateRoundRotDeltaMatrix(double WRX, double WRY, double WRZ, double WR, double WLX, double WLY, double WLZ, double WL, double Wpsi)
{
	// round W_MroundWpsi_delta
	rot_W_Mdelta_roundWpsi(0, 0) = -Wpsi;
	rot_W_Mdelta_roundWpsi(0, 1) = pow(Wpsi, 2) / (4 * pow(1 - pow(Wpsi, 2) / 4, 1 / 2)) - pow(1 - pow(Wpsi, 2) / 4, 1 / 2);
	rot_W_Mdelta_roundWpsi(0, 2) = 0;


	rot_W_Mdelta_roundWpsi(1, 0) = pow(1 - pow(Wpsi, 2) / 4, 1 / 2) - pow(Wpsi, 2) / (4 * pow(1 - pow(Wpsi, 2) / 4, 1 / 2));
	rot_W_Mdelta_roundWpsi(1, 1) = -Wpsi;
	rot_W_Mdelta_roundWpsi(1, 2) = 0;


	rot_W_Mdelta_roundWpsi(2, 0) = 0;
	rot_W_Mdelta_roundWpsi(2, 1) = 0;
	rot_W_Mdelta_roundWpsi(2, 2) = 0;

	// round  M_R0roundWRX_delta
	rot_M_R0delta_roundWRX(0, 0) = 0;
	rot_M_R0delta_roundWRX(0, 1) = WRY / 2 + (WRX * WRZ) / (4 * pow(1 - pow(WRY, 2) / 4 - pow(WRZ, 2) / 4 - pow(WRX, 2) / 4, 1 / 2));
	rot_M_R0delta_roundWRX(0, 2) = WRZ / 2 - (WRX * WRY) / (4 * pow(1 - pow(WRY, 2) / 4 - pow(WRZ, 2) / 4 - pow(WRX, 2) / 4, 1 / 2));


	rot_M_R0delta_roundWRX(1, 0) = WRY / 2 - (WRX * WRZ) / (4 * pow(1 - pow(WRY, 2) / 4 - pow(WRZ, 2) / 4 - pow(WRX, 2) / 4, 1 / 2));
	rot_M_R0delta_roundWRX(1, 1) = -WRX;
	rot_M_R0delta_roundWRX(1, 2) = pow(WRX, 2) / (4 * pow(1 - pow(WRY, 2) / 4 - pow(WRZ, 2) / 4 - pow(WRX, 2) / 4, 1 / 2)) - pow(1 - pow(WRY, 2) / 4 - pow(WRZ, 2) / 4 - pow(WRX, 2) / 4, 1 / 2);


	rot_M_R0delta_roundWRX(2, 0) = WRZ / 2 + (WRX * WRY) / (4 * pow(1 - pow(WRY, 2) / 4 - pow(WRZ, 2) / 4 - pow(WRX, 2) / 4, 1 / 2));
	rot_M_R0delta_roundWRX(2, 1) = pow(1 - pow(WRY, 2) / 4 - pow(WRZ, 2) / 4 - pow(WRX, 2) / 4, 1 / 2) - pow(WRX, 2) / (4 * pow(1 - pow(WRY, 2) / 4 - pow(WRZ, 2) / 4 - pow(WRX, 2) / 4, 1 / 2));
	rot_M_R0delta_roundWRX(2, 2) = -WRX;

	// round M_R0roundWRY_delta
	rot_M_R0delta_roundWRY(0, 0) = -WRY;
	rot_M_R0delta_roundWRY(0, 1) = WRX / 2 + (WRY * WRZ) / (4 * pow(1 - pow(WRY, 2) / 4 - pow(WRZ, 2) / 4 - pow(WRX, 2) / 4, 1 / 2));
	rot_M_R0delta_roundWRY(0, 2) = pow(1 - pow(WRY, 2) / 4 - pow(WRZ, 2) / 4 - pow(WRX, 2) / 4, 1 / 2) - pow(WRY, 2) / (4 * pow(1 - pow(WRY, 2) / 4 - pow(WRZ, 2) / 4 - pow(WRX, 2) / 4, 1 / 2));


	rot_M_R0delta_roundWRY(1, 0) = WRX / 2 - (WRY * WRZ) / (4 * pow(1 - pow(WRY, 2) / 4 - pow(WRZ, 2) / 4 - pow(WRX, 2) / 4, 1 / 2));
	rot_M_R0delta_roundWRY(1, 1) = 0;
	rot_M_R0delta_roundWRY(1, 2) = WRZ / 2 + (WRX * WRY) / (4 * pow(1 - pow(WRY, 2) / 4 - pow(WRZ, 2) / 4 - pow(WRX, 2) / 4, 1 / 2));


	rot_M_R0delta_roundWRY(2, 0) = pow(WRY, 2) / (4 * pow(1 - pow(WRY, 2) / 4 - pow(WRZ, 2) / 4 - pow(WRX, 2) / 4, 1 / 2)) - pow(1 - pow(WRY, 2) / 4 - pow(WRZ, 2) / 4 - pow(WRX, 2) / 4, 1 / 2);
	rot_M_R0delta_roundWRY(2, 1) = WRZ / 2 - (WRX * WRY) / (4 * pow(1 - pow(WRY, 2) / 4 - pow(WRZ, 2) / 4 - pow(WRX, 2) / 4, 1 / 2));
	rot_M_R0delta_roundWRY(2, 2) = -WRY;

	// round M_R0roundWRZ_delta
	rot_M_R0delta_roundWRZ(0, 0) = -WRZ;
	rot_M_R0delta_roundWRZ(0, 1) = pow(WRZ, 2) / (4 * pow(1 - pow(WRY, 2) / 4 - pow(WRZ, 2) / 4 - pow(WRX, 2) / 4, 1 / 2)) - pow(1 - pow(WRY, 2) / 4 - pow(WRZ, 2) / 4 - pow(WRX, 2) / 4, 1 / 2);
	rot_M_R0delta_roundWRZ(0, 2) = WRX / 2 - (WRY * WRZ) / (4 * pow(1 - pow(WRY, 2) / 4 - pow(WRZ, 2) / 4 - pow(WRX, 2) / 4, 1 / 2));


	rot_M_R0delta_roundWRZ(1, 0) = pow(1 - pow(WRY, 2) / 4 - pow(WRZ, 2) / 4 - pow(WRX, 2) / 4, 1 / 2) - pow(WRZ, 2) / (4 * pow(1 - pow(WRY, 2) / 4 - pow(WRZ, 2) / 4 - pow(WRX, 2) / 4, 1 / 2));
	rot_M_R0delta_roundWRZ(1, 1) = -WRZ;
	rot_M_R0delta_roundWRZ(1, 2) = WRY / 2 + (WRX * WRZ) / (4 * pow(1 - pow(WRY, 2) / 4 - pow(WRZ, 2) / 4 - pow(WRX, 2) / 4, 1 / 2));


	rot_M_R0delta_roundWRZ(2, 0) = WRX / 2 + (WRY * WRZ) / (4 * pow(1 - pow(WRY, 2) / 4 - pow(WRZ, 2) / 4 - pow(WRX, 2) / 4, 1 / 2));
	rot_M_R0delta_roundWRZ(2, 1) = WRY / 2 - (WRX * WRZ) / (4 * pow(1 - pow(WRY, 2) / 4 - pow(WRZ, 2) / 4 - pow(WRX, 2) / 4, 1 / 2));
	rot_M_R0delta_roundWRZ(2, 2) = 0;



	// round R0_R1roundWR_delta
	rot_R0_R1delta_roundWR(0, 0) = -WR;
	rot_R0_R1delta_roundWR(0, 1) = pow(WR, 2) / (4 * pow(1 - pow(WR, 2) / 4, 1 / 2)) - pow(1 - pow(WR, 2) / 4, 1 / 2);
	rot_R0_R1delta_roundWR(0, 2) = 0;


	rot_R0_R1delta_roundWR(1, 0) = pow(1 - pow(WR, 2) / 4, 1 / 2) - pow(WR, 2) / (4 * pow(1 - pow(WR, 2) / 4, 1 / 2));
	rot_R0_R1delta_roundWR(1, 1) = -WR;
	rot_R0_R1delta_roundWR(1, 2) = 0;


	rot_R0_R1delta_roundWR(2, 0) = 0;
	rot_R0_R1delta_roundWR(2, 1) = 0;
	rot_R0_R1delta_roundWR(2, 2) = 0;



	// round M_L0roundWLX_delta
	rot_M_L0delta_roundWLX(0, 0) = 0;
	rot_M_L0delta_roundWLX(0, 1) = WLY / 2 + (WLX * WLZ) / (4 * pow(1 - pow(WLY, 2) / 4 - pow(WLZ, 2) / 4 - pow(WLX, 2) / 4, 1 / 2));
	rot_M_L0delta_roundWLX(0, 2) = WLZ / 2 - (WLX * WLY) / (4 * pow(1 - pow(WLY, 2) / 4 - pow(WLZ, 2) / 4 - pow(WLX, 2) / 4, 1 / 2));


	rot_M_L0delta_roundWLX(1, 0) = WLY / 2 - (WLX * WLZ) / (4 * pow(1 - pow(WLY, 2) / 4 - pow(WLZ, 2) / 4 - pow(WLX, 2) / 4, 1 / 2));
	rot_M_L0delta_roundWLX(1, 1) = -WLX;
	rot_M_L0delta_roundWLX(1, 2) = pow(WLX, 2) / (4 * pow(1 - pow(WLY, 2) / 4 - pow(WLZ, 2) / 4 - pow(WLX, 2) / 4, 1 / 2)) - pow(1 - pow(WLY, 2) / 4 - pow(WLZ, 2) / 4 - pow(WLX, 2) / 4, 1 / 2);


	rot_M_L0delta_roundWLX(2, 0) = WLZ / 2 + (WLX * WLY) / (4 * pow(1 - pow(WLY, 2) / 4 - pow(WLZ, 2) / 4 - pow(WLX, 2) / 4, 1 / 2));
	rot_M_L0delta_roundWLX(2, 1) = pow(1 - pow(WLY, 2) / 4 - pow(WLZ, 2) / 4 - pow(WLX, 2) / 4, 1 / 2) - pow(WLX, 2) / (4 * pow(1 - pow(WLY, 2) / 4 - pow(WLZ, 2) / 4 - pow(WLX, 2) / 4, 1 / 2));
	rot_M_L0delta_roundWLX(2, 2) = -WLX;


	// round M_L0roundWLY_delta
	rot_M_L0delta_roundWLY(0, 0) = -WLY;
	rot_M_L0delta_roundWLY(0, 1) = WLX / 2 + (WLY * WLZ) / (4 * pow(1 - pow(WLY, 2) / 4 - pow(WLZ, 2) / 4 - pow(WLX, 2) / 4, 1 / 2));
	rot_M_L0delta_roundWLY(0, 2) = pow(1 - pow(WLY, 2) / 4 - pow(WLZ, 2) / 4 - pow(WLX, 2) / 4, 1 / 2) - pow(WLY, 2) / (4 * pow(1 - pow(WLY, 2) / 4 - pow(WLZ, 2) / 4 - pow(WLX, 2) / 4, 1 / 2));


	rot_M_L0delta_roundWLY(1, 0) = WLX / 2 - (WLY * WLZ) / (4 * pow(1 - pow(WLY, 2) / 4 - pow(WLZ, 2) / 4 - pow(WLX, 2) / 4, 1 / 2));
	rot_M_L0delta_roundWLY(1, 1) = 0;
	rot_M_L0delta_roundWLY(1, 2) = WLZ / 2 + (WLX * WLY) / (4 * pow(1 - pow(WLY, 2) / 4 - pow(WLZ, 2) / 4 - pow(WLX, 2) / 4, 1 / 2));


	rot_M_L0delta_roundWLY(2, 0) = pow(WLY, 2) / (4 * pow(1 - pow(WLY, 2) / 4 - pow(WLZ, 2) / 4 - pow(WLX, 2) / 4, 1 / 2)) - pow(1 - pow(WLY, 2) / 4 - pow(WLZ, 2) / 4 - pow(WLX, 2) / 4, 1 / 2);
	rot_M_L0delta_roundWLY(2, 1) = WLZ / 2 - (WLX * WLY) / (4 * pow(1 - pow(WLY, 2) / 4 - pow(WLZ, 2) / 4 - pow(WLX, 2) / 4, 1 / 2));
	rot_M_L0delta_roundWLY(2, 2) = -WLY;


	// round M_L0roundWLZ_delta
	rot_M_L0delta_roundWLZ(0, 0) = -WLZ;
	rot_M_L0delta_roundWLZ(0, 1) = pow(WLZ, 2) / (4 * pow(1 - pow(WLY, 2) / 4 - pow(WLZ, 2) / 4 - pow(WLX, 2) / 4, 1 / 2)) - pow(1 - pow(WLY, 2) / 4 - pow(WLZ, 2) / 4 - pow(WLX, 2) / 4, 1 / 2);
	rot_M_L0delta_roundWLZ(0, 2) = WLX / 2 - (WLY * WLZ) / (4 * pow(1 - pow(WLY, 2) / 4 - pow(WLZ, 2) / 4 - pow(WLX, 2) / 4, 1 / 2));


	rot_M_L0delta_roundWLZ(1, 0) = pow(1 - pow(WLY, 2) / 4 - pow(WLZ, 2) / 4 - pow(WLX, 2) / 4, 1 / 2) - pow(WLZ, 2) / (4 * pow(1 - pow(WLY, 2) / 4 - pow(WLZ, 2) / 4 - pow(WLX, 2) / 4, 1 / 2));
	rot_M_L0delta_roundWLZ(1, 1) = -WLZ;
	rot_M_L0delta_roundWLZ(1, 2) = WLY / 2 + (WLX * WLZ) / (4 * pow(1 - pow(WLY, 2) / 4 - pow(WLZ, 2) / 4 - pow(WLX, 2) / 4, 1 / 2));


	rot_M_L0delta_roundWLZ(2, 0) = WLX / 2 + (WLY * WLZ) / (4 * pow(1 - pow(WLY, 2) / 4 - pow(WLZ, 2) / 4 - pow(WLX, 2) / 4, 1 / 2));
	rot_M_L0delta_roundWLZ(2, 1) = WLY / 2 - (WLX * WLZ) / (4 * pow(1 - pow(WLY, 2) / 4 - pow(WLZ, 2) / 4 - pow(WLX, 2) / 4, 1 / 2));
	rot_M_L0delta_roundWLZ(2, 2) = 0;


	// round L0_L1roundWL_delta
	rot_L0_L1delta_roundWL(0, 0) = -WL;
	rot_L0_L1delta_roundWL(0, 1) = pow(WL, 2) / (4 * pow(1 - pow(WL, 2) / 4, 1 / 2)) - pow(1 - pow(WL, 2) / 4, 1 / 2);
	rot_L0_L1delta_roundWL(0, 2) = 0;


	rot_L0_L1delta_roundWL(1, 0) = pow(1 - pow(WL, 2) / 4, 1 / 2) - pow(WL, 2) / (4 * pow(1 - pow(WL, 2) / 4, 1 / 2));
	rot_L0_L1delta_roundWL(1, 1) = -WL;
	rot_L0_L1delta_roundWL(1, 2) = 0;


	rot_L0_L1delta_roundWL(2, 0) = 0;
	rot_L0_L1delta_roundWL(2, 1) = 0;
	rot_L0_L1delta_roundWL(2, 2) = 0;
}



// 이거 식은 완성됨. ==> 이름 바꿨음.
void GetRoundCameraCoordinate2roundXk()
{
	RoundCoordRoundXk_Cam.block(3 * MID_HIP, XK_TX, 3, 1) = rot_C_W.block(0, 0, 3, 1);
	RoundCoordRoundXk_Cam.block(3 * MID_HIP, XK_TY, 3, 1) = rot_C_W.block(0, 1, 3, 1);
	RoundCoordRoundXk_Cam.block(3 * MID_HIP, XK_TZ, 3, 1) = RoundCoordRoundXk_Cam.block(3 * MID_HIP, XK_B, 3, 1) = RoundCoordRoundXk_Cam.block(3 * MID_HIP, XK_C, 3, 1) = rot_C_W.block(0, 2, 3, 1);

	RoundCoordRoundXk_Cam.block(3 * R_HIP, XK_A, 3, 1) = rot_C_W * rot_W_M_new.block(0, 0, 3, 1);
	RoundCoordRoundXk_Cam.block(3 * L_HIP, XK_A, 3, 1) = -rot_C_W * rot_W_M_new.block(0, 0, 3, 1);
	RoundCoordRoundXk_Cam.block(3 * R_HIP, XK_TX, 3, 1) = rot_C_W.block(0, 0, 3, 1);
	RoundCoordRoundXk_Cam.block(3 * R_HIP, XK_TY, 3, 1) = rot_C_W.block(0, 1, 3, 1);
	RoundCoordRoundXk_Cam.block(3 * L_HIP, XK_TX, 3, 1) = rot_C_W.block(0, 0, 3, 1);
	RoundCoordRoundXk_Cam.block(3 * L_HIP, XK_TY, 3, 1) = rot_C_W.block(0, 1, 3, 1);
	RoundCoordRoundXk_Cam.block(3 * R_HIP, XK_TZ, 3, 1) = RoundCoordRoundXk_Cam.block(3 * R_HIP, XK_B, 3, 1) = RoundCoordRoundXk_Cam.block(3 * R_HIP, XK_C, 3, 1) = rot_C_W.block(0, 2, 3, 1);
	RoundCoordRoundXk_Cam.block(3 * L_HIP, XK_TZ, 3, 1) = RoundCoordRoundXk_Cam.block(3 * L_HIP, XK_B, 3, 1) = RoundCoordRoundXk_Cam.block(3 * L_HIP, XK_C, 3, 1) = rot_C_W.block(0, 2, 3, 1);
	
	
	// 이거 뭐지 ?? 
	// trans_M_R0
	RoundCoordRoundXk_Cam.block(3 * R_HIP, XK_WPSI, 3, 1) = rot_C_W * rot_W_Mdelta_roundWpsi * rot_W_M_old * trans_M_R0;
	RoundCoordRoundXk_Cam.block(3 * L_HIP, XK_WPSI, 3, 1) = rot_C_W * rot_W_Mdelta_roundWpsi * rot_W_M_old * trans_M_L0;

	RoundCoordRoundXk_Cam.block(3 * R_KNEE, XK_WPSI, 3, 1) = rot_C_W * rot_W_Mdelta_roundWpsi * rot_W_M_old * (rot_M_R0_new * trans_R0_R1 + trans_M_R0);
	RoundCoordRoundXk_Cam.block(3 * L_KNEE, XK_WPSI, 3, 1) = rot_C_W * rot_W_Mdelta_roundWpsi * rot_W_M_old * (rot_M_L0_new * trans_L0_L1 + trans_M_L0);
	RoundCoordRoundXk_Cam.block(3 * R_KNEE, XK_RX, 3, 1) = rot_C_W * rot_W_M_new * rot_M_R0delta_roundWRX * rot_M_R0_old * trans_R0_R1;
	RoundCoordRoundXk_Cam.block(3 * R_KNEE, XK_RY, 3, 1) = rot_C_W * rot_W_M_new * rot_M_R0delta_roundWRY * rot_M_R0_old * trans_R0_R1;
	RoundCoordRoundXk_Cam.block(3 * R_KNEE, XK_RZ, 3, 1) = rot_C_W * rot_W_M_new * rot_M_R0delta_roundWRZ * rot_M_R0_old * trans_R0_R1;
	RoundCoordRoundXk_Cam.block(3 * L_KNEE, XK_RX, 3, 1) = rot_C_W * rot_W_M_new * rot_M_L0delta_roundWLX * rot_M_L0_old * trans_L0_L1;
	RoundCoordRoundXk_Cam.block(3 * L_KNEE, XK_RY, 3, 1) = rot_C_W * rot_W_M_new * rot_M_L0delta_roundWLY * rot_M_L0_old * trans_L0_L1;
	RoundCoordRoundXk_Cam.block(3 * L_KNEE, XK_RZ, 3, 1) = rot_C_W * rot_W_M_new * rot_M_L0delta_roundWLZ * rot_M_L0_old * trans_L0_L1;
	RoundCoordRoundXk_Cam.block(3 * R_KNEE, XK_A, 3, 1) = (rot_C_W * rot_W_M_new).block(0, 0, 3, 1);
	RoundCoordRoundXk_Cam.block(3 * L_KNEE, XK_A, 3, 1) = -(rot_C_W * rot_W_M_new).block(0, 0, 3, 1);
	RoundCoordRoundXk_Cam.block(3 * R_KNEE, XK_B, 3, 1) = (rot_C_W * rot_W_M_new * rot_M_R0_new).block(0, 0, 3, 1) + rot_C_W.block(0, 2, 3, 1);
	RoundCoordRoundXk_Cam.block(3 * L_KNEE, XK_B, 3, 1) = (rot_C_W * rot_W_M_new * rot_M_L0_new).block(0, 0, 3, 1) + rot_C_W.block(0, 2, 3, 1);
	RoundCoordRoundXk_Cam.block(3 * R_KNEE, XK_C, 3, 1) = RoundCoordRoundXk_Cam.block(3 * L_KNEE, XK_C, 3, 1) = rot_C_W.block(0, 2, 3, 1);
	RoundCoordRoundXk_Cam.block(3 * R_KNEE, XK_TX, 3, 1) = RoundCoordRoundXk_Cam.block(3 * L_KNEE, XK_TX, 3, 1) = rot_C_W.block(0, 0, 3, 1);
	RoundCoordRoundXk_Cam.block(3 * R_KNEE, XK_TY, 3, 1) = RoundCoordRoundXk_Cam.block(3 * L_KNEE, XK_TY, 3, 1) = rot_C_W.block(0, 1, 3, 1);
	RoundCoordRoundXk_Cam.block(3 * R_KNEE, XK_TZ, 3, 1) = RoundCoordRoundXk_Cam.block(3 * L_KNEE, XK_TZ, 3, 1) = rot_C_W.block(0, 2, 3, 1);

	RoundCoordRoundXk_Cam.block(3 * R_ANKLE, XK_WPSI, 3, 1) = rot_C_W * rot_W_Mdelta_roundWpsi * rot_W_M_old * (rot_M_R0_new * (rot_R0_R1_new * trans_R1_R2 + trans_R0_R1) + trans_M_R0);
	RoundCoordRoundXk_Cam.block(3 * L_ANKLE, XK_WPSI, 3, 1) = rot_C_W * rot_W_Mdelta_roundWpsi * rot_W_M_old * (rot_M_L0_new * (rot_L0_L1_new * trans_L1_L2 + trans_L0_L1) + trans_M_L0);
	RoundCoordRoundXk_Cam.block(3 * R_ANKLE, XK_RX, 3, 1) = rot_C_W * rot_W_M_new * rot_M_R0delta_roundWRX * rot_M_R0_old * (rot_R0_R1_new * trans_R1_R2 + trans_R0_R1);
	RoundCoordRoundXk_Cam.block(3 * R_ANKLE, XK_RY, 3, 1) = rot_C_W * rot_W_M_new * rot_M_R0delta_roundWRY * rot_M_R0_old * (rot_R0_R1_new * trans_R1_R2 + trans_R0_R1);
	RoundCoordRoundXk_Cam.block(3 * R_ANKLE, XK_RZ, 3, 1) = rot_C_W * rot_W_M_new * rot_M_R0delta_roundWRZ * rot_M_R0_old * (rot_R0_R1_new * trans_R1_R2 + trans_R0_R1);
	RoundCoordRoundXk_Cam.block(3 * L_ANKLE, XK_RX, 3, 1) = rot_C_W * rot_W_M_new * rot_M_L0delta_roundWLX * rot_M_L0_old * (rot_L0_L1_new * trans_L1_L2 + trans_L0_L1);
	RoundCoordRoundXk_Cam.block(3 * L_ANKLE, XK_RY, 3, 1) = rot_C_W * rot_W_M_new * rot_M_L0delta_roundWLY * rot_M_L0_old * (rot_L0_L1_new * trans_L1_L2 + trans_L0_L1);
	RoundCoordRoundXk_Cam.block(3 * L_ANKLE, XK_RZ, 3, 1) = rot_C_W * rot_W_M_new * rot_M_L0delta_roundWLZ * rot_M_L0_old * (rot_L0_L1_new * trans_L1_L2 + trans_L0_L1);
	RoundCoordRoundXk_Cam.block(3 * R_ANKLE, XK_R, 3, 1) = rot_C_W * rot_W_M_new * rot_M_R0_new * rot_R0_R1delta_roundWR * rot_R0_R1_old * trans_R1_R2;
	RoundCoordRoundXk_Cam.block(3 * L_ANKLE, XK_L, 3, 1) = rot_C_W * rot_W_M_new * rot_M_L0_new * rot_L0_L1delta_roundWL * rot_L0_L1_old * trans_L1_L2;
	RoundCoordRoundXk_Cam.block(3 * R_ANKLE, XK_A, 3, 1) = (rot_C_W * rot_W_M_new).block(0, 0, 3, 1);
	RoundCoordRoundXk_Cam.block(3 * L_ANKLE, XK_A, 3, 1) = -(rot_C_W * rot_W_M_new).block(0, 0, 3, 1);
	RoundCoordRoundXk_Cam.block(3 * R_ANKLE, XK_B, 3, 1) = (rot_C_W * rot_W_M_new * rot_M_R0_new).block(0, 0, 3, 1) + rot_C_W.block(0, 2, 3, 1);
	RoundCoordRoundXk_Cam.block(3 * L_ANKLE, XK_B, 3, 1) = (rot_C_W * rot_W_M_new * rot_M_L0_new).block(0, 0, 3, 1) + rot_C_W.block(0, 2, 3, 1);
	RoundCoordRoundXk_Cam.block(3 * R_ANKLE, XK_C, 3, 1) = (rot_C_W * rot_W_M_new * rot_M_R0_new * rot_R0_R1_new).block(0, 0, 3, 1) + rot_C_W.block(0, 2, 3, 1);
	RoundCoordRoundXk_Cam.block(3 * L_ANKLE, XK_C, 3, 1) = (rot_C_W * rot_W_M_new * rot_M_L0_new * rot_L0_L1_new).block(0, 0, 3, 1) + rot_C_W.block(0, 2, 3, 1);
	RoundCoordRoundXk_Cam.block(3 * R_ANKLE, XK_TX, 3, 1) = RoundCoordRoundXk_Cam.block(3 * L_ANKLE, XK_TX, 3, 1) = rot_C_W.block(0, 0, 3, 1);
	RoundCoordRoundXk_Cam.block(3 * R_ANKLE, XK_TY, 3, 1) = RoundCoordRoundXk_Cam.block(3 * L_ANKLE, XK_TY, 3, 1) = rot_C_W.block(0, 1, 3, 1);
	RoundCoordRoundXk_Cam.block(3 * R_ANKLE, XK_TZ, 3, 1) = RoundCoordRoundXk_Cam.block(3 * L_ANKLE, XK_TZ, 3, 1) = rot_C_W.block(0, 2, 3, 1);



}


// 흐름 조심해야해
MatrixXd Calculate_hx()
{
	MatrixXd hx = MatrixXd(14, 1);      // hx matrix
	double tmpZc=0.0;
	for (int i = 0; i < Coord_CamXcYcZc.cols(); i++)
	{
		// get Zc
		tmpZc = Coord_CamXcYcZc(Z, i);
		// Ximg
		hx(2 * i, 0) = (rot_P_C  * Coord_CamXcYcZc.col(i) / tmpZc)(X, 0);
		// Yimg
		hx(2 * i + 1, 0) = (rot_P_C * Coord_CamXcYcZc.col(i) / tmpZc)(Y, 0);
		
	}
	
	return hx;
}

MatrixXd GetObservation(std::vector<coordinate> vJoints)
{
	MatrixXd Zk = MatrixXd(14, 1);
	
	for (int i = 0; i < vJoints.size(); i++)
	{
		double u = vJoints[i].j; // x
		double v = vJoints[i].i; // y
		
		Zk(2 * i, 0) = u; // 관측된 joints의 x좌표
		Zk(2 * i + 1, 0) = v; // 관측된 joints의 y좌표
	}
	return Zk;
}

void EKFpredict()
{
	ekfP = ekfP + ekfQ;
}

void EKFupdate(MatrixXd Zk, MatrixXd H, MatrixXd hx)
{

	MatrixXd Y = Zk - hx;
	//std::cout << Y << std::endl;
	MatrixXd S = H * ekfP * H.transpose() + ekfR;
	//std::cout << S << std::endl;
	MatrixXd Kk = ekfP * H.transpose() * S.inverse();
	//std::cout << Kk << std::endl;
	MatrixXd I = MatrixXd::Identity(15, 15);

	// update Xk, Pk
	Xk = Xk + Kk * Y;
	//std::cout << Kk << std::endl;
	ekfP = (I - Kk * H) * ekfP;
}


void printRot_Trans()
{
	cout << "------------- rotation ---------------"  << endl;
	cout << "C to W \n" << rot_C_W << endl;
	cout << "W to M \n" << rot_W_M_new << endl;
	cout << "M to R0 \n" << rot_M_R0_new << endl;
	cout << "R0 to R1 \n" << rot_R0_R1_new << endl;
	cout << "M to L0 \n" << rot_M_L0_new << endl;
	cout << "L0 to L1 \n" << rot_L0_L1_new << endl;
	
	cout << "------------- translation ---------------" << endl;
	cout << "C to W \n" << trans_C_W << endl;
	cout << "W to M \n" << trans_W_M << endl;
	cout << "M to R0 \n" << trans_M_R0 << endl;
	cout << "R0 to R1 \n" << trans_R0_R1 << endl;
	cout << "R1 to R2 \n" << trans_R1_R2 << endl;
	cout << "M to L0 \n" << trans_M_L0 << endl;
	cout << "L0 to L1 \n" << trans_L0_L1 << endl;
	cout << "L1 to L2 \n" << trans_L1_L2 << endl;
}


int main()
{

	FileIO fio;

	double cameraParams[10] = { 1436.35689, 1435.47675, 951.08945, 549.32842, 0, 0.05382, -0.16741, -0.00558, -0.00092, 0 };
	EKF ekf(cameraParams);


	// 마지막 ekf 테스트 용 함수
	InitEKF();
	

	// 저장된 데이터를 가져와서 칼만필터 실행(테스트)하는 부분
	std::ifstream reader;
	reader.open("body_data/1.가만히ver_3.txt");
	bool finalFlag = false;
	if (reader.is_open())
	{
		while (!reader.eof())
		{
			std::vector<coordinate> vJoints;

			//printf("여기부터 새로 읽는다\n");
			for (int i = 0; i < 7; i++) // joints 갯수 7개
			{
				// 6줄 읽고 vJoint에 넣어
				std::string str;
				std::getline(reader, str);

				if (str.length() == 0) // end of file
				{
					finalFlag = true;
					break;
				}
				else // non-end
				{
					// vJoints 에 넣는 과정
					double* dData;
					dData = fio.Strline2doubleArr(str);;
					double dX = dData[0], dY = dData[1];
					vJoints.push_back(coordinate(dY, dX));
				}
			}
			if (!finalFlag)
			{
				/*for (int i = 0; i < 7; i++)
				{
					std::cout << "[" << i << "]번째 좌표 : (x,y) : " << vJoints[i].j << ", " << vJoints[i].i << std::endl;
				}*/
				//ekf.Execute(vJoints); // vjoint 왜곡보정 실행

				assert(vJoints.size() == 7);
				MatrixXd Zk = GetObservation(vJoints);
				EKFpredict();
				MatrixXd H = GetJacobianMatrix()


				MatrixXd hx = Calculate_hx();



				std::cout << "------------Zk 출력 -----------\n" << Zk << std::endl;
				//-----  loop 돌때마다 업데이트해주어야하는 것들이 있다. -----//
				// delta rotation 업데이트
				updateRotation_delta(Xk(XK_RX, 0), Xk(XK_RY, 0), Xk(XK_RZ, 0), Xk(XK_R, 0), Xk(XK_LX, 0), Xk(XK_LY, 0), Xk(XK_LZ, 0), Xk(XK_L, 0), Xk(XK_WPSI, 0));
				std::cout << "------------Xk 출력 ------------\n" << Xk << std::endl;

				//rotation_new update; // R_new = R_delta * R_old
				updateRotation_new(); // hx에서나 H에서 사용하기 위한 R_new 를 만든다.
				updateTranslation(); // 새로 추정한 Xk대로 trans를 업데이트한다.
				findCoordinateFromCamera(); // Xc Yc Zc 구한게 hx, H 다 쓰임 

				//printRot_Trans();
				//위에서 hx를 구하기 위한 업데이트들이 끝나면 구한다.
				MatrixXd hx = Calculate_hx();
				std::cout << "------------hx 출력 ------------\n" << hx << std::endl;

				std::cout << "------------hx , Zk 차이 출력 ------------\n" << hx - Zk << std::endl;

				
				//rotation round update
				updateRoundRotDeltaMatrix(Xk(XK_RX, 0), Xk(XK_RY, 0), Xk(XK_RZ, 0), Xk(XK_R, 0), Xk(XK_LX, 0), Xk(XK_LY, 0), Xk(XK_LZ, 0), Xk(XK_L, 0), Xk(XK_WPSI, 0));
				MatrixXd H = GetJacobianMatrix(); // Zc^2 으로 나누면서 자코비안을 구한다.
				//std::cout << "------------H 출력 ------------\n" << H << std::endl;

				updateRotation_old();
				
				//std::cout << ekfP << std::endl;
				
				EKFupdate(Zk, H, hx); // X, P 가 업데이트 된다.
				
				//-------- print -----//
				/*std::cout << "------hx-------" << std::endl;
				std::cout << hx << std::endl;
				std::cout << "**************" << std::endl;
				std::cout << "--------Xk--------" << std::endl;
				std::cout << Xk << std::endl;
				std::cout << "************" << std::endl << std::endl;*/
			}
		}
	}
	reader.close();

	
	/*while(1)
	{

		MatrixXd Zk = GetObservation();
		//-----  loop 돌때마다 업데이트해주어야하는 것들이 있다. -----//
		// delta rotation 업데이트
		updateRotation_delta(Xk(XK_RX, 0), Xk(XK_RY, 0), Xk(XK_RZ, 0), Xk(XK_R, 0), Xk(XK_LX, 0), Xk(XK_LY, 0), Xk(XK_LZ, 0), Xk(XK_L, 0), Xk(XK_WPSI, 0));
		//rotation_new update; // R_new = R_delta * R_old
		updateRotation_new(); // hx에서나 H에서 사용하기 위한 R_new 를 만든다.
		updateTranslation(); // 새로 추정한 Xk대로 trans를 업데이트한다.
		findCoordinateFromCamera(); // Xc Yc Zc 구한게 hx, H 다 쓰임 
		
		//위에서 hx를 구하기 위한 업데이트들이 끝나면 구한다.
		MatrixXd hx = Calculate_hx();
		
		//rotation round update
		updateRoundRotDeltaMatrix(Xk(XK_RX, 0), Xk(XK_RY, 0), Xk(XK_RZ, 0), Xk(XK_R, 0), Xk(XK_LX, 0), Xk(XK_LY, 0), Xk(XK_LZ, 0), Xk(XK_L, 0), Xk(XK_WPSI, 0));
		MatrixXd H = GetJacobianMatrix(); // Zc^2 으로 나누면서 자코비안을 구한다.
		
		updateRotation_old();

		EKFpredict();
		EKFupdate(Zk, H, hx); // X, P 가 업데이트 된다.
		
	}*/
	
	

	
}


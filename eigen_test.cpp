













//#include <iostream>
//#include <math.h>
//#include <Eigen/Dense>
//
//using namespace Eigen;
//
//enum stateNum {
//	XK_A,         // assigned 0
//	XK_B,         // assigned 1
//	XK_C,      // assigned 2
//	XK_RX,   // assigned 3
//	XK_RY,   // assigned 4
//	XK_RZ,   // assigned 5
//	XK_R,      // assigned 6
//	XK_LX,   // assigned 7
//	XK_LY,   // assigned 8
//	XK_LZ,   // assigned 9
//	XK_L,      // assigned 10
//	XK_WPSI,      // assigned 11
//	XK_TX,      // assigned 12
//	XK_TY,      // assigned 13
//	XK_TZ      // assigned 14
//};
//
//enum Joint {
//	MID_HIP,      // assigned 0
//	R_HIP,      // assigned 1
//	R_KNEE,      // assigned 2
//	R_ANKLE,   // assigned 3
//	L_HIP,      // assigned 3
//	L_KNEE,      // assigned 4
//	L_ANKLE      // assigned 5
//};
//
//enum Coordinate3d {
//	X,         // assigned 0
//	Y,         // assigned 1
//	Z         // assigned 2
//};
//
//// set ekf matrix gain
//int R_sigma = 3; // 3 sigma
//double R_const[14] = { 1.915, 1.968, 1.85, 0.115, 0.044, 1.918, 1.927, 1.959, 0.045, 1.911, 0.015, 0.061, 0.015, 0.061 };      // R matrix's gain
//double Q_const[15] = { 0.03, 0.03, 0.03, 0.08, 0.08, 0.08, 0.08, 0.08, 0.08, 0.08, 0.08, 0.08, 0.03, 0.03, 0.03 };  // Q matrix's gain								 // P matrix's gain
//double P_const[15] = { 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0 };
//
//MatrixXd roundCoordroundXk_Cam = MatrixXd(21, 15);
//MatrixXd rot_C_W = MatrixXd(3, 3);
//
//MatrixXd rot_W_M_delta = MatrixXd(3, 3);
//MatrixXd rot_W_M_old = MatrixXd(3, 3);
//MatrixXd rot_W_M_new = MatrixXd(3, 3);
//
//MatrixXd rot_M_R0_delta = MatrixXd(3, 3);
//MatrixXd rot_M_R0_old = MatrixXd(3, 3);
//MatrixXd rot_M_R0_new = MatrixXd(3, 3);
//MatrixXd rot_R0_R1_delta = MatrixXd(3, 3);
//MatrixXd rot_R0_R1_old = MatrixXd(3, 3);
//MatrixXd rot_R0_R1_new = MatrixXd(3, 3);
//
//MatrixXd rot_M_L0_delta = MatrixXd(3, 3);
//MatrixXd rot_M_L0_old = MatrixXd(3, 3);
//MatrixXd rot_M_L0_new = MatrixXd(3, 3);
//MatrixXd rot_L0_L1_delta = MatrixXd(3, 3);
//MatrixXd rot_L0_L1_old = MatrixXd(3, 3);
//MatrixXd rot_L0_L1_new = MatrixXd(3, 3);
//
//// Translation matrix
//MatrixXd trans_C_W = MatrixXd(3, 1);
//MatrixXd trans_W_M = MatrixXd(3, 1);
//
//MatrixXd trans_M_R0 = MatrixXd(3, 1);
//MatrixXd trans_R0_R1 = MatrixXd(3, 1);
//MatrixXd trans_R1_R2 = MatrixXd(3, 1);
//
//MatrixXd trans_M_L0 = MatrixXd(3, 1);
//MatrixXd trans_L0_L1 = MatrixXd(3, 1);
//MatrixXd trans_L1_L2 = MatrixXd(3, 1);
//
//
//
//// round delta rotation matrix
//MatrixXd rot_W_Mdelta_roundWpsi = MatrixXd(3, 3);
//MatrixXd rot_M_R0delta_roundWRX = MatrixXd(3, 3);
//MatrixXd rot_M_R0delta_roundWRY = MatrixXd(3, 3);
//MatrixXd rot_M_R0delta_roundWRZ = MatrixXd(3, 3);
//MatrixXd rot_R0_R1delta_roundWR = MatrixXd(3, 3);
//MatrixXd rot_M_L0delta_roundWLX = MatrixXd(3, 3);
//MatrixXd rot_M_L0delta_roundWLY = MatrixXd(3, 3);
//MatrixXd rot_M_L0delta_roundWLZ = MatrixXd(3, 3);
//MatrixXd rot_L0_L1delta_roundWL = MatrixXd(3, 3);
//
//// round camera coodinate round Xk
//MatrixXd RoundCoordRoundXk_Cam = MatrixXd(21, 15);
//
//// camera coordinate ( Xc, Yc, Zc )
//MatrixXd Coord_Cam = MatrixXd(3, 7);
//
//// ekf matrix
//MatrixXd Hmatrix = MatrixXd(14, 15); // jacobian matrix
//MatrixXd K = MatrixXd(15, 14);	//kalman gain
//MatrixXd P = MatrixXd::Identity(15, 15);	//calculated covariance
//MatrixXd Q = MatrixXd(15, 15);	//covariance
//MatrixXd R = MatrixXd(14, 14);	//noise
//
//
//// 잠시 선언해둠!!!~~
//double FX = 0.0, FY = 0.0;
//
//// 자코비안 행렬구하기
//void findJacobianMatrix()
//{
//	// every joint
//	for (int i = 0; i < Hmatrix.rows(); i = +2) {
//		double tmpXc = Coord_Cam(X, i / 2);
//		double tmpYc = Coord_Cam(Y, i / 2);
//		double tmpZc = Coord_Cam(Z, i / 2);
//		// every state variable( Xk)
//		for (int j = 0; j < Hmatrix.cols(); j++) {
//			// round Xc / Zc round Xk
//			double tmpRoundXc = RoundCoordRoundXk_Cam(i / 2, j);
//			double tmpRoundYc = RoundCoordRoundXk_Cam(i / 2 + 1, j);
//			double tmpRoundZc = RoundCoordRoundXk_Cam(i / 2 + 2, j);
//
//			Hmatrix(i, j) = FX * (tmpRoundXc * tmpZc - tmpXc * tmpRoundZc) / pow(tmpZc, 2);
//			Hmatrix(i + 1, j) = FY * (tmpRoundYc * tmpZc - tmpYc * tmpRoundZc) / pow(tmpZc, 2);
//		}
//
//	}
//	//    for (int i = 0; i < Coord_Cam.cols(); i++) {
//	//        for (int j = 0; j < _H_.cols(); j++) {
//	//            //roundXc(Yc)/Zc - Xc(Yc) * roundZc / Zc^2
//	//            _H_.block(2 * i, j, 2, 1) = RoundCoordRoundXk_Cam.block(3 * i, j, 2, 1) / Coord_Cam(2, i) - XYZc.block(0, i, 2, 1) * (roundCam(3 * i, j) / pow(XYZc(2, i), 2));
//	//        }
//	//    }
//}
//
//// 카메라계 좌표 구하기
//void findCoordinateFromCamera()
//{
//	Coord_Cam.col(MID_HIP) = rot_C_W * trans_W_M + trans_C_W;
//	Coord_Cam.col(R_HIP) = rot_C_W * (rot_W_M_new * trans_M_R0 + trans_W_M) + trans_C_W;
//	Coord_Cam.col(R_KNEE) = rot_C_W * (rot_W_M_new * (rot_M_R0_new * trans_R0_R1 + trans_M_R0) + trans_W_M) + trans_C_W;
//	Coord_Cam.col(R_ANKLE) = rot_C_W * (rot_W_M_new * (rot_M_R0_new * (rot_R0_R1_new * (trans_R1_R2)+trans_R0_R1) + trans_M_R0) + trans_W_M) + trans_C_W;
//	Coord_Cam.col(L_HIP) = rot_C_W * (rot_W_M_new * trans_M_L0 + trans_W_M) + trans_C_W;
//	Coord_Cam.col(L_KNEE) = rot_C_W * (rot_W_M_new * (rot_M_L0_new * trans_L0_L1 + trans_M_L0) + trans_W_M) + trans_C_W;
//	Coord_Cam.col(L_ANKLE) = rot_C_W * (rot_W_M_new * (rot_M_L0_new * (rot_L0_L1_new * (trans_L1_L2)+trans_L0_L1) + trans_M_L0) + trans_W_M) + trans_C_W;
//}
//
//// 델타 로테이션 행렬을 각 state variable로 편미분한 행렬
//void updateRoundRotDeltaMatrix(double WRX, double WRY, double WRZ, double WR, double WLX, double WLY, double WLZ, double WL, double Wpsi)
//{
//	// round W_MroundWpsi_delta
//	rot_W_Mdelta_roundWpsi(0, 0) = -Wpsi;
//	rot_W_Mdelta_roundWpsi(0, 1) = pow(Wpsi, 2) / (4 * pow(1 - pow(Wpsi, 2) / 4, 1 / 2)) - pow(1 - pow(Wpsi, 2) / 4, 1 / 2);
//	rot_W_Mdelta_roundWpsi(0, 2) = 0;
//
//
//	rot_W_Mdelta_roundWpsi(1, 0) = pow(1 - pow(Wpsi, 2) / 4, 1 / 2) - pow(Wpsi, 2) / (4 * pow(1 - pow(Wpsi, 2) / 4, 1 / 2));
//	rot_W_Mdelta_roundWpsi(1, 1) = -Wpsi;
//	rot_W_Mdelta_roundWpsi(1, 2) = 0;
//
//
//	rot_W_Mdelta_roundWpsi(2, 0) = 0;
//	rot_W_Mdelta_roundWpsi(2, 1) = 0;
//	rot_W_Mdelta_roundWpsi(2, 2) = 0;
//
//	// round  M_R0roundWRX_delta
//	rot_M_R0delta_roundWRX(0, 0) = 0;
//	rot_M_R0delta_roundWRX(0, 1) = WRY / 2 + (WRX * WRZ) / (4 * pow(1 - pow(WRY, 2) / 4 - pow(WRZ, 2) / 4 - pow(WRX, 2) / 4, 1 / 2));
//	rot_M_R0delta_roundWRX(0, 2) = WRZ / 2 - (WRX * WRY) / (4 * pow(1 - pow(WRY, 2) / 4 - pow(WRZ, 2) / 4 - pow(WRX, 2) / 4, 1 / 2));
//
//
//	rot_M_R0delta_roundWRX(1, 0) = WRY / 2 - (WRX * WRZ) / (4 * pow(1 - pow(WRY, 2) / 4 - pow(WRZ, 2) / 4 - pow(WRX, 2) / 4, 1 / 2));
//	rot_M_R0delta_roundWRX(1, 1) = -WRX;
//	rot_M_R0delta_roundWRX(1, 2) = pow(WRX, 2) / (4 * pow(1 - pow(WRY, 2) / 4 - pow(WRZ, 2) / 4 - pow(WRX, 2) / 4, 1 / 2)) - pow(1 - pow(WRY, 2) / 4 - pow(WRZ, 2) / 4 - pow(WRX, 2) / 4, 1 / 2);
//
//
//	rot_M_R0delta_roundWRX(2, 0) = WRZ / 2 + (WRX * WRY) / (4 * pow(1 - pow(WRY, 2) / 4 - pow(WRZ, 2) / 4 - pow(WRX, 2) / 4, 1 / 2));
//	rot_M_R0delta_roundWRX(2, 1) = pow(1 - pow(WRY, 2) / 4 - pow(WRZ, 2) / 4 - pow(WRX, 2) / 4, 1 / 2) - pow(WRX, 2) / (4 * pow(1 - pow(WRY, 2) / 4 - pow(WRZ, 2) / 4 - pow(WRX, 2) / 4, 1 / 2));
//	rot_M_R0delta_roundWRX(2, 2) = -WRX;
//
//	// round M_R0roundWRY_delta
//	rot_M_R0delta_roundWRY(0, 0) = -WRY;
//	rot_M_R0delta_roundWRY(0, 1) = WRX / 2 + (WRY * WRZ) / (4 * pow(1 - pow(WRY, 2) / 4 - pow(WRZ, 2) / 4 - pow(WRX, 2) / 4, 1 / 2));
//	rot_M_R0delta_roundWRY(0, 2) = pow(1 - pow(WRY, 2) / 4 - pow(WRZ, 2) / 4 - pow(WRX, 2) / 4, 1 / 2) - pow(WRY, 2) / (4 * pow(1 - pow(WRY, 2) / 4 - pow(WRZ, 2) / 4 - pow(WRX, 2) / 4, 1 / 2));
//
//
//	rot_M_R0delta_roundWRY(1, 0) = WRX / 2 - (WRY * WRZ) / (4 * pow(1 - pow(WRY, 2) / 4 - pow(WRZ, 2) / 4 - pow(WRX, 2) / 4, 1 / 2));
//	rot_M_R0delta_roundWRY(1, 1) = 0;
//	rot_M_R0delta_roundWRY(1, 2) = WRZ / 2 + (WRX * WRY) / (4 * pow(1 - pow(WRY, 2) / 4 - pow(WRZ, 2) / 4 - pow(WRX, 2) / 4, 1 / 2));
//
//
//	rot_M_R0delta_roundWRY(2, 0) = pow(WRY, 2) / (4 * pow(1 - pow(WRY, 2) / 4 - pow(WRZ, 2) / 4 - pow(WRX, 2) / 4, 1 / 2)) - pow(1 - pow(WRY, 2) / 4 - pow(WRZ, 2) / 4 - pow(WRX, 2) / 4, 1 / 2);
//	rot_M_R0delta_roundWRY(2, 1) = WRZ / 2 - (WRX * WRY) / (4 * pow(1 - pow(WRY, 2) / 4 - pow(WRZ, 2) / 4 - pow(WRX, 2) / 4, 1 / 2));
//	rot_M_R0delta_roundWRY(2, 2) = -WRY;
//
//	// round M_R0roundWRZ_delta
//	rot_M_R0delta_roundWRZ(0, 0) = -WRZ;
//	rot_M_R0delta_roundWRZ(0, 1) = pow(WRZ, 2) / (4 * pow(1 - pow(WRY, 2) / 4 - pow(WRZ, 2) / 4 - pow(WRX, 2) / 4, 1 / 2)) - pow(1 - pow(WRY, 2) / 4 - pow(WRZ, 2) / 4 - pow(WRX, 2) / 4, 1 / 2);
//	rot_M_R0delta_roundWRZ(0, 2) = WRX / 2 - (WRY * WRZ) / (4 * pow(1 - pow(WRY, 2) / 4 - pow(WRZ, 2) / 4 - pow(WRX, 2) / 4, 1 / 2));
//
//
//	rot_M_R0delta_roundWRZ(1, 0) = pow(1 - pow(WRY, 2) / 4 - pow(WRZ, 2) / 4 - pow(WRX, 2) / 4, 1 / 2) - pow(WRZ, 2) / (4 * pow(1 - pow(WRY, 2) / 4 - pow(WRZ, 2) / 4 - pow(WRX, 2) / 4, 1 / 2));
//	rot_M_R0delta_roundWRZ(1, 1) = -WRZ;
//	rot_M_R0delta_roundWRZ(1, 2) = WRY / 2 + (WRX * WRZ) / (4 * pow(1 - pow(WRY, 2) / 4 - pow(WRZ, 2) / 4 - pow(WRX, 2) / 4, 1 / 2));
//
//
//	rot_M_R0delta_roundWRZ(2, 0) = WRX / 2 + (WRY * WRZ) / (4 * pow(1 - pow(WRY, 2) / 4 - pow(WRZ, 2) / 4 - pow(WRX, 2) / 4, 1 / 2));
//	rot_M_R0delta_roundWRZ(2, 1) = WRY / 2 - (WRX * WRZ) / (4 * pow(1 - pow(WRY, 2) / 4 - pow(WRZ, 2) / 4 - pow(WRX, 2) / 4, 1 / 2));
//	rot_M_R0delta_roundWRZ(2, 2) = 0;
//
//
//
//	// round R0_R1roundWR_delta
//	rot_R0_R1delta_roundWR(0, 0) = -WR;
//	rot_R0_R1delta_roundWR(0, 1) = pow(WR, 2) / (4 * pow(1 - pow(WR, 2) / 4, 1 / 2)) - pow(1 - pow(WR, 2) / 4, 1 / 2);
//	rot_R0_R1delta_roundWR(0, 2) = 0;
//
//
//	rot_R0_R1delta_roundWR(1, 0) = pow(1 - pow(WR, 2) / 4, 1 / 2) - pow(WR, 2) / (4 * pow(1 - pow(WR, 2) / 4, 1 / 2));
//	rot_R0_R1delta_roundWR(1, 1) = -WR;
//	rot_R0_R1delta_roundWR(1, 2) = 0;
//
//
//	rot_R0_R1delta_roundWR(2, 0) = 0;
//	rot_R0_R1delta_roundWR(2, 1) = 0;
//	rot_R0_R1delta_roundWR(2, 2) = 0;
//
//
//
//	// round M_L0roundWLX_delta
//	rot_M_L0delta_roundWLX(0, 0) = 0;
//	rot_M_L0delta_roundWLX(0, 1) = WLY / 2 + (WLX * WLZ) / (4 * pow(1 - pow(WLY, 2) / 4 - pow(WLZ, 2) / 4 - pow(WLX, 2) / 4, 1 / 2));
//	rot_M_L0delta_roundWLX(0, 2) = WLZ / 2 - (WLX * WLY) / (4 * pow(1 - pow(WLY, 2) / 4 - pow(WLZ, 2) / 4 - pow(WLX, 2) / 4, 1 / 2));
//
//
//	rot_M_L0delta_roundWLX(1, 0) = WLY / 2 - (WLX * WLZ) / (4 * pow(1 - pow(WLY, 2) / 4 - pow(WLZ, 2) / 4 - pow(WLX, 2) / 4, 1 / 2));
//	rot_M_L0delta_roundWLX(1, 1) = -WLX;
//	rot_M_L0delta_roundWLX(1, 2) = pow(WLX, 2) / (4 * pow(1 - pow(WLY, 2) / 4 - pow(WLZ, 2) / 4 - pow(WLX, 2) / 4, 1 / 2)) - pow(1 - pow(WLY, 2) / 4 - pow(WLZ, 2) / 4 - pow(WLX, 2) / 4, 1 / 2);
//
//
//	rot_M_L0delta_roundWLX(2, 0) = WLZ / 2 + (WLX * WLY) / (4 * pow(1 - pow(WLY, 2) / 4 - pow(WLZ, 2) / 4 - pow(WLX, 2) / 4, 1 / 2));
//	rot_M_L0delta_roundWLX(2, 1) = pow(1 - pow(WLY, 2) / 4 - pow(WLZ, 2) / 4 - pow(WLX, 2) / 4, 1 / 2) - pow(WLX, 2) / (4 * pow(1 - pow(WLY, 2) / 4 - pow(WLZ, 2) / 4 - pow(WLX, 2) / 4, 1 / 2));
//	rot_M_L0delta_roundWLX(2, 2) = -WLX;
//
//
//	// round M_L0roundWLY_delta
//	rot_M_L0delta_roundWLY(0, 0) = -WLY;
//	rot_M_L0delta_roundWLY(0, 1) = WLX / 2 + (WLY * WLZ) / (4 * pow(1 - pow(WLY, 2) / 4 - pow(WLZ, 2) / 4 - pow(WLX, 2) / 4, 1 / 2));
//	rot_M_L0delta_roundWLY(0, 2) = pow(1 - pow(WLY, 2) / 4 - pow(WLZ, 2) / 4 - pow(WLX, 2) / 4, 1 / 2) - pow(WLY, 2) / (4 * pow(1 - pow(WLY, 2) / 4 - pow(WLZ, 2) / 4 - pow(WLX, 2) / 4, 1 / 2));
//
//
//	rot_M_L0delta_roundWLY(1, 0) = WLX / 2 - (WLY * WLZ) / (4 * pow(1 - pow(WLY, 2) / 4 - pow(WLZ, 2) / 4 - pow(WLX, 2) / 4, 1 / 2));
//	rot_M_L0delta_roundWLY(1, 1) = 0;
//	rot_M_L0delta_roundWLY(1, 2) = WLZ / 2 + (WLX * WLY) / (4 * pow(1 - pow(WLY, 2) / 4 - pow(WLZ, 2) / 4 - pow(WLX, 2) / 4, 1 / 2));
//
//
//	rot_M_L0delta_roundWLY(2, 0) = pow(WLY, 2) / (4 * pow(1 - pow(WLY, 2) / 4 - pow(WLZ, 2) / 4 - pow(WLX, 2) / 4, 1 / 2)) - pow(1 - pow(WLY, 2) / 4 - pow(WLZ, 2) / 4 - pow(WLX, 2) / 4, 1 / 2);
//	rot_M_L0delta_roundWLY(2, 1) = WLZ / 2 - (WLX * WLY) / (4 * pow(1 - pow(WLY, 2) / 4 - pow(WLZ, 2) / 4 - pow(WLX, 2) / 4, 1 / 2));
//	rot_M_L0delta_roundWLY(2, 2) = -WLY;
//
//
//	// round M_L0roundWLZ_delta
//	rot_M_L0delta_roundWLZ(0, 0) = -WLZ;
//	rot_M_L0delta_roundWLZ(0, 1) = pow(WLZ, 2) / (4 * pow(1 - pow(WLY, 2) / 4 - pow(WLZ, 2) / 4 - pow(WLX, 2) / 4, 1 / 2)) - pow(1 - pow(WLY, 2) / 4 - pow(WLZ, 2) / 4 - pow(WLX, 2) / 4, 1 / 2);
//	rot_M_L0delta_roundWLZ(0, 2) = WLX / 2 - (WLY * WLZ) / (4 * pow(1 - pow(WLY, 2) / 4 - pow(WLZ, 2) / 4 - pow(WLX, 2) / 4, 1 / 2));
//
//
//	rot_M_L0delta_roundWLZ(1, 0) = pow(1 - pow(WLY, 2) / 4 - pow(WLZ, 2) / 4 - pow(WLX, 2) / 4, 1 / 2) - pow(WLZ, 2) / (4 * pow(1 - pow(WLY, 2) / 4 - pow(WLZ, 2) / 4 - pow(WLX, 2) / 4, 1 / 2));
//	rot_M_L0delta_roundWLZ(1, 1) = -WLZ;
//	rot_M_L0delta_roundWLZ(1, 2) = WLY / 2 + (WLX * WLZ) / (4 * pow(1 - pow(WLY, 2) / 4 - pow(WLZ, 2) / 4 - pow(WLX, 2) / 4, 1 / 2));
//
//
//	rot_M_L0delta_roundWLZ(2, 0) = WLX / 2 + (WLY * WLZ) / (4 * pow(1 - pow(WLY, 2) / 4 - pow(WLZ, 2) / 4 - pow(WLX, 2) / 4, 1 / 2));
//	rot_M_L0delta_roundWLZ(2, 1) = WLY / 2 - (WLX * WLZ) / (4 * pow(1 - pow(WLY, 2) / 4 - pow(WLZ, 2) / 4 - pow(WLX, 2) / 4, 1 / 2));
//	rot_M_L0delta_roundWLZ(2, 2) = 0;
//
//
//	// round L0_L1roundWL_delta
//	rot_L0_L1delta_roundWL(0, 0) = -WL;
//	rot_L0_L1delta_roundWL(0, 1) = pow(WL, 2) / (4 * pow(1 - pow(WL, 2) / 4, 1 / 2)) - pow(1 - pow(WL, 2) / 4, 1 / 2);
//	rot_L0_L1delta_roundWL(0, 2) = 0;
//
//
//	rot_L0_L1delta_roundWL(1, 0) = pow(1 - pow(WL, 2) / 4, 1 / 2) - pow(WL, 2) / (4 * pow(1 - pow(WL, 2) / 4, 1 / 2));
//	rot_L0_L1delta_roundWL(1, 1) = -WL;
//	rot_L0_L1delta_roundWL(1, 2) = 0;
//
//
//	rot_L0_L1delta_roundWL(2, 0) = 0;
//	rot_L0_L1delta_roundWL(2, 1) = 0;
//	rot_L0_L1delta_roundWL(2, 2) = 0;
//}
//
//// 카메라 좌표를 각 state variable로 편미분한 행렬
//void roundCameraCoordinate2roundXk()
//{
//	RoundCoordRoundXk_Cam.block(3 * MID_HIP, XK_TX, 3, 1) = rot_C_W.block(0, 0, 3, 1);
//	RoundCoordRoundXk_Cam.block(3 * MID_HIP, XK_TY, 3, 1) = rot_C_W.block(0, 1, 3, 1);
//	RoundCoordRoundXk_Cam.block(3 * MID_HIP, XK_TZ, 3, 1) = RoundCoordRoundXk_Cam.block(MID_HIP, XK_B, 3, 1) = RoundCoordRoundXk_Cam.block(MID_HIP, XK_C, 3, 1) = rot_C_W.block(0, 2, 3, 1);
//
//	RoundCoordRoundXk_Cam.block(3 * R_HIP, XK_A, 3, 1) = rot_C_W * rot_W_M_new.block(0, 0, 3, 1);
//	RoundCoordRoundXk_Cam.block(3 * L_HIP, XK_A, 3, 1) = -rot_C_W * rot_W_M_new.block(0, 0, 3, 1);
//	RoundCoordRoundXk_Cam.block(3 * R_HIP, XK_TX, 3, 1) = rot_C_W.block(0, 0, 3, 1);
//	RoundCoordRoundXk_Cam.block(3 * R_HIP, XK_TY, 3, 1) = rot_C_W.block(0, 1, 3, 1);
//	RoundCoordRoundXk_Cam.block(3 * L_HIP, XK_TX, 3, 1) = rot_C_W.block(0, 0, 3, 1);
//	RoundCoordRoundXk_Cam.block(3 * L_HIP, XK_TY, 3, 1) = rot_C_W.block(0, 1, 3, 1);
//	RoundCoordRoundXk_Cam.block(3 * R_HIP, XK_TZ, 3, 1) = RoundCoordRoundXk_Cam.block(R_HIP, XK_B, 3, 1) = RoundCoordRoundXk_Cam.block(R_HIP, XK_C, 3, 1) = rot_C_W.block(0, 2, 3, 1);
//	RoundCoordRoundXk_Cam.block(3 * L_HIP, XK_TZ, 3, 1) = RoundCoordRoundXk_Cam.block(L_HIP, XK_B, 3, 1) = RoundCoordRoundXk_Cam.block(L_HIP, XK_C, 3, 1) = rot_C_W.block(0, 2, 3, 1);
//	RoundCoordRoundXk_Cam.block(3 * R_HIP, XK_WPSI, 3, 1) = rot_C_W * rot_W_Mdelta_roundWpsi * rot_W_M_old * trans_M_R0;
//	RoundCoordRoundXk_Cam.block(3 * L_HIP, XK_WPSI, 3, 1) = rot_C_W * rot_W_Mdelta_roundWpsi * rot_W_M_old * trans_M_L0;
//
//	RoundCoordRoundXk_Cam.block(3 * R_KNEE, XK_WPSI, 3, 1) = rot_C_W * rot_W_Mdelta_roundWpsi * rot_W_M_old * (rot_M_R0_new * trans_R0_R1 + trans_M_R0);
//	RoundCoordRoundXk_Cam.block(3 * L_KNEE, XK_WPSI, 3, 1) = rot_C_W * rot_W_Mdelta_roundWpsi * rot_W_M_old * (rot_M_L0_new * trans_L0_L1 + trans_M_L0);
//	RoundCoordRoundXk_Cam.block(3 * R_KNEE, XK_RX, 3, 1) = rot_C_W * rot_W_M_new * rot_M_R0delta_roundWRX * rot_M_R0_old * trans_R0_R1;
//	RoundCoordRoundXk_Cam.block(3 * R_KNEE, XK_RY, 3, 1) = rot_C_W * rot_W_M_new * rot_M_R0delta_roundWRY * rot_M_R0_old * trans_R0_R1;
//	RoundCoordRoundXk_Cam.block(3 * R_KNEE, XK_RZ, 3, 1) = rot_C_W * rot_W_M_new * rot_M_R0delta_roundWRZ * rot_M_R0_old * trans_R0_R1;
//	RoundCoordRoundXk_Cam.block(3 * L_KNEE, XK_RX, 3, 1) = rot_C_W * rot_W_M_new * rot_M_L0delta_roundWLX * rot_M_L0_old * trans_L0_L1;
//	RoundCoordRoundXk_Cam.block(3 * L_KNEE, XK_RY, 3, 1) = rot_C_W * rot_W_M_new * rot_M_L0delta_roundWLY * rot_M_L0_old * trans_L0_L1;
//	RoundCoordRoundXk_Cam.block(3 * L_KNEE, XK_RZ, 3, 1) = rot_C_W * rot_W_M_new * rot_M_L0delta_roundWLZ * rot_M_L0_old * trans_L0_L1;
//	RoundCoordRoundXk_Cam.block(3 * R_KNEE, XK_A, 3, 1) = rot_C_W * rot_W_M_new.block(0, 0, 3, 1);
//	RoundCoordRoundXk_Cam.block(3 * L_KNEE, XK_A, 3, 1) = -rot_C_W * rot_W_M_new.block(0, 0, 3, 1);
//	RoundCoordRoundXk_Cam.block(3 * R_KNEE, XK_B, 3, 1) = rot_C_W * rot_W_M_new * rot_M_R0_new.block(0, 0, 3, 1) + rot_C_W.block(2, 0, 3, 1);
//	RoundCoordRoundXk_Cam.block(3 * L_KNEE, XK_B, 3, 1) = rot_C_W * rot_W_M_new * rot_M_L0_new.block(0, 0, 3, 1) + rot_C_W.block(2, 0, 3, 1);
//	RoundCoordRoundXk_Cam.block(3 * R_KNEE, XK_C, 3, 1) = RoundCoordRoundXk_Cam.block(L_KNEE, XK_C, 3, 1) = rot_C_W.block(2, 0, 3, 1);
//	RoundCoordRoundXk_Cam.block(3 * R_KNEE, XK_TX, 3, 1) = RoundCoordRoundXk_Cam.block(L_KNEE, XK_TX, 3, 1) = rot_C_W.block(0, 0, 3, 1);
//	RoundCoordRoundXk_Cam.block(3 * R_KNEE, XK_TY, 3, 1) = RoundCoordRoundXk_Cam.block(L_KNEE, XK_TY, 3, 1) = rot_C_W.block(0, 1, 3, 1);
//	RoundCoordRoundXk_Cam.block(3 * R_KNEE, XK_TZ, 3, 1) = RoundCoordRoundXk_Cam.block(L_KNEE, XK_TZ, 3, 1) = rot_C_W.block(0, 2, 3, 1);
//
//	RoundCoordRoundXk_Cam.block(3 * R_ANKLE, XK_WPSI, 3, 1) = rot_C_W * rot_W_Mdelta_roundWpsi * rot_W_M_old * (rot_M_R0_new * (rot_R0_R1_new * trans_R1_R2 + trans_R0_R1) + trans_M_R0);
//	RoundCoordRoundXk_Cam.block(3 * L_ANKLE, XK_WPSI, 3, 1) = rot_C_W * rot_W_Mdelta_roundWpsi * rot_W_M_old * (rot_M_L0_new * (rot_L0_L1_new * trans_L1_L2 + trans_L0_L1) + trans_M_L0);
//	RoundCoordRoundXk_Cam.block(3 * R_ANKLE, XK_RX, 3, 1) = rot_C_W * rot_W_M_new * rot_M_R0delta_roundWRX * rot_M_R0_old * (rot_R0_R1_new * trans_R1_R2 + trans_R0_R1);
//	RoundCoordRoundXk_Cam.block(3 * R_ANKLE, XK_RY, 3, 1) = rot_C_W * rot_W_M_new * rot_M_R0delta_roundWRY * rot_M_R0_old * (rot_R0_R1_new * trans_R1_R2 + trans_R0_R1);
//	RoundCoordRoundXk_Cam.block(3 * R_ANKLE, XK_RZ, 3, 1) = rot_C_W * rot_W_M_new * rot_M_R0delta_roundWRZ * rot_M_R0_old * (rot_R0_R1_new * trans_R1_R2 + trans_R0_R1);
//	RoundCoordRoundXk_Cam.block(3 * L_ANKLE, XK_RX, 3, 1) = rot_C_W * rot_W_M_new * rot_M_L0delta_roundWLX * rot_M_L0_old * (rot_L0_L1_new * trans_L1_L2 + trans_L0_L1);
//	RoundCoordRoundXk_Cam.block(3 * L_ANKLE, XK_RY, 3, 1) = rot_C_W * rot_W_M_new * rot_M_L0delta_roundWLY * rot_M_L0_old * (rot_L0_L1_new * trans_L1_L2 + trans_L0_L1);
//	RoundCoordRoundXk_Cam.block(3 * L_ANKLE, XK_RZ, 3, 1) = rot_C_W * rot_W_M_new * rot_M_L0delta_roundWLZ * rot_M_L0_old * (rot_L0_L1_new * trans_L1_L2 + trans_L0_L1);
//	RoundCoordRoundXk_Cam.block(3 * R_ANKLE, XK_R, 3, 1) = rot_C_W * rot_W_M_new * rot_M_R0_new * rot_R0_R1delta_roundWR * rot_R0_R1_old * trans_R1_R2;
//	RoundCoordRoundXk_Cam.block(3 * L_ANKLE, XK_L, 3, 1) = rot_C_W * rot_W_M_new * rot_M_L0_new * rot_L0_L1delta_roundWL * rot_L0_L1_old * trans_L1_L2;
//	RoundCoordRoundXk_Cam.block(3 * R_ANKLE, XK_A, 3, 1) = rot_C_W * rot_W_M_new.block(0, 0, 3, 1);
//	RoundCoordRoundXk_Cam.block(3 * L_ANKLE, XK_A, 3, 1) = -rot_C_W * rot_W_M_new.block(0, 0, 3, 1);
//	RoundCoordRoundXk_Cam.block(3 * R_ANKLE, XK_B, 3, 1) = rot_C_W * rot_W_M_new * rot_M_R0_new.block(0, 0, 3, 1) + rot_C_W.block(2, 0, 3, 1);
//	RoundCoordRoundXk_Cam.block(3 * L_ANKLE, XK_B, 3, 1) = rot_C_W * rot_W_M_new * rot_M_L0_new.block(0, 0, 3, 1) + rot_C_W.block(2, 0, 3, 1);
//	RoundCoordRoundXk_Cam.block(3 * R_ANKLE, XK_C, 3, 1) = rot_C_W * rot_W_M_new * rot_M_R0_new * rot_R0_R1_new.block(0, 0, 3, 1) + rot_C_W.block(2, 0, 3, 1);
//	RoundCoordRoundXk_Cam.block(3 * L_ANKLE, XK_C, 3, 1) = rot_C_W * rot_W_M_new * rot_M_L0_new * rot_L0_L1_new.block(0, 0, 3, 1) + rot_C_W.block(2, 0, 3, 1);
//	RoundCoordRoundXk_Cam.block(3 * R_KNEE, XK_TX, 3, 1) = RoundCoordRoundXk_Cam.block(L_KNEE, XK_TX, 3, 1) = rot_C_W.block(0, 0, 3, 1);
//	RoundCoordRoundXk_Cam.block(3 * R_KNEE, XK_TY, 3, 1) = RoundCoordRoundXk_Cam.block(L_KNEE, XK_TY, 3, 1) = rot_C_W.block(0, 1, 3, 1);
//	RoundCoordRoundXk_Cam.block(3 * R_KNEE, XK_TZ, 3, 1) = RoundCoordRoundXk_Cam.block(L_KNEE, XK_TZ, 3, 1) = rot_C_W.block(0, 2, 3, 1);
//}
//
//// hx 구하기
//void calculateHx()
//{
//
//}
//
//void initEKFmatrix()
//{
//	// init R matrix
//	for (int i = 0; i < 14; i++) {
//		R_const[i] /= 2;  // 3 * sigma
//		R(i, i) = pow(R_const[i] / R_sigma, 2);
//	}
//
//	// init _Q_ : a(10cm/3), b(10cm/3), c(10cm/3), WLX(5deg/3), WLY(5deg/3), WLZ(5deg/3), WL(5deg/3), WRX(5deg/3), WRY(5deg/3), WRZ(5deg/3), WR(5deg/3), Wpsi(5deg/3), tx(10cm/3), ty(10cm/3), tz(10cm/3)
//	// init P, Q matrix
//	for (int i = 0; i < 15; i++) {
//
//		Q(i, i) = pow(Q_const[i], 2);
//		P(i, i) = P_const[i];
//		//_P_(i, i) = _Q_(i, i);
//
//	}
//}

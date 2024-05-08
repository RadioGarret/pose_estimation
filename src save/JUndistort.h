//#ifndef j_UNDISTORT

//#define j_UNDISTORT
#ifndef OPENPOSE_MYHEADER_JUNDISTORT_HPP
#define OPENPOSE_MYHEADER_JUNDISTORT_HPP
#include <iostream>
#include <vector>

#define _SQR(x)		((x) * (x))

typedef struct coordinate
{
	double i; // i == y == v == row
	double j; // j == x == u == col
	coordinate() {}
	coordinate(double dI, double dJ) : i(dI), j(dJ) {}

}dCoordinate;


class JUnDistort
{
public:
	JUnDistort();
	JUnDistort(double fx, double fy, double cx, double cy, double alpha_c,
		double k1, double k2, double p1, double p2, double k3);

	void DistortPixel(int& px, int& py);
	void SetParams(double fx, double fy, double cx, double cy, double alpha_c,
		double k1, double k2, double p1, double p2, double k3);

	void UndistortJoints(std::vector<coordinate>& vSrc);
	void UndistortPixel(coordinate& _X_pd);
private:
	double _fx, _fy, _cx, _cy, _alpha_c, _k1, _k2, _p1, _p2, _k3;

protected:
	void Normalize(double &x, double& y);
	void Denormalize(double &x, double& y);
	void DistortNormal(double& x, double& y);
};
#endif // !UNDISTORT



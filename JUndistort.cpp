#include <openpose/myheader/JUndistort.h>

JUnDistort::JUnDistort()
{
	_fx = 0;
	_fy = 0;
	_cx = 0;
	_cy = 0;
	_alpha_c = 0;
	_k1 = 0;
	_k2 = 0;
	_p1 = 0;
	_p2 = 0;
	_k3 = 0;
}

void JUnDistort::SetParams(double fx, double fy, double cx, double cy, double alpha_c,
	double k1, double k2, double p1, double p2, double k3)
{
	_fx = fx;
	_fy = fy;
	_cx = cx;
	_cy = cy;
	_alpha_c = alpha_c;
	_k1 = k1;
	_k2 = k2;
	_p1 = p1;
	_p2 = p2;
	_k3 = k3;
}

JUnDistort::JUnDistort(double fx, double fy, double cx, double cy, double alpha_c,
	double k1, double k2, double p1, double p2, double k3)
{
	_fx = fx;
	_fy = fy;
	_cx = cx;
	_cy = cy;
	_alpha_c = alpha_c;
	_k1 = k1;
	_k2 = k2;
	_p1 = p1;
	_p2 = p2;
	_k3 = k3;
}

void JUnDistort::UndistortJoints(std::vector<coordinate>& vSrc)
{
	for (int i = 0, ii = vSrc.size(); i < ii; i++)
	{
		UndistortPixel(vSrc[i]); // apply Undistortion each joints
	}
}

void JUnDistort::DistortPixel(int& px, int& py)
{
	double x = px;
	double y = py;

	Normalize(x, y);
	DistortNormal(x, y);
	Denormalize(x, y);

	px = (int)(x + 0.5);
	py = (int)(y + 0.5);
}
void JUnDistort::UndistortPixel(coordinate& _X_pd)
{
	double xd = _X_pd.j;
	double yd = _X_pd.i;

	Normalize(xd, yd);
	double xu = xd;
	double yu = yd;

	double err_thr = (0.1 / _fx)*(0.1 / _fx) + (0.1 / _fy)*(0.1 / _fy);

	while (1)
	{
		double xud = xu;
		double yud = yu;
		DistortNormal(xud, yud);

		double err_x = xud - xd;
		double err_y = yud - yd;
		double err = err_x * err_x + err_y * err_y;

		xu = xu - err_x;
		yu = yu - err_y;

		if (err < err_thr) break;
	}

	Denormalize(xu, yu);

	/*_X_pd.j = (int)(xu + 0.5);
	_X_pd.i = (int)(yu + 0.5);*/
	_X_pd.j = xu;
	_X_pd.i = yu;

}

void JUnDistort::Normalize(double &x, double& y)
{
	double y_n = (y - _cy) / _fy;
	double x_n = (x - _cx) / _fx - _alpha_c * y_n;
	x = x_n;
	y = y_n;
}
void JUnDistort::Denormalize(double &x, double& y)
{
	double x_p = _fx * (x + _alpha_c * y) + _cx;
	double y_p = _fy * y + _cy;

	x = x_p;
	y = y_p;
}
void JUnDistort::DistortNormal(double& x, double& y)
{
	double r2 = x * x + y * y;
	double radial_d = 1 + _k1 * r2 + _k2 * r2*r2 + _k3 * r2*r2*r2;
	double x_d = radial_d * x + 2 * _p1*x*y + _p2 * (r2 + 2 * x*x);
	double y_d = radial_d * y + _p1 * (r2 + 2 * y*y) + 2 * _p2*x*y;

	x = x_d;
	y = y_d;
}
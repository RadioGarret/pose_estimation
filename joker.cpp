#include <iostream>

//using namespace std;
#define _CUBE(x)		((x) * (x) * (x))
#define _SQR(x)			((x)*(x))
int main()
{
	double x = 161.0;
	double y = 0.0;
	y = 0.0001*x*x*x - 0.0352*x*x + 3.1937*x - 15.7920;
	//double y = -0.0001*_CUBE(x)+ 0.0282*_SQR(x) - 0.5234*x + 6.2672;
	


	/*135	98
		161	126
		166	141*/

	std::cout << y;

	return 0;
}
#include "EKF.h"

int main()
{
	FileIO fio;

	double cameraParams[10] = { 1436.35689, 1435.47675, 951.08945, 549.32842, 0, 0.05382, -0.16741, -0.00558, -0.00092, 0 };
	EKF ekf(cameraParams);

	// fio로 데이터를 txt에 저장하는 예제
	//double arr[6][2] = {
	//					{100,200},
	//					{333,444},
	//					{555,666},
	//					{777,888},
	//					{999,111},
	//					{222,333} };

	//for (int k = 0; k < 40; k++)  // 데이터가 40 loop번 들어간다고 치자.
	//{
	//	for (int ii = 0; ii < 6; ii++)
	//	{
	//		double i = arr[ii][0];
	//		double j = arr[ii][1];
	//		std::string data = fio.StrConcate(i, j, ":");
	//		fio.FileWriter("write_test.txt", data);
	//	}
	//}


	// 저장된 데이터를 가져와서 칼만필터 실행(테스트)하는 부분
	std::ifstream reader;
	reader.open("body_data/1.가만히서있을때.txt");
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
				for (int i = 0; i < 7; i++)
				{
					std::cout << "[" << i << "]번째 좌표 : (x,y) : " << vJoints[i].j << ", " << vJoints[i].i << std::endl;
				}
				ekf.Execute(vJoints); // vjoint 왜곡보정 실행



			}
		}
	}
	reader.close();
	getchar();
	return 0;
}

#include "EKF.h"

int main()
{
	FileIO fio;

	double cameraParams[10] = { 1436.35689, 1435.47675, 951.08945, 549.32842, 0, 0.05382, -0.16741, -0.00558, -0.00092, 0 };
	EKF ekf(cameraParams);

	// fio�� �����͸� txt�� �����ϴ� ����
	//double arr[6][2] = {
	//					{100,200},
	//					{333,444},
	//					{555,666},
	//					{777,888},
	//					{999,111},
	//					{222,333} };

	//for (int k = 0; k < 40; k++)  // �����Ͱ� 40 loop�� ���ٰ� ġ��.
	//{
	//	for (int ii = 0; ii < 6; ii++)
	//	{
	//		double i = arr[ii][0];
	//		double j = arr[ii][1];
	//		std::string data = fio.StrConcate(i, j, ":");
	//		fio.FileWriter("write_test.txt", data);
	//	}
	//}


	// ����� �����͸� �����ͼ� Į������ ����(�׽�Ʈ)�ϴ� �κ�
	std::ifstream reader;
	reader.open("body_data/1.��������������.txt");
	bool finalFlag = false;
	if (reader.is_open())
	{
		while (!reader.eof())
		{
			std::vector<coordinate> vJoints;

			//printf("������� ���� �д´�\n");
			for (int i = 0; i < 7; i++) // joints ���� 7��
			{
				// 6�� �а� vJoint�� �־�
				std::string str;
				std::getline(reader, str);

				if (str.length() == 0) // end of file
				{
					finalFlag = true;
					break;
				}
				else // non-end
				{
					// vJoints �� �ִ� ����
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
					std::cout << "[" << i << "]��° ��ǥ : (x,y) : " << vJoints[i].j << ", " << vJoints[i].i << std::endl;
				}
				ekf.Execute(vJoints); // vjoint �ְ�� ����



			}
		}
	}
	reader.close();
	getchar();
	return 0;
}

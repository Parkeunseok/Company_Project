#define _CRT_SECURE_NO_WARNINGS
#include <iostream>
#include <pcl/registration/icp.h>
#include "MasterFile.h"
#include "RealFile.h"
#include "PointCloudICP.h"


// ���α׷� ������ ���� main��
int main()
{
	std::cout << "Program start" << std::endl;

	// icp process
	PointCloudICP icp; // registraion�� ���� Ŭ���� ��ü ����
	icp.Origin2RealICP(); // ICP������ ���� �Լ� ����

	return 0;
}
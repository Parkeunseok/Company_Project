#define _CRT_SECURE_NO_WARNINGS
#include <iostream>
#include <pcl/registration/icp.h>
#include "MasterFile.h"
#include "RealFile.h"
#include "PointCloudICP.h"


// 프로그램 시작을 위한 main문
int main()
{
	std::cout << "Program start" << std::endl;

	// icp process
	PointCloudICP icp; // registraion을 위한 클래스 객체 생성
	icp.Origin2RealICP(); // ICP동작을 위한 함수 실행

	return 0;
}
#include "MasterFile.h"

MasterFile::MasterFile()
{
	// 생성자
}

MasterFile::~MasterFile()
{
	// 소멸자
}

void MasterFile::loadFileMaster() // Master파일 로드 (제공해주신 PDF파일에 나와있는 데이터 로드 방법을 참고)
{
	// Master bottom파일 로드
	std::string filePath_bottom = "C://my_job//aiv_project//3D_치수_과제_영상//MASTER_BOTTOM.tiff";
	// file read
	idealDepthMapBottom_ = cv::imread(filePath_bottom, cv::IMREAD_UNCHANGED);
	dataFullSizeBottom_ = idealDepthMapBottom_.total();
	auto imgPtr_bottom_ = idealDepthMapBottom_.ptr<cv::Vec3f>(0);

	for (auto i = 0; i < dataFullSizeBottom_; ++i) // bottom파일을 point cloud로 저장
	{
		pcl::PointXYZ pt_;
		const auto& zyx = imgPtr_bottom_[i];
		pt_.x = zyx[2];
		pt_.y = zyx[1];
		pt_.z = zyx[0];

		idealPointBottom_.push_back(pt_);
	}
	//std::cout << idealPointBottom_.size() << std::endl;
	

	// master top파일 로드
	std::string filePath_top_ = "C://my_job//aiv_project//3D_치수_과제_영상//MASTER_TOP.tiff";
	idealDepthMapTop_ = cv::imread(filePath_top_, cv::IMREAD_UNCHANGED);
	dataFullSizeTop_ = idealDepthMapTop_.total();
	auto imgPtr_top_ = idealDepthMapTop_.ptr<cv::Vec3f>(0);

	for (int j = 0; j < dataFullSizeTop_; ++j) // top파일을 point cloud로 저장
	{
		pcl::PointXYZ pt_;
		const auto& zyx = imgPtr_top_[j];
		pt_.x = zyx[2];
		pt_.y = zyx[1];
		pt_.z = zyx[0];

		idealPointTop_.push_back(pt_);
	}

	//std::cout << idealPointTop_.size() << std::endl;

	// master side파일 로드
	std::string filePath_side_ = "C://my_job//aiv_project//3D_치수_과제_영상//MASTER_SIDE.tiff";
	idealDepthMapSide_ = cv::imread(filePath_side_, cv::IMREAD_UNCHANGED);
	dataFullSizeSide_ = idealDepthMapSide_.total();
	auto imgPtr_side_ = idealDepthMapSide_.ptr<cv::Vec3f>(0);

	for (int k = 0; k < dataFullSizeSide_; ++k) // side파일을 point cloud로 저장
	{
		pcl::PointXYZ pt_;
		const auto& zyx = imgPtr_side_[k];
		pt_.x = zyx[2];
		pt_.y = zyx[1];
		pt_.z = zyx[0];

		idealPointSide_.push_back(pt_);
	}

	//std::cout << idealPointSide_.size() << std::endl;
	std::cout << "Master Data file load complete" << std::endl;
}

pcl::PointCloud<pcl::PointXYZ> MasterFile::returnIdealPointBottom()
{
	return idealPointBottom_; // point cloud로 저장한 bottom데이터를 리턴
}

pcl::PointCloud<pcl::PointXYZ> MasterFile::returnIdealPointTop()
{
	return idealPointTop_; // point cloud로 저장한 top데이터를 리턴
}

pcl::PointCloud<pcl::PointXYZ> MasterFile::returnIdealPointSide()
{
	return idealPointSide_; // point cloud로 저장한 side데이터를 리턴
}
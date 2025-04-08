#include "MasterFile.h"

MasterFile::MasterFile()
{
	// ������
}

MasterFile::~MasterFile()
{
	// �Ҹ���
}

void MasterFile::loadFileMaster() // Master���� �ε� (�������ֽ� PDF���Ͽ� �����ִ� ������ �ε� ����� ����)
{
	// Master bottom���� �ε�
	std::string filePath_bottom = "C://my_job//aiv_project//3D_ġ��_����_����//MASTER_BOTTOM.tiff";
	// file read
	idealDepthMapBottom_ = cv::imread(filePath_bottom, cv::IMREAD_UNCHANGED);
	dataFullSizeBottom_ = idealDepthMapBottom_.total();
	auto imgPtr_bottom_ = idealDepthMapBottom_.ptr<cv::Vec3f>(0);

	for (auto i = 0; i < dataFullSizeBottom_; ++i) // bottom������ point cloud�� ����
	{
		pcl::PointXYZ pt_;
		const auto& zyx = imgPtr_bottom_[i];
		pt_.x = zyx[2];
		pt_.y = zyx[1];
		pt_.z = zyx[0];

		idealPointBottom_.push_back(pt_);
	}
	//std::cout << idealPointBottom_.size() << std::endl;
	

	// master top���� �ε�
	std::string filePath_top_ = "C://my_job//aiv_project//3D_ġ��_����_����//MASTER_TOP.tiff";
	idealDepthMapTop_ = cv::imread(filePath_top_, cv::IMREAD_UNCHANGED);
	dataFullSizeTop_ = idealDepthMapTop_.total();
	auto imgPtr_top_ = idealDepthMapTop_.ptr<cv::Vec3f>(0);

	for (int j = 0; j < dataFullSizeTop_; ++j) // top������ point cloud�� ����
	{
		pcl::PointXYZ pt_;
		const auto& zyx = imgPtr_top_[j];
		pt_.x = zyx[2];
		pt_.y = zyx[1];
		pt_.z = zyx[0];

		idealPointTop_.push_back(pt_);
	}

	//std::cout << idealPointTop_.size() << std::endl;

	// master side���� �ε�
	std::string filePath_side_ = "C://my_job//aiv_project//3D_ġ��_����_����//MASTER_SIDE.tiff";
	idealDepthMapSide_ = cv::imread(filePath_side_, cv::IMREAD_UNCHANGED);
	dataFullSizeSide_ = idealDepthMapSide_.total();
	auto imgPtr_side_ = idealDepthMapSide_.ptr<cv::Vec3f>(0);

	for (int k = 0; k < dataFullSizeSide_; ++k) // side������ point cloud�� ����
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
	return idealPointBottom_; // point cloud�� ������ bottom�����͸� ����
}

pcl::PointCloud<pcl::PointXYZ> MasterFile::returnIdealPointTop()
{
	return idealPointTop_; // point cloud�� ������ top�����͸� ����
}

pcl::PointCloud<pcl::PointXYZ> MasterFile::returnIdealPointSide()
{
	return idealPointSide_; // point cloud�� ������ side�����͸� ����
}
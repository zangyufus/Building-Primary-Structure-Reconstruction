#pragma once
#include "DataClass.h"

class COutdata
{
public:
	COutdata();
	~COutdata();


	void outputTxtPts(char* filename, std::vector<POINT_3D> &Pts3d);

	void outputTxtPts(char* filename, std::vector<cv::Point2d> &Pts2d);

	void outputLinesPts(char * filename, std::vector<Line_2D>& Lines, double & scale);

	void outputLinesPts(char* filename, std::vector<Line_3D> & Lines, double & scale);

};


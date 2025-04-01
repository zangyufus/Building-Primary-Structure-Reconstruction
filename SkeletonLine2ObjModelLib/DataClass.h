#pragma once
#include ".\CVector3D.h"

#define DOUBLE_MAX_VALUE 1e15
#define DOUBLE_MIN_VALUE -1e15

struct POINT_3D
{
	double p_x;
	double p_y;
	double p_z;
	double normVector[3];	
	int isUsed;
	POINT_3D()
	{
		p_x = 0.0;
		p_y = 0.0;
		p_z = 0.0;
		memset(normVector, 0.0, sizeof(double) * 3);
		isUsed = 0;
	}
};

struct Line_2D
{
	cv::Point2d BegPt;
	cv::Point2d EndPt;
	Line_2D()
	{
		BegPt.x = 0.0;
		BegPt.y = 0.0;
		EndPt.x = 0.0;
		EndPt.y = 0.0;
	}
};

struct Line_3D
{
	POINT_3D BegPt;
	POINT_3D EndPt;
	Line_3D()
	{
		BegPt.p_x = 0.0;
		BegPt.p_y = 0.0;
		BegPt.p_z = 0.0;
		EndPt.p_x = 0.0;
		EndPt.p_y = 0.0;
		EndPt.p_z = 0.0;
	}
};

class CDataClass
{
public:
	CDataClass();
	~CDataClass();
};


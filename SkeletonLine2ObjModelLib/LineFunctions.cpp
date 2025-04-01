#include "stdafx.h"
#include "LineFunctions.h"

CLineFunctions::CLineFunctions()
{
}
CLineFunctions::~CLineFunctions()
{
}

/**求取x,y平面上两点间的距离
*/
double CLineFunctions::DistanceOfTwoPts(cv::Point2d & FirstPt, cv::Point2d & SecondPt)
{
	return 	sqrt((FirstPt.x - SecondPt.x)*(FirstPt.x - SecondPt.x) + (FirstPt.y - SecondPt.y)* (FirstPt.y - SecondPt.y));
}

/**求点到一条直线的距离
*/
double CLineFunctions::DistanceFromPt2Line(cv::Point2d & OnePT, Line_2D & OneLine)
{
	double FA = OneLine.BegPt.y - OneLine.EndPt.y;
	double FB = OneLine.EndPt.x - OneLine.BegPt.x;
	double FC = OneLine.BegPt.x * OneLine.EndPt.y - OneLine.EndPt.x * OneLine.BegPt.y;
	double dist_ = fabs(FA * OnePT.x + FB * OnePT.y + FC) / (sqrt(FA * FA + FB * FB));
	return dist_;
}

double CLineFunctions::DistanceFromPt2LineEnd(cv::Point2d & OnePt, Line_2D & OneLine)
{
	double pt2Left;
	double pt2Right;
	pt2Left = DistanceOfTwoPts(OnePt, OneLine.BegPt);
	pt2Right = DistanceOfTwoPts(OnePt, OneLine.EndPt);

	double Mindist = pt2Left;
	if (Mindist > pt2Right)
	{
		Mindist = pt2Right;
	}
	return Mindist;
}

double CLineFunctions::LineLength(Line_2D & OneLine)
{
	double dist_;
	dist_ = DistanceOfTwoPts(OneLine.BegPt, OneLine.EndPt);
	return dist_;
}

/**求两直线的交点
*/
cv::Point2d CLineFunctions::GetIntersectionOfTwoLines(Line_2D & OneLine, Line_2D & SecondLine)
{
	cv::Point2d IntersectPt;
	OneLine.BegPt;
	OneLine.EndPt;
	double A1 = OneLine.EndPt.y - OneLine.BegPt.y;
	double B1 = OneLine.BegPt.x - OneLine.EndPt.x;
	double C1 = OneLine.EndPt.x * OneLine.BegPt.y - OneLine.BegPt.x * OneLine.EndPt.y;	
	double A2 = SecondLine.EndPt.y - SecondLine.BegPt.y;
	double B2 = SecondLine.BegPt.x - SecondLine.EndPt.x;
	double C2 = SecondLine.EndPt.x * SecondLine.BegPt.y - SecondLine.BegPt.x * SecondLine.EndPt.y;
	double m = A1*B2 - A2*B1;
	if (m == 0)
	{
		IntersectPt.x = 0.0;
		IntersectPt.y = 0.0;
	}
	else
	{
		IntersectPt.x = (C2*B1 - C1*B2) / m;
		IntersectPt.y = (C1*A2 - C2*A1) / m;
	}
	return IntersectPt;
}

double CLineFunctions::CalMindistOfTwoLines(Line_2D & OneLine, Line_2D & SecondLine)
{
	double FirstLeft2Left;
	double FirstLeft2Right;
	double FirstRight2Left;
	double FirstRight2Right;
	FirstLeft2Left = DistanceOfTwoPts(OneLine.BegPt, SecondLine.BegPt);
	FirstLeft2Right = DistanceOfTwoPts(OneLine.BegPt, SecondLine.EndPt);
	FirstRight2Left = DistanceOfTwoPts(OneLine.EndPt, SecondLine.BegPt);
	FirstRight2Right = DistanceOfTwoPts(OneLine.EndPt, SecondLine.EndPt);
	double Mindist = FirstLeft2Left;
	if (Mindist > FirstLeft2Right)
	{
		Mindist = FirstLeft2Right;
	}
	if (Mindist > FirstRight2Left)
	{
		Mindist = FirstRight2Left;
	}
	if (Mindist > FirstRight2Right)
	{
		Mindist = FirstRight2Right;
	}
	return Mindist;
}

double CLineFunctions::CalAngleOfTwoLines(Line_2D & OneLine, Line_2D & SecondLine)
{
	double line1Vec[2];
	line1Vec[0] = OneLine.EndPt.x - OneLine.BegPt.x;
	line1Vec[1] = OneLine.EndPt.y - OneLine.BegPt.y;
	double length_01 = sqrt(line1Vec[0] * line1Vec[0] + line1Vec[1] * line1Vec[1]);
	double line2Vec[2];
	line2Vec[0] = SecondLine.EndPt.x - SecondLine.BegPt.x;
	line2Vec[1] = SecondLine.EndPt.y - SecondLine.BegPt.y;
	double length_02 = sqrt(line2Vec[0] * line2Vec[0] + line2Vec[1] * line2Vec[1]);
	double Ang = fabs((line1Vec[0] * line2Vec[0] + line1Vec[1] * line2Vec[1]) / (length_01 * length_02));
	double angle;
	angle = acos(Ang) * 180 / CV_PI;
	return angle;
}

bool CLineFunctions::isPolygonContainsPoint(std::vector<cv::Point2d>& pts2d, cv::Point2d & onePt)
{
	std::vector<cv::Point2d> min_maxPt2d;
	min_maxPt2d = cal_min_maxP2d(pts2d);
	if (onePt.x<min_maxPt2d[0].x || onePt.x>min_maxPt2d[1].x
		|| onePt.y<min_maxPt2d[0].y || onePt.y>min_maxPt2d[1].y)
		return false;
	if (isPointInPolygonBoundary(pts2d, onePt))
		return false;
	bool inside = false;
	for (int i = 1; i <= pts2d.size(); i++)
	{
		if ((pts2d[i%pts2d.size()].y <= onePt.y && onePt.y < pts2d[i - 1].y) || (pts2d[i - 1].y <= onePt.y && onePt.y < pts2d[i%pts2d.size()].y))
		{
			float t = (onePt.x - pts2d[i%pts2d.size()].x)*(pts2d[i - 1].y - pts2d[i%pts2d.size()].y) - (pts2d[i - 1].x - pts2d[i%pts2d.size()].x)*(onePt.y - pts2d[i%pts2d.size()].y);
			if (pts2d[i - 1].y < pts2d[i%pts2d.size()].y)
				t = -t;
			if (t < 0)
				inside = !inside;
		}
	}
	return inside;
}

bool CLineFunctions::isPointInPolygonBoundary(std::vector<cv::Point2d>& pts2d, cv::Point2d & onePt)
{
	for (int i = 0; i < pts2d.size(); i++)
	{
		cv::Point2d p1 = pts2d[i];
		cv::Point2d p2 = pts2d[(i + 1) % pts2d.size()];
		if (onePt.y<min(p1.y, p2.y))
			continue;
		if (onePt.y>max(p1.y, p2.y))
			continue;
		if (p1.y == p2.y)
		{
			double minX = min(p1.x, p2.x);
			double maxX = max(p1.x, p2.x);
			if ((onePt.y == p1.y) && (onePt.x >= minX && onePt.x <= maxX))
			{
				return true;
			}
		}
		else
		{
			double x = (onePt.y - p1.y) * (p2.x - p1.x) / (p2.y - p1.y) + p1.x;
			if (x == onePt.x) // 说明onePt在p1p2线段上  
				return true;
		}
		return false;
	}
}

bool CLineFunctions::_ifPt_visible(int & curVertIdx
	, std::vector<int>& judgePolyVert, std::vector<cv::Point2d>& Line_vertex)
{
	for (int si = 0; si < judgePolyVert.size(); si++)
	{
		int curJudgeIdx = judgePolyVert[si];
		double f_A = Line_vertex[curJudgeIdx].y - Line_vertex[curVertIdx].y;
		double f_B = Line_vertex[curVertIdx].x - Line_vertex[curJudgeIdx].x;
		double f_C = Line_vertex[curJudgeIdx].x * Line_vertex[curVertIdx].y - Line_vertex[curVertIdx].x *  Line_vertex[curJudgeIdx].y;
		cv::Point2d jug_pt = Line_vertex[si];
		for (int j = 0; j < Line_vertex.size(); j++)
		{
			if (j == si || (j + 1) == curVertIdx || j == curVertIdx)
				continue;
			double f_A2 = Line_vertex[j].y - Line_vertex[(j + 1) % Line_vertex.size()].y;
			double f_B2 = Line_vertex[(j + 1) % Line_vertex.size()].x - Line_vertex[j].x;
			double f_C2 = Line_vertex[j].x * Line_vertex[(j + 1) % Line_vertex.size()].y
				- Line_vertex[(j + 1) % Line_vertex.size()].x * Line_vertex[j].y;
			double det = f_A*f_B2 - f_A2 * f_B;
			if (det == 0)
				continue;
			cv::Point2d crossPoint;
			crossPoint.x = (f_C2*f_B - f_C*f_B2) / det;
			crossPoint.y = (f_A2*f_C - f_A*f_C2) / det;
			double jug_oneend = f_A* Line_vertex[j].x + f_B * Line_vertex[j].y + f_C;
			double jug_twoend = f_A* Line_vertex[(j + 1) % Line_vertex.size()].x + f_B * Line_vertex[(j + 1) % Line_vertex.size()].y + f_C;
			double cross_Pt2jugPt[2];
			cross_Pt2jugPt[0] = jug_pt.x - crossPoint.x;
			cross_Pt2jugPt[1] = jug_pt.y - crossPoint.y;
			double cross_Pt2staPt[2];
			cross_Pt2staPt[0] = Line_vertex[curVertIdx].x - crossPoint.x;
			cross_Pt2staPt[1] = Line_vertex[curVertIdx].y - crossPoint.y;
			double lie_jugPt2staPt = cross_Pt2jugPt[0] * cross_Pt2staPt[0] + cross_Pt2jugPt[1] * cross_Pt2staPt[1];
			if (jug_oneend * jug_twoend < 0 && lie_jugPt2staPt < 0)
			{
				return false;
			}
		}
	}

	return true;
}

#pragma once
#include "DataClass.h"

class CLineFunctions
{
public:
	CLineFunctions();
	~CLineFunctions();

	/**求取x,y平面上两点间的距离
	*/
	static double DistanceOfTwoPts(cv::Point2d & FirstPt, cv::Point2d & SecondPt);
	/**求点到一条直线的距离
	*/
	static double DistanceFromPt2Line(cv::Point2d & OnePt, Line_2D & OneLine);
	/**求点到直线两端点的最小距离
	*/
	static double DistanceFromPt2LineEnd(cv::Point2d & OnePt, Line_2D & OneLine);
	/**求直线的长度
	*/
	static double LineLength(Line_2D & OneLine);
	/**求两直线的交点
	*/
	static cv::Point2d GetIntersectionOfTwoLines(Line_2D& OneLine, Line_2D& SecondLine);
	/**计算两条线的端点的最短距离(端点)
	*/
	static double CalMindistOfTwoLines(Line_2D& OneLine, Line_2D& SecondLine);
	/**计算两条线的夹角
	*/
	static double CalAngleOfTwoLines(Line_2D& OneLine, Line_2D& SecondLine);

	/**\
	* 返回一个点是否在一个多边形区域内
	* @param pts2d 多边形坐标点列表
	* @param onePt   待判断点
	* @return true 多边形包含这个点,false 多边形未包含这个点。
	*/
	static bool isPolygonContainsPoint(std::vector<cv::Point2d> &pts2d, cv::Point2d & onePt);
	/**
	* 返回一个点是否在一个多边形边界上
	* @param pts2d 多边形坐标点列表
	* @param onePt   待判断点
	* @return true 点在多边形边上,false 点不在多边形边上。
	*/
	static bool isPointInPolygonBoundary(std::vector<cv::Point2d> &pts2d, cv::Point2d & onePt);
	/**
	* return 当前点是否可见
	* @param curVertIdx 顶点
	* @param judgePolyVert 多边形序列
	* @param Line_vertex 边界顶点集合
	*/
	static bool  _ifPt_visible(int &curVertIdx, std::vector<int> &judgePolyVert, std::vector<cv::Point2d> &Line_vertex);
	/**
	* 返回 最小最大点 std::vector<cv::Point2d>，第一个点为最小xy，第二个点为最大xy
	* @param Pts2d 二维点集
	*/
	static std::vector<cv::Point2d> cal_min_maxP2d(std::vector<cv::Point2d> &Pts2d)
	{
		std::vector<cv::Point2d> min_max_pts;
		min_max_pts.resize(2);
		min_max_pts[0].x = DOUBLE_MAX_VALUE;
		min_max_pts[0].y = DOUBLE_MAX_VALUE;
		min_max_pts[1].x = DOUBLE_MIN_VALUE;
		min_max_pts[1].y = DOUBLE_MIN_VALUE;
		for (int i = 0; i<Pts2d.size(); ++i)
		{
			if (Pts2d[i].x < min_max_pts[0].x) { min_max_pts[0].x = Pts2d[i].x; }
			if (Pts2d[i].x > min_max_pts[1].x) { min_max_pts[1].x = Pts2d[i].x; }
			if (Pts2d[i].y < min_max_pts[0].y) { min_max_pts[0].y = Pts2d[i].y; }
			if (Pts2d[i].y > min_max_pts[1].y) { min_max_pts[1].y = Pts2d[i].y; }
		}
		return min_max_pts;
	}
	/**求vector最大最小值
	*/
	template <class T>
	static void findMaxMinIdx(std::vector<T > &all_Values, int MinMaxIdx[2])
	{
		MinMaxIdx[0] = min_element(all_Values.begin(), all_Values.end()) - all_Values.begin();
		MinMaxIdx[1] = max_element(all_Values.begin(), all_Values.end()) - all_Values.begin();
	}
};
#pragma once
#include "DataClass.h"

class CLineFunctions
{
public:
	CLineFunctions();
	~CLineFunctions();

	/**��ȡx,yƽ���������ľ���
	*/
	static double DistanceOfTwoPts(cv::Point2d & FirstPt, cv::Point2d & SecondPt);
	/**��㵽һ��ֱ�ߵľ���
	*/
	static double DistanceFromPt2Line(cv::Point2d & OnePt, Line_2D & OneLine);
	/**��㵽ֱ�����˵����С����
	*/
	static double DistanceFromPt2LineEnd(cv::Point2d & OnePt, Line_2D & OneLine);
	/**��ֱ�ߵĳ���
	*/
	static double LineLength(Line_2D & OneLine);
	/**����ֱ�ߵĽ���
	*/
	static cv::Point2d GetIntersectionOfTwoLines(Line_2D& OneLine, Line_2D& SecondLine);
	/**���������ߵĶ˵����̾���(�˵�)
	*/
	static double CalMindistOfTwoLines(Line_2D& OneLine, Line_2D& SecondLine);
	/**���������ߵļн�
	*/
	static double CalAngleOfTwoLines(Line_2D& OneLine, Line_2D& SecondLine);

	/**\
	* ����һ�����Ƿ���һ�������������
	* @param pts2d �����������б�
	* @param onePt   ���жϵ�
	* @return true ����ΰ��������,false �����δ��������㡣
	*/
	static bool isPolygonContainsPoint(std::vector<cv::Point2d> &pts2d, cv::Point2d & onePt);
	/**
	* ����һ�����Ƿ���һ������α߽���
	* @param pts2d �����������б�
	* @param onePt   ���жϵ�
	* @return true ���ڶ���α���,false �㲻�ڶ���α��ϡ�
	*/
	static bool isPointInPolygonBoundary(std::vector<cv::Point2d> &pts2d, cv::Point2d & onePt);
	/**
	* return ��ǰ���Ƿ�ɼ�
	* @param curVertIdx ����
	* @param judgePolyVert ���������
	* @param Line_vertex �߽綥�㼯��
	*/
	static bool  _ifPt_visible(int &curVertIdx, std::vector<int> &judgePolyVert, std::vector<cv::Point2d> &Line_vertex);
	/**
	* ���� ��С���� std::vector<cv::Point2d>����һ����Ϊ��Сxy���ڶ�����Ϊ���xy
	* @param Pts2d ��ά�㼯
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
	/**��vector�����Сֵ
	*/
	template <class T>
	static void findMaxMinIdx(std::vector<T > &all_Values, int MinMaxIdx[2])
	{
		MinMaxIdx[0] = min_element(all_Values.begin(), all_Values.end()) - all_Values.begin();
		MinMaxIdx[1] = max_element(all_Values.begin(), all_Values.end()) - all_Values.begin();
	}
};
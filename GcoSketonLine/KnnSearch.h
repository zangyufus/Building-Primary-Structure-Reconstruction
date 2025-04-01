#ifndef _KNN_SEARCH_H_
#define _KNN_SEARCH_H_
#pragma once
#include "stdafx.h"
#include "typesdef.h"
#include "Parameters.h"

class CKnnSearch
{
public:
	CKnnSearch();
	~CKnnSearch();

	/**\���վ���Ŀ��������ĸ���
	*/
	void KNN_nums(std::vector<point3D>& ptclouds, int k);

	/**\���վ���Ŀ��������ľ���
	*/
	void KNN_dist(std::vector<point3D>& ptclouds, double &dist);

	/**\Ѱ������Χ��ͬ���ٽ���
	*/
	void KNN_NearLine(std::vector<line>& lines_array, int &k_numsLine);

	/**\���ղ������ģ�����Ŀ��������ĸ���
	*/
	void KNN_searchLine(std::vector<point3D>& ptclouds, point3D& targetPt, int k);

	/**\ʹ��DBSCAN�����㷨����
	*/
	void DBSCANcluster(std::vector<point3D>& pts_array, std::vector<std::vector<int>> & cluster_idx);
	/**\��10��VOXEL_SIZE�ķ�Χ��Ѱ�����MINPTS������
	*/
	void KernelSelectByKNN(std::vector<POINT_STRUCT>& Pts_array, std::vector<int>& kernelPtIdx);
};
#endif //_KNN_SEARCH_H_
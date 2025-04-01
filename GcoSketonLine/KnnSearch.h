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

	/**\按照距离目标点最近点的个数
	*/
	void KNN_nums(std::vector<point3D>& ptclouds, int k);

	/**\按照距离目标点最近点的距离
	*/
	void KNN_dist(std::vector<point3D>& ptclouds, double &dist);

	/**\寻找线周围不同的临近线
	*/
	void KNN_NearLine(std::vector<line>& lines_array, int &k_numsLine);

	/**\按照查找中心，搜索目标点最近点的个数
	*/
	void KNN_searchLine(std::vector<point3D>& ptclouds, point3D& targetPt, int k);

	/**\使用DBSCAN聚类算法聚类
	*/
	void DBSCANcluster(std::vector<point3D>& pts_array, std::vector<std::vector<int>> & cluster_idx);
	/**\于10倍VOXEL_SIZE的范围，寻找最多MINPTS个点数
	*/
	void KernelSelectByKNN(std::vector<POINT_STRUCT>& Pts_array, std::vector<int>& kernelPtIdx);
};
#endif //_KNN_SEARCH_H_
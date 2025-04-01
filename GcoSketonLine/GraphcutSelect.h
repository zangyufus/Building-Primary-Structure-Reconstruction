#pragma once
#include "stdafx.h"
#include "typesdef.h"
#include "BuildGrid.h"
#include "CVector3D.h"
#include "Outdata.h"
#include "Parameters.h"
#include "KnnSearch.h"
#include "../3rd_graphcut/GCoptimization.h" // for Graph cut optimization.

class CGraphcutSelect
{
public:
	CGraphcutSelect();
	~CGraphcutSelect();

	/**\利用graphcut进行主体选取、优化
	*/
	void run(std::vector<point3D>& srcPts, std::vector<line> &srcLines);

	/**\图割法分类+优化
	*/
	void graphcutClassify(std::vector<line>& Lines_array);

	/**\搜索每个格网的邻近格网
	*/
	void NeighborSearch(std::vector<voxel>& grid_array);

	/**\统计线同方向
	*/
	void CalDirecLineRatio(std::vector<line>& Lines_array);

	/**\使用DBSCAN聚类算法初步聚类一下
	*/
	void Divided2class(std::vector<voxel>& grid_array);

	/**\搜索线段附近的其余的线（图割法，线），Nums为2的倍数
	*/
	void searchNeighborLine(std::vector<line>& Lines_array, int Nums);

	//**\图割法分类+优化（直接优化线）
	//*/
	//void graphcutClassifyLine(std::vector<line>& Lines_array);

	//double CalMindistOfTwoLines(line& FirstLine, line& SecondLine);

	/**\查找数组中的最大值
	*/
	template<class V> std::pair<double, int> findMaxVal(std::vector<V>& t1)
	{
		std::pair<double, int> result;
		double maxval = DOUBLE_MIN_VAL;
		int Index = -1;
		for (int i = 0; i < t1.size(); i++)
		{
			if (maxval<t1[i])
			{
				maxval = t1[i];
				Index = i;
			}
		}
		result.first = maxval;
		result.second = Index;
		return result;
	}

public:
	std::vector<point3D> Pts_data;//输入数据
	std::vector<line> Lines_array;//得到的特征线
	std::vector<voxel> Grid_array;//得到的构建的格网
};
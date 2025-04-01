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

	/**\����graphcut��������ѡȡ���Ż�
	*/
	void run(std::vector<point3D>& srcPts, std::vector<line> &srcLines);

	/**\ͼ�����+�Ż�
	*/
	void graphcutClassify(std::vector<line>& Lines_array);

	/**\����ÿ���������ڽ�����
	*/
	void NeighborSearch(std::vector<voxel>& grid_array);

	/**\ͳ����ͬ����
	*/
	void CalDirecLineRatio(std::vector<line>& Lines_array);

	/**\ʹ��DBSCAN�����㷨��������һ��
	*/
	void Divided2class(std::vector<voxel>& grid_array);

	/**\�����߶θ�����������ߣ�ͼ����ߣ���NumsΪ2�ı���
	*/
	void searchNeighborLine(std::vector<line>& Lines_array, int Nums);

	//**\ͼ�����+�Ż���ֱ���Ż��ߣ�
	//*/
	//void graphcutClassifyLine(std::vector<line>& Lines_array);

	//double CalMindistOfTwoLines(line& FirstLine, line& SecondLine);

	/**\���������е����ֵ
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
	std::vector<point3D> Pts_data;//��������
	std::vector<line> Lines_array;//�õ���������
	std::vector<voxel> Grid_array;//�õ��Ĺ����ĸ���
};
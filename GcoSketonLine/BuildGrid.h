#pragma once
#include "stdafx.h"
#include "typesdef.h"
#include "Parameters.h"
#include "Outdata.h"
#include <pcl/filters/voxel_grid.h>

class CBuildGrid
{
public:
	CBuildGrid();
	~CBuildGrid();

	/**\��������
	*/
	void run();

	/**\PCL���е��ƽ�����
	*/
	void Voxeldownsampling(std::vector<point3D>& pointData, std::vector<point3D>& Samplepts);

public:
	std::vector<point3D> Pts_data;//��������
	std::vector<line> Lines_data;//��������
	std::vector<voxel> grid_array;//�������
};


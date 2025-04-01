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

	/**\构建格网
	*/
	void run();

	/**\PCL进行点云降采样
	*/
	void Voxeldownsampling(std::vector<point3D>& pointData, std::vector<point3D>& Samplepts);

public:
	std::vector<point3D> Pts_data;//输入数据
	std::vector<line> Lines_data;//输入数据
	std::vector<voxel> grid_array;//输出数据
};


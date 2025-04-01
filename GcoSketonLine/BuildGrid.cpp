#include "BuildGrid.h"

CBuildGrid::CBuildGrid()
{
}

CBuildGrid::~CBuildGrid()
{
}

/**\构建格网
*/
void CBuildGrid::run()
{
	COutdata Resamplepts;
	std::vector<point3D> Pts_array;
	for (int i = 0; i < Lines_data.size(); i++)
	{
		std::vector<point3D> curLinePts;
		Resamplepts.ResampleLinePts(Lines_data[i], curLinePts);
		for (int j = 0; j < curLinePts.size();j++)
		{
			curLinePts[j].index_Line = i;
		}
		Pts_array.insert(Pts_array.end(), curLinePts.begin(), curLinePts.end());
	}
	//step1 对点云划分格网，确定xyz格网个数
	double minx = DOUBLE_MAX_VAL, maxx = DOUBLE_MIN_VAL;
	double miny = DOUBLE_MAX_VAL, maxy = DOUBLE_MIN_VAL;
	double minz = DOUBLE_MAX_VAL, maxz = DOUBLE_MIN_VAL;
	for (int i = 0; i < Pts_array.size(); i++)
	{
		if (Pts_array[i].p_x<minx)
		{
			minx = Pts_array[i].p_x;
		}
		if (Pts_array[i].p_x>maxx)
		{
			maxx = Pts_array[i].p_x;
		}
		if (Pts_array[i].p_y<miny)
		{
			miny = Pts_array[i].p_y;
		}
		if (Pts_array[i].p_y>maxy)
		{
			maxy = Pts_array[i].p_y;
		}
		if (Pts_array[i].p_z<minz)
		{
			minz = Pts_array[i].p_z;
		}
		if (Pts_array[i].p_z>maxz)
		{
			maxz = Pts_array[i].p_z;
		}
	}
	//获取所有点中的坐标最值
	double region_minx, region_maxx;
	double region_miny, region_maxy;
	double region_minz, region_maxz;
	region_maxx = maxx;	region_minx = minx;
	region_maxy = maxy;	region_miny = miny;
	region_maxz = maxz;	region_minz = minz;
	int cols_x = ceil((maxx - minx) / VOXEL_SIZE);
	int rows_y = ceil((maxy - miny) / VOXEL_SIZE);
	int lays_z = ceil((maxz - minz) / VOXEL_SIZE);
	//step2根据确定的个数，生成grid_array数组
	for (int ix = 0; ix <cols_x; ix++)
	{
		for (int jy = 0; jy <rows_y; jy++)
		{
			for (int kz = 0; kz < lays_z; kz++)
			{
				VOXEL_STRUCT one_grid;
				one_grid.x_i = ix;
				one_grid.y_j = jy;
				one_grid.z_k = kz;
				grid_array.push_back(one_grid);
			}
		}
	}
	//step3将所有点投到格网数组中
	//3.1计算点所对应的格网序号
	for (int i = 0; i < Pts_array.size(); i++)
	{
		point3D cur_pt;
		cur_pt = Pts_array[i];
		int cur_xcol, cur_yrow, cur_zlay;
		cur_xcol = int((cur_pt.p_x - minx) / VOXEL_SIZE);
		cur_yrow = int((cur_pt.p_y - miny) / VOXEL_SIZE);
		cur_zlay = int((cur_pt.p_z - minz) / VOXEL_SIZE);
		//3.2根据格网的序号，找到在数组中的序号
		int curvector_index = -1;
		if (cur_zlay == lays_z)
		{			curvector_index = cur_xcol * rows_y * lays_z + cur_yrow * lays_z + cur_zlay - 1;		}
		else
		{			curvector_index = cur_xcol * rows_y * lays_z + cur_yrow * lays_z + cur_zlay;		}
		if (curvector_index<0)
		{			break;		}
		grid_array[curvector_index].fallinpts_index.push_back(i);//这里的点不是真实存在的
		vector<int>::iterator result = 
			find(grid_array[curvector_index].Idx_line.begin(), grid_array[curvector_index].Idx_line.end(), cur_pt.index_Line); 
		if (grid_array[curvector_index].Idx_line.size()==0|| result == grid_array[curvector_index].Idx_line.end())
		{
			grid_array[curvector_index].Idx_line.push_back(cur_pt.index_Line);
		}
	}
	std::vector<voxel> empty_tempGrid;
	for (int i = 0; i < grid_array.size(); i++)
	{
		if (grid_array[i].fallinpts_index.size() != 0)	////格网有点，计算格网中心点；
		{
			double Cur_X = grid_array[i].x_i * VOXEL_SIZE + 0.5 * VOXEL_SIZE;
			double Cur_Y = grid_array[i].y_j * VOXEL_SIZE + 0.5 * VOXEL_SIZE;
			double Cur_Z = grid_array[i].z_k * VOXEL_SIZE + 0.5 * VOXEL_SIZE;
			grid_array[i].centerPt.p_x = Cur_X + region_minx;
			grid_array[i].centerPt.p_y = Cur_Y + region_miny;
			grid_array[i].centerPt.p_z = Cur_Z + region_minz;
			empty_tempGrid.push_back(grid_array[i]);
		}
	}
	grid_array.clear();
	grid_array = empty_tempGrid;
	for (int i = 0; i < grid_array.size(); i++)
	{
		grid_array[i].fallinpts_index.clear();
	}
	for (int i = 0; i < grid_array.size(); i++)//格网关联线
	{
		for (int j = 0; j<grid_array[i].Idx_line.size(); j++)
		{
			int curLineIdx = grid_array[i].Idx_line[j];
			Lines_data[curLineIdx].curLinegrid.push_back(grid_array[i]);
		}
	}
	std::vector<voxel> tempGrid;//总的格网，有重复的
	for (int i = 0; i < Lines_data.size(); i++)
	{
		for (int j = 0; j < Lines_data[i].curLinegrid.size(); j++)
		{
			Lines_data[i].curLinegrid[j].index_Line = i;
			tempGrid.push_back(Lines_data[i].curLinegrid[j]);
		}
	}
	grid_array.clear();
	grid_array = tempGrid;
	//******格网创建完毕***********
	//针对线的可靠性，计算线上面有多少点支持该线
	std::vector<point3D> Samplepts;
	Voxeldownsampling(Pts_data, Samplepts);//点云降采样
	for (int i = 0; i < Lines_data.size(); i++)
	{
		for (int j = 0; j < Samplepts.size();j++)
		{
			double curDist = distPt2Line(Samplepts[j], Lines_data[i]);//点到线的距离
			if (curDist<VOXEL_SIZE / 2)
			{
				for (int k = 0; k < Lines_data[i].curLinegrid.size(); k++)
				{
					double CalPt2Pt = getPtsDist(Samplepts[j], Lines_data[i].curLinegrid[k].centerPt);//两点之间距离
					if (CalPt2Pt<VOXEL_SIZE / 2)
					{
						Lines_data[i].curLinegrid[k].fallinpts_index.push_back(j);
					}
				}
			}
		}
	}
	for (int i = 0; i < Lines_data.size(); i++)
	{
		int count = 0;
		for (int j = 0; j < Lines_data[i].curLinegrid.size();j++)
		{
			if (Lines_data[i].curLinegrid[j].fallinpts_index.size())
			{
				count++;
			}
		}
		Lines_data[i].NoEmptyGNum = count;
	}

}

void CBuildGrid::Voxeldownsampling(std::vector<point3D>& pointData, std::vector<point3D>& Samplepts)
{
	// 创建点云对象指针
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_voxel_downsample(new pcl::PointCloud<pcl::PointXYZ>);
	// 创建随机点云
	cloud->width = pointData.size();
	cloud->height = 1;
	cloud->points.resize(cloud->width * cloud->height);
	for (size_t i = 0; i < cloud->points.size(); i++)
	{
		cloud->points[i].x = pointData[i].p_x;
		cloud->points[i].y = pointData[i].p_y;
		cloud->points[i].z = pointData[i].p_z;
	}
	//创建VexelGrid体素降采样对象
	pcl::VoxelGrid<pcl::PointXYZ> vg;
	vg.setInputCloud(cloud);
	vg.setLeafSize(VOXEL_SIZE/4, VOXEL_SIZE/4, VOXEL_SIZE / 4);  //设置体素大小为(VOXEL_SIZE/2) m x (VOXEL_SIZE/2)  m x (VOXEL_SIZE/2)  m
	vg.filter(*cloud_voxel_downsample);
	for (int i = 0; i < cloud_voxel_downsample->points.size(); i++)
	{
		point3D curPt;
		curPt.p_x = cloud_voxel_downsample->points[i].x;
		curPt.p_y = cloud_voxel_downsample->points[i].y;
		curPt.p_z = cloud_voxel_downsample->points[i].z;
		Samplepts.push_back(curPt);
	}
}

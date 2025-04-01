#include "KnnSearch.h"


CKnnSearch::CKnnSearch()
{
}

CKnnSearch::~CKnnSearch()
{
}

//���վ���Ŀ��������ĸ���
void CKnnSearch::KNN_nums(std::vector<point3D>& ptclouds, int k)
{
	// �������ƶ���ָ��
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

	// �����������
	cloud->width = ptclouds.size();
	cloud->height = 1;
	cloud->points.resize(cloud->width * cloud->height);
	for (size_t i = 0; i < cloud->points.size(); i++)
	{
		cloud->points[i].x = ptclouds[i].p_x;
		cloud->points[i].y = ptclouds[i].p_y;
		cloud->points[i].z = ptclouds[i].p_z;
	}
	// ����һ�� KdTree ����
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;

	// ��ǰ�洴�������������Ϊ KdTree ����
	kdtree.setInputCloud(cloud);
//#pragma omp parallel for
	for (int i = 0; i < ptclouds.size(); i++)
	{
		// ����һ����������
		pcl::PointXYZ searchPoint;
		searchPoint.x = ptclouds[i].p_x;
		searchPoint.y = ptclouds[i].p_y;
		searchPoint.z = ptclouds[i].p_z;

		/************K ���ڲ���************/
		// ���������������ֱ��Ž��ڵ�����ֵ�����ڵ����ľ�
		std::vector<int> pointIdxNKNSearch(k+1);
		std::vector<float> pointNKNSquaredDistance(k+1);
		if (kdtree.nearestKSearch(searchPoint,k+1,pointIdxNKNSearch,pointNKNSquaredDistance)>0)
		{
			//int num = pointIdxNKNSearch.size();
			for (int j = 1; j < pointIdxNKNSearch.size(); j++)
			{
				int index = pointIdxNKNSearch[j];
				//double distance = pointNKNSquaredDistance[j];
				//��һ�����������pointIdxNKNSearch[0]
				ptclouds[i].IdofKnnPts.push_back(index);
			}
		}
	}
}

//���վ���Ŀ��������ľ���
void CKnnSearch::KNN_dist(std::vector<point3D>& ptclouds, double &dist)
{
	// �������ƶ���ָ��
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

	// �����������
	cloud->width = ptclouds.size();
	cloud->height = 1;
	cloud->points.resize(cloud->width * cloud->height);
	for (size_t i = 0; i < cloud->points.size(); i++)
	{
		cloud->points[i].x = ptclouds[i].p_x;
		cloud->points[i].y = ptclouds[i].p_y;
		cloud->points[i].z = ptclouds[i].p_z;
	}
	// ����һ�� KdTree ����
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;

	// ��ǰ�洴�������������Ϊ KdTree ����
	kdtree.setInputCloud(cloud);
//#pragma omp parallel for
	for (int i = 0; i < ptclouds.size(); i++)
	{
		// ����һ����������
		pcl::PointXYZ searchPoint;
		searchPoint.x = ptclouds[i].p_x;
		searchPoint.y = ptclouds[i].p_y;
		searchPoint.z = ptclouds[i].p_z;

		// �뾶�������ڰ뾶dist���������ڡ�
		vector<int> pointIdxRadiusSearch;  // �洢����������
		vector<float> pointRadiusSquaredDistance;   // �洢���ڶ�Ӧ��ƽ�����롣
		if (kdtree.radiusSearch(searchPoint, dist, pointIdxRadiusSearch, pointRadiusSquaredDistance)>0)
		{
			//int num = pointIdxNKNSearch.size();
			for (int j = 0; j < pointIdxRadiusSearch.size(); j++)
			{
				int index = pointIdxRadiusSearch[j];
				//double distance = pointNKNSquaredDistance[j];
				//��һ�����������pointIdxNKNSearch[0]
				ptclouds[i].IdofKnnPts.push_back(index);
			}
		}
	}
}

/**\Ѱ������Χ��ͬ���ٽ���
*/
void CKnnSearch::KNN_NearLine(std::vector<line>& lines_array, int &k_numsLine)
{
	std::vector<point3D> allEndPts;
	for (int i = 0; i < lines_array.size(); i++)
	{
		point3D onePt;
		onePt.ownIndex = i * 2;
		onePt.index_Line = i;
		onePt = lines_array[i].endPts[0];
		point3D secondPt;
		secondPt.ownIndex = i * 2 + 1;
		secondPt.index_Line = i;
		secondPt = lines_array[i].endPts[1];
		allEndPts.push_back(onePt);
		allEndPts.push_back(secondPt);
	}
	std::vector<line>useforNearline;//������������ߵ�
	useforNearline.resize(lines_array.size());
	int half_k_numsLine = k_numsLine / 2;
	//ptclouds
	// �������ƶ���ָ��
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	// �����������
	cloud->width = allEndPts.size();
	cloud->height = 1;
	cloud->points.resize(cloud->width * cloud->height);
	for (size_t i = 0; i < cloud->points.size(); i++)
	{
		cloud->points[i].x = allEndPts[i].p_x;
		cloud->points[i].y = allEndPts[i].p_y;
		cloud->points[i].z = allEndPts[i].p_z;
	}
	// ����һ�� KdTree ����
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
	// ��ǰ�洴�������������Ϊ KdTree ����
	kdtree.setInputCloud(cloud);
	for (int i = 0; i < allEndPts.size(); i++)
	{
		// ����һ����������
		pcl::PointXYZ searchPoint;
		searchPoint.x = allEndPts[i].p_x;
		searchPoint.y = allEndPts[i].p_y;
		searchPoint.z = allEndPts[i].p_z;
		int numsNeibor = 8 * half_k_numsLine;
		int CurLineIdx = allEndPts[i].index_Line;
		while (1)
		{
			int count = 0;
			/************K ���ڲ���************/
			// ���������������ֱ��Ž��ڵ�����ֵ�����ڵ����ľ�
			std::vector<int> pointIdxNKNSearch(numsNeibor);
			std::vector<float> pointNKNSquaredDistance(numsNeibor);
			if (kdtree.nearestKSearch(searchPoint, numsNeibor, pointIdxNKNSearch, pointNKNSquaredDistance)>0)
			{
				for (int j = 1; j < pointIdxNKNSearch.size(); j++)
				{
					int index = pointIdxNKNSearch[j];
					int nearLineIdx = allEndPts[index].index_Line;
					if (CurLineIdx != nearLineIdx)
					{
						int cal_counts = 0;
						//vector<int>::iterator it_label = find(allEndPts[i].IdofKnnPts.begin(), allEndPts[i].IdofKnnPts.end(), index);	
						for (int kn = 0; kn < useforNearline[CurLineIdx].nearLineIdx.size(); kn++)
						{
							int curidx = useforNearline[CurLineIdx].nearLineIdx[kn];
							if (nearLineIdx == curidx)
							{
								cal_counts++;
							}
						}
						if (cal_counts == 0)
						{
							useforNearline[CurLineIdx].nearLineIdx.push_back(nearLineIdx);
							count++;
						}
						if (count == half_k_numsLine)
						{
							break;
						}
					}
				}
			}
			if (count < half_k_numsLine)
			{				
				numsNeibor = numsNeibor*2;
			}
			else
			{
				break;
			}
		}
	}
	for (int i = 0; i < useforNearline.size(); i++)
	{
		lines_array[i].nearLineIdx = useforNearline[i].nearLineIdx;
	}
}

void CKnnSearch::KNN_searchLine(std::vector<point3D>& ptclouds, point3D& targetPt, int k)
{
	// �������ƶ���ָ��
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	// �����������
	cloud->width = ptclouds.size();
	cloud->height = 1;
	cloud->points.resize(cloud->width * cloud->height);
	for (size_t i = 0; i < cloud->points.size(); i++)
	{
		cloud->points[i].x = ptclouds[i].p_x;
		cloud->points[i].y = ptclouds[i].p_y;
		cloud->points[i].z = ptclouds[i].p_z;
	}
	// ����һ�� KdTree ����
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
	// ��ǰ�洴�������������Ϊ KdTree ����
	kdtree.setInputCloud(cloud);
	// ����һ����������
	pcl::PointXYZ searchPoint;
	searchPoint.x = targetPt.p_x;
	searchPoint.y = targetPt.p_y;
	searchPoint.z = targetPt.p_z;
	/************K ���ڲ���************/
	// ���������������ֱ��Ž��ڵ�����ֵ�����ڵ����ľ�
	std::vector<int> pointIdxNKNSearch(ptclouds.size());
	std::vector<float> pointNKNSquaredDistance(ptclouds.size());
	if (kdtree.nearestKSearch(searchPoint, k, pointIdxNKNSearch, pointNKNSquaredDistance)>0)
	{
		for (int j = 0; j < pointIdxNKNSearch.size(); j++)
		{
			int index = ptclouds[pointIdxNKNSearch[j]].ownIndex;
			targetPt.IdofKnnPts.push_back(index);
		}
	}
}

/**\ʹ��DBSCAN�����㷨����
*/
void CKnnSearch::DBSCANcluster(std::vector<point3D>& pts_array, std::vector<std::vector<int>> &cluster_idx)
{
	const int NOT_CLASSIFIED = -1;	//δ����
	std::vector<int> CurKernelPtIdx;
	KernelSelectByKNN(pts_array, CurKernelPtIdx);
	int k = -1;
	for (int i = 0; i < CurKernelPtIdx.size(); i++)//��С��MINPTS�ĵ��index
	{
		if (pts_array[CurKernelPtIdx[i]].judgeOrNot != NOT_CLASSIFIED)
		{
			continue;
		}
		std::vector<int> seed_queue;
		seed_queue.push_back(CurKernelPtIdx[i]);
		pts_array[CurKernelPtIdx[i]].judgeOrNot = ++k;
		while (!seed_queue.empty())
		{
			int index1 = seed_queue.back();	//�������һ�����Ķ�����Χ����
			seed_queue.pop_back();
			if (pts_array[index1].IdofKnnPts.size()>MINPTS)
			{
				for (int n = 0; n < pts_array[index1].IdofKnnPts.size(); n++)
				{
					int index_of_onept;
					index_of_onept = pts_array[index1].IdofKnnPts[n];
					if (k == (pts_array[index_of_onept].judgeOrNot))
					{
						continue;
					}
					seed_queue.push_back(index_of_onept);
					pts_array[index_of_onept].judgeOrNot = k;
				}
			}
		}
	}
	cluster_idx.resize(k + 1);//*****
	for (int i = 0; i < pts_array.size(); i++)
	{
		if (pts_array[i].judgeOrNot == -1)
		{
			continue;
		}
		cluster_idx[pts_array[i].judgeOrNot].push_back(i);
	}
}

/**\��10��VOXEL_SIZE�ķ�Χ��Ѱ�����MINPTS������
*/
void CKnnSearch::KernelSelectByKNN(std::vector<POINT_STRUCT>& Pts_array, std::vector<int>& kernelPtIdx)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	cloud->points.resize(Pts_array.size());
	for (int i = 0; i < Pts_array.size(); i++)
	{
		cloud->points[i].x = Pts_array[i].p_x;
		cloud->points[i].y = Pts_array[i].p_y;
		cloud->points[i].z = Pts_array[i].p_z;
	}
	//����KD��
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
	// ��ǰ�洴�������������Ϊ KdTree ����
	kdtree.setInputCloud(cloud);
	for (int i = 0; i < Pts_array.size(); i++)
	{
		/************K ���ڲ���************/
		// ���������������ֱ��Ž��ڵ�����ֵ�����ڵ����ľ�
		std::vector<int> pointIdxNKNSearch(Pts_array.size());
		std::vector<float> pointNKNSquaredDistance(Pts_array.size());
		kdtree.radiusSearch(cloud->points[i], 10 * VOXEL_SIZE, pointIdxNKNSearch, pointNKNSquaredDistance);
		for (int j = 1; j < pointIdxNKNSearch.size(); j++)
		{
			int index0 = pointIdxNKNSearch[j];
			Pts_array[i].IdofKnnPts.push_back(index0);
		}
		if (pointIdxNKNSearch.size() >= MINPTS)
		{
			kernelPtIdx.push_back(i);
		}
	}
}
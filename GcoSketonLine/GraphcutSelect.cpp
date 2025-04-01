#include "GraphcutSelect.h"

CGraphcutSelect::CGraphcutSelect()
{
}

CGraphcutSelect::~CGraphcutSelect()
{
}


void CGraphcutSelect::run(std::vector<point3D>& srcPts, std::vector<line> &srcLines)
{
	this->Pts_data = Pts_data;
	//step1 datacost parameters 数据项的参数

	//step2 smoothcost parameters 平滑项的参数
	std::cout << "     2.2: Building Grid ..." << endl;
	CBuildGrid grid_build;
	grid_build.Lines_data = this ->Lines_array;
	grid_build.Pts_data = this ->Pts_data;
	grid_build.run();
	this->Grid_array = grid_build.grid_array;

	//step3 利用格网+graghcut优化直线
	std::cout << "2.3 line label..." << endl;
	graphcutClassify(grid_build.Lines_data);

}

void CGraphcutSelect::graphcutClassify(std::vector<line>& Lines_array)
{
	int numSite = Grid_array.size();
	int numLines = Lines_array.size();
	GCoptimizationGeneralGraph *gc = new GCoptimizationGeneralGraph(numSite, LABEL_NUM);
	//-----1. set up the array for data costs-----  关于图割公式中数据项Edata（b)的设置
	float *data = new float[numSite*LABEL_NUM];
	std::vector<double> allLength;
	for (int i = 0; i < Lines_array.size(); i++)
	{
		allLength.push_back(Lines_array[i].length);
	}
	std::pair<double, int>maxLength = findMaxVal(allLength);
	searchNeighborLine(Lines_array, LINENUMs_NEARBY);//查找每条线周围的邻居线
	for (int gridId = 0; gridId < Grid_array.size(); gridId++)
	{
		double DCOST_alpha = 1.0;
		double DCOST_beta = 0.0;
		double DCOST_gamma = 0.0;
		int lineId = Grid_array[gridId].index_Line;
		//*****1. 1 First term for data costs***** 
		DCOST_alpha =1 - double(Lines_array[lineId].NoEmptyGNum)/ double(Lines_array[lineId].curLinegrid.size());
		DCOST_gamma = (1 - DCOST_alpha) / 3.0;
		DCOST_beta = 2 * DCOST_gamma;
		double data_first = DCOST_alpha *double(Lines_array[lineId].NoEmptyGNum)/ double(Lines_array[lineId].curLinegrid.size());
		//*****1. 2 Second term for data costs*****  
		CalDirecLineRatio(Lines_array);//每条线计算出方向相近的线的数量
		double data_second = DCOST_beta * (double(Lines_array[lineId].NearDirecLNums)/ double(numLines)) 
			* exp(1 / (fabs(Lines_array[lineId].length - maxLength.first) + 1));
		//*****1. 3 Third term for data costs*****  
		double data_third = 0.0;
		//计算邻域线距离
		for (int Ln = 0; Ln < Lines_array[lineId].nearLineIdx.size(); Ln++)
		{
			int idofCurNL = Lines_array[lineId].nearLineIdx[Ln];
			double _dist = minDistofTwoLines(Lines_array[lineId], Lines_array[idofCurNL]);
			data_third += exp(-_dist  * _dist);	
		}
		data_third = DCOST_gamma * data_third;
		double _score = data_first + data_second + data_third - 1;
		printf("_score=:%.8lf \n", _score);
		if (_score > 0)
		{
			data[gridId*LABEL_NUM] = 1000;
			data[gridId*LABEL_NUM + 1] = 0;
		}
		else
		{
			data[gridId*LABEL_NUM] = 0;
			data[gridId*LABEL_NUM + 1] = 1000;
		}
	}
	//-----2. set neighbors

	//查找邻域
	NeighborSearch(Grid_array);
	for (int si = 0; si < Grid_array.size(); si++)
	{
		std::vector<int> & neigb = Grid_array[si].NeighborIndex;
		std::vector<int> ifdifferLine;
		ifdifferLine.push_back(Grid_array[si].index_Line);
		for (int n_i = 0; n_i < neigb.size(); n_i++)
		{
			int & tempId = neigb[n_i];
			if (Grid_array[si].index_Line != Grid_array[tempId].index_Line)
			{
				int count = 0;
				for (int j = 0; j < ifdifferLine.size(); j++)
				{
					if (ifdifferLine[j] == Grid_array[tempId].index_Line)
					{
						count++;
					}
				}
				if (count == 0)
				{
					ifdifferLine.push_back(Grid_array[tempId].index_Line);
				}
			}
		}
		for (int n_i = 0; n_i < neigb.size(); n_i++)
		{
			int & tempId = neigb[n_i];
			if (tempId<si)
				continue;
			else
			{
				double NeiborWeight;
				if (ifdifferLine.size()>3)
				{
					double dist0 = getPtsDist(Grid_array[si].centerPt, Grid_array[tempId].centerPt);
					NeiborWeight = 1/((dist0)+1);
					cout << "     NeiborWeight ： " << NeiborWeight << endl;
				}
				else
				{
					NeiborWeight = 1;
				}
				gc->setNeighbors(si, tempId);
			}
		}
		for (int n_i = 0; n_i<neigb.size(); n_i++)
		{
			int & tempId = neigb[n_i];
			if (tempId<si)
				continue;
			else
				gc->setNeighbors(si, tempId);
		}
	}

	//-----3. set up the array for smooth costs-----   关于平滑项Esmooth（b)的设置
	float *smooth = new float[LABEL_NUM*LABEL_NUM];
	for (int l1 = 0; l1 < LABEL_NUM; l1++)
	{
		for (int l2 = 0; l2 < LABEL_NUM; l2++)
		{
			smooth[l1 + l2*LABEL_NUM] = (l1 - l2)*(l1 - l2) == 0 ? 0 : OPTIM_LAMBDA; //1(p,q)potts model
		}
	}
	//-----4. graph cut -----   进行图割公式推演计算
	try
	{		
		gc->setDataCost(data);
		gc->setSmoothCost(smooth);
		COutdata::outStr("Before optimization energy is " + COutdata::convDouble2Str(gc->compute_energy()) + "\n");
		gc->expansion(2);// run expansion for 2 iterations. For swap use gc->swap(num_iterations);
		COutdata::outStr("After optimization energy is " + COutdata::convDouble2Str(gc->compute_energy()) + "\n");
		for (int i = 0; i < numSite; i++)
		{
			Grid_array[i].label = gc->whatLabel(i);
		}
		delete gc;
	}
	catch (GCException e)
	{
		e.Report();
	}
	delete[] smooth;
	delete[] data;
	//*******test********
	int count = 0;
	std::vector<point3D> remainsPts;
	std::vector<line> remainsLines;
	for (int i = 0; i < Grid_array.size();i++)
	{
		if (Grid_array[i].label==1)
		{
			count++;
			remainsPts.push_back(Grid_array[i].centerPt);
			int indexremainsLine = Grid_array[i].index_Line;
			remainsLines.push_back(Lines_array[indexremainsLine]);
		}
	}
	std::cout <<"      格网总数： " << Grid_array.size() << endl;
	std::cout <<"留下来的格网数： " << count << endl;
	//*******************
	COutdata outptsTest;
	char* remianPts = "remainsGrids.txt";
	outptsTest.PtsOut(remianPts, remainsPts);
	char* remainsLName = "remainsLines.txt";
	outptsTest.LineOutput(remainsLName, remainsLines);
}

/**\搜索每个格网的邻近格网
*/
void CGraphcutSelect::NeighborSearch(std::vector<voxel>& grid_array)
{
	std::vector<point3D> GridPt;
	for (int i = 0; i < grid_array.size(); i++)
	{
		point3D curPt;
		curPt = grid_array[i].centerPt;
		curPt.IdofGrid = i;
		curPt.index_Line = grid_array[i].index_Line;
		GridPt.push_back(curPt);
	}
	CKnnSearch searchKnn;
	//searchKnn.KNN_Grids(GridPt, NUMS_NEIGHBOR);//不同线
	searchKnn.KNN_nums(GridPt, NUMS_NEIGHBOR);//相同线
	//std::vector<point> testoutNeibor;
	for (int i = 0; i < GridPt.size();i++)
	{
		//GridPt[i].p_r = 255;
		//GridPt[i].p_g = 0;
		//GridPt[i].p_b = 0;
		//testoutNeibor.push_back(GridPt[i]);
		int curIdGrid = GridPt[i].IdofGrid;
		for (int j = 0; j < GridPt[i].IdofKnnPts.size(); j++)
		{
			
			int neighborPtID = GridPt[i].IdofKnnPts[j];
			int neighborGId = GridPt[neighborPtID].IdofGrid;
			grid_array[curIdGrid].NeighborIndex.push_back(neighborGId);
			//testoutNeibor.push_back(GridPt[neighborPtID]);
		}
		////*****
		//COutdata outTest;
		//char *putPath = "neighborPts.txt";
		//outTest.PtsOut(putPath, testoutNeibor);
		////*****
	}
}

/**\统计线同方向
*/
void CGraphcutSelect::CalDirecLineRatio(std::vector<line>& Lines_array)
{
	std::vector<std::vector<int>> LineIndex;
	double thCosAngle = cos(TH_ANGLE / 180.0*M_PI);
	std::vector<std::pair<int, int> > idxLabel(Lines_array.size());
	for (int i = 0; i < Lines_array.size(); i++)//进行标记
	{
		idxLabel[i].first = i;
		idxLabel[i].second = 0;
	}
	for (int i = 0; i < Lines_array.size();i++)
	{
		if (idxLabel[i].second == 1)
		{
			continue;
		}
		std::vector<int> curCluster;
		curCluster.push_back(i);
		line curline = Lines_array[i];
		CVector3D curDirec = curline.getLineDirec();
		for (int j = 0; j < Lines_array.size(); j++)
		{
			if (i ==j || idxLabel[j].second == 1)
			{
				continue;
			}
			//求夹角
			CVector3D calLineDir = Lines_array[j].getLineDirec();
			double cos_Angel =fabs((curDirec * calLineDir) / (curDirec.length() * calLineDir.length()));
			if (cos_Angel > thCosAngle)//
			{
				curCluster.push_back(j);
				idxLabel[j].second = 1;
			}
		}
		LineIndex.push_back(curCluster);
		idxLabel[i].second = 1;
	}
	for (int i = 0; i < LineIndex.size(); i++)
	{
		for (int j = 0; j < LineIndex[i].size(); j++)
		{
			int curLineindex = LineIndex[i][j];
			Lines_array[curLineindex].NearDirecLNums = LineIndex[i].size()-1;
		}
	}
}

/**\使用DBSCAN聚类算法初步聚类
*/
void CGraphcutSelect::Divided2class(std::vector<voxel>& grid_array)
{
	std::vector<point3D> centerPts;
	for (int i = 0; i < grid_array.size(); i++)
	{
		centerPts.push_back(grid_array[i].centerPt);
	}
	CKnnSearch cluster;
	std::vector<std::vector<int>> ClusterIndex;
	cluster.DBSCANcluster(centerPts, ClusterIndex);
	std::vector<int>clusterSize;
	for (int i = 0; i < ClusterIndex.size(); i++)
	{
		int numsSize = ClusterIndex[i].size();
		clusterSize.push_back(numsSize);
	}
	std::pair<double, int> maxsize = findMaxVal(clusterSize);
	int indexMax = maxsize.second;
	for (int i = 0; i < ClusterIndex[indexMax].size();i++)
	{
		int curIndex = ClusterIndex[indexMax][i];
		grid_array[curIndex].label = 1;
	}
	for (int i = 0; i < grid_array.size(); i++)
	{
		if (grid_array[i].label ==1 )
		{
			continue;
		}
		grid_array[i].label = 0;
	}
}

/**\搜索线段附近的其余的线, Nums为2的倍数
*/
void CGraphcutSelect::searchNeighborLine(std::vector<line>& Lines_array, int Nums)
{
	CKnnSearch neighborLine;	neighborLine.KNN_NearLine(Lines_array, Nums);
	////********outTest*********
	//COutdata outtest;
	//char* nearLine = "nearline1_38.txt";
	//std::vector<line> test;
	//test.push_back(Lines_array[38]);
	//for (int i = 0; i < Lines_array[38].nearLineIdx.size();i++)
	//{
	//	int curIndex = Lines_array[38].nearLineIdx[i];
	//	test.push_back(Lines_array[curIndex]);
	//}
	//outtest.LineOutput(nearLine, test);
	////****************************
}

/**\图割法分类+优化（直接优化线）
*/

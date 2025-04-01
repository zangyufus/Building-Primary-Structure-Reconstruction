#include "stdafx.h"
#include "CgetFacadeObj.h"


CgetFacadeObj::CgetFacadeObj()
{
}
CgetFacadeObj::~CgetFacadeObj()
{
}

void CgetFacadeObj::run(const string & path_name, std::vector<Line_3D> &srcLines3d)
{
	if (srcLines3d.size() == 0)
	{
		SaveEmptyObj(path_name);
		return;
	}
	//step1.  线生成三角面片
	edgeLine2mesh(srcLines3d);
	//step2. 读取顶点与面片

	//step3.  SaveToObj
	SaveToObj(path_name);
}

void CgetFacadeObj::edgeLine2mesh(std::vector<Line_3D> srcLines3d)
{
	//1、储存顶点
	std::vector<POINT_3D> allVertices;
	for (int i = 0; i < srcLines3d.size(); i++)
	{
		allVertices.push_back(srcLines3d[i].BegPt);
	}
	pushVecVertice(allVertices);
	int VertexNums = allVertices.size();
	//2、由边界线分割小面片，并记录每个小面片(顶点序列号)
	int judgeLoopStop = 1;
	//计算初始的生长点序列号
	CVector3D begVertex(allVertices[0].p_x, allVertices[0].p_y, allVertices[0].p_z);
	CVector3D beg1Vertex(allVertices[1].p_x, allVertices[1].p_y, allVertices[1].p_z);
	CVector3D endVertex(allVertices[VertexNums - 1].p_x, allVertices[VertexNums - 1].p_y, allVertices[VertexNums - 1].p_z);
	CVector3D faceNorZ = (endVertex - begVertex) ^ (beg1Vertex - begVertex);
	CVector3D Xaxis = endVertex - begVertex;
	CVector3D Yaxis = Xaxis^faceNorZ;
	Xaxis.normalize();
	Yaxis.normalize();
	std::vector<cv::Point2d> turn2pts2d;
	for (int i = 0; i < VertexNums; ++i)
	{
		CVector3D curPt3d(allVertices[i].p_x, allVertices[i].p_y, allVertices[i].p_z);
		cv::Point2d curPt2d;
		curPt2d.x = curPt3d * Xaxis;
		curPt2d.y = curPt3d * Yaxis;
		turn2pts2d.push_back(curPt2d);
	}
	//输出线和点
	std::vector<Line_2D>Lines2d;
	for (int i = 0; i < turn2pts2d.size(); i++)
	{
		Line_2D curline;
		curline.BegPt = turn2pts2d[i];
		curline.EndPt = turn2pts2d[(i + 1) % turn2pts2d.size() ];
		Lines2d.push_back(curline);
	}
	COutdata outTest;
	char * allpts2d = "allLines2d.txt";
	double sacle = 1;
	outTest.outputLinesPts(allpts2d, Lines2d, sacle);

	for (int i = 0; i < VertexNums; ++i)
	{
		CVector3D curPt = o_Vertex[i].v;
		CVector3D NextCurPt = o_Vertex[(i+1)% VertexNums].v;
		CVector3D BeforCurPt;
		if (i==0)
			BeforCurPt = o_Vertex[VertexNums - 1].v;
		else
			BeforCurPt = o_Vertex[i - 1].v;
		CVector3D cent = (curPt + NextCurPt + BeforCurPt) / 3.0;
		cv::Point2d curCentPt2d;
		curCentPt2d.x = cent * Xaxis;
		curCentPt2d.y = cent * Yaxis;
		bool judgeIn = CLineFunctions::isPolygonContainsPoint(turn2pts2d, curCentPt2d);
		if (judgeIn)
		{
			faceNorZ = (BeforCurPt - curPt) ^ (NextCurPt - curPt);
			break;
		}
	}
	std::vector<int> growPolygon;
	growPolygon.resize(VertexNums);
	iota(growPolygon.begin(), growPolygon.end(), 0);
	int count = 0;
	while (judgeLoopStop)
	{
		//顺时针方向寻找
		std::vector<int> oneHalfMesh;
		std::vector<int> oneHalfGrowIdx;
		for (int i = 0; i < growPolygon.size(); i++)
		{
			CVector3D curNor;
			if (i == 0)
				curNor = (o_Vertex[growPolygon[growPolygon.size() - 1]].v - o_Vertex[growPolygon[i]].v)
				^ (o_Vertex[growPolygon[i + 1 ]].v - o_Vertex[growPolygon[i]].v);
			else
				curNor = (o_Vertex[growPolygon[i - 1]].v - o_Vertex[growPolygon[i]].v )
				^ (o_Vertex[growPolygon[(i + 1)% growPolygon.size()]].v - o_Vertex[growPolygon[i]].v);
			if (curNor * faceNorZ > 0.0)
			{
				oneHalfMesh.push_back(growPolygon[i]);
				oneHalfGrowIdx.push_back(i);
			}
			else
			{
				oneHalfMesh.push_back(growPolygon[i]);
				oneHalfGrowIdx.push_back(i);
				break;
			}
		}
		std::vector<cv::Point2d> realPtPoly;
		for (int i = 0; i < growPolygon.size(); i++)
		{
			cv::Point2d curPt2d;
			curPt2d.x = o_Vertex[growPolygon[i]].v * Xaxis;
			curPt2d.y = o_Vertex[growPolygon[i]].v * Yaxis;
			realPtPoly.push_back(curPt2d);
		}
		if (oneHalfMesh.size()== VertexNums)
		{
			PushOneFace(growPolygon);
			break;
		}

		//逆时针时针方向寻找
		std::vector<int> twoHalfMesh;
		std::vector<int> twoHalfGrowIdx;
		std::vector<Line_2D> remainLines2d;
		for (int i = growPolygon.size() - 1; i >= 0; i--)
		{
			CVector3D curNor_;
			if (i== 0)
			{
				curNor_ = (o_Vertex[growPolygon[growPolygon.size() - 1]].v - o_Vertex[growPolygon[i]].v)
					^ (o_Vertex[growPolygon[i + 1]].v - o_Vertex[growPolygon[i]].v);
			}
			else
				curNor_ = (o_Vertex[growPolygon[i - 1]].v - o_Vertex[growPolygon[i]].v )
				^ (o_Vertex[growPolygon[(i + 1) % growPolygon.size()]].v - o_Vertex[growPolygon[i]].v);
			bool judgeInter = CLineFunctions::_ifPt_visible(i, oneHalfGrowIdx, realPtPoly);
			if (judgeInter&&(curNor_ * faceNorZ)> 0.0)
			{
				if (judgeInter)
				{
					twoHalfMesh.push_back(growPolygon[i]);
					twoHalfGrowIdx.push_back(i);
					if (growPolygon[i] == oneHalfMesh[oneHalfMesh.size() - 1 ])
						twoHalfMesh.push_back(growPolygon[i]);
				}
			}
			else
			{
				if (judgeInter)
				{
					twoHalfMesh.push_back(growPolygon[i]);
					twoHalfGrowIdx.push_back(i);
					if (growPolygon[i] == oneHalfMesh[oneHalfMesh.size() - 1])
						twoHalfMesh.push_back(growPolygon[i]);
				}
				break;
			}
		}
		std::vector<int> curMesh;
		sort(oneHalfMesh.rbegin(), oneHalfMesh.rend());
		curMesh.insert(curMesh.end(), oneHalfMesh.begin(), oneHalfMesh.end());
		curMesh.insert(curMesh.end(), twoHalfMesh.begin(), twoHalfMesh.end());

		if (curMesh.size()== VertexNums)
		{
			PushOneFace(curMesh);
			break;
		}
		if (curMesh.size() < 3)
		{
			break;
		}
		for (int i = 1; i < curMesh.size() - 1; i++)
		{
			allVertices[curMesh[i]].isUsed = 1;
		}
		PushOneFace(curMesh);
		//update polygon
		growPolygon.clear();
		for (int i = 0; i < allVertices.size(); i++)
		{
			if(allVertices[i].isUsed == 0)
				growPolygon.push_back(i);
		}
		//judge loop stop
		int countTimes = 0;
		for (int i = 0; i < VertexNums; ++i)
		{
			if (allVertices[i].isUsed == 0)
			{
				countTimes++;
			}
		}
		if (countTimes == 0)
			break;
		count++;
	}


}

void CgetFacadeObj::pushVecVertice(std::vector<POINT_3D>& pnts)
{
	for (int i = 0; i<pnts.size(); i++)
	{
		CVector3D curPts(pnts[i].p_x, pnts[i].p_y, pnts[i].p_z);
		MeshVertex mv(curPts);
		o_Vertex.push_back(mv);
	}
}

void CgetFacadeObj::PushOneFace(std::vector<int> &face_Idx)
{
	MeshPolygon f;
	f.Vertex_Idx = face_Idx;
	this->o_Triangle.push_back(f);
}

void CgetFacadeObj::SaveToObj(const string & path_name)
{
	string Temp = path_name;
	const char * filename = Temp.c_str();
	FILE * m_pFile;
	fopen_s(&m_pFile, filename, "w");
	int countNumVer = (int)o_Vertex.size();
	int countTriNum = (int)o_Triangle.size();
	fprintf_s(m_pFile, "# %d vertices\n", countNumVer);
	for (int i = 0; i < countNumVer; i++)
	{
		fprintf_s(m_pFile, "v %.3f %.3f %.3f\n", o_Vertex[i].v.x_(), o_Vertex[i].v.y_(), o_Vertex[i].v.z_());
	}
	fprintf_s(m_pFile, "# %d faces\n", countTriNum);
	for (int i = 0; i < countTriNum; i++)
	{
		// In obj formate file, vertex index starts from 1. 
		fprintf_s(m_pFile, "f ");
		for (int j = 0; j < o_Triangle[i].Vertex_Idx.size(); ++j)
		{
			fprintf_s(m_pFile, "%d ", o_Triangle[i].Vertex_Idx[j] + 1);
		}
		fprintf_s(m_pFile, "\n");
	}
	fclose(m_pFile);
}

void CgetFacadeObj::SaveEmptyObj(const string & path_name)
{
	string Temp = path_name;
	const char * filename = Temp.c_str();
	FILE * m_pFile;
	fopen_s(&m_pFile, filename, "w");

	fclose(m_pFile);
}

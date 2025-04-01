#ifndef _TYPESDEF_H_
#define _TYPESDEF_H_

#include "stdafx.h"
#include "Parameters.h"
#include "CVector3D.h"

/**\点结构体，可以实现
   *\初始化
   *\互相赋值
   *\判断点是否相等
   *\利用<<直接输出点
   *\计算向量方向
   *\计算两点距离
   */
typedef struct POINT_STRUCT
{
	double p_x;
	double p_y;
	double p_z;
	int p_r;
	int p_g;
	int p_b;

	int ownIndex;//记录点的索引
	std::vector<int> IdofKnnPts;//记录临近点
	int IdofGrid;//记录所在格网
	int index_Line;//记录线的索引
	int judgeOrNot;//判断是否被判断,聚类用，初始-1

	POINT_STRUCT()
	{
		p_x =0.0;
		p_y = 0.0;
		p_z = 0.0;
		p_r = 255;
		p_g = 255;
		p_b = 255;
		ownIndex = -1;
		IdofGrid = -1;
		index_Line = -1;
		judgeOrNot = -1;
	}

	POINT_STRUCT(double xx, double yy, double zz)
	{
		p_x = xx;
		p_y = yy;
		p_z = zz;
	}

	//POINT_STRUCT (const POINT_STRUCT &pnt)
	//{
	//	p_x = pnt.p_x;
	//	p_x = pnt.p_y;
	//	p_x = pnt.p_z;
	//}

	POINT_STRUCT & operator=(const POINT_STRUCT &pnt)
	{
		p_x = pnt.p_x;
		p_y = pnt.p_y;
		p_z = pnt.p_z;
		p_r = pnt.p_r;
		p_g = pnt.p_g;
		p_b = pnt.p_b;
		ownIndex = pnt.ownIndex;
		IdofGrid = pnt.IdofGrid;
		index_Line = pnt.index_Line;
		IdofKnnPts = pnt.IdofKnnPts;//记录临近点
		return (*this);
	}

	POINT_STRUCT & operator=(const CVector3D &pnt)
	{
		//CVector3D curPt;
		p_x = pnt.pVec[0];
		p_y = pnt.pVec[1];
		p_z = pnt.pVec[2];
		return (*this);
	}

	friend bool operator==(const POINT_STRUCT &pnt1, const POINT_STRUCT &pnt2)
	{
		float diff_x = pnt1.p_x - pnt2.p_x;
		float diff_y = pnt1.p_y - pnt2.p_y;
		float diff_z = pnt1.p_z - pnt2.p_z;

		return (-NEAREST_ZERO < diff_x && diff_x < NEAREST_ZERO)
			&& (-NEAREST_ZERO < diff_y && diff_y < NEAREST_ZERO)
			&& (-NEAREST_ZERO < diff_z && diff_z < NEAREST_ZERO);
	}

	friend std::ostream & operator<<(std::ostream & os, const POINT_STRUCT &pnt)
	{
		os << pnt.p_x << "; " << pnt.p_y << "; " << pnt.p_z << " " << std::endl;
		return os;
	}

	inline double getVeclength()
	{
		return sqrt(p_x * p_x  + p_y * p_y + p_z * p_z);
	}

	friend double getPtsDist(const POINT_STRUCT & pnt1, const POINT_STRUCT & pnt2)
	{
		double diff_x = pnt1.p_x - pnt2.p_x, diff_y = pnt1.p_y- pnt2.p_y, diff_z = pnt1.p_z- pnt2.p_z;
		return sqrt(diff_x * diff_x + diff_y * diff_y + diff_z * diff_z);
	}

}point3D;

/**\格网结构体，可以实现
     *\初始化
     *\中心点
     *\所在直线
     *\落入点的索引
*/
typedef struct VOXEL_STRUCT
{
	int x_i;
	int y_j;
	int z_k;
	int label;
	point3D centerPt;
	int index_Line;
	std::vector<int> Idx_line;

	std::vector<int> fallinpts_index;
	std::vector<int> NeighborIndex;

	VOXEL_STRUCT()
	{
		x_i = -1;
		y_j = -1;
		z_k = -1;
		label = -1;
		index_Line = -1;
	}
}voxel;

/**\线结构体，可实现
   *\线段中点
   *\线段方向
   *\线段归一化方向
   *\线段长度
*/
typedef struct LINE_STRUCT
{
	CVector3D endPts[2];
	double length;

	int label;//0是未选中，1是选中

	int NearDirecLNums;//平行线的数量
	int NoEmptyGNum;//线上面非空格网的数量
	std::vector<int> nearLineIdx;
	std::vector<voxel> curLinegrid;
	LINE_STRUCT()
	{
		length = 0;
		label = -1;
		NearDirecLNums = -1;
		NoEmptyGNum = - 1;
	}

	LINE_STRUCT & operator=(const LINE_STRUCT &pnt)
	{
		endPts[0] = pnt.endPts[0];
		endPts[1] = pnt.endPts[1];
		length = pnt.length;
		return (*this);
	}

	inline CVector3D MidPt()
	{
		CVector3D Onept= (endPts[0] + endPts[1]) / 2;
		return (Onept);
	}

	inline CVector3D getLineDirec()
	{
		CVector3D direc;
		direc = endPts[1] - endPts[0];
		return direc;
	}

	inline CVector3D LineDirecNormalize()
	{
		CVector3D Normaldirec;
		Normaldirec = endPts[1] - endPts[0];
		double dist = Normaldirec.length();
		Normaldirec /= dist;
		return (Normaldirec);
	}

	inline double getLinelength()
	{
		return ((endPts[0] - endPts[1]).length());
	}

	friend double minDistofTwoLines(const LINE_STRUCT & Line1, const LINE_STRUCT & Line2)
	{
		point3D FLineL;
		point3D FLineR;
		point3D SLineL;
		point3D SLineR;
		FLineL = Line1.endPts[0];
		FLineR = Line1.endPts[1];
		SLineL = Line2.endPts[0];
		SLineR = Line2.endPts[1];
		double FirstLeft2Left = getPtsDist(FLineL, SLineL);
		double FirstLeft2Right = getPtsDist(FLineL, SLineR);
		double FirstRight2Left = getPtsDist(FLineR, SLineL);
		double FirstRight2Right = getPtsDist(FLineR, SLineR);
		//距离计算
		double Mindist = FirstLeft2Left;
		if (Mindist > FirstLeft2Right)
		{
			Mindist = FirstLeft2Right;
		}
		if (Mindist > FirstRight2Left)
		{
			Mindist = FirstRight2Left;
		}
		if (Mindist > FirstRight2Right)
		{
			Mindist = FirstRight2Right;
		}
		return Mindist;
	}

	friend double minDistPt2EndLine(const point3D & onePt, const LINE_STRUCT & OneLine)
	{
		point3D LineL;
		point3D LineR;
		LineL = OneLine.endPts[0];
		LineR = OneLine.endPts[1];
		double Pt2Left = getPtsDist(onePt, LineL);
		double Pt2Right = getPtsDist(onePt, LineR);
		double Mindist = Pt2Left;
		if (Mindist > Pt2Right)
			Mindist = Pt2Right;
		return Mindist;
	}

	friend double distPt2Line(const POINT_STRUCT & pt, const LINE_STRUCT & oneline)
	{
		LINE_STRUCT curLine = oneline;
		CVector3D curPt;
		curPt.pVec[0] = pt.p_x;
		curPt.pVec[1] = pt.p_y;
		curPt.pVec[2] = pt.p_z;
		CVector3D linedirec = curLine.getLineDirec();
		CVector3D Pt2OneEnd = curPt - curLine.endPts[0];
		double lengthLine = linedirec.length();
		double lengthPt2End = Pt2OneEnd.length();
		double radian = acos(linedirec * Pt2OneEnd / (linedirec.length() * lengthPt2End));
		double ds = fabs(lengthPt2End*sin(radian));
		return ds;
	}
	//判断点是否在线段两端点之间
	bool Pt(const POINT_STRUCT & pt, const LINE_STRUCT & oneline)
	{
		LINE_STRUCT curLine = oneline;
		CVector3D curPt;
		curPt.pVec[0] = pt.p_x;
		curPt.pVec[1] = pt.p_y;
		curPt.pVec[2] = pt.p_z;
		CVector3D curPt2OneEnd = oneline.endPts[0]  - curPt;
		CVector3D curPt2TwoEnd = oneline.endPts[1] - curPt;
		if (curPt2OneEnd*curPt2TwoEnd<0)
		{
			return true;
		}
		else
		{
			return false;
		}
	}
}line;

typedef struct FACE_STRUCT
{
	std::vector<point3D> vertex;//记录有顺序的顶点，顺时针，逆时针都可以应该
}face;

#endif    //_TYPESDEF_H_

#pragma once
#include "stdafx.h"
#include "DataClass.h"
#include "Outdata.h"
#include "LineFunctions.h"

class CgetFacadeObj
{
public:
	CgetFacadeObj();
	~CgetFacadeObj();

public:
	struct MeshVertex
	{
		CVector3D v;
		MeshVertex(const CVector3D & vv) : v(vv) { }
		MeshVertex() {}
	};
	struct MeshPolygon
	{
		std::vector<int> Vertex_Idx;
		MeshPolygon() {}
	};

public:
	std::vector< MeshVertex >	o_Vertex;
	std::vector< MeshPolygon >	o_Triangle;  // Here, the index starts from 0. But attention. In obj formate file, vertex index starts from 1.
	std::vector<int> o_contour;
	
public:
	void run(const string & path_name, std::vector<Line_3D> &srcLines3d);
	void edgeLine2mesh(std::vector<Line_3D>srcLines3d);
	
	void pushVecVertice(std::vector<POINT_3D> & pnts);
	void PushOneFace(std::vector<int> &face_Idx);
	
	void SaveToObj(const string & path_name);

	void SaveEmptyObj(const string & path_name);
};


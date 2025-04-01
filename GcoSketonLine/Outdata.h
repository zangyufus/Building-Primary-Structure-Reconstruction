#pragma once
#include "typesdef.h"
#include "CVector3D.h"

class COutdata
{
public:
	COutdata();
	~COutdata();

	/**\��txt����ply��ʽ�����
	*/
	void PtsOut(const char* filename, std::vector<POINT_STRUCT>& ptsArray);
	/**\��txt��ʽ�����
	*/
	void TxtOutput(const char * filename, std::vector<POINT_STRUCT>& Pts_array);
	/**\��ply��ʽ�����
	*/
	void PlyOutput(const char * filename, std::vector<POINT_STRUCT>& Pts_array);

	/**\���ֱ��
	*/
	void LineOutput(const char * filename, std::vector<line>& Line_array);

	/**\���ֱ��ʱ������
	*/
	void ResampleLinePts(line& OneLine, std::vector<point3D>& SamplePts);

	/**\double��ֵת��Ϊ�ַ���
	*/
	static std::string convDouble2Str(double val);

	/**\int��ֵת��Ϊ�ַ���
	*/
	static std::string convInt2Str(int val);

	/**\���msg
	*/
	static int outStr(const std::string& msg);

	/**\���obj
	*/
	void OutputObjFormat(const char * filename, FACE_STRUCT &Face_array);

};


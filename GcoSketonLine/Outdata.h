#pragma once
#include "typesdef.h"
#include "CVector3D.h"

class COutdata
{
public:
	COutdata();
	~COutdata();

	/**\以txt或者ply格式输出点
	*/
	void PtsOut(const char* filename, std::vector<POINT_STRUCT>& ptsArray);
	/**\以txt格式输出点
	*/
	void TxtOutput(const char * filename, std::vector<POINT_STRUCT>& Pts_array);
	/**\以ply格式输出点
	*/
	void PlyOutput(const char * filename, std::vector<POINT_STRUCT>& Pts_array);

	/**\输出直线
	*/
	void LineOutput(const char * filename, std::vector<line>& Line_array);

	/**\输出直线时采样点
	*/
	void ResampleLinePts(line& OneLine, std::vector<point3D>& SamplePts);

	/**\double数值转变为字符串
	*/
	static std::string convDouble2Str(double val);

	/**\int数值转变为字符串
	*/
	static std::string convInt2Str(int val);

	/**\输出msg
	*/
	static int outStr(const std::string& msg);

	/**\输出obj
	*/
	void OutputObjFormat(const char * filename, FACE_STRUCT &Face_array);

};


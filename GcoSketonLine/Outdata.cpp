#include "stdafx.h"
#include "Outdata.h"


COutdata::COutdata()
{
}


COutdata::~COutdata()
{
}

/**\输出点
*/
void COutdata::PtsOut(const char * filename, std::vector<POINT_STRUCT>& ptsArray)
{
	string outpath = filename;
	int cutIndex = outpath.find_last_of(".");
	std::string FileName = outpath.substr(0, cutIndex);
	std::string FileExtension = outpath.substr(cutIndex + 1, -1);
	if ((FileExtension == "ply") || (FileExtension == "Ply") || (FileExtension == "PLY"))
	{
		FileName = FileName + ".ply";
		const char *oneformatPath = FileName.c_str() ;
		PlyOutput(oneformatPath, ptsArray);
	}
	else if (FileExtension == "txt" || FileExtension == "Txt" || FileExtension == "TXT")
	{
		FileName = FileName + ".txt";
		const char *oneformatPath = FileName.c_str();
		TxtOutput(oneformatPath, ptsArray);
	}
}

/* *\输出txt格式的点云数据
*/
void COutdata::TxtOutput(const char * filename, std::vector<POINT_STRUCT>& Pts_array)
{
	FILE* fp = NULL;
	int err = fopen_s(&fp, filename, "w");
	if (err != 0)
		return;
	for (int i = 0; i < Pts_array.size(); i++)
	{
		fprintf(fp, "%lf\t%lf\t%lf\t%d\t%d\t%d\n", Pts_array[i].p_x, Pts_array[i].p_y, Pts_array[i].p_z, Pts_array[i].p_r, Pts_array[i].p_g, Pts_array[i].p_b);
	}
	fclose(fp);
}

/* *\输出ply格式的点云数据
*/
void COutdata::PlyOutput(const char * filename, std::vector<POINT_STRUCT>& Pts_array)
{
	FILE* fp = NULL;
	int err = fopen_s(&fp, filename, "w");
	if (err != 0)
		return;

	fprintf(fp, "ply\n");
	fprintf(fp, "format ascii 1.0\n");
	fprintf(fp, "comment VCGLIB generated\n");
	fprintf(fp, "element vertex %ld\n", Pts_array.size());
	fprintf(fp, "property float x\n");
	fprintf(fp, "property float y\n");
	fprintf(fp, "property float z\n");
	fprintf(fp, "property	uchar	red\n");
	fprintf(fp, "property	uchar	green\n");
	fprintf(fp, "property	uchar	blue\n");
	fprintf(fp, "element face %ld\n", 0);
	fprintf(fp, "property list uchar int vertex_indices\n");
	fprintf(fp, "end_header\n");

	for (int i = 0; i < Pts_array.size(); i++)
	{
		POINT_STRUCT& point3D = Pts_array[i];
		fprintf(fp, "%lf\t%lf\t%lf\t", point3D.p_x, point3D.p_y, point3D.p_z);
		fprintf(fp, "%d\t%d\t%d\n", point3D.p_r, point3D.p_g, point3D.p_b);
	}
	fclose(fp);
}

/**\输出直线段
*/
void COutdata::LineOutput(const char * filename, std::vector<line>& Line_array)
{
	std::vector<point3D> allLinePts;
	for (int i = 0; i < Line_array.size();i++)
	{
		std::vector<point3D> CurLinePts;
		ResampleLinePts(Line_array[i], CurLinePts);
		allLinePts.insert(allLinePts.end(), CurLinePts.begin(), CurLinePts.end());
	}
	PtsOut(filename, allLinePts);
}

/**\输出线时，采样的点
*/
void COutdata::ResampleLinePts(line& OneLine, std::vector<point3D>& SamplePts)
{
	//计算单位方向向量
	CVector3D LineDirec;
	LineDirec =OneLine.LineDirecNormalize();
	double lengthbegin2end;
	lengthbegin2end = OneLine.getLinelength();
	//根据起点和末点和方向向量，计算出各个采样点
	int i = 0;
	int color1 = rand() % 255;
	int color2 = rand() % 255;
	int color3 = rand() % 255;
	while (1)
	{
		double k_multiple = 0;
		double dist = i*LINE_SAMPLE_INTERVAL;
		k_multiple = sqrt(dist * dist / LineDirec.length2());
		POINT_STRUCT onesamplept;
		onesamplept.p_x = OneLine.endPts[0].x_() + k_multiple*LineDirec.pVec[0];
		onesamplept.p_y = OneLine.endPts[0].y_() + k_multiple*LineDirec.pVec[1];
		onesamplept.p_z = OneLine.endPts[0].z_() + k_multiple*LineDirec.pVec[2];
		onesamplept.p_r = color1;
		onesamplept.p_g = color2;
		onesamplept.p_b = color3;
		POINT_STRUCT testPt;
		testPt.p_x = OneLine.endPts[0].x_();
		testPt.p_y = OneLine.endPts[0].y_();
		testPt.p_z = OneLine.endPts[0].z_();
		double disttest;
		disttest = getPtsDist(onesamplept, testPt);
		SamplePts.push_back(onesamplept);
		if (i*LINE_SAMPLE_INTERVAL > lengthbegin2end || i*LINE_SAMPLE_INTERVAL == lengthbegin2end)
		{
			POINT_STRUCT LastPt;
			LastPt.p_x = OneLine.endPts[1].x_();
			LastPt.p_y = OneLine.endPts[1].y_();
			LastPt.p_z = OneLine.endPts[1].z_();
			LastPt.p_r = color1;
			LastPt.p_g = color2;
			LastPt.p_b = color3;
			SamplePts.push_back(LastPt);
			break;
		}
		i++;
	}
}

std::string COutdata::convDouble2Str(double val)
{
	std::ostringstream sso;
	sso << val;
	std::string str = sso.str();
	sso.clear();
	return str;
}

std::string COutdata::convInt2Str(int val)
{
	std::ostringstream sso;
	sso << val;
	std::string str = sso.str();
	sso.clear();
	return str;
}

int COutdata::outStr(const std::string & msg)
{
	std::cout << msg;
	return 0;
}

void COutdata::OutputObjFormat(const char * filename, FACE_STRUCT &Face_array)
{
	FILE* fp = NULL;
	fp = fopen(filename, "w");
	if (fp == NULL)
		return;

	fprintf(fp, "# %d vertices\n", (Face_array.vertex.size()));
	int count = 0;
	for (int j = 0; j < Face_array.vertex.size(); j++)
	{
		fprintf(fp, "v %lf\t%lf\t%lf\t\n", Face_array.vertex[j].p_x, Face_array.vertex[j].p_y, Face_array.vertex[j].p_z);
		count++;
	}
	fprintf(fp, "# %d faces\n", Face_array.vertex.size() - 2);
	for (int cn = 0; cn <Face_array.vertex.size() - 2; cn++)
	{
		fprintf(fp, "f %d %d %d\n", count, (count - cn - 1), (count - cn - 2));
	}
	fclose(fp);
}


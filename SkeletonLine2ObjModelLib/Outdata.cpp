#include "stdafx.h"
#include "Outdata.h"


COutdata::COutdata()
{
}
COutdata::~COutdata()
{
}


void COutdata::outputTxtPts(char* filename, std::vector<POINT_3D> &Pts3d)
{
	FILE* fp = NULL;
	int err = fopen_s(&fp, filename, "w");
	if (err != 0)
	{
		printf("open file %s fail!\n", filename);
		return;
	}

	for (int i = 0; i < Pts3d.size(); i++)
	{
		POINT_3D& point = Pts3d[i];
		fprintf(fp, "%lf\t%lf\t%lf\n", point.p_x, point.p_y, point.p_z);

	}

	fclose(fp);
}

void COutdata::outputTxtPts(char* filename, std::vector<cv::Point2d> &Pts2d)
{
	FILE* fp = NULL;
	int err = fopen_s(&fp, filename, "w");
	if (err != 0)
	{
		printf("open file %s fail!\n", filename);
		return;
	}

	for (int i = 0; i < Pts2d.size(); i++)
	{
		fprintf(fp, "%lf\t%lf\n", Pts2d[i].x, Pts2d[i].y);
	}

	fclose(fp);
}

void COutdata::outputLinesPts(char * filename, std::vector<Line_2D>& Lines, double & scale)
{
	FILE *fp2 = fopen(filename, "w");
	for (int li = 0; li<Lines.size(); ++li)
	{
		int R = rand() % 255;
		int G = rand() % 255;
		int B = rand() % 255;
		cv::Point2d dev = Lines[li].EndPt - Lines[li].BegPt;
		double L = sqrt(dev.x*dev.x + dev.y*dev.y);
		int k = L / (scale / 10);
		double x = Lines[li].BegPt.x, y = Lines[li].BegPt.y;
		double dx = dev.x / k, dy = dev.y / k;
		double zz = 0.0;
		for (int lj = 0; lj<k; ++lj)
		{
			x += dx;
			y += dy;
			fprintf(fp2, "%.6lf   %.6lf   %.6lf", x, y, zz);
			fprintf(fp2, "%d   %d   %d   %d\n", R, G, B, li);
		}
	}
	fclose(fp2);
}


void COutdata::outputLinesPts(char* filename, std::vector<Line_3D> & Lines, double & scale) 
{
	FILE *fp2 = fopen(filename, "w");
	for (int li = 0; li<Lines.size(); ++li)
	{
		int R = rand() % 255;
		int G = rand() % 255;
		int B = rand() % 255;
		cv::Point3d dev(Lines[li].EndPt.p_x - Lines[li].BegPt.p_x, Lines[li].EndPt.p_y - Lines[li].BegPt.p_y, Lines[li].EndPt.p_z - Lines[li].BegPt.p_z);
		double L = sqrt(dev.x*dev.x + dev.y*dev.y + dev.z*dev.z);
		int k = L / (scale / 10);
		double x = Lines[li].BegPt.p_x, y = Lines[li].BegPt.p_y, z = Lines[li].BegPt.p_z;
		double dx = dev.x / k, dy = dev.y / k, dz = dev.z / k;
		for (int lj = 0; lj<k; ++lj)
		{
			x += dx;
			y += dy;
			z += dz;
			fprintf(fp2, "%.6lf   %.6lf   %.6lf", x, y, z);
			fprintf(fp2, "%d   %d   %d   %d\n", R, G, B, li);
		}
	}
	fclose(fp2);
}

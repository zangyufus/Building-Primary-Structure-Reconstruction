========================================================================
    控制台应用程序：SkeletonLine2ObjModelLib 项目概述
========================================================================

应用程序向导已为您创建了此 SkeletonLine2ObjModelLib 应用程序。

本文件概要介绍组成 SkeletonLine2ObjModelLib 应用程序的每个文件的内容。


SkeletonLine2ObjModelLib.vcxproj
    这是使用应用程序向导生成的 VC++ 项目的主项目文件，其中包含生成该文件的 Visual C++ 的版本信息，以及有关使用应用程序向导选择的平台、配置和项目功能的信息。

SkeletonLine2ObjModelLib.vcxproj.filters
    这是使用“应用程序向导”生成的 VC++ 项目筛选器文件。它包含有关项目文件与筛选器之间的关联信息。在 IDE 中，通过这种关联，在特定节点下以分组形式显示具有相似扩展名的文件。例如，“.cpp”文件与“源文件”筛选器关联。

SkeletonLine2ObjModelLib.cpp
    这是主应用程序源文件。

/////////////////////////////////////////////////////////////////////////////
其他标准文件:

StdAfx.h, StdAfx.cpp
    这些文件用于生成名为 SkeletonLine2ObjModelLib.pch 的预编译头 (PCH) 文件和名为 StdAfx.obj 的预编译类型文件。

/////////////////////////////////////////////////////////////////////////////
其他注释:

应用程序向导使用“TODO:”注释来指示应添加或自定义的源代码部分。

/////////////////////////////////////////////////////////////////////////////
	//1.读入墙面线
	CIndata inputdata;
	char * inputLLinesEnd = "realLinesEnd.txt";
	std::vector<POINT_3D> allLinesEnds;
	std::vector<Line_3D>srcLines3d;
	inputdata.readTxtPtsXYZ(inputLLinesEnd, allLinesEnds);
	inputdata.linesEnd2Lines(allLinesEnds, srcLines3d);

	//2.根据墙面线来生成墙面obj格式模型
	CgetFacadeObj getObj;
	char * facadeObj = "facade.obj";
	getObj.run(facadeObj, srcLines3d);





// pclOrthProjectionLib.cpp : 定义控制台应用程序的入口点。
#include "stdafx.h"
#include "DataClass.h"
#include "UniversTool.h"
#include "Indata.h"
#include "OrthProjection.h"
#include "Outdata.h"
#include "GeneTempImage.h"
#include "OptimizeImage.h"
#include "CalRegionNorm.h"
#include "Get3DLines.h"


int main()
{
	clock_t start, finish;
	start = clock();

	//*******************************************************************
	////1.输入的投影法向量和投影点
	//POINT_3D startPt;
	//startPt.p_x = 32.1092; startPt.p_y = 71.7825;
	//POINT_3D endPt;
	//endPt.p_x = 31.9864;   endPt.p_y = 71.4608;
	//CUniversTool universTool;
	//double normVector[3];
	////universTool.cal3dNormVector(startPt, endPt, normVector);
	//universTool.cal3dNormVector(endPt, startPt, normVector);
	////normVector

	POINT_3D PRJCenter;
	PRJCenter.p_x = 80.8388; PRJCenter.p_y = 69.1514; PRJCenter.p_z = 106.897;

	const string path_name = ".\\dormitory\\regions";
	const string outImgepath = ".\\dormitory\\regionsImg";

	const string ImgLinepath = ".\\dormitory\\regionImgLines";
	const string outRealLinesEnd = ".\\dormitory\\RealLines";

	const string plyExten = ".ply";
	std::vector<string> src_file_names;
	CIndata indata;
	COutdata outdata;

	//批量读取regions点云数据路径
	CIndata::bathReadFileNamesInFolders(path_name, plyExten, src_file_names);	
	std::vector<string> src_Line_names;
	const string lineExten = ".txt";
	CIndata::bathReadFileNamesInFolders(ImgLinepath, lineExten, src_Line_names);
	int countNums = 0;
	while (src_file_names.size()- countNums)
	{
		//处理当前数据
		//2.读入墙面点云
		const string infile = src_file_names[countNums];
		std::vector<POINT_3D> facadePts;
		indata.readPlyPts(infile, facadePts);
		if (facadePts.size() == 0)
		{
			printf("Please check filename!\n");
			return 0;
		}
		printf("read data done...\n");

		//3.计算墙面点云法向量
		double normVector[3];
		CCalRegionNorm calNorm;
		POINT_3D facMeanPts;
		calNorm.regNormCal(facadePts, normVector, facMeanPts);
		printf("normal calculate done...\n");

		//4.计算各点在投影面上的坐标（x0,y0,z0,rang0）(在空间依然是三维)
		COrthProjection orthProjection;
		std::vector<POINT_3D> PRJPts;
		orthProjection.generatePRJPts(normVector, PRJCenter, facadePts, PRJPts);
		printf("orthProjection done...\n");

		////output
		//char* txtfilename = "PRJtest.txt";
		//outdata.outputTxtPts(txtfilename, PRJPts);

		//5.计算各投影点的临时pixel坐标（row,col）(临时pixel坐标以PRJcenter为中心，由三维->二维坐标)  关键：确定pixel的尺寸大小
		CGeneTempImage geneTempImage;
		geneTempImage.run(normVector, PRJCenter, PRJPts);
		printf("geneTempImage done...\n");

		//6.根据墙面点的分布和范围，重新规划新图像的原点和范围->new image（保留各pixel和点云点的映射关系）
		COptimizeImage optimizeImage;
		std::vector<std::vector<PIXEL_2D> > PRJImage;
		std::vector<cv::Point2d> ImgMinMaxPt;
		cv::Mat cv_image;
		optimizeImage.run(PRJPts, PRJImage, ImgMinMaxPt, cv_image);
		printf("optimizeImage done...\n");

		//生成当前影像
		string::size_type iPosPly = src_file_names[countNums].find_last_of('\\') + 1;
		string plyfilename = src_file_names[countNums].substr(iPosPly, src_file_names[countNums].length() - iPosPly);	
		string plyNam = plyfilename.substr(0, plyfilename.rfind("."));
		string output_imagefile = outImgepath + "\\"+plyNam + ".png";
		cv::imwrite(output_imagefile, cv_image);

		//7.根据二维影读取线
		//需要考虑影像，点对应关系
		const string lineInfile = src_Line_names[countNums];
		std::vector<Line_2D> imglines;
		indata.readImgLines(lineInfile, imglines);


		//8.影像中的线完成规则化，输出原始点云中的对应线段的端点
		std::vector<Line_3D> realLines3D;
		CGet3DLines CurRegLines3D;
		CurRegLines3D.getReg3Dline(ImgMinMaxPt, PRJPts, imglines, realLines3D, PRJImage, normVector, PRJCenter, facMeanPts);
		std::vector<POINT_3D> allLines3dEnd;
		for (int i = 0; i < realLines3D.size(); i++)
		{
			allLines3dEnd.push_back(realLines3D[i].BegPt);
		}
		string::size_type iPostxt = src_Line_names[countNums].find_last_of('\\') + 1;
		string txtfilename = src_Line_names[countNums].substr(iPostxt, src_Line_names[countNums].length() - iPostxt);
		string txtNam = txtfilename.substr(0, txtfilename.rfind("."));
		string output_txtfile = outRealLinesEnd + "\\" + txtNam + ".txt";
		double scale0 = 2;
		outdata.outputLinesPts(output_txtfile, realLines3D, scale0);

		//char* alllLinesEnd = "realLinesEnd.txt";
		//outdata.outputTxtPts(alllLinesEnd, allLines3dEnd);

		countNums++;
	}

	finish = clock();
	double duration = (double)(finish - start) / CLOCKS_PER_SEC;
	printf("%f sec\n", duration);

    return 0;
}

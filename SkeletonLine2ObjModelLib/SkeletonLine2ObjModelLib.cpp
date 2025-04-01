// SkeletonLine2ObjModelLib.cpp : 定义控制台应用程序的入口点。
//
#include "stdafx.h"
#include "Indata.h"
#include "CgetFacadeObj.h"

int main()
{
	clock_t start, finish;
	start = clock();

	const string path_name = ".\\Teaching_building\\paperResult";
	const string outIObjpath = ".\\Teaching_building\\obj_result";
	const string txtExten = ".txt";
	std::vector<string> src_file_names;
	CIndata indata;
	COutdata outdata;
	//批量读取多边形点云数据路径
	CIndata::bathReadFileNamesInFolders(path_name, txtExten, src_file_names);
	int countNums = 0;
	while (src_file_names.size() - countNums)
	{
		//1.读入边界顶点
		const string infile = src_file_names[countNums];

		std::vector<POINT_3D> allLinesEnds;
		std::vector<Line_3D>srcLines3d;
		indata.readTxtPtsXYZ(infile.c_str(), allLinesEnds);
		indata.linesEnd2Lines(allLinesEnds, srcLines3d);

		//2.根据顶点边界线来生成obj格式模型
		if (countNums==2)
		{
			int dsadsadsd = 233;
			dsadsadsd++;
		}
		CgetFacadeObj getObj;
		string::size_type iPostxt = src_file_names[countNums].find_last_of('\\') + 1;
		string txtfilename = src_file_names[countNums].substr(iPostxt, src_file_names[countNums].length() - iPostxt);
		string txtNam = txtfilename.substr(0, txtfilename.rfind("."));
		string output_txtfile = outIObjpath + "\\" + txtNam + ".obj";
		getObj.run(output_txtfile, srcLines3d);

		countNums++;
	}

	finish = clock();
	double duration = (double)(finish - start) / CLOCKS_PER_SEC;
	printf("%f sec\n", duration);
    return 0;
}


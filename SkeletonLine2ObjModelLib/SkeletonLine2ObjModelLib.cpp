// SkeletonLine2ObjModelLib.cpp : �������̨Ӧ�ó������ڵ㡣
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
	//������ȡ����ε�������·��
	CIndata::bathReadFileNamesInFolders(path_name, txtExten, src_file_names);
	int countNums = 0;
	while (src_file_names.size() - countNums)
	{
		//1.����߽綥��
		const string infile = src_file_names[countNums];

		std::vector<POINT_3D> allLinesEnds;
		std::vector<Line_3D>srcLines3d;
		indata.readTxtPtsXYZ(infile.c_str(), allLinesEnds);
		indata.linesEnd2Lines(allLinesEnds, srcLines3d);

		//2.���ݶ���߽���������obj��ʽģ��
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


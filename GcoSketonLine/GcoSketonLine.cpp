// GcoSketonLine.cpp : �������̨Ӧ�ó������ڵ㡣
//

#include "stdafx.h"
#include "Indata.h"
#include "typesdef.h"
#include "GraphcutSelect.h"
#include "Outdata.h"

int main()
{
	clock_t start, finish;
	start = clock();
	printf("	Step1:  Read data...\n");
	//Read_pts...
	std::vector<point3D> srcPts;
	char * OriginPts = "dormitory";
	///**\Teaching_building��house��dormitory��HankouRoad
	CIndata::readPts(OriginPts, srcPts);//��ȡ�����������
	//Read_lines...
	std::vector<line> srcLines;
	CIndata::readLines(OriginPts, srcLines);//��ȡ��ȡ���߶�
	printf("	Step2:  calculate parameter and graph cut optimize...");
	CGraphcutSelect LineLabel;
	LineLabel.run(srcPts, srcLines);//ͼ���Ż���ȡ���ṹ������

	printf("	Step3:  out result...\n");
	COutdata;

	finish = clock();
	double duration = (double)(finish - start) / CLOCKS_PER_SEC;
	printf("%f sec\n", duration);
	system("pause");
    return 0;
}


#pragma once

#include "stdafx.h"
#include "typesdef.h"
#include "divideStr.h"

class CIndata
{
public:
	CIndata();
	~CIndata();

	/**\��ȡ��������
	*\ply��ʽ
	*\txt��ʽ
	*/
	static bool readPts(const char* infileName, std::vector<POINT_STRUCT>& ptsArray);
	static bool ply_input(const string &filename, std::vector<POINT_STRUCT>& ptsArray);
	static bool txt_input(const string &filename, std::vector<POINT_STRUCT>& ptsArray);

	/**\��ȡֱ������
	*\txt��ʽ
	*/
	static bool readLines(const char* filename, std::vector<line>& linesArray);

	/*
	* @brief  �õ�һ���ļ��к�׺��Ϊextension�������ļ�
	* param[in]  folderName �ļ�����  extension ��Ҫ��õĺ�׺�� vec_filenames �ļ�����������
	*/
	static bool ReadFolderFiles(const string &folderName, const string &extension, vector<string> &vec_filenames);
};
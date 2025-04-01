#pragma once

#include "stdafx.h"
#include "typesdef.h"
#include "divideStr.h"

class CIndata
{
public:
	CIndata();
	~CIndata();

	/**\读取点云数据
	*\ply格式
	*\txt格式
	*/
	static bool readPts(const char* infileName, std::vector<POINT_STRUCT>& ptsArray);
	static bool ply_input(const string &filename, std::vector<POINT_STRUCT>& ptsArray);
	static bool txt_input(const string &filename, std::vector<POINT_STRUCT>& ptsArray);

	/**\读取直线数据
	*\txt格式
	*/
	static bool readLines(const char* filename, std::vector<line>& linesArray);

	/*
	* @brief  得到一个文件夹后缀名为extension的所有文件
	* param[in]  folderName 文件夹名  extension 需要获得的后缀名 vec_filenames 文件名存入容器
	*/
	static bool ReadFolderFiles(const string &folderName, const string &extension, vector<string> &vec_filenames);
};
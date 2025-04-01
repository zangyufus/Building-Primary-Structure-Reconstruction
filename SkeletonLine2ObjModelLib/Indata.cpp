#include "stdafx.h"
#include "Indata.h"
#include "divideStr.h"

CIndata::CIndata()
{
}
CIndata::~CIndata()
{
}

void CIndata::readTxtPtsXYZRGB(char* infile, std::vector<POINT_3D>& ptsArray)
{
		FILE* fpIn = NULL;
		size_t err = fopen_s(&fpIn, infile, "r");
		if (err != 0)
		{
			printf("open file %s fail!\n", infile);
			return;
		}	
		char buffer[512];
		while (fgets(buffer, 512, fpIn))
		{
			POINT_3D onept;
			int r, g, b;
			sscanf_s(buffer, "%lf%lf%lf%d%d%d", &onept.p_x, &onept.p_y, &onept.p_z, &r, &g, &b);
			ptsArray.push_back(onept);
		}	
		fclose(fpIn);
}

void CIndata::readTxtPtsXYZ(const string & path_name, std::vector<POINT_3D>& ptsArray)
{
	string Temp = path_name;
	const char * infile = Temp.c_str();
	FILE* fpIn = NULL;
	size_t err = fopen_s(&fpIn, infile, "r");
	if (err != 0)
	{
		printf("open file %s fail!\n", infile);
		return;
	}
	char buffer[512];
	while (fgets(buffer, 512, fpIn))
	{
		POINT_3D onept;
		sscanf_s(buffer, "%lf%lf%lf", &onept.p_x, &onept.p_y, &onept.p_z);
		ptsArray.push_back(onept);
	}
	fclose(fpIn);
}

void CIndata::linesEnd2Lines(std::vector<POINT_3D>& ptsArray, std::vector<Line_3D>& srcLines3d)
{
	for (int i = 0; i < ptsArray.size(); ++i)
	{
		Line_3D curLine;
		curLine.BegPt = ptsArray[i];
		curLine.EndPt = ptsArray[(i + 1) % ptsArray.size()];
		srcLines3d.push_back(curLine);
	}
}

/*
* @brief  得到一个文件夹后缀名为extension的所有文件
* param[in]  folderName 文件夹名  extension 需要获得的后缀名 vec_filenames 文件名存入容器
*/
bool CIndata::bathReadFileNamesInFolders(const string folderName, const string extension, vector<string> &vec_filenames)
{
	if (!boost::filesystem::exists(folderName))
	{
		return 0;
	}
	else
	{
		boost::filesystem::directory_iterator end_iter;
		for (boost::filesystem::directory_iterator iter(folderName); iter != end_iter; ++iter)
		{
			if (boost::filesystem::is_regular_file(iter->status()))
			{
				string file_name = iter->path().string();
				boost::filesystem::path dir(file_name);

				if (!dir.extension().string().empty())
				{
					if (!file_name.substr(file_name.rfind('.')).compare(extension))
					{
						vec_filenames.push_back(file_name);
					}
				}
			}

		}
	}
	return 1;
}
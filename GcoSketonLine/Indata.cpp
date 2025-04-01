#include "stdafx.h"
#include "Indata.h"
#include "divideStr.h"

CIndata::CIndata()
{
}

CIndata::~CIndata()
{
}

/**\读取点云数据
   *\ply格式
   *\txt格式
*/
bool CIndata::readPts(const char* infileName, std::vector<POINT_STRUCT>& ptsArray)
{
	//读取ply以及txt 格式点云数据
	string filename = infileName;
	string nametxt = filename + ".txt";
	string nameply = filename + ".ply";
	//int cutIndex = Inputpath.find_last_of(".");
	//std::string FileName = Inputpath.substr(0, cutIndex);
	//std::string FileExtension = Inputpath.substr(cutIndex + 1, -1);
	//std::cout << "folderPath:\t" << FileName << std::endl;
	//std::cout << "extendName:\t" << FileExtension << std::endl;
	if (!ply_input(nameply, ptsArray))
	{
		if (txt_input(nametxt, ptsArray))
			return true;
		else
		{
			printf("no %s txt or ply file  !\n", infileName);
			return false;
		}
	}
	else
		return true;
}
/**\read ply format
*/
bool CIndata::ply_input(const string &filename, std::vector<POINT_STRUCT>& ptsArray)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	if (pcl::io::loadPLYFile<pcl::PointXYZ>(filename, *cloud) == -1)
	{
		PCL_ERROR("Couldnot read file.\n");
		system("pause");
		return(-1);
	}
	for (int i = 0; i < cloud->size(); i++)
	{
		point3D onept;
		onept.p_x = cloud->points[i].x;
		onept.p_y = cloud->points[i].y;
		onept.p_z = cloud->points[i].z;
		ptsArray.push_back(onept);
	}
	return 1;


	return true;
}
/**\read txt format
*/
bool CIndata::txt_input(const string &filename, std::vector<POINT_STRUCT>& ptsArray)
{
	string Temp = filename;
	const char * infile = Temp.c_str();
	FILE* fpIn = NULL;
	size_t err = fopen_s(&fpIn, infile, "r");
	if (err != 0)
	{
		printf("open file %s fail!\n", infile);
		return false;
	}
	int nPts = 0;
	char buffer[512];
	CdivideStr divideStr;
	//空间申请小了 就会内存不足
	char** ptr_array = (char**)malloc(sizeof(char*) * 50);

	char new_buffer[512];
	divideStr.divideStr(buffer, ptr_array);
	while (fgets(buffer, 512, fpIn))
	{
		POINT_STRUCT pt;
		sscanf_s(buffer, "%lf%lf%lf", &pt.p_x, &pt.p_y, &pt.p_z);
		pt.ownIndex = nPts;
		ptsArray.push_back(pt);
		nPts++;
	}
	cout << "      点的总个数：" << nPts << endl;
	fclose(fpIn);
	return true;
}

bool CIndata::readLines(const char * filename, std::vector<line>& linesArray)
{
	string Temp = filename;
	string folderName = Temp + "\\Lines";
	const string txtExten = ".txt";
	std::vector<string> src_lines_names;
	ReadFolderFiles(folderName, txtExten, src_lines_names);
	int count = 0;
	while (src_lines_names.size() - count)
	{
		const char * infile = src_lines_names[count].c_str();
		FILE* fpIn = NULL;
		size_t err = fopen_s(&fpIn, infile, "r");
		if (err != 0)
			continue;
		char buffer[512];
		CdivideStr divideStr;
		//空间申请小了 就会内存不足
		char** ptr_array = (char**)malloc(sizeof(char*) * 50);
		char new_buffer[512];
		divideStr.divideStr(buffer, ptr_array);
		CVector3D Linecolor(0,0,0);
		CVector3D oneEnd(DOUBLE_MAX_VAL, DOUBLE_MAX_VAL, DOUBLE_MAX_VAL);
		CVector3D twoEnd(DOUBLE_MAX_VAL, DOUBLE_MAX_VAL, DOUBLE_MAX_VAL);
		CVector3D growPt;
		int calLines = 0;
		while (fgets(buffer, 512, fpIn))
		{
			CVector3D Curcolor;
			CVector3D CurLinePts;
			sscanf_s(buffer, "%lf%lf%lf%lf%lf%lf", &CurLinePts[0], &CurLinePts[1], &CurLinePts[2], &Curcolor[0], &Curcolor[1], &Curcolor[2]);
			if (!(Linecolor == Curcolor))
			{
				calLines++;
				if ((calLines % 2))
				{
					Linecolor = Curcolor;
					if(!(calLines-1))
					oneEnd = CurLinePts;
				}
				else
				{
					twoEnd = growPt;
					line curLine;
					curLine.endPts[0] = oneEnd;
					curLine.endPts[1] = twoEnd;
					linesArray.push_back(curLine);
					oneEnd = CurLinePts;
				}
			}
			else
				growPt = CurLinePts;
		}
		fclose(fpIn);
		count++;
	}
	printf("	Lines numbers:%d", linesArray.size());
	return true;
}

/*
* @brief  得到一个文件夹后缀名为extension的所有文件
* param[in]  folderName 文件夹名  extension 需要获得的后缀名 vec_filenames 文件名存入容器
*/
bool CIndata::ReadFolderFiles(const string &folderName, const string &extension, vector<string> &vec_filenames)
{
	if (!boost::filesystem::exists(folderName))
	{
		return false;
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
	return true;
}
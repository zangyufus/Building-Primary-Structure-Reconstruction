#pragma once
#include "DataClass.h"

class CIndata
{
public:
	CIndata();
	~CIndata();

	void readTxtPtsXYZRGB(char* infile, std::vector<POINT_3D>& ptsArray);

	void readTxtPtsXYZ(const string & path_name, std::vector<POINT_3D>& ptsArray);

	static void linesEnd2Lines(std::vector<POINT_3D>& ptsArray, std::vector<Line_3D>& srcLines3d);
	
	static bool bathReadFileNamesInFolders(const string folderName, const string extension, vector<string> &vec_filenames);
};


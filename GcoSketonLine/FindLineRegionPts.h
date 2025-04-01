#pragma once
#include "typesdef.h"
#include "stdafx.h"

class CFindLineRegionPts
{
public:
	CFindLineRegionPts();
	~CFindLineRegionPts();

	void run(std::vector<point3D>& srcPts, std::vector<line> &srcLines);
};


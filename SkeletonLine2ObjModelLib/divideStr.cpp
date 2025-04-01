#include "StdAfx.h"
#include "divideStr.h"

CdivideStr::CdivideStr(void)
{
}

CdivideStr::~CdivideStr(void)
{
}
//加了个空格
void CdivideStr::divideStr(char* str, char** ptr_array)
{
	char seps[] = " :;,.`，。；：“\t\n";
	char *token = NULL;

	char *next_token = NULL;
	token = strtok_s(str, seps, &next_token);
	size_t calnum = 1;
	while (token != NULL)
	{
		ptr_array[calnum] = token;

		token = strtok_s(NULL, seps, &next_token);
		calnum++;
	}

   char numString[25]; 
   _itoa_s(calnum, numString, 25, 10); 
   ptr_array[0] = numString;
}

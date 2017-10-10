/*

Copyright(C) 2017 Julian Löhr
All rights reserved.

Licensed under:
MIT License

*/
#include "stdafx.h"

#include "Application.h"

int main()
{
	std::locale::global(std::locale(""));

	Application Application;
	Application.Run();

	return 0;
}


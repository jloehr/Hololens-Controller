/*

Copyright(C) 2017 Julian Löhr
All rights reserved.

Licensed under:
MIT License

*/
#include "stdafx.h"
#include "UpdatingCloud.h"


UpdatingCloud::UpdatingCloud()
	:ClearFlag(false)
	,Cloud(new PointCloud)
{
}

void UpdatingCloud::Load(const std::string & FilePath)
{
	FileCloud.reset(new PointCloud());
	pcl::io::loadPCDFile(FilePath, *FileCloud);
}

void UpdatingCloud::Clear()
{
	UpdateQueue.Clear();
	ClearFlag = true;
}

bool UpdatingCloud::HasUpdates() const
{
	return ((!UpdateQueue.IsEmpty()) || (FileCloud));
}

const PointCloud::ConstPtr UpdatingCloud::GetCurrentCloud()
{
	if (ClearFlag)
	{
		Reset();
		ClearFlag = true;
	}

	if (FileCloud)
	{
		Cloud = FileCloud;
		FileCloud.reset();
	}

	// Process Update Queue;

	return PointCloud::ConstPtr(Cloud);
}

void UpdatingCloud::Reset()
{
	FileCloud.reset();
	Cloud.reset(new PointCloud);
}


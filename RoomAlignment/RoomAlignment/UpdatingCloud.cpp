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

	Update();

	return PointCloud::ConstPtr(Cloud);
}

void UpdatingCloud::Reset()
{
	FileCloud.reset();
	Cloud.reset(new PointCloud);
}

void UpdatingCloud::Update()
{
	if (UpdateQueue.IsEmpty())
		return;

	UpdateMessageQueue::QueueType Updates = UpdateQueue.Dequeue();
	for (auto Update : Updates)
	{
		UpdateGridCells(Update);
	}

	// Generate new concatenated cloud
	Cloud.reset(new PointCloud);
	for (auto GridCell : Grid)
	{
		*Cloud += *GridCell.second;
	}
}

void UpdatingCloud::UpdateGridCells(const std::string & JSONString)
{
	const nlohmann::json AsJSON = nlohmann::json::parse(JSONString.c_str());
	
	for (auto & Mesh : AsJSON["Meshes"])
	{
		UpdateGridCell(Mesh);
	}
}

void UpdatingCloud::UpdateGridCell(const nlohmann::json & Mesh)
{
	size_t NumPoints = std::max((Mesh["Vertices"].size() / 3), (Mesh["Normals"].size() / 3));

	PointCloud::Ptr NewCloud(new PointCloud(NumPoints, 1));

	auto VertexIt = Mesh["Vertices"].begin();
	auto NormalIt = Mesh["Normals"].begin();

	for (size_t i = 0; i < NumPoints; i++)
	{
		NewCloud->points[i].x = (*VertexIt++);
		NewCloud->points[i].y = (*VertexIt++);
		NewCloud->points[i].z = (*VertexIt++);
		NewCloud->points[i].normal_x = (*NormalIt++);
		NewCloud->points[i].normal_y = (*NormalIt++);
		NewCloud->points[i].normal_z = (*NormalIt++);
	}

	IndexKey Index{ Mesh["Index"][0], Mesh["Index"][1], Mesh["Index"][2] };
	Grid[Index] = NewCloud;
}


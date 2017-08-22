/*

Copyright(C) 2017 Julian Löhr
All rights reserved.

Licensed under:
MIT License

*/
#include "stdafx.h"
#include "UpdatingCloud.h"


UpdatingCloud::UpdatingCloud(bool ConvertFrame)
	:ClearFlag(false)
	,Cloud(new PointCloud)
	,ConvertFrame(ConvertFrame)
{
}

void UpdatingCloud::Load(const std::string & FilePath)
{
	FileCloud.reset(new PointCloud());
	pcl::io::loadPCDFile(FilePath, *FileCloud);

	if (ConvertFrame)
	{
		ConvertFromAndroidToPCLFrame(FileCloud);
	}
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
		ClearFlag = false;
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
	Grid.clear();
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
		if (!ConvertFrame)
		{
			NewCloud->points[i].x = (*VertexIt++);
			NewCloud->points[i].y = (*VertexIt++);
			NewCloud->points[i].z = (*VertexIt++);
			NewCloud->points[i].normal_x = (*NormalIt++);
			NewCloud->points[i].normal_y = (*NormalIt++);
			NewCloud->points[i].normal_z = (*NormalIt++);
		}
		else
		{
			NewCloud->points[i].x = (*VertexIt++);
			NewCloud->points[i].z = (*VertexIt++);
			NewCloud->points[i].z *= -1.f;
			NewCloud->points[i].y = (*VertexIt++);
			NewCloud->points[i].normal_x = (*NormalIt++);
			NewCloud->points[i].normal_z = (*NormalIt++);
			NewCloud->points[i].normal_z *= -1.f;
			NewCloud->points[i].normal_y = (*NormalIt++);
		}
	}

	IndexKey Index{ Mesh["Index"][0], Mesh["Index"][1], Mesh["Index"][2] };
	Grid[Index] = NewCloud;
}

void UpdatingCloud::ConvertFromAndroidToPCLFrame(PointCloud::Ptr Cloud)
{
	Eigen::Matrix4f Rotation = Eigen::Matrix4f::Identity();
	Rotation(1, 1) = 0;
	Rotation(1, 2) = 1;
	Rotation(2, 1) = -1;
	Rotation(2, 2) = 0;

	pcl::transformPointCloud(*Cloud, *Cloud, Rotation);
}

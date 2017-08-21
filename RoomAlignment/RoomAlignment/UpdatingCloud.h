/*

Copyright(C) 2017 Julian Löhr
All rights reserved.

Licensed under:
MIT License

*/
#pragma once

#include "MonitoredQueue.h"

class UpdatingCloud
{
public:
	typedef MonitoredQueue<std::string> UpdateMessageQueue;
	UpdatingCloud();

	void Load(const std::string & FilePath);
	void Clear();

	bool HasUpdates() const;
	const PointCloud::ConstPtr GetCurrentCloud();

	UpdateMessageQueue UpdateQueue;

private:
	typedef std::array<int, 3> IndexKey;
	typedef std::map<IndexKey, PointCloud::ConstPtr> CloudGrid;

	std::atomic_bool ClearFlag;
	PointCloud::Ptr FileCloud;
	PointCloud::Ptr Cloud;

	CloudGrid Grid;

	void Reset();
	void Update();
	void UpdateGridCells(const std::string & JSONString);
	void UpdateGridCell(const nlohmann::json & Mesh);
};


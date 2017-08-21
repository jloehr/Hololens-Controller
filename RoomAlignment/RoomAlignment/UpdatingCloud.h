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
	UpdatingCloud();

	void Load(const std::string & FilePath);
	void Clear();

	bool HasUpdates() const;
	const PointCloud::ConstPtr GetCurrentCloud();

	MonitoredQueue<std::string> UpdateQueue;

private:
	std::atomic_bool ClearFlag;
	PointCloud::Ptr FileCloud;
	PointCloud::Ptr Cloud;

	void Reset();
};


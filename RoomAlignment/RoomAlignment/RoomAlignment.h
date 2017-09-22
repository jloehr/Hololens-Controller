/*

Copyright(C) 2017 Julian Löhr
All rights reserved.

Licensed under:
MIT License

*/
#pragma once

#include "Mosquitto.h"
#include "MonitoredQueue.h"
#include "UpdatingCloud.h"
#include "Signal.h"

class RoomAlignment
{
public:
	RoomAlignment(Mosquitto & MQTT);
	~RoomAlignment() = default;

	/* Called from Main thread */
	void Initialize();
	void Start();

private:
	const std::string HololensScanFile = "../../Data/SpatialMapping-Hololens-Office-CleanEmpty.pcd";
	const std::string HololensScanTopic = "HololensController/RoomScan/Update";
	const std::string HololensScanClearTopic = "HololensController/RoomScan/Clear";
	const std::string TangoScanFile = "../../Data/Tango-Office-CleanEmpty.pcd";
	const std::string TangoScanTopic = "TangoController/RoomScan/Update";
	const std::string TangoScanResetTopic = "TangoController/RoomScan/Reset";

	const std::string TimeTextId = "TimeText";
	const std::string PointTextId = "PointText";
	const std::string HololensCloudId = "Hololens";
	const std::string TangoCloudId = "Tango";
	const std::string ICPCloudId = "ICP";

	Mosquitto & MQTT;	
	Signal CloudUpdate;
	UpdatingCloud HololensScan;
	UpdatingCloud TangoScan;

	PointCloud::ConstPtr HololensRoom;
	PointCloud::ConstPtr TangoRoom;
	PointCloud::Ptr AlignedTangoRoom;

	Eigen::Matrix4f Transformation;
	pcl::IterativeClosestPoint<Point, Point> ICP;
	pcl::visualization::CloudViewer Viewer;

	void Run();

	void ShowLatest();
	void ShowCloud(PointCloud::ConstPtr Cloud, const std::string & CloudName);

	void VisualizationCallback(pcl::visualization::PCLVisualizer& viewer);
};


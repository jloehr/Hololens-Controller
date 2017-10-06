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
#include "HeadingTransformEstimation.h"

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
	const std::string HololensScanHeadingTopic = "HololensController/RoomScan/Heading";
	const std::string TangoScanFile = "../../Data/Tango-Office-CleanEmpty.pcd";
	const std::string TangoScanTopic = "TangoController/RoomScan/Update";
	const std::string TangoScanResetTopic = "TangoController/RoomScan/Reset";

	const std::string RoomAlignmentTestRunFinished = "RoomAlignment/Run/Finished";

	const std::string TimeTextId = "TimeText";
	const std::string PointTextId = "PointText";
	const std::string HololensCloudId = "Hololens";
	const std::string TangoCloudId = "Tango";
	const std::string ICPCloudId = "ICP";

	Mosquitto & MQTT;	
	Signal CloudUpdate;
	UpdatingCloud HololensScan;
	UpdatingCloud TangoScan;
	HeadingTransformEstimation HeadingEstimation;

	PointCloud::ConstPtr HololensRoom;
	PointCloud::ConstPtr TangoRoom;
	PointCloud::Ptr AlignedTangoRoom;

	Eigen::Matrix4f Transformation;
	pcl::IterativeClosestPoint<Point, Point> ICP;
	pcl::visualization::CloudViewer Viewer;

	std::atomic_bool TestFinishedFlag;

	void Run();

	bool HasConverged(const Eigen::Matrix4f & Previous, const Eigen::Matrix4f & Current);

	void ShowLatest();
	void ShowCloud(PointCloud::ConstPtr Cloud, const std::string & CloudName, uint8_t R, uint8_t G, uint8_t B);

	void VisualizationCallback(pcl::visualization::PCLVisualizer& viewer);
};


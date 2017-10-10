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
	const std::string HololensScanFile = "../../Data/HoloLens-OfficeScan.pcd";
	const std::string HololensScanTopic = "HololensController/RoomScan/Update";
	const std::string HololensScanClearTopic = "HololensController/RoomScan/Clear";
	const std::string HololensScanHeadingTopic = "HololensController/RoomScan/Heading";
	const std::string TangoScanFile = "../../Data/Tango-OfficeScan.pcd";
	const std::string TangoScanTopic = "TangoController/RoomScan/Update";
	const std::string TangoScanResetTopic = "TangoController/RoomScan/Reset";

	const std::string RoomAlignmentTransform = "RoomAlignment/Transform";
	const std::string RoomAlignmentRunFinished = "RoomAlignment/Run/Finished";
	const std::string RoomAlignmentReplayFinished = "RoomAlignment/Replay/Finished";

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
	//pcl::IterativeClosestPoint<Point, Point> ICP;
	//pcl::IterativeClosestPointNonLinear<Point, Point> ICP;
	//pcl::IterativeClosestPointWithNormals<Point, Point> ICP;
	pcl::GeneralizedIterativeClosestPoint<Point, Point> ICP;
	pcl::visualization::CloudViewer Viewer;

	std::atomic_bool ReplayFinishedFlag;

	void Run();

	bool HasConverged(const Eigen::Matrix4f & Previous, const Eigen::Matrix4f & Current);
	void SendTransform();

	void ShowLatest();
	void ShowCloud(PointCloud::ConstPtr Cloud, const std::string & CloudName, uint8_t R, uint8_t G, uint8_t B);

	void VisualizationCallback(pcl::visualization::PCLVisualizer& viewer);
};


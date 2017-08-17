/*

Copyright(C) 2017 Julian Löhr
All rights reserved.

Licensed under:
MIT License

*/
#pragma once

#include "Mosquitto.h"

class RoomAlignment
{
public:
	RoomAlignment(Mosquitto & MQTT);
	~RoomAlignment() = default;

	/* Called from Main thread */
	void Initialize();
	void Start();

private:
	typedef pcl::PointXYZINormal Point;
	typedef pcl::PointCloud<Point> PointCloud;

	const std::string HololensScanFile = "../../Data/SpatialMapping-Hololens-Office-CleanEmpty.pcd";
	const std::string TangoScanFile = "../../Data/Tango-Office-CleanEmpty.pcd";

	const std::string TimeTextId = "TimeText";
	const std::string PointTextId = "PointText";
	const std::string HololensCloudId = "Hololens";
	const std::string TangoCloudId = "Tango";
	const std::string ICPCloudId = "ICP";

	Mosquitto & MQTT;

	PointCloud::Ptr HololensRoom;
	PointCloud::Ptr TangoRoom;
	PointCloud::Ptr AlignedTangoRoom;
	Eigen::Matrix4f Transformation;
	pcl::IterativeClosestPoint<Point, Point> ICP;
	pcl::visualization::CloudViewer Viewer;

	void Run();

	void ShowLatest();
	void ShowCloud(PointCloud::Ptr Cloud, const std::string & CloudName);

	void VisualizationCallback(pcl::visualization::PCLVisualizer& viewer);
};


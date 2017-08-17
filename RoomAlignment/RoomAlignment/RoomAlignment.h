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

	Mosquitto & MQTT;

	pcl::visualization::CloudViewer Viewer;
	PointCloud::Ptr HololensRoom;
	PointCloud::Ptr TangoRoom;

	void Run();

	void ShowLatest();
	void ShowCloud(PointCloud::Ptr Cloud, const std::string & CloudName);

	void VisualizationCallback(pcl::visualization::PCLVisualizer& viewer);
};


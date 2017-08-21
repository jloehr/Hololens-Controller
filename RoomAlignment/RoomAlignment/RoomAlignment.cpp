/*

Copyright(C) 2017 Julian Löhr
All rights reserved.

Licensed under:
MIT License

*/
#include "stdafx.h"
#include "RoomAlignment.h"

#include "Application.h"

RoomAlignment::RoomAlignment(Mosquitto & MQTT)
	:MQTT(MQTT),
	Viewer("Latest Rooms"),
	HololensRoom(new PointCloud()), TangoRoom(new PointCloud()), AlignedTangoRoom(new PointCloud()),
	Transformation(Eigen::Matrix4f::Identity())
{
}

void RoomAlignment::Initialize()
{
	Viewer.runOnVisualizationThread([this](pcl::visualization::PCLVisualizer & viewer) {VisualizationCallback(viewer);});
}

void RoomAlignment::Start()
{
	std::thread WorkerThread(&RoomAlignment::Run, this);
	WorkerThread.detach();
}

[[ noreturn ]]
void RoomAlignment::Run()
{
	// Load Clouds
	pcl::io::loadPCDFile(HololensScanFile, *HololensRoom);
	pcl::io::loadPCDFile(TangoScanFile, *TangoRoom);

	while (true)
	{
		// Update Clouds

		// Do Alignment
		ICP.setInputSource(TangoRoom);
		ICP.setInputTarget(HololensRoom);
		ICP.align(*AlignedTangoRoom, Transformation);
		Transformation = ICP.getFinalTransformation();

		// Send result
		ShowLatest();
	}
}

void RoomAlignment::ShowLatest()
{
	ShowCloud(HololensRoom, "HoloLens");
	ShowCloud(TangoRoom, "Tango");
	ShowCloud(AlignedTangoRoom, "AlignedTango");
}

void RoomAlignment::ShowCloud(PointCloud::Ptr Cloud, const std::string & CloudName)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr Buffer(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::copyPointCloud(*Cloud, *Buffer);
	pcl::PointCloud<pcl::PointXYZ>::ConstPtr ConstBuffer(Buffer);
	Viewer.showCloud(ConstBuffer, CloudName);
}

void RoomAlignment::VisualizationCallback(pcl::visualization::PCLVisualizer & viewer)
{
	if (viewer.wasStopped())
		Application::Close();
}

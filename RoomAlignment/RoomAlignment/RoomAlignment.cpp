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
	CloudUpdate([&]()->bool {return (HololensScan.HasUpdates() || TangoScan.HasUpdates());}),
	AlignedTangoRoom(new PointCloud()),
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
	//Subscribe to Topics
	MQTT.Subscribe(RoomAlignment::HololensScanTopic, [this](const std::string Topic, const std::string & Payload) { HololensScan.UpdateQueue.Enqueue(Payload); }, Mosquitto::AtLeastOnce);
	MQTT.Subscribe(RoomAlignment::HololensScanClearTopic, [this](const std::string Topic, const std::string & Payload) { HololensScan.Clear(); }, Mosquitto::ExactlyOnce);
	MQTT.Subscribe(RoomAlignment::TangoScanTopic, [this](const std::string Topic, const std::string & Payload) { TangoScan.UpdateQueue.Enqueue(Payload); }, Mosquitto::AtLeastOnce);
	MQTT.Subscribe(RoomAlignment::TangoScanClearTopic, [this](const std::string Topic, const std::string & Payload) { TangoScan.Clear(); }, Mosquitto::ExactlyOnce);

	// Load Clouds
	HololensScan.Load(HololensScanFile);
	//TangoScan.Load(TangoScanFile);

	while (true)
	{
		CloudUpdate.Wait();

		do
		{
			// Update Clouds
			HololensRoom = HololensScan.GetCurrentCloud();
			TangoRoom = TangoScan.GetCurrentCloud();

			// Do Alignment
			ICP.setInputSource(TangoRoom);
			ICP.setInputTarget(HololensRoom);
			ICP.align(*AlignedTangoRoom, Transformation);

			// Get Result
			Transformation = ICP.getFinalTransformation();

			// Send result
			ShowLatest();

		} while (ICP.hasConverged());
	}
}

void RoomAlignment::ShowLatest()
{
	ShowCloud(HololensRoom, RoomAlignment::HololensCloudId);
	ShowCloud(TangoRoom, RoomAlignment::TangoCloudId);
	ShowCloud(AlignedTangoRoom, RoomAlignment::ICPCloudId);
}

void RoomAlignment::ShowCloud(PointCloud::ConstPtr Cloud, const std::string & CloudName)
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

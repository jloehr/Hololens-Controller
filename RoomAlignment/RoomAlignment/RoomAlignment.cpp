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
	HololensScan(false),
	TangoScan(true),
	CloudUpdate([&]()->bool {return (HololensScan.HasUpdates() || TangoScan.HasUpdates());}),
	AlignedTangoRoom(new PointCloud()),
	Transformation(Eigen::Matrix4f::Identity())
{
}

void RoomAlignment::Initialize()
{
	Viewer.runOnVisualizationThread([this](pcl::visualization::PCLVisualizer & viewer) {VisualizationCallback(viewer);});
	ICP.setMaximumIterations(1);
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
	MQTT.Subscribe(RoomAlignment::HololensScanTopic, [this](const std::string Topic, const std::string & Payload) { HololensScan.UpdateQueue.Enqueue(Payload); CloudUpdate.Wake(); }, Mosquitto::AtLeastOnce);
	MQTT.Subscribe(RoomAlignment::HololensScanClearTopic, [this](const std::string Topic, const std::string & Payload) { HololensScan.Clear(); }, Mosquitto::ExactlyOnce);
	MQTT.Subscribe(RoomAlignment::HololensScanHeadingTopic, [this](const std::string Topic, const std::string & Payload) { HeadingEstimation.HoloLensHeadingQueue.Enqueue(Payload); }, Mosquitto::ExactlyOnce);
	MQTT.Subscribe(RoomAlignment::TangoScanTopic, [this](const std::string Topic, const std::string & Payload) { TangoScan.UpdateQueue.Enqueue(Payload);  CloudUpdate.Wake(); }, Mosquitto::AtLeastOnce);
	MQTT.Subscribe(RoomAlignment::TangoScanResetTopic, [this](const std::string Topic, const std::string & Payload) { TangoScan.Clear(); HeadingEstimation.TangoHeadingQueue.Enqueue(Payload); }, Mosquitto::ExactlyOnce);

	// Load Clouds
	HololensScan.Load(HololensScanFile);
	TangoScan.Load(TangoScanFile);
	// Wait for Headings
	while(!HeadingEstimation.HasBothHeadings())
	{
		// Update Clouds
		HololensRoom = HololensScan.GetCurrentCloud();
		TangoRoom = TangoScan.GetCurrentCloud();
	}

	// Get Transform Estimate 
	Transformation = HeadingEstimation.GetTransformEstimation();
	
	while (true)
	{
		//CloudUpdate.Wait();

		size_t Iterations = 0;
		std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();

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

			Iterations++;
			std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
			std::cout << "Iteration " << Iterations << " took " << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() << "ms.\n";

		} while (ICP.hasConverged());
	}
}

void RoomAlignment::ShowLatest()
{
	ShowCloud(HololensRoom, RoomAlignment::HololensCloudId, 0, 255, 0);
	ShowCloud(TangoRoom, RoomAlignment::TangoCloudId, 0, 0, 0);
	ShowCloud(AlignedTangoRoom, RoomAlignment::ICPCloudId, 255, 0, 0);
}

void RoomAlignment::ShowCloud(PointCloud::ConstPtr Cloud, const std::string & CloudName, uint8_t R, uint8_t G, uint8_t B)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr Buffer(new pcl::PointCloud<pcl::PointXYZRGB>());
	pcl::copyPointCloud(*Cloud, *Buffer);
	for (pcl::PointXYZRGB & Point : *Buffer)
	{
		Point.r = R;
		Point.g = G;
		Point.b = B;
	}
	pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr ConstBuffer(Buffer);
	Viewer.showCloud(ConstBuffer, CloudName);
}

void RoomAlignment::VisualizationCallback(pcl::visualization::PCLVisualizer & viewer)
{
	if (viewer.wasStopped())
		Application::Close();
}

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
	HololensScan(UpdatingCloud::Source::Unity),
	TangoScan(UpdatingCloud::Source::Tango),
	CloudUpdate([&]()->bool {return (HololensScan.HasUpdates() || TangoScan.HasUpdates());}),
	AlignedTangoRoom(new PointCloud()),
	Transformation(Eigen::Matrix4f::Identity())
{
}

void RoomAlignment::Initialize()
{
	Viewer.runOnVisualizationThread([this](pcl::visualization::PCLVisualizer & viewer) {VisualizationCallback(viewer);});
	ICP.setMaximumIterations(1);
	ICP.setMaxCorrespondenceDistance(1);
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
	MQTT.Subscribe(RoomAlignment::RoomAlignmentTestRunFinished, [this](const std::string Topic, const std::string & Payload) { TestFinishedFlag = true; }, Mosquitto::ExactlyOnce);

	// Load Clouds
	// HololensScan.Load(HololensScanFile);
	// TangoScan.Load(TangoScanFile);
	
	while (true)
	{
		bool Converged = false;
		TestFinishedFlag = false;
		HeadingEstimation.Reset();

		// Wait for Headings
		while (!HeadingEstimation.HasBothHeadings())
		{
			// Update Clouds
			HololensRoom = HololensScan.GetCurrentCloud();
			TangoRoom = TangoScan.GetCurrentCloud();

			ShowLatest();
		}

		// Get Transform Estimate 
		Transformation = HeadingEstimation.GetTransformEstimation();

		ShowLatest();
		std::cout << "Start" << std::endl;
		size_t Iterations = 0;
		std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();
		std::chrono::steady_clock::time_point end;

		// Wait until both clouds are populated
		while (((HololensRoom->size() == 0) || (TangoRoom->size() == 0)) && !TestFinishedFlag)
		{
			HololensRoom = HololensScan.GetCurrentCloud();
			TangoRoom = TangoScan.GetCurrentCloud();
		}

		while ((!Converged || !TestFinishedFlag) && ((HololensRoom->size() >= 0) && (TangoRoom->size() >= 0)))
		{
			// Update Clouds
			HololensRoom = HololensScan.GetCurrentCloud();
			TangoRoom = TangoScan.GetCurrentCloud();

			// Do Alignment
			ICP.setInputSource(TangoRoom);
			ICP.setInputTarget(HololensRoom);
			ICP.align(*AlignedTangoRoom, Transformation);

			// Get Result
			Eigen::Matrix4f PreviousTransformation = Transformation;
			Transformation = ICP.getFinalTransformation();

			if (!Converged)
				end = std::chrono::steady_clock::now();
			Converged = HasConverged(PreviousTransformation, Transformation);

			// if converged
				// Send Transform

			Iterations++;

			ShowLatest();
		} 

		if (Iterations == 0)
		{
			std::cout << "No Matching performed!" << std::endl;
			std::cout << "HoloLens: " << std::setw(8) << HololensRoom->size() << "\tTango: " << std::setw(8) << TangoRoom->size() << std::endl;
		}
		else
		{
			std::cout << "Iterations " << Iterations << " took " << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() << "ms.\n";
			std::cout << "HoloLens: " << std::setw(8) << HololensRoom->size() << "\tTango: " << std::setw(8) << TangoRoom->size() << std::endl;
			std::cout << "Transform: " << std::endl;

			constexpr std::streamsize Width = 15;
			constexpr std::streamsize Precision = 10;
			std::cout.precision(Precision);
			std::cout << std::fixed << std::setw(Width) << Transformation(0, 0) << '\t' << std::setw(Width) << Transformation(0, 1) << '\t' << std::setw(Width) << Transformation(0, 2) << '\t' << std::setw(Width) << Transformation(0, 3) << std::endl;
			std::cout << std::fixed << std::setw(Width) << Transformation(1, 0) << '\t' << std::setw(Width) << Transformation(1, 1) << '\t' << std::setw(Width) << Transformation(1, 2) << '\t' << std::setw(Width) << Transformation(1, 3) << std::endl;
			std::cout << std::fixed << std::setw(Width) << Transformation(2, 0) << '\t' << std::setw(Width) << Transformation(2, 1) << '\t' << std::setw(Width) << Transformation(2, 2) << '\t' << std::setw(Width) << Transformation(2, 3) << std::endl;
			std::cout << std::fixed << std::setw(Width) << Transformation(3, 0) << '\t' << std::setw(Width) << Transformation(3, 1) << '\t' << std::setw(Width) << Transformation(3, 2) << '\t' << std::setw(Width) << Transformation(3, 3) << std::endl;
		}

		// Send result
		ShowLatest();
	}
}

bool RoomAlignment::HasConverged(const Eigen::Matrix4f & Previous, const Eigen::Matrix4f & Current)
{
	constexpr double Epsilon = 1e-10;
	double MSE = 0.0;

	for (int i = 0; i < 4; i++)
		for (int j = 0; j < 4; j++)
			MSE += pow(Previous(i, j) - Current(i, j), 2);

	MSE /= 16.;

	std::cout << "MSE: " << std::fixed << std::setw(15) << std::setprecision(15) << MSE << std::endl;

	return (MSE < Epsilon);
}

void RoomAlignment::ShowLatest()
{
	ShowCloud(HololensRoom, RoomAlignment::HololensCloudId, 255, 0, 0);
	ShowCloud(TangoRoom, RoomAlignment::TangoCloudId, 0, 0, 255);
	ShowCloud(AlignedTangoRoom, RoomAlignment::ICPCloudId, 0, 255, 0);
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

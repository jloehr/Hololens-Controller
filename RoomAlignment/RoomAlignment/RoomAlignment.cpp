/*

Copyright(C) 2017 Julian Löhr
All rights reserved.

Licensed under:
MIT License

*/
#include "stdafx.h"
#include "RoomAlignment.h"

#include "Application.h"

constexpr size_t MaxIterations = 0;
constexpr float Leafsize = 0.000f;
//constexpr float Leafsize = 0.01f;
//constexpr float Leafsize = 0.05f;
//constexpr size_t MaxIterations = 1000;
//constexpr float Leafsize = 0.1f;
//constexpr size_t MaxIterations = 2000;
//constexpr float Leafsize = 0.2f;
//constexpr size_t MaxIterations = 5000;
//constexpr size_t MaxIterations = 10000;
//constexpr float Leafsize = 0.5f;
//constexpr size_t MaxIterations = 15000;
//constexpr size_t MaxIterations = 30000;

static std::atomic_bool TakeScreen = false;
static std::atomic_uint64_t Seconds = 0;

RoomAlignment::RoomAlignment(Mosquitto & MQTT)
	:MQTT(MQTT),
	Viewer("Latest Rooms"),
	HololensScan(UpdatingCloud::Source::Unity, Leafsize),
	TangoScan(UpdatingCloud::Source::Tango, Leafsize),
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
	MQTT.Subscribe(RoomAlignment::RoomAlignmentReplayFinished, [this](const std::string Topic, const std::string & Payload) { ReplayFinishedFlag = true; }, Mosquitto::ExactlyOnce);


	std::stringstream IterationStr;
	std::stringstream TimeStr;
	std::stringstream HoloLensStr;
	std::stringstream TangoStr;
	std::stringstream Trans1Str;
	std::stringstream Trans2Str;
	std::stringstream Trans3Str;
	std::stringstream Trans4Str;
	// Load Clouds
	/*
	PointCloud::Ptr Hololens(new PointCloud());
	PointCloud::Ptr Tango(new PointCloud());
	pcl::io::loadPCDFile(HololensScanFile, *Hololens);
	pcl::io::loadPCDFile(TangoScanFile, *Tango);
	std::vector<float> LeafSizes = { 0.005f, 0.01f, 0.02f, 0.05f, 0.1f, 0.2f, 0.5f };
	pcl::VoxelGrid<Point> Filter;

	std::cout << "Points HoloLens / Tango" << std::endl;
	std::cout << "Orig -> " << Hololens->size() << " / " << Tango->size() << std::endl;
	for (float Leaf : LeafSizes)
	{
		PointCloud HololensFiltered;
		PointCloud TangoFiltered;

		Filter.setLeafSize(Leaf, Leaf, Leaf);

		Filter.setInputCloud(Hololens);
		Filter.filter(HololensFiltered);

		Filter.setInputCloud(Tango);
		Filter.filter(TangoFiltered);

		std::cout << std::fixed << std::setw(5) << Leaf << " -> " << HololensFiltered.size() << " / " << TangoFiltered.size() << std::endl;
	}
	*/
	while (true)
	{
		bool Converged = false;
		ReplayFinishedFlag = false;
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

		/*
		while (true)
		{
			// Update Clouds
			HololensRoom = HololensScan.GetCurrentCloud();
			TangoRoom = TangoScan.GetCurrentCloud();

			pcl::transformPointCloud(*TangoRoom, *AlignedTangoRoom, Transformation);

			ShowLatest();

			if (ReplayFinishedFlag)
			{
				Seconds++;
				TakeScreen = true;
				while (TakeScreen == true)
					std::this_thread::sleep_for(std::chrono::microseconds(10));

				ReplayFinishedFlag = false;
				// Signale Replay to send next
				MQTT.Publish(RoomAlignmentRunFinished, std::string(), Mosquitto::ExactlyOnce, false);
			}
		}*/

		ShowLatest();
		std::cout << "Start" << std::endl;
		size_t Iterations = 0;
		std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();
		std::chrono::steady_clock::time_point end;

		// Wait until both clouds are populated
		while (((HololensRoom->size() == 0) || (TangoRoom->size() == 0)) && !ReplayFinishedFlag)
		{
			HololensRoom = HololensScan.GetCurrentCloud();
			TangoRoom = TangoScan.GetCurrentCloud();
		}

		while (((!Converged && ((Iterations < MaxIterations) || (MaxIterations == 0))) || !ReplayFinishedFlag) && ((HololensRoom->size() >= 0) && (TangoRoom->size() >= 0)))
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

			bool PreviousConverged = Converged;
			Converged = HasConverged(PreviousTransformation, Transformation);

			// If this very alignment made it converge
			if (!PreviousConverged && Converged)
			{
				SendTransform();
				end = std::chrono::steady_clock::now();
			}

			Iterations++;

			//ShowLatest();
		} 

		if (Iterations == 0)
		{
			std::cout << "No Matching performed!" << std::endl;
			std::cout << "HoloLens: " << std::setw(8) << HololensRoom->size() << "\tTango: " << std::setw(8) << TangoRoom->size() << std::endl;

			IterationStr << "NA;";
			TimeStr << "NA;";
			HoloLensStr  << HololensRoom->size() << ";";
			TangoStr << TangoRoom->size() << ";";
			Trans1Str << "NA;NA;NA;NA;";
			Trans2Str << "NA;NA;NA;NA;";
			Trans3Str << "NA;NA;NA;NA;";
			Trans4Str << "NA;NA;NA;NA;";
		}
		else
		{
			std::cout << "Iterations " << Iterations << " took " << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() << "ms.\n";
			std::cout << "HoloLens: " << std::setw(8) << HololensRoom->size() << "\tTango: " << std::setw(8) << TangoRoom->size() << std::endl;
			std::cout << "Transform: " << std::endl;


			IterationStr << Iterations <<";";
			TimeStr << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() << ";";
			HoloLensStr << HololensRoom->size() << ";";
			TangoStr << TangoRoom->size() << ";";

			constexpr std::streamsize Width = 15;
			constexpr std::streamsize Precision = 10;
			std::cout.precision(Precision);
			Trans1Str << std::fixed << std::setprecision(Precision) << std::setw(Width) << Transformation(0, 0) << ';' << std::setw(Width) << Transformation(0, 1) << ';' << std::setw(Width) << Transformation(0, 2) << ';' << std::setw(Width) << Transformation(0, 3) << ';';
			Trans2Str << std::fixed << std::setprecision(Precision) << std::setw(Width) << Transformation(1, 0) << ';' << std::setw(Width) << Transformation(1, 1) << ';' << std::setw(Width) << Transformation(1, 2) << ';' << std::setw(Width) << Transformation(1, 3) << ';';
			Trans3Str << std::fixed << std::setprecision(Precision) << std::setw(Width) << Transformation(2, 0) << ';' << std::setw(Width) << Transformation(2, 1) << ';' << std::setw(Width) << Transformation(2, 2) << ';' << std::setw(Width) << Transformation(2, 3) << ';';
			Trans4Str << std::fixed << std::setprecision(Precision) << std::setw(Width) << Transformation(3, 0) << ';' << std::setw(Width) << Transformation(3, 1) << ';' << std::setw(Width) << Transformation(3, 2) << ';' << std::setw(Width) << Transformation(3, 3) << ';';
		}

		std::cout << IterationStr.str() << std::endl;
		std::cout << TimeStr.str() << std::endl;
		std::cout << HoloLensStr.str() << std::endl;
		std::cout << TangoStr.str() << std::endl;
		std::cout << Trans1Str.str() << std::endl;
		std::cout << Trans2Str.str() << std::endl;
		std::cout << Trans3Str.str() << std::endl;
		std::cout << Trans4Str.str() << std::endl;

		// Send result
		ShowLatest();

		// Signale Replay to send next
		MQTT.Publish(RoomAlignmentRunFinished, std::string(), Mosquitto::ExactlyOnce, false);
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

	//std::cout << "MSE: " << std::fixed << std::setw(15) << std::setprecision(15) << MSE << std::endl;

	return (MSE < Epsilon);
}

void RoomAlignment::SendTransform()
{
	nlohmann::json JSON;


	for (int i = 0; i < 4; i++)
		for (int j = 0; j < 4; j++)
		{
			std::stringstream Name;
			Name << "m" << i << j;
			JSON[Name.str()] = Transformation(i, j);
		}

	std::string Payload = JSON.dump();
	MQTT.Publish(RoomAlignmentTransform, JSON.dump(), Mosquitto::AtLeastOnce, false);
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

	if (TakeScreen)
	{
		std::stringstream Name;
		Name << "Tango" << Seconds << ".png";
		viewer.saveScreenshot(Name.str());
		TakeScreen = false;
	}
}

/*

Copyright(C) 2017 Julian Löhr
All rights reserved.

Licensed under:
MIT License

*/
#include "stdafx.h"
#include "RoomAlignment.h"

RoomAlignment::RoomAlignment(Mosquitto & MQTT)
	:MQTT(MQTT),
	Viewer("Latest Rooms"),
	HololensRoom(new PointCloud()), TangoRoom(new PointCloud())
{
}

void RoomAlignment::Initialize()
{

}

void RoomAlignment::Start()
{
	std::thread WorkerThread(&RoomAlignment::Run, this);
	WorkerThread.detach();
}

void RoomAlignment::Run()
{
	// Load Clouds
	pcl::io::loadPCDFile(HololensScanFile, *HololensRoom);
	pcl::io::loadPCDFile(TangoScanFile, *TangoRoom);

	while (true)
	{
		// Update Clouds

		// Do Alignment

		// Send result
		ShowLatest();
	}
}

void RoomAlignment::ShowLatest()
{
	ShowCloud(HololensRoom, "HoloLens");
	ShowCloud(TangoRoom, "Tango");
}

void RoomAlignment::ShowCloud(PointCloud::Ptr Cloud, const std::string & CloudName)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr Buffer(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::copyPointCloud(*Cloud, *Buffer);
	pcl::PointCloud<pcl::PointXYZ>::ConstPtr ConstBuffer(Buffer);
	Viewer.showCloud(ConstBuffer, CloudName);
}

/*

Copyright(C) 2017 Julian Löhr
All rights reserved.

Licensed under:
MIT License

*/
#include "stdafx.h"
#include "RoomAlignment.h"

RoomAlignment::RoomAlignment(Mosquitto & MQTT)
	:MQTT(MQTT)
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

	while (true)
	{
		// Update Clouds

		// Do Alignment

		// Send result
	}
}

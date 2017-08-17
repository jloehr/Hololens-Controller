/*

Copyright(C) 2017 Julian Löhr
All rights reserved.

Licensed under:
MIT License

*/
#include "stdafx.h"
#include "RoomAlignment.h"

std::atomic_bool RoomAlignment::Aborted = false;
std::condition_variable_any RoomAlignment::MainThreadSleep;
std::condition_variable_any RoomAlignment::CtrlHandlerThreadSleep;
std::mutex RoomAlignment::AbortMutex;

BOOL RoomAlignment::CtrlHandler(DWORD CtrlType)
{
	switch (CtrlType)
	{
	case CTRL_C_EVENT:
	case CTRL_CLOSE_EVENT:
	{
		std::lock_guard<std::mutex> lock(AbortMutex);
		if (Aborted)
			return TRUE;
		std::cout << "Window is closing or received CTL-C" << std::endl;
		Aborted = true;
		MainThreadSleep.notify_all();
		CtrlHandlerThreadSleep.wait(AbortMutex);
		return TRUE;
	}
	default:
		return FALSE;
	}
}

RoomAlignment::RoomAlignment()
{
}


RoomAlignment::~RoomAlignment()
{
}

void RoomAlignment::Run()
{
	std::cout << "Run ..." << std::endl;

	SetConsoleCtrlHandler(reinterpret_cast<PHANDLER_ROUTINE>(CtrlHandler), true);

	MQTT.Initialize();
	// Init PCL

	MQTT.Connect();
	// Start PCL thread

	// Sleep
	SleepMainThread();

	MQTT.Disconnect();
	MQTT.Finalize();

	WakeCtrlHandler();
}

void RoomAlignment::SleepMainThread()
{
	std::lock_guard<std::mutex> lock(AbortMutex);

	if (Aborted)
		return;

	std::cout << "Main thread going to sleep..." << std::endl;
	MainThreadSleep.wait(AbortMutex);
	std::cout << "Main thread continues..." << std::endl;
}

void RoomAlignment::WakeCtrlHandler()
{
	std::lock_guard<std::mutex> lock(AbortMutex);
	CtrlHandlerThreadSleep.notify_all();
}

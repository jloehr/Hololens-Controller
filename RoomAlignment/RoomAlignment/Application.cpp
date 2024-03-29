/*

Copyright(C) 2017 Julian Löhr
All rights reserved.

Licensed under:
MIT License

*/
#include "stdafx.h"
#include "Application.h"

std::atomic_bool Application::Aborted = false;
std::condition_variable_any Application::MainThreadSleep;
std::condition_variable_any Application::CtrlHandlerThreadSleep;
std::recursive_mutex Application::AbortMutex;

void Application::Close()
{
	std::lock_guard<std::recursive_mutex> lock(AbortMutex);
	if (Aborted)
		return;
	Aborted = true;
	MainThreadSleep.notify_all();
}

BOOL Application::CtrlHandler(DWORD CtrlType)
{
	switch (CtrlType)
	{
	case CTRL_C_EVENT:
	case CTRL_CLOSE_EVENT:
	{
		std::lock_guard<std::recursive_mutex> lock(AbortMutex);
		if (Aborted)
			return TRUE;
		std::cout << "Window is closing or received CTL-C" << std::endl;
		Close();
		CtrlHandlerThreadSleep.wait(AbortMutex);
		return TRUE;
	}
	default:
		return FALSE;
	}
}

Application::Application()
	:RoomAlignment(MQTT)
{
}


Application::~Application()
{
}

[[ noreturn ]]
void Application::Run()
{
	std::cout << "Run ..." << std::endl;

	SetConsoleCtrlHandler(reinterpret_cast<PHANDLER_ROUTINE>(CtrlHandler), true);

	MQTT.Initialize();
	RoomAlignment.Initialize();

	MQTT.Connect();
	RoomAlignment.Start();

	SleepMainThread();

	MQTT.Disconnect();
	MQTT.Finalize();

	WakeCtrlHandler();
	std::exit(0);
}

void Application::SleepMainThread()
{
	std::lock_guard<std::recursive_mutex> lock(AbortMutex);

	if (Aborted)
		return;

	std::cout << "Main thread going to sleep..." << std::endl;
	MainThreadSleep.wait(AbortMutex);
	std::cout << "Main thread continues..." << std::endl;
}

void Application::WakeCtrlHandler()
{
	std::lock_guard<std::recursive_mutex> lock(AbortMutex);
	CtrlHandlerThreadSleep.notify_all();
}

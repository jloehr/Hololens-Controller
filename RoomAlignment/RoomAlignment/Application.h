/*

Copyright(C) 2017 Julian Löhr
All rights reserved.

Licensed under:
MIT License

*/
#pragma once

#include "Mosquitto.h"
#include "RoomAlignment.h"

class Application
{
public:
	Application();
	~Application();

	void Run();

	static std::atomic_bool Aborted;
	static std::condition_variable_any MainThreadSleep;
	static std::condition_variable_any CtrlHandlerThreadSleep;
	static std::mutex AbortMutex;
	static BOOL CtrlHandler(DWORD CtrlType);

private:
	Mosquitto MQTT;
	RoomAlignment RoomAlignment;

	void SleepMainThread();
	void WakeCtrlHandler();
};


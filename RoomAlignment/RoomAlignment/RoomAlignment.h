/*

Copyright(C) 2017 Julian Löhr
All rights reserved.

Licensed under:
MIT License

*/
#pragma once

#include "Mosquitto.h"

class RoomAlignment
{
public:
	RoomAlignment();
	~RoomAlignment();

	void Run();

	static std::atomic_bool Aborted;
	static std::condition_variable_any MainThreadSleep;
	static std::condition_variable_any CtrlHandlerThreadSleep;
	static std::mutex AbortMutex;
	static BOOL CtrlHandler(DWORD CtrlType);

private:
	Mosquitto MQTT;

	void SleepMainThread();
	void WakeCtrlHandler();
};


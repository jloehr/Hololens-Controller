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
	RoomAlignment(Mosquitto & MQTT);
	~RoomAlignment() = default;

	/* Called from Main thread */
	void Initialize();
	void Start();

private:
	Mosquitto & MQTT;

	void Run();
};


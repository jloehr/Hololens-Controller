/*

Copyright(C) 2017 Julian Löhr
All rights reserved.

Licensed under:
MIT License

*/
#include "stdafx.h"
#include "Mosquitto.h"


Mosquitto::Mosquitto()
	:MosquittoContext(nullptr)
{

}

Mosquitto::~Mosquitto()
{
}

void Mosquitto::Initialize()
{
	int Result;

	Result = mosquitto_lib_init();
	if (Result != MOSQ_ERR_SUCCESS)
	{
		std::cout << "Error initializing mosquitto: " << Result << std::endl;
		return;
	}

	MosquittoContext = mosquitto_new(nullptr, true, this);
	mosquitto_threaded_set(MosquittoContext, true);
	mosquitto_connect_callback_set(MosquittoContext, [](PMosquittoContext MosquittoContext, void * UserData, int Result) { std::cout << "Connect Result: " << Result << std::endl; });
	mosquitto_disconnect_callback_set(MosquittoContext, [](PMosquittoContext MosquittoContext, void * UserData, int Result) {std::cout << "Disconnect Result: " << Result << std::endl;});
	mosquitto_message_callback_set(MosquittoContext, [](PMosquittoContext MosquittoContext, void * UserData, CPMosquittoMessage Message) {});
}

void Mosquitto::Connect()
{
	int Result;

	if (MosquittoContext == nullptr)
		return;

	Result = mosquitto_connect(MosquittoContext, Hostname.c_str(), Port, 60);
	if (Result != MOSQ_ERR_SUCCESS)
	{
		std::cout << "Error connecting mosquitto: " << Result << std::endl;
		return;
	}

	MosquittoLoop = std::thread([this]() { mosquitto_loop_forever(MosquittoContext, -1, 1); });
}

void Mosquitto::Disconnect()
{
	if (MosquittoContext == nullptr)
		return;

	mosquitto_disconnect(MosquittoContext);

	if (MosquittoLoop.joinable())
		MosquittoLoop.join();
}

void Mosquitto::Finalize()
{
	mosquitto_destroy(MosquittoContext);
	mosquitto_lib_cleanup();
}

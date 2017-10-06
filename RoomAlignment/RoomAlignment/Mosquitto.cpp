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
	mosquitto_message_callback_set(MosquittoContext, OnMessageWrapper);
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
	MosquittoContext = nullptr;
	mosquitto_lib_cleanup();
}

void Mosquitto::Subscribe(const std::string & Topic, SubscriptionCallback Callback, QoS SubscriptionQoS)
{
	if (MosquittoContext == nullptr)
		return;

	SubscriptionCallbackList & CallbackList = Subscriptions[Topic];

	if (CallbackList.empty())
	{
		mosquitto_subscribe(MosquittoContext, nullptr, Topic.c_str(), SubscriptionQoS);
	}

	CallbackList.push_back(Callback);
}

void Mosquitto::Publish(const std::string & Topic, const std::string & Payload, QoS MessageQoS, bool Retain) const
{
	if (MosquittoContext == nullptr)
		return;

	int Result = mosquitto_publish(MosquittoContext, nullptr, Topic.c_str(), static_cast<int>(Payload.length()), Payload.c_str(), MessageQoS, Retain);
	std::cout << "Publish Result: " << Result << std::endl;
}

void Mosquitto::OnMessageWrapper(PMosquittoContext MosquittoContext, void * UserData, CPMosquittoMessage Message)
{
	Mosquitto * Instance = reinterpret_cast<Mosquitto *>(UserData);
	Instance->OnMessage(Message);
}

void Mosquitto::OnMessage(CPMosquittoMessage Message) const
{
	const std::string Topic(Message->topic);
	const std::string Payload(static_cast<const char *>(Message->payload), Message->payloadlen);

	//std::cout << "Message received for: " << Topic << " (" << (Payload.size()/1024) << " kB)" << std::endl;

	auto CallbackListIt = Subscriptions.find(Topic);
	if (CallbackListIt != Subscriptions.end())
	{
		for (const SubscriptionCallback & Callback : CallbackListIt->second)
		{
			Callback(Topic, Payload);
		}
	}
}

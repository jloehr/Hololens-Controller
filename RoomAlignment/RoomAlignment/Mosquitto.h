/*

Copyright(C) 2017 Julian Löhr
All rights reserved.

Licensed under:
MIT License

*/
#pragma once

class Mosquitto
{
public:
	enum QoS { FireAndForget = 0, AtLeastOnce = 1, ExactlyOnce = 2 };
	typedef std::function<void(const std::string & Topic, const std::string & Payload)> SubscriptionCallback;

	Mosquitto();
	~Mosquitto();

	void Initialize();
	void Connect();
	void Disconnect();
	void Finalize();

	void Subscribe(const std::string & Topic, SubscriptionCallback Callback, QoS SubscriptionQoS);
	void Publish(const std::string & Topic, const std::string & Payload, QoS MessageQoS, bool Retain) const;

private:
	typedef mosquitto * PMosquittoContext;
	typedef mosquitto_message * PMosquittoMessage;
	typedef const mosquitto_message * CPMosquittoMessage;
	typedef std::vector<SubscriptionCallback> SubscriptionCallbackList;
	typedef std::map<const std::string, SubscriptionCallbackList> TopicSubscriptionMap;

	const std::string Hostname = "192.168.188.2";
	const int Port = 1883;

	PMosquittoContext MosquittoContext;
	std::thread MosquittoLoop;

	TopicSubscriptionMap Subscriptions;

	static void OnMessageWrapper(PMosquittoContext MosquittoContext, void * UserData, CPMosquittoMessage Message);
	void OnMessage(CPMosquittoMessage Message) const;
};


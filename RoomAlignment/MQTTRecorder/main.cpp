/*

Copyright(C) 2017 Julian Löhr
All rights reserved.

Licensed under:
MIT License

*/
#include "stdafx.h"

static const std::string Hostname = "192.168.188.2";
static const int Port = 1883;
static const std::string FileName = "Recording.mqtt";

static std::atomic_bool Aborted = false;
static bool Connected = false;

static void Record();
static void Play();
static mosquitto* InitMosquitto();
static void FinitMosquitto(mosquitto * & Context);
static void OnMessage(mosquitto* Context, void * UserData, const mosquitto_message  * Message);

struct Message {
	uint64_t Timestamp;
	std::string Topic;
	std::vector<char> Payload;
};

BOOL CtrlHandler(DWORD CtrlType)
{
	switch (CtrlType)
	{
	case CTRL_C_EVENT:
	case CTRL_CLOSE_EVENT:
	{
		Aborted = true;
		return TRUE;
	}
	default:
		return FALSE;
	}
}

int main()
{
	//Record();
	Play();

	return 0;
}

static void Record()
{
	SetConsoleCtrlHandler(reinterpret_cast<PHANDLER_ROUTINE>(CtrlHandler), true);

	mosquitto* Context = InitMosquitto();

	mosquitto_subscribe(Context, nullptr, "HololensController/#", 2);
	mosquitto_subscribe(Context, nullptr, "TangoController/#", 2);

	do {
		mosquitto_loop(Context, 1, INT_MAX);
	} while (!Aborted);

	FinitMosquitto(Context);
}

static void Play()
{
	std::ifstream File(FileName, std::ofstream::binary);
	typedef std::vector<Message> MessageList;
	bool FillPreBatch = true;
	uint64_t ResetTime = 0;
	MessageList PreBatch;
	MessageList List;

	if (!File.is_open())
	{
		std::cout << "File not opened: " << FileName << std::endl;
		system("pause");
		return;
	}

	while (File)
	{
		Message Buffer;
		uint64_t DataLength;

		File.read(reinterpret_cast<char *>(&Buffer.Timestamp), sizeof(Buffer.Timestamp));
		std::getline(File, Buffer.Topic, '\0');
		File.read(reinterpret_cast<char *>(&DataLength), sizeof(DataLength));
		Buffer.Payload.resize(DataLength);
		File.read(Buffer.Payload.data(), Buffer.Payload.size());

		if (FillPreBatch && (Buffer.Topic == "TangoController/RoomScan/Reset"))
		{
			FillPreBatch = false;
			ResetTime = Buffer.Timestamp;
		}

		Buffer.Timestamp -= ResetTime;

		if(FillPreBatch)
			PreBatch.emplace_back(std::move(Buffer));
		else
			List.emplace_back(std::move(Buffer));
	}

	std::cout << "File ready: " << FileName << std::endl;
	system("pause");

	mosquitto* Context = InitMosquitto();
	mosquitto_threaded_set(Context, true);

	while(!Connected) 
		mosquitto_loop(Context, 1, INT_MAX);

	std::thread LoopThread = std::thread([&]() { mosquitto_loop_forever(Context, -1, 1); });

	while (true)
	{
		for (unsigned int Seconds = 1; Seconds <= 20; Seconds++)
		{
			std::cout << "Sending " << Seconds << " seconds of Data" << std::endl;

			// Send Prebatch Immediately
			for (const Message & Msg : PreBatch)
			{
				mosquitto_publish(Context, nullptr, Msg.Topic.c_str(), Msg.Payload.size(), Msg.Payload.data(), 2, false);
			}

			// Sleep one second, so RoomAlignment can process data
			std::this_thread::sleep_for(std::chrono::milliseconds(1000));

			auto Start = std::chrono::steady_clock::now();
			auto End = Start + std::chrono::seconds(Seconds);

			for (const Message & Msg : List)
			{
				auto Time = Start + std::chrono::milliseconds(Msg.Timestamp);

				// If next update is beyond current data timespan finish up
				if (Time > End)
					break;

				std::this_thread::sleep_until(Time);
				mosquitto_publish(Context, nullptr, Msg.Topic.c_str(), Msg.Payload.size(), Msg.Payload.data(), 2, false);
			}

			mosquitto_publish(Context, nullptr, "RoomAlignment/Run/Finished", 0, nullptr, 2, false);
			std::cout << "Run finished!" << std::endl;
			system("pause");
		}

		std::cout << "Test finished!"<< std::endl;
		system("pause");
	}

	FinitMosquitto(Context);
}

static mosquitto* InitMosquitto()
{
	int Result;
	mosquitto * Context = nullptr;

	Result = mosquitto_lib_init();
	if (Result != MOSQ_ERR_SUCCESS)
	{
		std::cout << "Error initializing mosquitto: " << Result << std::endl;
		return Context;
	}

	Context = mosquitto_new(nullptr, true, nullptr);
	mosquitto_message_callback_set(Context, OnMessage);
	mosquitto_connect_callback_set(Context, [](mosquitto * Context, void * UserData, int Result) { Connected = true; std::cout << "Connect Result: " << Result << std::endl; });
	mosquitto_disconnect_callback_set(Context, [](mosquitto * Context, void * UserData, int Result) {Connected = false; std::cout << "Disconnect Result: " << Result << std::endl;});
	mosquitto_subscribe_callback_set(Context, [](mosquitto * Context, void * UserData, int Result, int, const int *) {std::cout << "Subscribe Result: " << Result << std::endl;});

	Result = mosquitto_connect(Context, Hostname.c_str(), Port, 60);
	if (Result != MOSQ_ERR_SUCCESS)
	{
		std::cout << "Error connecting mosquitto: " << Result << std::endl;
		return Context;
	}

	return Context;
}

static void FinitMosquitto(mosquitto * &Context)
{
	mosquitto_disconnect(Context);
	mosquitto_loop(Context, 1, INT_MAX);
	mosquitto_destroy(Context);
	Context = nullptr;
	mosquitto_lib_cleanup();
}

static void OnMessage(mosquitto* Context, void * UserData, const mosquitto_message  * Message)
{
	static std::ofstream File(FileName, std::ofstream::binary | std::ofstream::trunc);
	static std::chrono::steady_clock::time_point Start = std::chrono::steady_clock::now();

	uint64_t Time = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - Start).count();
	File.write(reinterpret_cast<char*>(&Time), sizeof(Time));

	char * Topic = Message->topic;
	do { File.put(*Topic); } while ((*(Topic++)) != '\0');
	uint64_t Length = static_cast<uint64_t>(Message->payloadlen);
	File.write(reinterpret_cast<char*>(&Length), sizeof(Length));
	File.write(static_cast<char *>(Message->payload), Message->payloadlen);

	File.flush();
}
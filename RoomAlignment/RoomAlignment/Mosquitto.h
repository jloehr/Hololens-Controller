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
	Mosquitto();
	~Mosquitto();

	void Initialize();
	void Connect();
	void Disconnect();
	void Finalize();

private:
	typedef mosquitto * PMosquittoContext;
	typedef mosquitto_message * PMosquittoMessage;
	typedef const mosquitto_message * CPMosquittoMessage;

	PMosquittoContext MosquittoContext;
	std::thread MosquittoLoop;
};


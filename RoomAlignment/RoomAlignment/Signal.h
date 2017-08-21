/*

Copyright(C) 2017 Julian Löhr
All rights reserved.

Licensed under:
MIT License

*/
#pragma once

class Signal
{
public:
	typedef std::function<bool()> PredFunc;
	Signal(PredFunc Pred);

	void Wake();
	void Wait();

private:
	std::mutex Mutex;
	std::condition_variable_any ConVar;

	PredFunc Pred;
};


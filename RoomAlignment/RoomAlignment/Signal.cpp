/*

Copyright(C) 2017 Julian Löhr
All rights reserved.

Licensed under:
MIT License

*/
#include "stdafx.h"
#include "Signal.h"


Signal::Signal(PredFunc Pred)
	:Pred(Pred)
{
}

void Signal::Wake()
{
	std::lock_guard<std::mutex> lk(Mutex);
	ConVar.notify_all();
}

void Signal::Wait()
{
	std::lock_guard<std::mutex> lk(Mutex);
	ConVar.wait(Mutex, Pred);
}

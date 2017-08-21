/*

Copyright(C) 2017 Julian Löhr
All rights reserved.

Licensed under:
MIT License

*/
#pragma once

template<typename T>
class MonitoredQueue {
public:
	typedef std::vector<T> QueueType;

	MonitoredQueue()
		:Empty(true) {}

	bool IsEmpty() const 
	{ 
		return Empty; 
	}

	void Enqueue(const T & Data)
	{
		std::lock_guard<std::mutex> lock(Monitor);
		Queue.push_back(Data);
		Empty = false;
	}

	QueueType && Dequeue()
	{
		std::lock_guard<std::mutex> lock(Monitor);
		Empty = true;
		return std::move(Queue);
	}

	void Clear()
	{
		std::lock_guard<std::mutex> lock(Monitor);
		Empty = true;
		Queue.clear();
	}

private:
	std::mutex Monitor;
	std::atomic_bool Empty;
	std::vector<T> Queue;

};
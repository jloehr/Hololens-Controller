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

	void Enqueue(const T && Data)
	{
		std::lock_guard<std::mutex> lock(Monitor);
		Queue.push_back(Data);
		Empty = false;
		WakeDequeue.notify_one();
	}

	QueueType && Dequeue()
	{
		std::lock_guard<std::mutex> lock(Monitor);
		if (IsEmpty())
		{
			WakeDequeue.wait(Monitor);
		}
		Empty = true;
		return std::move(Queue);
	}

private:
	std::atomic_bool Empty;
	std::vector<T> Queue;

	std::mutex Monitor;
	std::condition_variable_any WakeDequeue;
};
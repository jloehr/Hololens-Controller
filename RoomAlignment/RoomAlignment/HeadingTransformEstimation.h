/*

Copyright(C) 2017 Julian Löhr
All rights reserved.

Licensed under:
MIT License

*/
#pragma once

#include "MonitoredQueue.h"

class HeadingTransformEstimation
{
public:
	typedef MonitoredQueue<std::string> MessageQueue;

	void Reset();
	bool HasBothHeadings() const;
	bool HasNewEstimation() const;
	Eigen::Matrix4f GetTransformEstimation();

	MessageQueue TangoHeadingQueue;
	MessageQueue HoloLensHeadingQueue;

private:
	bool BothHeadingsSet = false;
	Eigen::Matrix4f TangoTransform;
	Eigen::Matrix4f HoloLensTransform;
	Eigen::Matrix4f TransformEstimation;

	void UpdateTangoTransform();
	void UpdateHoloLensTransform();
	const nlohmann::json GetLatestTransformFromQueue(MessageQueue & Queue);

};


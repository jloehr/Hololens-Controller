/*

Copyright(C) 2017 Julian Löhr
All rights reserved.

Licensed under:
MIT License

*/
#include "stdafx.h"
#include "HeadingTransformEstimation.h"

bool HeadingTransformEstimation::HasBothHeadings() const
{
	// Either one estimation was already made, or both queues have messages
	return (BothHeadingsSet || (!TangoHeadingQueue.IsEmpty() && !HoloLensHeadingQueue.IsEmpty()));
}

bool HeadingTransformEstimation::HasNewEstimation() const
{
	return (!TangoHeadingQueue.IsEmpty() || !HoloLensHeadingQueue.IsEmpty());
}

Eigen::Matrix4f HeadingTransformEstimation::GetTransformEstimation()
{
	if (HasBothHeadings() && HasNewEstimation())
	{
		BothHeadingsSet = HasBothHeadings();

		UpdateTangoTransform();
		UpdateHoloLensTransform();

		TransformEstimation = TangoTransform.inverse() * HoloLensTransform;
	}

	return TransformEstimation;
}

void HeadingTransformEstimation::UpdateTangoTransform()
{
	const nlohmann::json AsJSON = GetLatestTransformFromQueue(TangoHeadingQueue);
	std::vector<float> AsVetor = AsJSON["DevicePoseTransform"];
	Eigen::Map<Eigen::Matrix<float, 4, 4, Eigen::RowMajor>> Temp(AsVetor.data());

	TangoTransform = Temp;
}

void HeadingTransformEstimation::UpdateHoloLensTransform()
{
	const nlohmann::json AsJSON = GetLatestTransformFromQueue(HoloLensHeadingQueue);

	HoloLensTransform = Eigen::Matrix4f::Identity();

	double theta = AsJSON["Angle"] * M_PI / 180;  // The angle of rotation in radians
	// X
	/*
	HoloLensTransform(1, 1) = cos(theta);
	HoloLensTransform(1, 2) = -sin(theta);
	HoloLensTransform(2, 1) = sin(theta);
	HoloLensTransform(2, 2) = cos(theta);
	*/
	// Y
	HoloLensTransform(0, 0) = cos(theta);
	HoloLensTransform(0, 2) = -sin(theta);
	HoloLensTransform(2, 0) = sin(theta);
	HoloLensTransform(2, 2) = cos(theta);
	// Z
	/*
	HoloLensTransform(0, 0) = cos(theta);
	HoloLensTransform(0, 1) = -sin(theta);
	HoloLensTransform(1, 0) = sin(theta);
	HoloLensTransform(1, 1) = cos(theta);
	*/

	HoloLensTransform(0, 3) = AsJSON["X"];
	HoloLensTransform(1, 3) = AsJSON["Y"];
	HoloLensTransform(2, 3) = AsJSON["Z"];
	HoloLensTransform(2, 3) *= -1.;
}

const nlohmann::json HeadingTransformEstimation::GetLatestTransformFromQueue(MessageQueue & Queue)
{
	std::vector<std::string> Messages = Queue.Dequeue();

	return nlohmann::json::parse(Messages.back().c_str());
}



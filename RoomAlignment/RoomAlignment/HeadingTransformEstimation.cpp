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
	HoloLensTransform <<
		AsJSON["m00"], AsJSON["m01"], AsJSON["m02"], AsJSON["m03"],
		AsJSON["m10"], AsJSON["m11"], AsJSON["m12"], AsJSON["m13"],
		AsJSON["m20"], AsJSON["m21"], AsJSON["m22"], AsJSON["m23"],
		AsJSON["m30"], AsJSON["m31"], AsJSON["m32"], AsJSON["m33"];
}

const nlohmann::json HeadingTransformEstimation::GetLatestTransformFromQueue(MessageQueue & Queue)
{
	std::vector<std::string> Messages = Queue.Dequeue();

	return nlohmann::json::parse(Messages.back().c_str());
}



//============================================================================================================
//
//
//                  Copyright (c) 2023, Qualcomm Innovation Center, Inc. All rights reserved.
//                              SPDX-License-Identifier: BSD-3-Clause
//
//============================================================================================================

#pragma once

#include <memory>
#include "Common/CompilerSpecificSIMD.h"


#include "Util/RapidRasterizer/RapidRasterizer.h"
#include "Util/RapidRasterizer/OccluderQuad.h"

namespace SDOCCommon
{
class SOCFrameInfo;
} // namespace common

namespace SOC
{

class SOCPrivate
{
public:

    SOCPrivate(const SOCPrivate &) = delete;
    SOCPrivate(SOCPrivate &&) = delete;
    SOCPrivate &operator=(const SOCPrivate &) = delete;
    SOCPrivate &operator=(SOCPrivate &&) = delete;

    // framebuffer resolution, near clip distance
    bool resize(unsigned int width, unsigned int height);
    void setNearPlane(float nearPlane);

	// start new frame
    void startNewFrame(const float *CameraPos, const float *ViewDir, const float *ViewProj);



	// batch query
    bool batchQuery(const float *bbox, unsigned int nPrim, bool *results, bool obbMode);

	void configPerformanceMode(unsigned int configValue);
	bool setConfig(unsigned int configTarget, unsigned int configValue);


    // dump depth map
	bool doDumpDepthMap(unsigned char *data, SDOCCommon::DumpImageMode mode);

#if defined(SDOC_NATIVE_DEBUG)&& defined(SDOC_NATIVE)
    // replay
    bool replay(const char *file_path, int config, int frameNum, uint64_t replaySetting, float * replayResult);
#endif

    // set approach
    void setAlgoApproach(SDOCCommon::AlgoEnum config);



	size_t getMemoryByteUsage();

	bool onFrameCaptureSet(int configValue);
	float SmallRotateDotAngleThreshold = 0.9999f; //rotating if angle dot value < 0.9999f, which means > 0.8 degree
	float LargeRotateDotAngleThreshold = 0.9962f; // rotating 5 degree

	float CameraNearDistanceThreshold = -1;
	SDOCCommon::AlgoEnum algoApproachMask = SDOCCommon::AlgoEnum::Rasterizer_FullTriangle;

	bool mResolutionChanged = false;
	uint16_t mRapidCoherentMode = 0;
	// SOC frame info
	SDOCCommon::SOCFrameInfo* m_frameInfo = nullptr;

protected:


	void setRenderType(int renderType);

public:

	SOCPrivate();
	~SOCPrivate();
	SDOCUtil::RapidRasterizer* m_rapidRasterizer;

	bool saveColorImage(unsigned char * fullImgData, const char* path);

	void socDestroy();
	bool socConfig(unsigned int width, unsigned int height, float nearPlane);

};

} // namespace SOC

//============================================================================================================
//
//
//                  Copyright (c) 2023, Qualcomm Innovation Center, Inc. All rights reserved.
//                              SPDX-License-Identifier: BSD-3-Clause
//
//============================================================================================================


#pragma once
#include <cmath>
#include <cstring>
#include "CompilerSpecificSIMD.h"
#include <vector>


#include <sys/types.h>
#include <sys/stat.h>
#include <string>

namespace common
{

	static const std::string CAPTURE_PREFIX = "SCAP";
	static const std::string CAPTURE_APPENDIX = ".cap";
	static const std::string FB_SETTING_HEADER = "Framebuffer Settings";
	static const std::string CAM_POS_HEADER = "Camera PosDir";
	static const std::string VIEW_PROJ_HEADER = "View Projection Matrix";
	static const std::string BATCHED_OCE_HEADER = "Batched Occludee";
	static const uint32_t    BBOX_STRIDE = 6;

	static constexpr bool bDumpCheckerboardImage = 0;
	static constexpr bool bDumpCheckerboardImageBlack = 0;
	static constexpr bool bDumpCheckerboardImageBinary = 0;

	static constexpr bool bCompareImoCSDOCResult = false;

#if defined(SDOC_NATIVE)
    static constexpr bool IS_ARM_PLATFORM = false;
#else
    static constexpr bool IS_ARM_PLATFORM = true;
#endif

	//reset to 1.0 for now
	//set to v1.2 2021/9/5: support resolution change, optimize query, show occludee result better
	static constexpr int VERSION_MAJOR = 1; 
	static constexpr int VERSION_SUB = 6;

	static constexpr int START_FRAME_COUNT = 2; //Frame 3 would be the first frame. 

	static constexpr bool bEnableCompressMode = 1;
	static constexpr int SuperCompressVertNum = 1 + (255 / 3);


	class FloatArray {
	public:
		static bool containSameData16(const float *a, const float*b) 
		{
			uint64_t * a64 = (uint64_t *)a;
			uint64_t * b64 = (uint64_t *)b;
			for (int i = 0; i < 8; i++)
			{
				if (a64[i] != b64[i])
				{
					return false;
				}
			}
			return true;
		}
		
		static float calculateSquareDistance3(const float *a, const float*b)
		{
			float d0 = a[0] - b[0];
			float d1 = a[1] - b[1];
			float d2 = a[2] - b[2];
			return d0 * d0 + d1 * d1 + d2 * d2;
		}
	};

    class Matrix4x4
    {
    public:
		void updateTranspose(const float *src)
		{
			Row[0] = _mm_setr_ps(src[0], src[4], src[8], src[12]);
			Row[1] = _mm_setr_ps(src[1], src[5], src[9], src[13]);
			Row[2] = _mm_setr_ps(src[2], src[6], src[10], src[14]);
			Row[3] = _mm_setr_ps(src[3], src[7], src[11], src[15]);
		}
		
		static void Multiply(const Matrix4x4 &a, const Matrix4x4 &b,  Matrix4x4 &c) 
		{
			for (unsigned int row = 0; row < 4; ++row)
			{
				__m128 curr = a.Row[row];
				__m128 internal = _mm_mul_ps(_mm_shuffle_ps_single_index(curr, 0), b.Row[0]);
				internal =  _mm_fmadd_ps(_mm_shuffle_ps_single_index(curr, 1), b.Row[1], internal);
				internal =  _mm_fmadd_ps(_mm_shuffle_ps_single_index(curr, 2), b.Row[2], internal);
				internal =  _mm_fmadd_ps(_mm_shuffle_ps_single_index(curr, 3), b.Row[3], internal);
				c.Row[row] = internal;
			}
		}
		
        __m128 Row[4];
    };

} // namespace common

//============================================================================================================
//
//
//                  Copyright (c) 2023, Qualcomm Innovation Center, Inc. All rights reserved.
//                              SPDX-License-Identifier: BSD-3-Clause
//
//============================================================================================================

#pragma once

#include <stdlib.h>
#include <new>


#include "Vec3f.h"
#ifdef SDOC_WIN
#include <intrin.h>
#endif
#ifdef SDOC_ANDROID
#include <byteswap.h> //GCC Clang
#endif

#ifdef __APPLE__
#include <libkern/OSByteOrder.h>
#endif

namespace common
{

	inline uint64_t sdoc_bswap_64(uint64_t mask)
	{
#if defined(SDOC_WIN)
		return _byteswap_uint64(mask); //bswap r64 is 2 uops on Intel CPUs
#elif defined(SDOC_ANDROID)
		return bswap_64(mask);
#else
        return _OSSwapInt64(mask);
#endif
	}
	

} // namespace common

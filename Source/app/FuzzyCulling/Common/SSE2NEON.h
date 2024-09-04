/*
** Include code taken from https://github.com/jratcliff63367/sse2neon/blob/master/SSE2NEON.h
**
** The MIT license:
**
** Permission is hereby granted, free of charge, to any person obtaining a copy
** of this software and associated documentation files (the "Software"), to deal
** in the Software without restriction, including without limitation the rights
** to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
** copies of the Software, and to permit persons to whom the Software is furnished
** to do so, subject to the following conditions:
**
** The above copyright notice and this permission notice shall be included in all
** copies or substantial portions of the Software.
** THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
** IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
** FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
** AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
** WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
** CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#ifndef SSE2NEON_H
#define SSE2NEON_H

// This header file provides a simple API translation layer
// between SSE intrinsics to their corresponding ARM NEON versions
//
// This header file does not (yet) translate *all* of the SSE intrinsics.
// Since this is in support of a specific porting effort, I have only
// included the intrinsics I needed to get my port to work.
//
// *********************************************************************************************************************
// apoty: March 17, 2017
// Current version was changed in most to fix issues and potential issues.
// All unit tests were rewritten as a part of forge lib project to cover all implemented functions.
// *********************************************************************************************************************
// Release notes for January 20, 2017 version:
//
// The unit tests have been refactored.  They no longer assert on an error, instead they return a pass/fail condition
// The unit-tests now test 10,000 random float and int values against each intrinsic.
//
// SSE2NEON now supports 95 SSE intrinsics.  39 of them have formal unit tests which have been implemented and
// fully tested on NEON/ARM.  The remaining 56 still need unit tests implemented.
//
// A struct is now defined in this header file called 'SIMDVec' which can be used by applications which
// attempt to access the contents of an _m128 struct directly.  It is important to note that accessing the __m128
//
// However, some legacy source code may try to access the contents of an __m128 struct directly so the developer
// can use the SIMDVec as an alias for it.  Any casting must be done manually by the developer, as you cannot
// cast or otherwise alias the base NEON data type for intrinsic operations.
//
// A bug was found with the _mm_shuffle_ps intrinsic.  If the shuffle permutation was not one of the ones with
// a custom/unique implementation causing it to fall through to the default shuffle implementation it was failing
// to return the correct value.  This is now fixed.
//
// A bug was found with the _mm_cvtps_epi32 intrinsic.  This converts floating point values to integers.
// It was not honoring the correct rounding mode.  In SSE the default rounding mode when converting from float to int
// is to use 'round to even' otherwise known as 'bankers rounding'.  ARMv7 did not support this feature but ARMv8 does.
// As it stands today, this header file assumes ARMv8.  If you are trying to target really old ARM devices, you may get
// a build error.
//
// Support for a number of new intrinsics was added, however, none of them yet have unit-tests to 100% confirm they are
// producing the correct results on NEON.  These unit tests will be added as soon as possible.
//
// Here is the list of new instrinsics which have been added:
//
// _mm_cvtss_f32     :  extracts the lower order floating point value from the parameter
// _mm_add_ss        : adds the scalar single - precision floating point values of a and b
// _mm_div_ps        : Divides the four single - precision, floating - point values of a and b.
// _mm_div_ss        : Divides the scalar single - precision floating point value of a by b.
// _mm_sqrt_ss       : Computes the approximation of the square root of the scalar single - precision floating point value of in.
// _mm_rsqrt_ps      : Computes the approximations of the reciprocal square roots of the four single - precision floating point values of in.
// _mm_comilt_ss     : Compares the lower single - precision floating point scalar values of a and b using a less than operation
// _mm_comigt_ss     : Compares the lower single - precision floating point scalar values of a and b using a greater than operation.
// _mm_comile_ss     :  Compares the lower single - precision floating point scalar values of a and b using a less than or equal operation.
// _mm_comige_ss     : Compares the lower single - precision floating point scalar values of a and b using a greater than or equal operation.
// _mm_comieq_ss     :  Compares the lower single - precision floating point scalar values of a and b using an equality operation.
// _mm_comineq_s     :  Compares the lower single - precision floating point scalar values of a and b using an inequality operation
// _mm_unpackhi_epi8 : Interleaves the upper 8 signed or unsigned 8 - bit integers in a with the upper 8 signed or unsigned 8 - bit integers in b.
// _mm_unpackhi_epi16:  Interleaves the upper 4 signed or unsigned 16 - bit integers in a with the upper 4 signed or unsigned 16 - bit integers in b.
//
// *********************************************************************************************************************

#define ENABLE_CPP_VERSION 1

#if defined(__GNUC__) || defined(__clang__)
#pragma push_macro("inline")
#pragma push_macro("ALIGN_STRUCT")
#define inline static inline __attribute__((always_inline))
#define ALIGN_STRUCT(x) __attribute__((aligned(x)))
#else
#error "Macro name collisions may happens with unknown compiler"
#define inline static inline
#define ALIGN_STRUCT(x) __declspec(align(x))
#endif

#include <stdint.h>
#include "arm_neon.h"

/*******************************************************/
/* MACRO for shuffle parameter for _mm_shuffle_ps().   */
/* Argument fp3 is a digit[0123] that represents the fp*/
/* from argument "b" of mm_shuffle_ps that will be     */
/* placed in fp3 of result. fp2 is the same for fp2 in */
/* result. fp1 is a digit[0123] that represents the fp */
/* from argument "a" of mm_shuffle_ps that will be     */
/* places in fp1 of result. fp0 is the same for fp0 of */
/* result                                              */
/*******************************************************/
#define _MM_SHUFFLE(fp3, fp2, fp1, fp0) \
    (((fp3) << 6) | ((fp2) << 4) | ((fp1) << 2) | ((fp0)))

/* indicate immediate constant argument in a given range */
#define __constrange(a, b) \
    const

typedef float32x4_t __m128;
typedef int32x4_t __m128i;

// ******************************************
// type-safe casting between types
// ******************************************

#define vreinterpretq_m128_f16(x) \
    vreinterpretq_f32_f16(x)

#define vreinterpretq_m128_f32(x) \
    (x)

#define vreinterpretq_m128_f64(x) \
    vreinterpretq_f32_f64(x)

#define vreinterpretq_m128_u8(x) \
    vreinterpretq_f32_u8(x)

#define vreinterpretq_m128_u16(x) \
    vreinterpretq_f32_u16(x)

#define vreinterpretq_m128_u32(x) \
    vreinterpretq_f32_u32(x)

#define vreinterpretq_m128_u64(x) \
    vreinterpretq_f32_u64(x)

#define vreinterpretq_m128_s8(x) \
    vreinterpretq_f32_s8(x)

#define vreinterpretq_m128_s16(x) \
    vreinterpretq_f32_s16(x)

#define vreinterpretq_m128_s32(x) \
    vreinterpretq_f32_s32(x)

#define vreinterpretq_m128_s64(x) \
    vreinterpretq_f32_s64(x)

#define vreinterpretq_f16_m128(x) \
    vreinterpretq_f16_f32(x)

#define vreinterpretq_f32_m128(x) \
    (x)

#define vreinterpretq_f64_m128(x) \
    vreinterpretq_f64_f32(x)

#define vreinterpretq_u8_m128(x) \
    vreinterpretq_u8_f32(x)

#define vreinterpretq_u16_m128(x) \
    vreinterpretq_u16_f32(x)

#define vreinterpretq_u32_m128(x) \
    vreinterpretq_u32_f32(x)

#define vreinterpretq_u64_m128(x) \
    vreinterpretq_u64_f32(x)

#define vreinterpretq_s8_m128(x) \
    vreinterpretq_s8_f32(x)

#define vreinterpretq_s16_m128(x) \
    vreinterpretq_s16_f32(x)

#define vreinterpretq_s32_m128(x) \
    vreinterpretq_s32_f32(x)

#define vreinterpretq_s64_m128(x) \
    vreinterpretq_s64_f32(x)

#define vreinterpretq_m128i_s8(x) \
    vreinterpretq_s32_s8(x)

#define vreinterpretq_m128i_s16(x) \
    vreinterpretq_s32_s16(x)

#define vreinterpretq_m128i_s32(x) \
    (x)

#define vreinterpretq_m128i_s64(x) \
    vreinterpretq_s32_s64(x)

#define vreinterpretq_m128i_u8(x) \
    vreinterpretq_s32_u8(x)

#define vreinterpretq_m128i_u16(x) \
    vreinterpretq_s32_u16(x)

#define vreinterpretq_m128i_u32(x) \
    vreinterpretq_s32_u32(x)

#define vreinterpretq_m128i_u64(x) \
    vreinterpretq_s32_u64(x)

#define vreinterpretq_s8_m128i(x) \
    vreinterpretq_s8_s32(x)

#define vreinterpretq_s16_m128i(x) \
    vreinterpretq_s16_s32(x)

#define vreinterpretq_s32_m128i(x) \
    (x)

#define vreinterpretq_s64_m128i(x) \
    vreinterpretq_s64_s32(x)

#define vreinterpretq_u8_m128i(x) \
    vreinterpretq_u8_s32(x)

#define vreinterpretq_u16_m128i(x) \
    vreinterpretq_u16_s32(x)

#define vreinterpretq_u32_m128i(x) \
    vreinterpretq_u32_s32(x)

#define vreinterpretq_u64_m128i(x) \
    vreinterpretq_u64_s32(x)

// union intended to allow direct access to an __m128 variable using the names that the MSVC
// compiler provides.  This union should really only be used when trying to access the members
// of the vector as integer values.  GCC/clang allow native access to the float members through
// a simple array access operator (in C since 4.6, in C++ since 4.8).
//
// Ideally direct accesses to SIMD vectors should not be used since it can cause a performance
// hit.  If it really is needed however, the original __m128 variable can be aliased with a
// pointer to this union and used to access individual components.  The use of this union should
// be hidden behind a macro that is used throughout the codebase to access the members instead
// of always declaring this type of variable.
typedef union ALIGN_STRUCT(16) SIMDVec {
    float m128_f32[4];    // as floats - do not to use this.  Added for convenience.
    int8_t m128_i8[16];   // as signed 8-bit integers.
    int16_t m128_i16[8];  // as signed 16-bit integers.
    int32_t m128_i32[4];  // as signed 32-bit integers.
    int64_t m128_i64[2];  // as signed 64-bit integers.
    uint8_t m128_u8[16];  // as unsigned 8-bit integers.
    uint16_t m128_u16[8]; // as unsigned 16-bit integers.
    uint32_t m128_u32[4]; // as unsigned 32-bit integers.
    uint64_t m128_u64[2]; // as unsigned 64-bit integers.
} SIMDVec;

// ******************************************
// Set/get methods
// ******************************************

// extracts the lower order floating point value from the parameter
inline float _mm_cvtss_f32(__m128 a)
{
    return vgetq_lane_f32(vreinterpretq_f32_m128(a), 0);
}

// Sets the 128-bit value to zero
inline __m128i _mm_setzero_si128()
{
    return vreinterpretq_m128i_s32(vdupq_n_s32(0));
}

// Clears the four single-precision, floating-point values
inline __m128 _mm_setzero_ps(void)
{
    return vreinterpretq_m128_f32(vdupq_n_f32(0));
}

// Sets the four single-precision, floating-point values to w
inline __m128 _mm_set1_ps(float _w)
{
    return vreinterpretq_m128_f32(vdupq_n_f32(_w));
}

// Sets the four single-precision, floating-point values to w
inline __m128 _mm_set_ps1(float _w)
{
    return vreinterpretq_m128_f32(vdupq_n_f32(_w));
}

// Sets the four single-precision, floating-point values to the four inputs
inline __m128 _mm_set_ps(float w, float z, float y, float x)
{
    float __attribute__((aligned(16))) data[4] = {x, y, z, w};
    return vreinterpretq_m128_f32(vld1q_f32(data));
}

// Sets the four single-precision, floating-point values to the four inputs in reverse order
inline __m128 _mm_setr_ps(float w, float z, float y, float x)
{
    float __attribute__((aligned(16))) data[4] = {w, z, y, x};
    return vreinterpretq_m128_f32(vld1q_f32(data));
}

//Sets the 4 signed 32-bit integer values in reverse order
inline __m128i _mm_setr_epi32(int i3, int i2, int i1, int i0)
{
    int32_t __attribute__((aligned(16))) data[4] = {i3, i2, i1, i0};
    return vreinterpretq_m128i_s32(vld1q_s32(data));
}

inline __m128i _mm_setr_epi8(int8_t e0, int8_t e1, int8_t e2, int8_t e3, int8_t e4, int8_t e5, int8_t e6, int8_t e7, int8_t e8, int8_t e9, int8_t e10, int8_t e11, int8_t e12, int8_t e13, int8_t e14, int8_t e15)
{
    int8_t data[16] = {e0, e1, e2, e3, e4, e5, e6, e7, e8, e9, e10, e11, e12, e13, e14, e15};
    return vreinterpretq_m128i_s8(vld1q_s8(data));
}

//Sets the 16 signed 8-bit integer values to b
inline __m128i _mm_set1_epi8(char w)
{
    return vreinterpretq_m128i_s8(vdupq_n_s8(w));
}

//Sets the 8 signed 16-bit integer values to w
inline __m128i _mm_set1_epi16(short w)
{
    return vreinterpretq_m128i_s16(vdupq_n_s16(w));
}

//Sets the 8 signed 16-bit integer values
inline __m128i _mm_set_epi16(short i7, short i6, short i5, short i4, short i3, short i2, short i1, short i0)
{
	int16_t __attribute__((aligned(16))) data[8] = { i0, i1, i2, i3, i4, i5, i6, i7 };
	return vreinterpretq_m128i_s16(vld1q_s16(data));
}
//Sets the 8 signed 16-bit integer values
inline __m128i _mm_setr_epi16(short i7, short i6, short i5, short i4, short i3, short i2, short i1, short i0)
{
	int16_t __attribute__((aligned(16))) data[8] = { i7, i6, i5, i4, i3, i2, i1, i0 };
	return vreinterpretq_m128i_s16(vld1q_s16(data));
}

// Sets the 4 signed 32-bit integer values to i
inline __m128i _mm_set1_epi32(int _i)
{
    return vreinterpretq_m128i_s32(vdupq_n_s32(_i));
}

// Sets the 4 signed 32-bit integer values
inline __m128i _mm_set_epi32(int i3, int i2, int i1, int i0)
{
    int32_t __attribute__((aligned(16))) data[4] = {i0, i1, i2, i3};
    return vreinterpretq_m128i_s32(vld1q_s32(data));
}

// Stores four single-precision, floating-point values
inline void _mm_store_ps(float *p, __m128 a)
{
    vst1q_f32(p, vreinterpretq_f32_m128(a));
}

// Stores four single-precision, floating-point values
inline void _mm_storeu_ps(float *p, __m128 a)
{
    vst1q_f32(p, vreinterpretq_f32_m128(a));
}

// Stores four 32-bit integer values as (as a __m128i value) at the address p
inline void _mm_store_si128(__m128i *p, __m128i a)
{
    vst1q_s32((int32_t *)p, vreinterpretq_s32_m128i(a));
}

// Stores four 32-bit integer values as (as a __m128i value) at the address p
inline void _mm_storeu_si128(__m128i *p, __m128i a)
{
    vst1q_s32((int32_t *)p, vreinterpretq_s32_m128i(a));
}

// Stores the lower single - precision, floating - point value
inline void _mm_store_ss(float *p, __m128 a)
{
    vst1q_lane_f32(p, vreinterpretq_f32_m128(a), 0);
}

// Reads the lower 64 bits of b and stores them into the lower 64 bits of a
inline void _mm_storel_epi64(__m128i *a, __m128i b)
{
    uint64x1_t hi = vget_high_u64(vreinterpretq_u64_m128i(*a));
    uint64x1_t lo = vget_low_u64(vreinterpretq_u64_m128i(b));
    *a = vreinterpretq_m128i_u64(vcombine_u64(lo, hi));
}

// Loads a single single-precision, floating-point value, copying it into all four words
inline __m128 _mm_load1_ps(const float *p)
{
    return vreinterpretq_m128_f32(vld1q_dup_f32(p));
}

// Loads four single-precision, floating-point values
inline __m128 _mm_load_ps(const float *p)
{
    return vreinterpretq_m128_f32(vld1q_f32(p));
}

// Loads four single-precision, floating-point values
inline __m128 _mm_loadu_ps(const float *p)
{
    // for neon, alignment doesn't matter, so _mm_load_ps and _mm_loadu_ps are equivalent for neon
    return vreinterpretq_m128_f32(vld1q_f32(p));
}

// Loads an single - precision, floating - point value into the low word and clears the upper three words
inline __m128 _mm_load_ss(const float *p)
{
    return vreinterpretq_m128_f32(vsetq_lane_f32(*p, vdupq_n_f32(0), 0));
}

// ******************************************
// Logic/Binary operations
// ******************************************

// Compares for inequality
inline __m128 _mm_cmpneq_ps(__m128 a, __m128 b)
{
    return vreinterpretq_m128_u32(vmvnq_u32(vceqq_f32(vreinterpretq_f32_m128(a), vreinterpretq_f32_m128(b))));
}

// Computes the bitwise AND-NOT of the four single-precision, floating-point values of a and b
inline __m128 _mm_andnot_ps(__m128 a, __m128 b)
{
    return vreinterpretq_m128_s32(vbicq_s32(vreinterpretq_s32_m128(b), vreinterpretq_s32_m128(a))); // *NOTE* argument swap
}

// Computes the bitwise AND of the 128-bit value in b and the bitwise NOT of the 128-bit value in a
inline __m128i _mm_andnot_si128(__m128i a, __m128i b)
{
    return vreinterpretq_m128i_s32(vbicq_s32(vreinterpretq_s32_m128i(b), vreinterpretq_s32_m128i(a))); // *NOTE* argument swap
}

// Computes the bitwise AND of the 128-bit value in a and the 128-bit value in b
inline __m128i _mm_and_si128(__m128i a, __m128i b)
{
    return vreinterpretq_m128i_s32(vandq_s32(vreinterpretq_s32_m128i(a), vreinterpretq_s32_m128i(b)));
}

// Computes the bitwise AND of the four single-precision, floating-point values of a and b
inline __m128 _mm_and_ps(__m128 a, __m128 b)
{
    return vreinterpretq_m128_s32(vandq_s32(vreinterpretq_s32_m128(a), vreinterpretq_s32_m128(b)));
}

// Computes the bitwise OR of the four single-precision, floating-point values of a and b
inline __m128 _mm_or_ps(__m128 a, __m128 b)
{
    return vreinterpretq_m128_s32(vorrq_s32(vreinterpretq_s32_m128(a), vreinterpretq_s32_m128(b)));
}

// Computes bitwise EXOR (exclusive-or) of the four single-precision, floating-point values of a and b
inline __m128 _mm_xor_ps(__m128 a, __m128 b)
{
    return vreinterpretq_m128_s32(veorq_s32(vreinterpretq_s32_m128(a), vreinterpretq_s32_m128(b)));
}

// Computes the bitwise OR of the 128-bit value in a and the 128-bit value in b
inline __m128i _mm_or_si128(__m128i a, __m128i b)
{
    return vreinterpretq_m128i_s32(vorrq_s32(vreinterpretq_s32_m128i(a), vreinterpretq_s32_m128i(b)));
}

// Computes the bitwise XOR of the 128-bit value in a and the 128-bit value in b
inline __m128i _mm_xor_si128(__m128i a, __m128i b)
{
    return vreinterpretq_m128i_s32(veorq_s32(vreinterpretq_s32_m128i(a), vreinterpretq_s32_m128i(b)));
}

// NEON does not provide this method
// Creates a 4-bit mask from the most significant bits of the four single-precision, floating-point values
inline int _mm_movemask_ps(__m128 a)
{
	uint32x4_t input = vreinterpretq_u32_m128(a);
#if defined(__aarch64__)
	static const int32x4_t shift = { 0, 1, 2, 3 };
	uint32x4_t tmp = vshrq_n_u32(input, 31);
	return vaddvq_u32(vshlq_u32(tmp, shift));
#else
	// Uses the exact same method as _mm_movemask_epi8, see that for details.
	// Shift out everything but the sign bits with a 32-bit unsigned shift
	// right.
	uint64x2_t high_bits = vreinterpretq_u64_u32(vshrq_n_u32(input, 31));
	// Merge the two pairs together with a 64-bit unsigned shift right + add.
	uint8x16_t paired =
		vreinterpretq_u8_u64(vsraq_n_u64(high_bits, high_bits, 31));
	// Extract the result.
	return vgetq_lane_u8(paired, 0) | (vgetq_lane_u8(paired, 8) << 2);
#endif

////#if ENABLE_CPP_VERSION // I am not yet convinced that the NEON version is faster than the C version of this
////    uint32x4_t &ia = *(uint32x4_t *)&a;
////    return (ia[0] >> 31) | ((ia[1] >> 30) & 2) | ((ia[2] >> 29) & 4) | ((ia[3] >> 28) & 8);
////#else
////    static const uint32x4_t movemask = {1, 2, 4, 8};
////    static const uint32x4_t highbit = {0x80000000, 0x80000000, 0x80000000, 0x80000000};
////    uint32x4_t t0 = vreinterpretq_u32_m128(a);
////    uint32x4_t t1 = vtstq_u32(t0, highbit);
////    uint32x4_t t2 = vandq_u32(t1, movemask);
////    uint32x2_t t3 = vorr_u32(vget_low_u32(t2), vget_high_u32(t2));
////    return vget_lane_u32(t3, 0) | vget_lane_u32(t3, 1);
////#endif
}

inline bool _mm_same_sign0(__m128 a)
{
	//return _mm_movemask_ps(a) == 0;
	uint32x4_t &ia = *(uint32x4_t *)&a;
	return   ! (((ia[0] | ia[1]) | (ia[2] | ia[3])) >> 31);
}

inline bool _mm_same_sign1_soc(__m128 a)
{
	//return _mm_movemask_ps(a) == 0;
	uint32x4_t &ia = *(uint32x4_t *)&a;
	return ((ia[0] & ia[1])  & (ia[2] & ia[3]) )>> 31;
}


//inline bool _mm_same_sign0_firstThree_soc(__m128 a)
//{
//	uint32x4_t &ia = *(uint32x4_t *)&a;
//	return   !((ia[0] | ia[1] | ia[2]) >> 31);
//}

//inline bool _mm_any_sign0_firstThree_soc(__m128 a)
//{
//	uint32x4_t &ia = *(uint32x4_t *)&a;
//	return   (ia[0] & ia[1] & ia[2]) >> 31;
//}
//inline bool _mm_any_sign0_soc(__m128 a)
//{
//	uint32x4_t &ia = *(uint32x4_t *)&a;
//	return   (ia[0] & ia[1] & ia[2] & ia[3]) >> 31;
//}


inline bool _mm_same_sign1_firstThree_soc(__m128 a)
{
	uint32x4_t &ia = *(uint32x4_t *)&a;
	return   (ia[0] & ia[1] & ia[2]) >> 31;
}


// Takes the upper 64 bits of a and places it in the low end of the result
// Takes the lower 64 bits of b and places it into the high end of the result.
inline __m128 _mm_shuffle_ps_1032(__m128 a, __m128 b)
{
    float32x2_t a32 = vget_high_f32(vreinterpretq_f32_m128(a));
    float32x2_t b10 = vget_low_f32(vreinterpretq_f32_m128(b));
    return vreinterpretq_m128_f32(vcombine_f32(a32, b10));
}

// takes the lower two 32-bit values from a and swaps them and places in high end of result
// takes the higher two 32 bit values from b and swaps them and places in low end of result.
inline __m128 _mm_shuffle_ps_2301(__m128 a, __m128 b)
{
    float32x2_t a01 = vrev64_f32(vget_low_f32(vreinterpretq_f32_m128(a)));
    float32x2_t b23 = vrev64_f32(vget_high_f32(vreinterpretq_f32_m128(b)));
    return vreinterpretq_m128_f32(vcombine_f32(a01, b23));
}

inline __m128 _mm_shuffle_ps_0123(__m128 a, __m128 b)
{
    float32x2_t a23 = vrev64_f32(vget_high_f32(vreinterpretq_f32_m128(a)));
    float32x2_t b01 = vrev64_f32(vget_low_f32(vreinterpretq_f32_m128(b)));
    return vreinterpretq_m128_f32(vcombine_f32(a23, b01));
}

inline __m128 _mm_shuffle_ps_0321(__m128 a, __m128 b)
{
    float32x2_t a21 = vget_high_f32(vextq_f32(vreinterpretq_f32_m128(a), vreinterpretq_f32_m128(a), 3));
    float32x2_t b03 = vget_low_f32(vextq_f32(vreinterpretq_f32_m128(b), vreinterpretq_f32_m128(b), 3));
    return vreinterpretq_m128_f32(vcombine_f32(a21, b03));
}

inline __m128 _mm_shuffle_ps_2103(__m128 a, __m128 b)
{
    float32x2_t a03 = vget_low_f32(vextq_f32(vreinterpretq_f32_m128(a), vreinterpretq_f32_m128(a), 3));
    float32x2_t b21 = vget_high_f32(vextq_f32(vreinterpretq_f32_m128(b), vreinterpretq_f32_m128(b), 3));
    return vreinterpretq_m128_f32(vcombine_f32(a03, b21));
}

inline __m128 _mm_shuffle_ps_1010(__m128 a, __m128 b)
{
	float32x2_t a10 = vget_low_f32(vreinterpretq_f32_m128(a));
	float32x2_t b10 = vget_low_f32(vreinterpretq_f32_m128(b));
	return vreinterpretq_m128_f32(vcombine_f32(a10, b10));
}

inline __m128 _mm_shuffle_ps_1001(__m128 a, __m128 b)
{
    float32x2_t a01 = vrev64_f32(vget_low_f32(vreinterpretq_f32_m128(a)));
    float32x2_t b10 = vget_low_f32(vreinterpretq_f32_m128(b));
    return vreinterpretq_m128_f32(vcombine_f32(a01, b10));
}

inline __m128 _mm_shuffle_ps_0101(__m128 a, __m128 b)
{
    float32x2_t a01 = vrev64_f32(vget_low_f32(vreinterpretq_f32_m128(a)));
    float32x2_t b01 = vrev64_f32(vget_low_f32(vreinterpretq_f32_m128(b)));
    return vreinterpretq_m128_f32(vcombine_f32(a01, b01));
}

// keeps the low 64 bits of b in the low and puts the high 64 bits of a in the high
inline __m128 _mm_shuffle_ps_3210(__m128 a, __m128 b)
{
    float32x2_t a10 = vget_low_f32(vreinterpretq_f32_m128(a));
    float32x2_t b32 = vget_high_f32(vreinterpretq_f32_m128(b));
    return vreinterpretq_m128_f32(vcombine_f32(a10, b32));
}

inline __m128 _mm_shuffle_ps_0011(__m128 a, __m128 b)
{
    float32x2_t a11 = vdup_lane_f32(vget_low_f32(vreinterpretq_f32_m128(a)), 1);
    float32x2_t b00 = vdup_lane_f32(vget_low_f32(vreinterpretq_f32_m128(b)), 0);
    return vreinterpretq_m128_f32(vcombine_f32(a11, b00));
}

inline __m128 _mm_shuffle_ps_0022(__m128 a, __m128 b)
{
    float32x2_t a22 = vdup_lane_f32(vget_high_f32(vreinterpretq_f32_m128(a)), 0);
    float32x2_t b00 = vdup_lane_f32(vget_low_f32(vreinterpretq_f32_m128(b)), 0);
    return vreinterpretq_m128_f32(vcombine_f32(a22, b00));
}

inline __m128 _mm_shuffle_ps_2200(__m128 a, __m128 b)
{
    float32x2_t a00 = vdup_lane_f32(vget_low_f32(vreinterpretq_f32_m128(a)), 0);
    float32x2_t b22 = vdup_lane_f32(vget_high_f32(vreinterpretq_f32_m128(b)), 0);
    return vreinterpretq_m128_f32(vcombine_f32(a00, b22));
}

inline __m128 _mm_shuffle_ps_3202(__m128 a, __m128 b)
{
    float32_t a0 = vgetq_lane_f32(vreinterpretq_f32_m128(a), 0);
    float32x2_t a22 = vdup_lane_f32(vget_high_f32(vreinterpretq_f32_m128(a)), 0);
    float32x2_t a02 = vset_lane_f32(a0, a22, 1); /* apoty: TODO: use vzip ?*/
    float32x2_t b32 = vget_high_f32(vreinterpretq_f32_m128(b));
    return vreinterpretq_m128_f32(vcombine_f32(a02, b32));
}

inline __m128 _mm_shuffle_ps_1133(__m128 a, __m128 b)
{
    float32x2_t a33 = vdup_lane_f32(vget_high_f32(vreinterpretq_f32_m128(a)), 1);
    float32x2_t b11 = vdup_lane_f32(vget_low_f32(vreinterpretq_f32_m128(b)), 1);
    return vreinterpretq_m128_f32(vcombine_f32(a33, b11));
}

inline __m128 _mm_shuffle_ps_2010(__m128 a, __m128 b)
{
    float32x2_t a10 = vget_low_f32(vreinterpretq_f32_m128(a));
    float32_t b2 = vgetq_lane_f32(vreinterpretq_f32_m128(b), 2);
    float32x2_t b00 = vdup_lane_f32(vget_low_f32(vreinterpretq_f32_m128(b)), 0);
    float32x2_t b20 = vset_lane_f32(b2, b00, 1);
    return vreinterpretq_m128_f32(vcombine_f32(a10, b20));
}

inline __m128 _mm_shuffle_ps_2001(__m128 a, __m128 b)
{
    float32x2_t a01 = vrev64_f32(vget_low_f32(vreinterpretq_f32_m128(a)));
    float32_t b2 = vgetq_lane_f32(b, 2);
    float32x2_t b00 = vdup_lane_f32(vget_low_f32(vreinterpretq_f32_m128(b)), 0);
    float32x2_t b20 = vset_lane_f32(b2, b00, 1);
    return vreinterpretq_m128_f32(vcombine_f32(a01, b20));
}

inline __m128 _mm_shuffle_ps_2032(__m128 a, __m128 b)
{
    float32x2_t a32 = vget_high_f32(vreinterpretq_f32_m128(a));
    float32_t b2 = vgetq_lane_f32(b, 2);
    float32x2_t b00 = vdup_lane_f32(vget_low_f32(vreinterpretq_f32_m128(b)), 0);
    float32x2_t b20 = vset_lane_f32(b2, b00, 1);
    return vreinterpretq_m128_f32(vcombine_f32(a32, b20));
}

////inline __m128 _mm_shuffle_ps_3232_soc(__m128 a, __m128 b)
////{
////	float32x2_t a32 = vget_high_f32(vreinterpretq_f32_m128(a));
////	float32x2_t b32 = vget_high_f32(vreinterpretq_f32_m128(b));
////	return vreinterpretq_m128_f32(vcombine_f32(a32, b32));
////}
////inline __m128 _mm_shuffle_ps_1010_soc(__m128 a, __m128 b)
////{
////	float32x2_t a10 = vget_low_f32(vreinterpretq_f32_m128(a));
////	float32x2_t b10 = vget_low_f32(vreinterpretq_f32_m128(b));
////	return vreinterpretq_m128_f32(vcombine_f32(a10, b10));
////}
// NEON does not support a general purpose permute intrinsic
// Currently I am not sure whether the C implementation is faster or slower than the NEON version.
// Note, this has to be expanded as a template because the shuffle value must be an immediate value.
// The same is true on SSE as well.
// Selects four specific single-precision, floating-point values from a and b, based on the mask i.
#if ENABLE_CPP_VERSION // I am not convinced that the NEON version is faster than the C version yet.
inline __m128 _mm_shuffle_ps_default(__m128 a, __m128 b, __constrange(0, 255) int imm)
{
    __m128 ret;
    ret[0] = a[imm & 0x3];
    ret[1] = a[(imm >> 2) & 0x3];
    ret[2] = b[(imm >> 4) & 0x03];
    ret[3] = b[(imm >> 6) & 0x03];
    return ret;
}
#else
#define _mm_shuffle_ps_default(a, b, imm)                                                            \
    ({                                                                                               \
        float32x4_t ret;                                                                             \
        ret = vmovq_n_f32(vgetq_lane_f32(vreinterpretq_f32_m128(a), (imm)&0x3));                     \
        ret = vsetq_lane_f32(vgetq_lane_f32(vreinterpretq_f32_m128(a), ((imm) >> 2) & 0x3), ret, 1); \
        ret = vsetq_lane_f32(vgetq_lane_f32(vreinterpretq_f32_m128(b), ((imm) >> 4) & 0x3), ret, 2); \
        ret = vsetq_lane_f32(vgetq_lane_f32(vreinterpretq_f32_m128(b), ((imm) >> 6) & 0x3), ret, 3); \
        vreinterpretq_m128_f32(ret);                                                                 \
    })
#endif

//inline __m128 _mm_shuffle_ps(__m128 a, __m128 b, __constrange(0,255) int imm)
// ***** Can be add 3232 and 1010****
inline __m128 _mm_shuffle_ps(const __m128 &a, const __m128 &b, unsigned int imm)
{
    __m128 ret = _mm_setzero_ps();
    ret[0] = a[imm & 0x3];
    ret[1] = a[(imm >> 2) & 0x3];
    ret[2] = b[(imm >> 4) & 0x03];
    ret[3] = b[(imm >> 6) & 0x03];
    return ret;
}

// Single float shuffle
inline __m128 _mm_shuffle_ps_single_index(__m128 a, int idx)
{
    return _mm_set1_ps(a[idx]);
}

// Takes the upper 64 bits of a and places it in the low end of the result
// Takes the lower 64 bits of a and places it into the high end of the result.
inline __m128i _mm_shuffle_epi_1032(__m128i a)
{
    int32x2_t a32 = vget_high_s32(vreinterpretq_s32_m128i(a));
    int32x2_t a10 = vget_low_s32(vreinterpretq_s32_m128i(a));
    return vreinterpretq_m128i_s32(vcombine_s32(a32, a10));
}

// takes the lower two 32-bit values from a and swaps them and places in low end of result
// takes the higher two 32 bit values from a and swaps them and places in high end of result.
inline __m128i _mm_shuffle_epi_2301(__m128i a)
{
    int32x2_t a01 = vrev64_s32(vget_low_s32(vreinterpretq_s32_m128i(a)));
    int32x2_t a23 = vrev64_s32(vget_high_s32(vreinterpretq_s32_m128i(a)));
    return vreinterpretq_m128i_s32(vcombine_s32(a01, a23));
}

// rotates the least significant 32 bits into the most signficant 32 bits, and shifts the rest down
inline __m128i _mm_shuffle_epi_0321(__m128i a)
{
    return vreinterpretq_m128i_s32(vextq_s32(vreinterpretq_s32_m128i(a), vreinterpretq_s32_m128i(a), 1));
}

// rotates the most significant 32 bits into the least signficant 32 bits, and shifts the rest up
inline __m128i _mm_shuffle_epi_2103(__m128i a)
{
    return vreinterpretq_m128i_s32(vextq_s32(vreinterpretq_s32_m128i(a), vreinterpretq_s32_m128i(a), 3));
}

// gets the lower 64 bits of a, and places it in the upper 64 bits
// gets the lower 64 bits of a and places it in the lower 64 bits
inline __m128i _mm_shuffle_epi_1010(__m128i a)
{
    int32x2_t a10 = vget_low_s32(vreinterpretq_s32_m128i(a));
    return vreinterpretq_m128i_s32(vcombine_s32(a10, a10));
}

// gets the lower 64 bits of a, swaps the 0 and 1 elements, and places it in the lower 64 bits
// gets the lower 64 bits of a, and places it in the upper 64 bits
inline __m128i _mm_shuffle_epi_1001(__m128i a)
{
    int32x2_t a01 = vrev64_s32(vget_low_s32(vreinterpretq_s32_m128i(a)));
    int32x2_t a10 = vget_low_s32(vreinterpretq_s32_m128i(a));
    return vreinterpretq_m128i_s32(vcombine_s32(a01, a10));
}

// gets the lower 64 bits of a, swaps the 0 and 1 elements and places it in the upper 64 bits
// gets the lower 64 bits of a, swaps the 0 and 1 elements, and places it in the lower 64 bits
inline __m128i _mm_shuffle_epi_0101(__m128i a)
{
    int32x2_t a01 = vrev64_s32(vget_low_s32(vreinterpretq_s32_m128i(a)));
    return vreinterpretq_m128i_s32(vcombine_s32(a01, a01));
}

inline __m128i _mm_shuffle_epi_2211(__m128i a)
{
    int32x2_t a11 = vdup_lane_s32(vget_low_s32(vreinterpretq_s32_m128i(a)), 1);
    int32x2_t a22 = vdup_lane_s32(vget_high_s32(vreinterpretq_s32_m128i(a)), 0);
    return vreinterpretq_m128i_s32(vcombine_s32(a11, a22));
}

inline __m128i _mm_shuffle_epi_0122(__m128i a)
{
    int32x2_t a22 = vdup_lane_s32(vget_high_s32(vreinterpretq_s32_m128i(a)), 0);
    int32x2_t a01 = vrev64_s32(vget_low_s32(vreinterpretq_s32_m128i(a)));
    return vreinterpretq_m128i_s32(vcombine_s32(a22, a01));
}

inline __m128i _mm_shuffle_epi_3332(__m128i a)
{
    int32x2_t a32 = vget_high_s32(vreinterpretq_s32_m128i(a));
    int32x2_t a33 = vdup_lane_s32(vget_high_s32(vreinterpretq_s32_m128i(a)), 1);
    return vreinterpretq_m128i_s32(vcombine_s32(a32, a33));
}

//inline __m128i _mm_shuffle_epi32_default(__m128i a, __constrange(0,255) int imm)
#if ENABLE_CPP_VERSION
inline __m128i _mm_shuffle_epi32_default(__m128i a, __constrange(0, 255) int imm)
{
    __m128i ret;
    ret[0] = a[imm & 0x3];
    ret[1] = a[(imm >> 2) & 0x3];
    ret[2] = a[(imm >> 4) & 0x03];
    ret[3] = a[(imm >> 6) & 0x03];
    return ret;
}
#else
#define _mm_shuffle_epi32_default(a, imm)                                                             \
    ({                                                                                                \
        int32x4_t ret;                                                                                \
        ret = vmovq_n_s32(vgetq_lane_s32(vreinterpretq_s32_m128i(a), (imm)&0x3));                     \
        ret = vsetq_lane_s32(vgetq_lane_s32(vreinterpretq_s32_m128i(a), ((imm) >> 2) & 0x3), ret, 1); \
        ret = vsetq_lane_s32(vgetq_lane_s32(vreinterpretq_s32_m128i(a), ((imm) >> 4) & 0x3), ret, 2); \
        ret = vsetq_lane_s32(vgetq_lane_s32(vreinterpretq_s32_m128i(a), ((imm) >> 6) & 0x3), ret, 3); \
        vreinterpretq_m128i_s32(ret);                                                                 \
    })
#endif

//inline __m128i _mm_shuffle_epi32_splat(__m128i a, __constrange(0,255) int imm)
#if defined(__aarch64__)
#define _mm_shuffle_epi32_splat(a, imm)                                              \
    ({                                                                               \
        vreinterpretq_m128i_s32(vdupq_laneq_s32(vreinterpretq_s32_m128i(a), (imm))); \
    })
#else
#define _mm_shuffle_epi32_splat(a, imm)                                                          \
    ({                                                                                           \
        vreinterpretq_m128i_s32(vdupq_n_s32(vgetq_lane_s32(vreinterpretq_s32_m128i(a), (imm)))); \
    })
#endif

// Shuffles the 4 signed or unsigned 32-bit integers in a as specified by imm
//inline __m128i _mm_shuffle_epi32(__m128i a, __constrange(0,255) int imm)
#define _mm_shuffle_epi32(a, imm)                        \
    ({                                                   \
        __m128i ret;                                     \
        switch (imm)                                     \
        {                                                \
        case _MM_SHUFFLE(1, 0, 3, 2):                    \
            ret = _mm_shuffle_epi_1032((a));             \
            break;                                       \
        case _MM_SHUFFLE(2, 3, 0, 1):                    \
            ret = _mm_shuffle_epi_2301((a));             \
            break;                                       \
        case _MM_SHUFFLE(0, 3, 2, 1):                    \
            ret = _mm_shuffle_epi_0321((a));             \
            break;                                       \
        case _MM_SHUFFLE(2, 1, 0, 3):                    \
            ret = _mm_shuffle_epi_2103((a));             \
            break;                                       \
        case _MM_SHUFFLE(1, 0, 1, 0):                    \
            ret = _mm_shuffle_epi_1010((a));             \
            break;                                       \
        case _MM_SHUFFLE(1, 0, 0, 1):                    \
            ret = _mm_shuffle_epi_1001((a));             \
            break;                                       \
        case _MM_SHUFFLE(0, 1, 0, 1):                    \
            ret = _mm_shuffle_epi_0101((a));             \
            break;                                       \
        case _MM_SHUFFLE(2, 2, 1, 1):                    \
            ret = _mm_shuffle_epi_2211((a));             \
            break;                                       \
        case _MM_SHUFFLE(0, 1, 2, 2):                    \
            ret = _mm_shuffle_epi_0122((a));             \
            break;                                       \
        case _MM_SHUFFLE(3, 3, 3, 2):                    \
            ret = _mm_shuffle_epi_3332((a));             \
            break;                                       \
        case _MM_SHUFFLE(0, 0, 0, 0):                    \
            ret = _mm_shuffle_epi32_splat((a), 0);       \
            break;                                       \
        case _MM_SHUFFLE(1, 1, 1, 1):                    \
            ret = _mm_shuffle_epi32_splat((a), 1);       \
            break;                                       \
        case _MM_SHUFFLE(2, 2, 2, 2):                    \
            ret = _mm_shuffle_epi32_splat((a), 2);       \
            break;                                       \
        case _MM_SHUFFLE(3, 3, 3, 3):                    \
            ret = _mm_shuffle_epi32_splat((a), 3);       \
            break;                                       \
        default:                                         \
            ret = _mm_shuffle_epi32_default((a), (imm)); \
            break;                                       \
        }                                                \
        ret;                                             \
    })

// Shuffles the upper 4 signed or unsigned 16 - bit integers in a as specified by imm
//inline __m128i _mm_shufflehi_epi16_function(__m128i a, __constrange(0,255) int imm)
#define _mm_shufflehi_epi16_function(a, imm)                                       \
    ({                                                                             \
        int16x8_t ret = vreinterpretq_s16_s32(a);                                  \
        int16x4_t highBits = vget_high_s16(ret);                                   \
        ret = vsetq_lane_s16(vget_lane_s16(highBits, (imm)&0x3), ret, 4);          \
        ret = vsetq_lane_s16(vget_lane_s16(highBits, ((imm) >> 2) & 0x3), ret, 5); \
        ret = vsetq_lane_s16(vget_lane_s16(highBits, ((imm) >> 4) & 0x3), ret, 6); \
        ret = vsetq_lane_s16(vget_lane_s16(highBits, ((imm) >> 6) & 0x3), ret, 7); \
        vreinterpretq_s32_s16(ret);                                                \
    })

//inline __m128i _mm_shufflehi_epi16(__m128i a, __constrange(0,255) int imm)
#define _mm_shufflehi_epi16(a, imm) \
    _mm_shufflehi_epi16_function((a), (imm))

//Shifts the 8 signed or unsigned 16-bit integers in a left by count bits while shifting in zeros
#define _mm_slli_epi16(a, imm)                                                             \
    ({                                                                                     \
        __m128i ret;                                                                       \
        if ((imm) <= 0)                                                                    \
        {                                                                                  \
            ret = a;                                                                       \
        }                                                                                  \
        else if ((imm) > 31)                                                               \
        {                                                                                  \
            ret = _mm_setzero_si128();                                                     \
        }                                                                                  \
        else                                                                               \
        {                                                                                  \
            ret = vreinterpretq_m128i_s16(vshlq_n_s16(vreinterpretq_s16_m128i(a), (imm))); \
        }                                                                                  \
        ret;                                                                               \
    })

#define _mm_slli_epi8(a, imm)                                                             \
    ({                                                                                     \
        __m128i ret;                                                                       \
        if ((imm) <= 0)                                                                    \
        {                                                                                  \
            ret = a;                                                                       \
        }                                                                                  \
        else if ((imm) > 31)                                                               \
        {                                                                                  \
            ret = _mm_setzero_si128();                                                     \
        }                                                                                  \
        else                                                                               \
        {                                                                                  \
            ret = vreinterpretq_m128i_s8(vshlq_n_s8(vreinterpretq_s8_m128i(a), (imm))); \
        }                                                                                  \
        ret;                                                                               \
    })

// Shifts the 4 signed or unsigned 32-bit integers in a left by count bits while shifting in zeros
//inline __m128i _mm_slli_epi32(__m128i a, __constrange(0,255) int imm)
#define _mm_slli_epi32(a, imm)                                                             \
    ({                                                                                     \
        __m128i ret;                                                                       \
        if ((imm) <= 0)                                                                    \
        {                                                                                  \
            ret = a;                                                                       \
        }                                                                                  \
        else if ((imm) > 31)                                                               \
        {                                                                                  \
            ret = _mm_setzero_si128();                                                     \
        }                                                                                  \
        else                                                                               \
        {                                                                                  \
            ret = vreinterpretq_m128i_s32(vshlq_n_s32(vreinterpretq_s32_m128i(a), (imm))); \
        }                                                                                  \
        ret;                                                                               \
    })

// Shifts the 8 signed or unsigned 16-bit integers in a right by count bits while shifting in zeros.
#define _mm_srli_epi16(a, imm)                                                             \
    ({                                                                                     \
        __m128i ret;                                                                       \
        if ((imm) <= 0)                                                                    \
        {                                                                                  \
            ret = a;                                                                       \
        }                                                                                  \
        else if ((imm) > 31)                                                               \
        {                                                                                  \
            ret = _mm_setzero_si128();                                                     \
        }                                                                                  \
        else                                                                               \
        {                                                                                  \
            ret = vreinterpretq_m128i_u16(vshrq_n_u16(vreinterpretq_u16_m128i(a), (imm))); \
        }                                                                                  \
        ret;                                                                               \
    })


// Shifts the 8 signed or unsigned 16-bit integers in a right by count bits while shifting in zeros.
#define _mm_srli_epi8(a, imm)                                                             \
    ({                                                                                     \
        __m128i ret;                                                                       \
        if ((imm) <= 0)                                                                    \
        {                                                                                  \
            ret = a;                                                                       \
        }                                                                                  \
        else if ((imm) > 31)                                                               \
        {                                                                                  \
            ret = _mm_setzero_si128();                                                     \
        }                                                                                  \
        else                                                                               \
        {                                                                                  \
            ret = vreinterpretq_m128i_u8(vshrq_n_u8(vreinterpretq_u8_m128i(a), (imm))); \
        }                                                                                  \
        ret;                                                                               \
    })


//Shifts the 4 signed or unsigned 32-bit integers in a right by count bits while shifting in zeros
//inline __m128i _mm_srli_epi32(__m128i a, __constrange(0,255) int imm)
#define _mm_srli_epi32(a, imm)                                                             \
    ({                                                                                     \
        __m128i ret;                                                                       \
        if ((imm) <= 0)                                                                    \
        {                                                                                  \
            ret = a;                                                                       \
        }                                                                                  \
        else if ((imm) > 31)                                                               \
        {                                                                                  \
            ret = _mm_setzero_si128();                                                     \
        }                                                                                  \
        else                                                                               \
        {                                                                                  \
            ret = vreinterpretq_m128i_u32(vshrq_n_u32(vreinterpretq_u32_m128i(a), (imm))); \
        }                                                                                  \
        ret;                                                                               \
    })

// Shifts the 4 signed 32 - bit integers in a right by count bits while shifting in the sign bit
//inline __m128i _mm_srai_epi32(__m128i a, __constrange(0,255) int imm)
#define _mm_srai_epi32(a, imm)                                                             \
    ({                                                                                     \
        __m128i ret;                                                                       \
        if ((imm) <= 0)                                                                    \
        {                                                                                  \
            ret = a;                                                                       \
        }                                                                                  \
        else if ((imm) > 31)                                                               \
        {                                                                                  \
            ret = vreinterpretq_m128i_s32(vshrq_n_s32(vreinterpretq_s32_m128i(a), 16));    \
            ret = vreinterpretq_m128i_s32(vshrq_n_s32(vreinterpretq_s32_m128i(ret), 16));  \
        }                                                                                  \
        else                                                                               \
        {                                                                                  \
            ret = vreinterpretq_m128i_s32(vshrq_n_s32(vreinterpretq_s32_m128i(a), (imm))); \
        }                                                                                  \
        ret;                                                                               \
    })

// Shifts the 128 - bit value in a right by imm bytes while shifting in zeros.imm must be an immediate
//inline _mm_srli_si128(__m128i a, __constrange(0,255) int imm)
#define _mm_srli_si128(a, imm)                                                                       \
    ({                                                                                               \
        __m128i ret;                                                                                 \
        if ((imm) <= 0)                                                                              \
        {                                                                                            \
            ret = a;                                                                                 \
        }                                                                                            \
        else if ((imm) > 15)                                                                         \
        {                                                                                            \
            ret = _mm_setzero_si128();                                                               \
        }                                                                                            \
        else                                                                                         \
        {                                                                                            \
            ret = vreinterpretq_m128i_s8(vextq_s8(vreinterpretq_s8_m128i(a), vdupq_n_s8(0), (imm))); \
        }                                                                                            \
        ret;                                                                                         \
    })

// Shifts the 128-bit value in a left by imm bytes while shifting in zeros. imm must be an immediate
//inline __m128i _mm_slli_si128(__m128i a, __constrange(0,255) int imm)
#define _mm_slli_si128(a, imm)                                                                            \
    ({                                                                                                    \
        __m128i ret;                                                                                      \
        if ((imm) <= 0)                                                                                   \
        {                                                                                                 \
            ret = a;                                                                                      \
        }                                                                                                 \
        else if ((imm) > 15)                                                                              \
        {                                                                                                 \
            ret = _mm_setzero_si128();                                                                    \
        }                                                                                                 \
        else                                                                                              \
        {                                                                                                 \
            ret = vreinterpretq_m128i_s8(vextq_s8(vdupq_n_s8(0), vreinterpretq_s8_m128i(a), 16 - (imm))); \
        }                                                                                                 \
        ret;                                                                                              \
    })


inline int _mm_movemask_epi16_soc(__m128i a)
{
#if __LITTLE_ENDIAN__
	uint16x8_t input = vreinterpretq_u16_m128i(a);
	uint16x8_t high_bits = vshrq_n_u16(input, 15);
	uint64x2_t paired32 = vsraq_n_u32(high_bits, high_bits, 15);
	uint8x16_t paired64 = vsraq_n_u64(paired32, paired32, 30);
	return vgetq_lane_u8(paired64, 0) | ((int)vgetq_lane_u8(paired64, 8) << 4);
#else
	uint8x16_t ia = vreinterpretq_u8_m128i(a);
	static const uint8x16_t mask_and = vdupq_n_u8(0x80);
	ia = vandq_u8(ia, mask_and);

	int low = (ia[0] >> 7) |  (ia[2] >> 6) | (ia[4] >> 5) | (ia[6] >> 4);
	int high = (ia[8] >> 7) |  (ia[10] >> 6) |(ia[12] >> 5) | (ia[14] >> 4);

	return ((high << 4) | (low & 0xF));
#endif
}

inline int _mm_test_all_zeros(__m128i a, __m128i mask)
{
    int64x2_t a_and_mask =
        vandq_s64(vreinterpretq_s64_m128i(a), vreinterpretq_s64_m128i(mask));
    return (vgetq_lane_s64(a_and_mask, 0) | vgetq_lane_s64(a_and_mask, 1)) ? 0
                                                                           : 1;
}

// Create mask from the most significant bit of each 8-bit element in a, and store the result in dst.
// NEON does not provide a version of this function, here is an article about some ways to repro the results.
// Creates a 16-bit mask from the most significant bits of the 16 signed or unsigned 8-bit integers in a and zero extends the upper bits
inline int _mm_movemask_epi8(__m128i a)
{
#if defined(__aarch64__)
	uint8x16_t input = vreinterpretq_u8_m128i(a);
	const int8_t ALIGN_STRUCT(16)
		xr[16] = { -7, -6, -5, -4, -3, -2, -1, 0, -7, -6, -5, -4, -3, -2, -1, 0 };
	const uint8x16_t mask_and = vdupq_n_u8(0x80);
	const int8x16_t mask_shift = vld1q_s8(xr);
	const uint8x16_t mask_result =
		vshlq_u8(vandq_u8(input, mask_and), mask_shift);
	uint8x8_t lo = vget_low_u8(mask_result);
	uint8x8_t hi = vget_high_u8(mask_result);

	return vaddv_u8(lo) + (vaddv_u8(hi) << 8);
#elif __LITTLE_ENDIAN__

	// Use increasingly wide shifts+adds to collect the sign bits
	// together.
	// Since the widening shifts would be rather confusing to follow in little
	// endian, everything will be illustrated in big endian order instead. This
	// has a different result - the bits would actually be reversed on a big
	// endian machine.

	// Starting input (only half the elements are shown):
	// 89 ff 1d c0 00 10 99 33
	uint8x16_t input = vreinterpretq_u8_m128i(a);

	// Shift out everything but the sign bits with an unsigned shift right.
	//
	// Bytes of the vector::
	// 89 ff 1d c0 00 10 99 33
	// \  \  \  \  \  \  \  \    high_bits = (uint16x4_t)(input >> 7)
	//  |  |  |  |  |  |  |  |
	// 01 01 00 01 00 00 01 00
	//
	// Bits of first important lane(s):
	// 10001001 (89)
	// \______
	//        |
	// 00000001 (01)
	uint16x8_t high_bits = vreinterpretq_u16_u8(vshrq_n_u8(input, 7));

	// Merge the even lanes together with a 16-bit unsigned shift right + add.
	// 'xx' represents garbage data which will be ignored in the final result.
	// In the important bytes, the add functions like a binary OR.
	//
	// 01 01 00 01 00 00 01 00
	//  \_ |  \_ |  \_ |  \_ |   paired16 = (uint32x4_t)(input + (input >> 7))
	//    \|    \|    \|    \|
	// xx 03 xx 01 xx 00 xx 02
	//
	// 00000001 00000001 (01 01)
	//        \_______ |
	//                \|
	// xxxxxxxx xxxxxx11 (xx 03)
	uint32x4_t paired16 =
		vreinterpretq_u32_u16(vsraq_n_u16(high_bits, high_bits, 7));

	// Repeat with a wider 32-bit shift + add.
	// xx 03 xx 01 xx 00 xx 02
	//     \____ |     \____ |  paired32 = (uint64x1_t)(paired16 + (paired16 >>
	//     14))
	//          \|          \|
	// xx xx xx 0d xx xx xx 02
	//
	// 00000011 00000001 (03 01)
	//        \\_____ ||
	//         '----.\||
	// xxxxxxxx xxxx1101 (xx 0d)
	uint64x2_t paired32 =
		vreinterpretq_u64_u32(vsraq_n_u32(paired16, paired16, 14));

	// Last, an even wider 64-bit shift + add to get our result in the low 8 bit
	// lanes. xx xx xx 0d xx xx xx 02
	//            \_________ |   paired64 = (uint8x8_t)(paired32 + (paired32 >>
	//            28))
	//                      \|
	// xx xx xx xx xx xx xx d2
	//
	// 00001101 00000010 (0d 02)
	//     \   \___ |  |
	//      '---.  \|  |
	// xxxxxxxx 11010010 (xx d2)
	uint8x16_t paired64 =
		vreinterpretq_u8_u64(vsraq_n_u64(paired32, paired32, 28));

	// Extract the low 8 bits from each 64-bit lane with 2 8-bit extracts.
	// xx xx xx xx xx xx xx d2
	//                      ||  return paired64[0]
	//                      d2
	// Note: Little endian would return the correct value 4b (01001011) instead.
	return vgetq_lane_u8(paired64, 0) | ((int)vgetq_lane_u8(paired64, 8) << 8);
#else
	uint8x16_t ia = vreinterpretq_u8_m128i(a);
	static const uint8x16_t mask_and = vdupq_n_u8(0x80);
	ia = vandq_u8(ia, mask_and);

	int low = (ia[0] >> 7) | (ia[1] >> 6) | (ia[2] >> 5) | (ia[3] >> 4) |
		(ia[4] >> 3) | (ia[5] >> 2) | (ia[6] >> 1) | ia[7];
	int high = (ia[8] >> 7) | (ia[9] >> 6) | (ia[10] >> 5) | (ia[11] >> 4) |
		(ia[12] >> 3) | (ia[13] >> 2) | (ia[14] >> 1) | ia[15];

	return ((high << 8) | (low & 0xFF));
#endif
}

// ******************************************
// Math operations
// ******************************************

// Subtracts the four single-precision, floating-point values of a and b
inline __m128 _mm_sub_ps(__m128 a, __m128 b)
{
    return vreinterpretq_m128_f32(vsubq_f32(vreinterpretq_f32_m128(a), vreinterpretq_f32_m128(b)));
}

// Subtracts the 4 signed or unsigned 32-bit integers of b from the 4 signed or unsigned 32-bit integers of a
inline __m128i _mm_sub_epi32(__m128i a, __m128i b)
{
    return vreinterpretq_m128_f32(vsubq_s32(vreinterpretq_f32_m128(a), vreinterpretq_f32_m128(b)));
}

inline __m128i _mm_sub_epi16(__m128i a, __m128i b)
{
    return vreinterpretq_m128i_s16(vsubq_s16(vreinterpretq_s16_m128i(a), vreinterpretq_s16_m128i(b)));
}

inline __m128i _mm_sub_epi8(__m128i a, __m128i b)
{
    return vreinterpretq_m128i_s8(vsubq_s8(vreinterpretq_s8_m128i(a), vreinterpretq_s8_m128i(b)));
}

//Subtracts the 8 unsigned 16-bit integers of bfrom the 8 unsigned 16-bit integers of a and saturates
inline __m128i _mm_subs_epu16(__m128i a, __m128i b)
{
    return vreinterpretq_m128i_u16(vqsubq_u16(vreinterpretq_u16_m128i(a), vreinterpretq_u16_m128i(b)));
}

//Subtracts the 16 unsigned 8-bit integers of b from the 16 unsigned 8-bit integers of a and saturates
inline __m128i _mm_subs_epu8(__m128i a, __m128i b)
{
    return vreinterpretq_m128i_u8(vqsubq_u8(vreinterpretq_u8_m128i(a), vreinterpretq_u8_m128i(b)));
}

// Adds the four single-precision, floating-point values of a and b
inline __m128 _mm_add_ps(__m128 a, __m128 b)
{
    return vreinterpretq_m128_f32(vaddq_f32(vreinterpretq_f32_m128(a), vreinterpretq_f32_m128(b)));
}

// adds the scalar single-precision floating point values of a and b
inline __m128 _mm_add_ss(__m128 a, __m128 b)
{
    float32_t b0 = vgetq_lane_f32(vreinterpretq_f32_m128(b), 0);
    float32x4_t value = vsetq_lane_f32(b0, vdupq_n_f32(0), 0);
    //the upper values in the result must be the remnants of <a>.
    return vreinterpretq_m128_f32(vaddq_f32(a, value));
}

// Adds the 4 signed or unsigned 32-bit integers in a to the 4 signed or unsigned 32-bit integers in b
inline __m128i _mm_add_epi32(__m128i a, __m128i b)
{
    return vreinterpretq_m128i_s32(vaddq_s32(vreinterpretq_s32_m128i(a), vreinterpretq_s32_m128i(b)));
}

// Adds the 8 signed or unsigned 16-bit integers in a to the 8 signed or unsigned 16-bit integers in b
inline __m128i _mm_add_epi16(__m128i a, __m128i b)
{
    return vreinterpretq_m128i_s16(vaddq_s16(vreinterpretq_s16_m128i(a), vreinterpretq_s16_m128i(b)));
}

// Adds the 16 signed or unsigned 8-bit integers in a to the 16 signed or unsigned 8-bit integers in b
inline __m128i _mm_add_epi8(__m128i a, __m128i b)
{
    return vreinterpretq_m128i_s8(vaddq_s8(vreinterpretq_s8_m128i(a), vreinterpretq_s8_m128i(b)));
}

// Adds the 8 signed 16-bit integers in a to the 8 signed 16-bit integers in b and saturates
inline __m128i _mm_adds_epi16(__m128i a, __m128i b)
{
    return vreinterpretq_m128i_s16(vqaddq_s16(vreinterpretq_s16_m128i(a), vreinterpretq_s16_m128i(b)));
}

//Adds the 16 unsigned 8-bit integers in a to the 16 unsigned 8-bit integers in b and saturates
inline __m128i _mm_adds_epu8(__m128i a, __m128i b)
{
    return vreinterpretq_m128i_u8(vqaddq_u8(vreinterpretq_u8_m128i(a), vreinterpretq_u8_m128i(b)));
}

// Multiplies the 8 signed or unsigned 16-bit integers from a by the 8 signed or unsigned 16-bit integers from b
inline __m128i _mm_mullo_epi16(__m128i a, __m128i b)
{
    return vreinterpretq_m128i_s16(vmulq_s16(vreinterpretq_s16_m128i(a), vreinterpretq_s16_m128i(b)));
}

// Multiplies the 4 signed or unsigned 32-bit integers from a by the 4 signed or unsigned 32-bit integers from b
inline __m128i _mm_mullo_epi32(__m128i a, __m128i b)
{
    return vreinterpretq_m128i_s32(vmulq_s32(vreinterpretq_s32_m128i(a), vreinterpretq_s32_m128i(b)));
}

// Multiplies the four single-precision, floating-point values of a and b
inline __m128 _mm_mul_ps(__m128 a, __m128 b)
{
    return vreinterpretq_m128_f32(vmulq_f32(vreinterpretq_f32_m128(a), vreinterpretq_f32_m128(b)));
}

// Multiplies the four single-precision, floating-point values of a and float b
inline __m128 _mm_mul_ps_scalar_soc(__m128 a, float b)
{
	return vreinterpretq_m128_f32(vmulq_n_f32(vreinterpretq_f32_m128(a), b));
}

inline __m128 _mm_fmadd_ps_soc(const __m128 &a, float b, const __m128 &c)
{
	return vaddq_f32(vmulq_n_f32(a, b), c);
}

// Multiplies the four single-precision, floating-point values of a and float b
inline __m128 _mm_mul_ps_scalar_soc(__m128 a, __m128 b, int bIdx)
{	
	return vreinterpretq_m128_f32(vmulq_n_f32(vreinterpretq_f32_m128(a), b[bIdx]));
}
// https://www.cnblogs.com/fellow1988/p/12283792.html
inline __m128 _mm_abs_ps_soc(__m128 a)
{
	return vreinterpretq_m128_f32(vabsq_f32(vreinterpretq_f32_m128(a)));
}
// negates each element in a vector.
inline __m128 _mm_negate_ps_soc(__m128 a)
{
	return vreinterpretq_m128_f32(vnegq_f32(vreinterpretq_f32_m128(a)));
}


// Divides the four single-precision, floating-point values of a and b
inline __m128 _mm_div_ps(__m128 a, __m128 b)
{
#ifdef __aarch64__
    return vdivq_f32(a, b);
#else
//////    float arr1[4], arr2[4];
//////    vst1q_f32(arr1, a);
//////    vst1q_f32(arr2, b);
////    return _mm_setr_ps(a[0] / b[0], a[1] / b[1], a[2] / b[2], a[3] / b[3]);

	float32x4_t recip0 = vrecpeq_f32(vreinterpretq_f32_m128(b));
	float32x4_t recip1 =
		vmulq_f32(recip0, vrecpsq_f32(recip0, vreinterpretq_f32_m128(b)));
	return vreinterpretq_m128_f32(vmulq_f32(vreinterpretq_f32_m128(a), recip1));
#endif
}

// Divides the scalar single-precision floating point value of a by b
inline __m128 _mm_div_ss(__m128 a, __m128 b)
{
    float32_t value = vgetq_lane_f32(vreinterpretq_f32_m128(_mm_div_ps(a, b)), 0);
    return vreinterpretq_m128_f32(vsetq_lane_f32(value, vreinterpretq_f32_m128(a), 0));
}

// This version does additional iterations to improve accuracy.  Between 1 and 4 recommended.
// Computes the approximations of reciprocals of the four single-precision, floating-point values of a
inline __m128 recipq_newton(__m128 in, int n)
{
    int i;
    float32x4_t recip = vrecpeq_f32(vreinterpretq_f32_m128(in));
    for (i = 0; i < n; ++i)
    {
        recip = vmulq_f32(recip, vrecpsq_f32(recip, vreinterpretq_f32_m128(in)));
    }
    return vreinterpretq_m128_f32(recip);
}

// Computes the approximations of reciprocals of the four single-precision, floating-point values of a
inline __m128 _mm_rcp_ps(__m128 in)
{
    float32x4_t recip = vrecpeq_f32(vreinterpretq_f32_m128(in));
    recip = vmulq_f32(recip, vrecpsq_f32(recip, vreinterpretq_f32_m128(in)));
    return vreinterpretq_m128_f32(recip);
}

// Computes the approximations of square roots of the four single-precision, floating-point values of a. First computes reciprocal square roots and then reciprocals of the four values
inline __m128 _mm_sqrt_ps(__m128 in)
{
#if defined(__aarch64__)
	return vreinterpretq_m128_f32(vsqrtq_f32(vreinterpretq_f32_m128(in)));
#else
    float32x4_t recipsq = vrsqrteq_f32(vreinterpretq_f32_m128(in));
    float32x4_t sq = vrecpeq_f32(recipsq);
    // ??? use step versions of both sqrt and recip for better accuracy?
    return vreinterpretq_m128_f32(sq);
#endif
}

// Computes the approximation of the square root of the scalar single-precision floating point value of in
inline __m128 _mm_sqrt_ss(__m128 in)
{
    float32_t value = vgetq_lane_f32(vreinterpretq_f32_m128(_mm_sqrt_ps(in)), 0);
    return vreinterpretq_m128_f32(vsetq_lane_f32(value, vreinterpretq_f32_m128(in), 0));
}

// Computes the approximations of the reciprocal square roots of the four single-precision floating point values of in
inline __m128 _mm_rsqrt_ps(__m128 in)
{
    return vreinterpretq_m128_f32(vrsqrteq_f32(vreinterpretq_f32_m128(in)));
}

// Computes the maximums of the four single-precision, floating-point values of a and b
inline __m128 _mm_max_ps(__m128 a, __m128 b)
{
    return vreinterpretq_m128_f32(vmaxq_f32(vreinterpretq_f32_m128(a), vreinterpretq_f32_m128(b)));
}

// Computes the minima of the four single-precision, floating-point values of a and b
inline __m128 _mm_min_ps(__m128 a, __m128 b)
{
    return vreinterpretq_m128_f32(vminq_f32(vreinterpretq_f32_m128(a), vreinterpretq_f32_m128(b)));
}

// Computes the maximum of the two lower scalar single-precision floating point values of a and b
inline __m128 _mm_max_ss(__m128 a, __m128 b)
{
    float32_t value = vgetq_lane_f32(vmaxq_f32(vreinterpretq_f32_m128(a), vreinterpretq_f32_m128(b)), 0);
    return vreinterpretq_m128_f32(vsetq_lane_f32(value, vreinterpretq_f32_m128(a), 0));
}

// Computes the minimum of the two lower scalar single-precision floating point values of a and b
inline __m128 _mm_min_ss(__m128 a, __m128 b)
{
    float32_t value = vgetq_lane_f32(vminq_f32(vreinterpretq_f32_m128(a), vreinterpretq_f32_m128(b)), 0);
    return vreinterpretq_m128_f32(vsetq_lane_f32(value, vreinterpretq_f32_m128(a), 0));
}

//Computes the pairwise maxima of the 16 unsigned 8-bit integers from a and the 16 unsigned 8-bit integers from b
inline __m128i _mm_max_epu8(__m128i a, __m128i b)
{
    return vreinterpretq_m128i_u8(vmaxq_u8(vreinterpretq_u8_m128i(a), vreinterpretq_u8_m128i(b)));
}

//Computes the pairwise minima of the 16 unsigned 8-bit integers from a and the 16 unsigned 8-bit integers from b
inline __m128i _mm_min_epu8(__m128i a, __m128i b)
{
    return vreinterpretq_m128i_u8(vminq_u8(vreinterpretq_u8_m128i(a), vreinterpretq_u8_m128i(b)));
}

// Computes the pairwise minima of the 8 signed 16-bit integers from a and the 8 signed 16-bit integers from b
inline __m128i _mm_min_epi16(__m128i a, __m128i b)
{
    return vreinterpretq_m128i_s16(vminq_s16(vreinterpretq_s16_m128i(a), vreinterpretq_s16_m128i(b)));
}

//Computes the pairwise maxima of the 8 signed 16-bit integers from a and the 8 signed 16-bit integers from b
inline __m128i _mm_max_epi16(__m128i a, __m128i b)
{
    return vreinterpretq_m128i_s16(vmaxq_s16(vreinterpretq_s16_m128i(a), vreinterpretq_s16_m128i(b)));
}

// epi versions of min/max
// Computes the pariwise maximums of the four signed 32-bit integer values of a and b
inline __m128i _mm_max_epi32(__m128i a, __m128i b)
{
    return vreinterpretq_m128i_s32(vmaxq_s32(vreinterpretq_s32_m128i(a), vreinterpretq_s32_m128i(b)));
}

// Computes the pariwise minima of the four signed 32-bit integer values of a and b
inline __m128i _mm_min_epi32(__m128i a, __m128i b)
{
    return vreinterpretq_m128i_s32(vminq_s32(vreinterpretq_s32_m128i(a), vreinterpretq_s32_m128i(b)));
}

// Compare packed unsigned 32-bit integers in a and b, and store packed minimum
// values in dst.
// https://software.intel.com/sites/landingpage/IntrinsicsGuide/#text=_mm_max_epu32
inline __m128i _mm_min_epu32(__m128i a, __m128i b)
{
	return vreinterpretq_m128i_u32(
		vminq_u32(vreinterpretq_u32_m128i(a), vreinterpretq_u32_m128i(b)));
}

// Multiplies the 8 signed 16-bit integers from a by the 8 signed 16-bit integers from b
inline __m128i _mm_mulhi_epi16(__m128i a, __m128i b)
{
    /* apoty: issue with large values because of result saturation */
    //int16x8_t ret = vqdmulhq_s16(vreinterpretq_s16_m128i(a), vreinterpretq_s16_m128i(b)); /* =2*a*b */
    //return vreinterpretq_m128i_s16(vshrq_n_s16(ret, 1));
    int16x4_t a3210 = vget_low_s16(vreinterpretq_s16_m128i(a));
    int16x4_t b3210 = vget_low_s16(vreinterpretq_s16_m128i(b));
    int32x4_t ab3210 = vmull_s16(a3210, b3210); /* 3333222211110000 */
    int16x4_t a7654 = vget_high_s16(vreinterpretq_s16_m128i(a));
    int16x4_t b7654 = vget_high_s16(vreinterpretq_s16_m128i(b));
    int32x4_t ab7654 = vmull_s16(a7654, b7654); /* 7777666655554444 */
    uint16x8x2_t r = vuzpq_u16(vreinterpretq_u16_s32(ab3210), vreinterpretq_u16_s32(ab7654));
    return vreinterpretq_m128i_u16(r.val[1]);
}

inline uint64_t _mm_getUint16Max8_soc(__m128i a)
{
	return  (uint64_t) vqshrn_n_u16(vreinterpretq_s16_m128i(a), 8);
}

// Computes pairwise add of each argument as single-precision, floating-point values a and b.
inline __m128 _mm_hadd_ps(__m128 a, __m128 b)
{
#if defined(__aarch64__)
	return vreinterpretq_m128_f32(
		vpaddq_f32(vreinterpretq_f32_m128(a), vreinterpretq_f32_m128(b)));
#else
    float32x2_t a10 = vget_low_f32(vreinterpretq_f32_m128(a));
    float32x2_t a32 = vget_high_f32(vreinterpretq_f32_m128(a));
    float32x2_t b10 = vget_low_f32(vreinterpretq_f32_m128(b));
    float32x2_t b32 = vget_high_f32(vreinterpretq_f32_m128(b));
    return vreinterpretq_m128_f32(vcombine_f32(vpadd_f32(a10, a32), vpadd_f32(b10, b32)));
#endif
}

inline __m128 _mm_hadd_ps(__m128 a)
{
    float32x2_t a10 = vget_low_f32(vreinterpretq_f32_m128(a));
    float32x2_t a32 = vget_high_f32(vreinterpretq_f32_m128(a));
    float32x2_t sum = vpadd_f32(a10, a32);
    return vreinterpretq_m128_f32(vcombine_f32(sum, sum));
}

inline __m128 _mm_sum4_ps_soc(__m128 a)
{
	float32x2_t a10 = vget_low_f32(vreinterpretq_f32_m128(a));
	float32x2_t a32 = vget_high_f32(vreinterpretq_f32_m128(a));
	float32x2_t sum = vadd_f32(a10, a32);
	sum = vpadd_f32(sum, sum);
	return vreinterpretq_m128_f32(vcombine_f32(sum, sum));
}


inline __m128 _mm_max4_ps_soc(__m128 a)
{
	//__m128 maxExtent = _mm_max_ps(extents, _mm_shuffle_ps(extents, extents, _MM_SHUFFLE(1, 0, 3, 2)));
	//maxExtent = _mm_max_ps(maxExtent, _mm_shuffle_ps(maxExtent, maxExtent, _MM_SHUFFLE(2, 3, 0, 1)));

	float32x2_t a10 = vget_low_f32(vreinterpretq_f32_m128(a));
	float32x2_t a32 = vget_high_f32(vreinterpretq_f32_m128(a));
	float32x2_t max = vmax_f32(a10, a32);
	max = vpmax_f32(max, max);
	return vreinterpretq_m128_f32(vcombine_f32(max, max));
}

// ******************************************
// Compare operations
// ******************************************

// Compares for less than
inline __m128 _mm_cmplt_ps(__m128 a, __m128 b)
{
    return vreinterpretq_m128_u32(vcltq_f32(vreinterpretq_f32_m128(a), vreinterpretq_f32_m128(b)));
}

// Compares for greater than
inline __m128 _mm_cmpgt_ps(__m128 a, __m128 b)
{
    return vreinterpretq_m128_u32(vcgtq_f32(vreinterpretq_f32_m128(a), vreinterpretq_f32_m128(b)));
}

// Compares for greater than or equal
inline __m128 _mm_cmpge_ps(__m128 a, __m128 b)
{
    return vreinterpretq_m128_u32(vcgeq_f32(vreinterpretq_f32_m128(a), vreinterpretq_f32_m128(b)));
}

// Compares for less than or equal
inline __m128 _mm_cmple_ps(__m128 a, __m128 b)
{
    return vreinterpretq_m128_u32(vcleq_f32(vreinterpretq_f32_m128(a), vreinterpretq_f32_m128(b)));
}

// Compares for equality
inline __m128 _mm_cmpeq_ps(__m128 a, __m128 b)
{
    return vreinterpretq_m128_u32(vceqq_f32(vreinterpretq_f32_m128(a), vreinterpretq_f32_m128(b)));
}

//Compares the 16 signed or unsigned 8-bit integers in a and the 16 signed or unsigned 8-bit integers in b for equality
inline __m128i _mm_cmpeq_epi8(__m128i a, __m128i b)
{
    return vreinterpretq_m128i_u8(vceqq_s8(vreinterpretq_s8_m128i(a), vreinterpretq_s8_m128i(b)));
}

//Compares the 8 signed or unsigned 16-bit integers in a and the 8 signed or unsigned 16-bit integers in b for equality
inline __m128i _mm_cmpeq_epi16(__m128i a, __m128i b)
{
    return vreinterpretq_m128i_u16(vceqq_s16(vreinterpretq_s16_m128i(a), vreinterpretq_s16_m128i(b)));
}

//Compares the 8 signed or unsigned 16-bit integers in a and the 8 signed or unsigned 16-bit integers in b for equality
inline __m128i _mm_cmpeq_epi32(__m128i a, __m128i b)
{
    return vreinterpretq_m128i_u32(vceqq_s32(vreinterpretq_s32_m128i(a), vreinterpretq_s32_m128i(b)));
}

//Compares the 16 signed 8-bit integers in a and the 16 signed 8-bit integers in b for lesser than
inline __m128i _mm_cmplt_epi8(__m128i a, __m128i b)
{
    return vreinterpretq_m128i_u8(vcltq_s8(vreinterpretq_s8_m128i(a), vreinterpretq_s8_m128i(b)));
}

//Compares the 16 signed 8-bit integers in a and the 16 signed 8-bit integers in b for greater than
inline __m128i _mm_cmpgt_epi8(__m128i a, __m128i b)
{
    return vreinterpretq_m128i_u8(vcgtq_s8(vreinterpretq_s8_m128i(a), vreinterpretq_s8_m128i(b)));
}

//Compares the 8 signed 16-bit integers in a and the 8 signed 16-bit integers in b for greater than
inline __m128i _mm_cmpgt_epi16(__m128i a, __m128i b)
{
    return vreinterpretq_m128i_u16(vcgtq_s16(vreinterpretq_s16_m128i(a), vreinterpretq_s16_m128i(b)));
}

// Compares the 4 signed 32-bit integers in a and the 4 signed 32-bit integers in b for less than
inline __m128i _mm_cmplt_epi32(__m128i a, __m128i b)
{
    return vreinterpretq_m128i_u32(vcltq_s32(vreinterpretq_s32_m128i(a), vreinterpretq_s32_m128i(b)));
}

// Compares the 4 signed 32-bit integers in a and the 4 signed 32-bit integers in b for greater than
inline __m128i _mm_cmpgt_epi32(__m128i a, __m128i b)
{
    return vreinterpretq_m128i_u32(vcgtq_s32(vreinterpretq_s32_m128i(a), vreinterpretq_s32_m128i(b)));
}

// Compares the 4 signed 32-bit integers in a and the 4 signed 32-bit integers in b for greater than
inline __m128i _mm_cmple_epi32_soc(__m128i a, __m128i b)
{
	return vreinterpretq_m128i_u32(vcleq_s32(vreinterpretq_s32_m128i(a), vreinterpretq_s32_m128i(b)));
}


// Compares the four 32-bit floats in a and b to check if any values are NaN. Ordered compare between each value returns true for "orderable" and false for "not orderable" (NaN)
inline __m128 _mm_cmpord_ps(__m128 a, __m128 b)
{
    // Note: NEON does not have ordered compare builtin
    // Need to compare a eq a and b eq b to check for NaN
    // Do AND of results to get final
    uint32x4_t ceqaa = vceqq_f32(vreinterpretq_f32_m128(a), vreinterpretq_f32_m128(a));
    uint32x4_t ceqbb = vceqq_f32(vreinterpretq_f32_m128(b), vreinterpretq_f32_m128(b));
    return vreinterpretq_m128_u32(vandq_u32(ceqaa, ceqbb));
}

// Compares the lower single-precision floating point scalar values of a and b using a less than operation
// Important note!! The documentation on MSDN is incorrect!  If either of the values is a NAN the docs say you will get a one, but in fact, it will return a zero!!
inline int _mm_comilt_ss(__m128 a, __m128 b)
{
    uint32x4_t a_not_nan = vceqq_f32(vreinterpretq_f32_m128(a), vreinterpretq_f32_m128(a));
    uint32x4_t b_not_nan = vceqq_f32(vreinterpretq_f32_m128(b), vreinterpretq_f32_m128(b));
    uint32x4_t a_or_b_nan = vmvnq_u32(vandq_u32(a_not_nan, b_not_nan));
    uint32x4_t a_lt_b = vcltq_f32(vreinterpretq_f32_m128(a), vreinterpretq_f32_m128(b));
    return (vgetq_lane_u32(vorrq_u32(a_or_b_nan, a_lt_b), 0) != 0) ? 1 : 0;
}

// Compares the lower single-precision floating point scalar values of a and b using a greater than operation
inline int _mm_comigt_ss(__m128 a, __m128 b)
{
    //return vgetq_lane_u32(vcgtq_f32(vreinterpretq_f32_m128(a), vreinterpretq_f32_m128(b)), 0);
    uint32x4_t a_not_nan = vceqq_f32(vreinterpretq_f32_m128(a), vreinterpretq_f32_m128(a));
    uint32x4_t b_not_nan = vceqq_f32(vreinterpretq_f32_m128(b), vreinterpretq_f32_m128(b));
    uint32x4_t a_and_b_not_nan = vandq_u32(a_not_nan, b_not_nan);
    uint32x4_t a_gt_b = vcgtq_f32(vreinterpretq_f32_m128(a), vreinterpretq_f32_m128(b));
    return (vgetq_lane_u32(vandq_u32(a_and_b_not_nan, a_gt_b), 0) != 0) ? 1 : 0;
}

// Compares the lower single-precision floating point scalar values of a and b using a less than or equal operation
inline int _mm_comile_ss(__m128 a, __m128 b)
{
    //return vgetq_lane_u32(vcleq_f32(vreinterpretq_f32_m128(a), vreinterpretq_f32_m128(b)), 0);
    uint32x4_t a_not_nan = vceqq_f32(vreinterpretq_f32_m128(a), vreinterpretq_f32_m128(a));
    uint32x4_t b_not_nan = vceqq_f32(vreinterpretq_f32_m128(b), vreinterpretq_f32_m128(b));
    uint32x4_t a_or_b_nan = vmvnq_u32(vandq_u32(a_not_nan, b_not_nan));
    uint32x4_t a_le_b = vcleq_f32(vreinterpretq_f32_m128(a), vreinterpretq_f32_m128(b));
    return (vgetq_lane_u32(vorrq_u32(a_or_b_nan, a_le_b), 0) != 0) ? 1 : 0;
}

// Compares the lower single-precision floating point scalar values of a and b using a greater than or equal operation
inline int _mm_comige_ss(__m128 a, __m128 b)
{
    //return vgetq_lane_u32(vcgeq_f32(vreinterpretq_f32_m128(a), vreinterpretq_f32_m128(b)), 0);
    uint32x4_t a_not_nan = vceqq_f32(vreinterpretq_f32_m128(a), vreinterpretq_f32_m128(a));
    uint32x4_t b_not_nan = vceqq_f32(vreinterpretq_f32_m128(b), vreinterpretq_f32_m128(b));
    uint32x4_t a_and_b_not_nan = vandq_u32(a_not_nan, b_not_nan);
    uint32x4_t a_ge_b = vcgeq_f32(vreinterpretq_f32_m128(a), vreinterpretq_f32_m128(b));
    return (vgetq_lane_u32(vandq_u32(a_and_b_not_nan, a_ge_b), 0) != 0) ? 1 : 0;
}

// Compares the lower single-precision floating point scalar values of a and b using an equality operation
inline int _mm_comieq_ss(__m128 a, __m128 b)
{
    //return vgetq_lane_u32(vceqq_f32(vreinterpretq_f32_m128(a), vreinterpretq_f32_m128(b)), 0);
    uint32x4_t a_not_nan = vceqq_f32(vreinterpretq_f32_m128(a), vreinterpretq_f32_m128(a));
    uint32x4_t b_not_nan = vceqq_f32(vreinterpretq_f32_m128(b), vreinterpretq_f32_m128(b));
    uint32x4_t a_or_b_nan = vmvnq_u32(vandq_u32(a_not_nan, b_not_nan));
    uint32x4_t a_eq_b = vceqq_f32(vreinterpretq_f32_m128(a), vreinterpretq_f32_m128(b));
    return (vgetq_lane_u32(vorrq_u32(a_or_b_nan, a_eq_b), 0) != 0) ? 1 : 0;
}

// Compares the lower single-precision floating point scalar values of a and b using an inequality operation
inline int _mm_comineq_ss(__m128 a, __m128 b)
{
    //return !vgetq_lane_u32(vceqq_f32(vreinterpretq_f32_m128(a), vreinterpretq_f32_m128(b)), 0);
    uint32x4_t a_not_nan = vceqq_f32(vreinterpretq_f32_m128(a), vreinterpretq_f32_m128(a));
    uint32x4_t b_not_nan = vceqq_f32(vreinterpretq_f32_m128(b), vreinterpretq_f32_m128(b));
    uint32x4_t a_and_b_not_nan = vandq_u32(a_not_nan, b_not_nan);
    uint32x4_t a_neq_b = vmvnq_u32(vceqq_f32(vreinterpretq_f32_m128(a), vreinterpretq_f32_m128(b)));
    return (vgetq_lane_u32(vandq_u32(a_and_b_not_nan, a_neq_b), 0) != 0) ? 1 : 0;
}

// according to the documentation, these intrinsics behave the same as the non-'u' versions.  We'll just alias them here.
#define _mm_ucomilt_ss _mm_comilt_ss
#define _mm_ucomile_ss _mm_comile_ss
#define _mm_ucomigt_ss _mm_comigt_ss
#define _mm_ucomige_ss _mm_comige_ss
#define _mm_ucomieq_ss _mm_comieq_ss
#define _mm_ucomineq_ss _mm_comineq_ss

// ******************************************
// Conversions
// ******************************************

// Converts the four single-precision, floating-point values of a to signed 32-bit integer values using truncate
inline __m128i _mm_cvttps_epi32(__m128 a)
{
    return vreinterpretq_m128i_s32(vcvtq_s32_f32(vreinterpretq_f32_m128(a)));
}

// Converts the four signed 32-bit integer values of a to single-precision, floating-point values
inline __m128 _mm_cvtepi32_ps(__m128i a)
{
    return vreinterpretq_m128_f32(vcvtq_f32_s32(vreinterpretq_s32_m128i(a)));
}

// Converts the four unsigned 8-bit integers in the lower 32 bits to four unsigned 32-bit integers
inline __m128i _mm_cvtepu8_epi32(__m128i a)
{
    uint8x16_t u8x16 = vreinterpretq_u8_s32(a);        /* xxxx xxxx xxxx DCBA */
    uint16x8_t u16x8 = vmovl_u8(vget_low_u8(u8x16));   /* 0x0x 0x0x 0D0C 0B0A */
    uint32x4_t u32x4 = vmovl_u16(vget_low_u16(u16x8)); /* 000D 000C 000B 000A */
    return vreinterpretq_s32_u32(u32x4);
}

// Converts the four signed 16-bit integers in the lower 64 bits to four signed 32-bit integers
inline __m128i _mm_cvtepi16_epi32(__m128i a)
{
    return vreinterpretq_m128i_s32(vmovl_s16(vget_low_s16(vreinterpretq_s16_m128i(a))));
}

// Converts the four single-precision, floating-point values of a to signed 32-bit integer values
// *NOTE*. The default rounding mode on SSE is 'round to even', which ArmV7 does not support!
// It is supported on ARMv8 however.
inline __m128i _mm_cvtps_epi32(__m128 a)
{
#if defined(__aarch64__)
    return vcvtnq_s32_f32(a);
#else
    uint32x4_t signmask = vdupq_n_u32(0x80000000);
    float32x4_t half = vbslq_f32(signmask, vreinterpretq_f32_m128(a), vdupq_n_f32(0.5f));                  /* +/- 0.5 */
    int32x4_t r_normal = vcvtq_s32_f32(vaddq_f32(vreinterpretq_f32_m128(a), half));                        /* round to integer: [a + 0.5]*/
    int32x4_t r_trunc = vcvtq_s32_f32(vreinterpretq_f32_m128(a));                                          /* truncate to integer: [a] */
    int32x4_t plusone = vreinterpretq_s32_u32(vshrq_n_u32(vreinterpretq_u32_s32(vnegq_s32(r_trunc)), 31)); /* 1 or 0 */
    int32x4_t r_even = vbicq_s32(vaddq_s32(r_trunc, plusone), vdupq_n_s32(1));                             /* ([a] + {0,1}) & ~1 */
    float32x4_t delta = vsubq_f32(vreinterpretq_f32_m128(a), vcvtq_f32_s32(r_trunc));                      /* compute delta: delta = (a - [a]) */
    uint32x4_t is_delta_half = vceqq_f32(delta, half);                                                     /* delta == +/- 0.5 */
    return vreinterpretq_m128i_s32(vbslq_s32(is_delta_half, r_even, r_normal));
#endif
}

// Moves the least significant 32 bits of a to a 32-bit integer
inline int _mm_cvtsi128_si32(__m128i a)
{
    return vgetq_lane_s32(vreinterpretq_s32_m128i(a), 0);
}

// Moves 32-bit integer a to the least significant 32 bits of an __m128 object, zero extending the upper bits
inline __m128i _mm_cvtsi32_si128(int a)
{
    return vreinterpretq_m128i_s32(vsetq_lane_s32(a, vdupq_n_s32(0), 0));
}

// Applies a type cast to reinterpret four 32-bit floating point values passed in as a 128-bit parameter as packed 32-bit integers
inline __m128i _mm_castps_si128(__m128 a)
{
    return vreinterpretq_m128i_s32(vreinterpretq_s32_m128(a));
}

// Applies a type cast to reinterpret four 32-bit integers passed in as a 128-bit parameter as packed 32-bit floating point values
inline __m128 _mm_castsi128_ps(__m128i a)
{
    return vreinterpretq_m128_s32(vreinterpretq_s32_m128i(a));
}

// Loads 128-bit value
inline __m128i _mm_load_si128(const __m128i *p)
{
    return vreinterpretq_m128i_s32(vld1q_s32((int32_t *)p));
}

// Loads 128-bit value
inline __m128i _mm_loadu_si128(const __m128i *p)
{
    return vreinterpretq_m128i_s32(vld1q_s32((int32_t *)p));
}

#define _MM_ROUND_DOWN 1
#define _MM_ROUND_UP 2
#define _MM_ROUND_NEAREST 3

inline int _MM_GET_ROUNDING_MODE()
{
    return 0;
}

inline void _MM_SET_ROUNDING_MODE(int /*v*/)
{
}

// ******************************************
// Miscellaneous Operations
// ******************************************

// Packs the 16 signed 16-bit integers from a and b into 8-bit integers and saturates
inline __m128i _mm_packs_epi16(__m128i a, __m128i b)
{
    return vreinterpretq_m128i_s8(vcombine_s8(vqmovn_s16(vreinterpretq_s16_m128i(a)), vqmovn_s16(vreinterpretq_s16_m128i(b))));
}

// Packs the 16 signed 16 - bit integers from a and b into 8 - bit unsigned integers and saturates
inline __m128i _mm_packus_epi16(const __m128i a, const __m128i b)
{
    return vreinterpretq_m128i_u8(vcombine_u8(vqmovun_s16(vreinterpretq_s16_m128i(a)), vqmovun_s16(vreinterpretq_s16_m128i(b))));
}

// Packs the 8 signed 32-bit integers from a and b into signed 16-bit integers and saturates
inline __m128i _mm_packs_epi32(__m128i a, __m128i b)
{
    return vreinterpretq_m128i_s16(vcombine_s16(vqmovn_s32(vreinterpretq_s32_m128i(a)), vqmovn_s32(vreinterpretq_s32_m128i(b))));
}

// Interleaves the lower 8 signed or unsigned 8-bit integers in a with the lower 8 signed or unsigned 8-bit integers in b
inline __m128i _mm_unpacklo_epi8(__m128i a, __m128i b)
{
#if defined(__aarch64__)
	return vreinterpretq_m128i_s8(
		vzip1q_s8(vreinterpretq_s8_m128i(a), vreinterpretq_s8_m128i(b)));
#else
    int8x8_t a1 = vreinterpret_s8_s16(vget_low_s16(vreinterpretq_s16_m128i(a)));
    int8x8_t b1 = vreinterpret_s8_s16(vget_low_s16(vreinterpretq_s16_m128i(b)));
    int8x8x2_t result = vzip_s8(a1, b1);
    return vreinterpretq_m128i_s8(vcombine_s8(result.val[0], result.val[1]));
#endif
}

// Interleaves the lower 4 signed or unsigned 16-bit integers in a with the lower 4 signed or unsigned 16-bit integers in b
inline __m128i _mm_unpacklo_epi16(__m128i a, __m128i b)
{
#if defined(__aarch64__)
	return vreinterpretq_m128i_s16(
		vzip1q_s16(vreinterpretq_s16_m128i(a), vreinterpretq_s16_m128i(b)));
#else
    int16x4_t a1 = vget_low_s16(vreinterpretq_s16_m128i(a));
    int16x4_t b1 = vget_low_s16(vreinterpretq_s16_m128i(b));
    int16x4x2_t result = vzip_s16(a1, b1);
	return vreinterpretq_m128i_s16(vcombine_s16(result.val[0], result.val[1]));
#endif
}

// Interleaves the lower 2 signed or unsigned 32 - bit integers in a with the lower 2 signed or unsigned 32 - bit integers in b
inline __m128i _mm_unpacklo_epi32(__m128i a, __m128i b)
{
    int32x2_t a1 = vget_low_s32(vreinterpretq_s32_m128i(a));
    int32x2_t b1 = vget_low_s32(vreinterpretq_s32_m128i(b));
    int32x2x2_t result = vzip_s32(a1, b1);
    return vreinterpretq_m128i_s32(vcombine_s32(result.val[0], result.val[1]));
}

//inline __m128 _mm_min_minXY_min_maxXY_ps_soc(__m128 a, __m128 b)
//{
//	float32x2_t a1 = vget_low_f32(vreinterpretq_f32_m128(a));
//	float32x2_t a2 = vget_high_f32(vreinterpretq_f32_m128(a));
//	float32x2_t amin = vpmin_f32(a1, a2);
//
//	float32x2_t b1 = vget_low_f32(vreinterpretq_f32_m128(b));
//	float32x2_t b2 = vget_high_f32(vreinterpretq_f32_m128(b));
//	float32x2_t bmax = vpmin_f32(b1, b2);
//
//	return vreinterpretq_m128_f32(vcombine_s32(amin, bmax));
//}
// Selects and interleaves the lower two single-precision, floating-point values from a and b
inline __m128 _mm_unpacklo_ps(__m128 a, __m128 b)
{
#if defined(__aarch64__)
	return vreinterpretq_m128_f32(
		vzip1q_f32(vreinterpretq_f32_m128(a), vreinterpretq_f32_m128(b)));
#else
    float32x2_t a1 = vget_low_f32(vreinterpretq_f32_m128(a));
    float32x2_t b1 = vget_low_f32(vreinterpretq_f32_m128(b));
    float32x2x2_t result = vzip_f32(a1, b1);
	return vreinterpretq_m128_f32(vcombine_f32(result.val[0], result.val[1]));
#endif
}

// Selects and interleaves the upper two single-precision, floating-point values from a and b
inline __m128 _mm_unpackhi_ps(__m128 a, __m128 b)
{
#if defined(__aarch64__)
	return vreinterpretq_m128_f32(
		vzip2q_f32(vreinterpretq_f32_m128(a), vreinterpretq_f32_m128(b)));
#else
    float32x2_t a1 = vget_high_f32(vreinterpretq_f32_m128(a));
    float32x2_t b1 = vget_high_f32(vreinterpretq_f32_m128(b));
    float32x2x2_t result = vzip_f32(a1, b1);
	return vreinterpretq_m128_f32(vcombine_f32(result.val[0], result.val[1]));
#endif
}

// Interleaves the upper 8 signed or unsigned 8-bit integers in a with the upper 8 signed or unsigned 8-bit integers in b
inline __m128i _mm_unpackhi_epi8(__m128i a, __m128i b)
{
    int8x8_t a1 = vreinterpret_s8_s16(vget_high_s16(vreinterpretq_s16_m128i(a)));
    int8x8_t b1 = vreinterpret_s8_s16(vget_high_s16(vreinterpretq_s16_m128i(b)));
    int8x8x2_t result = vzip_s8(a1, b1);
    return vreinterpretq_m128i_s8(vcombine_s8(result.val[0], result.val[1]));
}

// Interleaves the upper 4 signed or unsigned 16-bit integers in a with the upper 4 signed or unsigned 16-bit integers in b
inline __m128i _mm_unpackhi_epi16(__m128i a, __m128i b)
{
    int16x4_t a1 = vget_high_s16(vreinterpretq_s16_m128i(a));
    int16x4_t b1 = vget_high_s16(vreinterpretq_s16_m128i(b));
    int16x4x2_t result = vzip_s16(a1, b1);
    return vreinterpretq_m128i_s16(vcombine_s16(result.val[0], result.val[1]));
}

// Interleaves the upper 2 signed or unsigned 32-bit integers in a with the upper 2 signed or unsigned 32-bit integers in b
inline __m128i _mm_unpackhi_epi32(__m128i a, __m128i b)
{
    int32x2_t a1 = vget_high_s32(vreinterpretq_s32_m128i(a));
    int32x2_t b1 = vget_high_s32(vreinterpretq_s32_m128i(b));
    int32x2x2_t result = vzip_s32(a1, b1);
    return vreinterpretq_m128i_s32(vcombine_s32(result.val[0], result.val[1]));
}

// Extracts the selected signed or unsigned 16-bit integer from a and zero extends
//inline int _mm_extract_epi16(__m128i a, __constrange(0,8) int imm)
#define _mm_extract_epi16(a, imm)                                           \
    ({                                                                      \
        (vgetq_lane_s16(vreinterpretq_s16_m128i(a), (imm)) & 0x0000ffffUL); \
    })

// Inserts the least significant 16 bits of b into the selected 16-bit integer of a
//inline __m128i _mm_insert_epi16(__m128i a, const int b, __constrange(0,8) int imm)
#define _mm_insert_epi16(a, b, imm)                                                      \
    ({                                                                                   \
        vreinterpretq_m128i_s16(vsetq_lane_s16((b), vreinterpretq_s16_m128i(a), (imm))); \
    })

// ******************************************
// Streaming Extensions
// ******************************************

// Guarantees that every preceding store is globally visible before any subsequent store
inline void _mm_sfence(void)
{
    __sync_synchronize();
}

// Stores the data in a to the address p without polluting the caches.  If the cache line containing address p is already in the cache, the cache will be updated.Address p must be 16 - byte aligned
inline void _mm_stream_si128(__m128i *p, __m128i a)
{
    *p = a;
}

#define _mm_srai_epi16(a, imm)                                                             \
    ({                                                                                     \
        __m128i ret;                                                                       \
        if ((imm) <= 0)                                                                    \
        {                                                                                  \
            ret = a;                                                                       \
        }                                                                                  \
        else if ((imm) > 15)                                                               \
        {                                                                                  \
            ret = vreinterpretq_m128i_s16(vshrq_n_s16(vreinterpretq_s16_m128i(a), 8));     \
            ret = vreinterpretq_m128i_s16(vshrq_n_s16(vreinterpretq_s16_m128i(ret), 8));   \
        }                                                                                  \
        else                                                                               \
        {                                                                                  \
            ret = vreinterpretq_m128i_s16(vshrq_n_s16(vreinterpretq_s16_m128i(a), (imm))); \
        }                                                                                  \
        ret;                                                                               \
    })

inline __m128i _mm_set_epi64x(__int64_t e1, __int64_t e0)
{
	int64_t ALIGN_STRUCT(16) data[2] = { e0, e1 };
	return vreinterpretq_m128i_s64(vld1q_s64(data));
    //return vcombine_s64(vdup_n_s64(e0), vdup_n_s64(e1));
}

inline __m128 _mm_fmadd_ps_soc(const __m128 &a, const __m128 &b, const __m128 &c)
{
#ifdef __aarch64__
    return vfmaq_f32(c, a, b);
#else
    return vaddq_f32(vmulq_f32(a, b), c);
#endif
}

inline __m128 _mm_fnmadd_ps_soc(const __m128 &a, const __m128 &b, const __m128 &c)
{
#ifdef __aarch64__
    // c - a * b
    return vfmsq_f32(c, a, b);
#else
    return vsubq_f32(c, vmulq_f32(a, b));
#endif
}

inline __m128 _mm_fmsub_ps_soc(const __m128 &a, const __m128 &b, const __m128 &c)
{
#ifdef __aarch64__
    // a * b - c
	//return vfmaq_f32(vsubq_f32(vdupq_n_s32(0.0f), c), a, b);
	return vfmaq_f32(vnegq_f32(c), a, b);	
#else
    return vsubq_f32(vmulq_f32(a, b), c);
#endif
}

inline __m128i _mm_packus_epi32(const __m128i &a, const __m128i &b)
{
    return vcombine_u16(vqmovn_u32(a), vqmovn_u32(b));
}

inline __m128i _mm_packus_epi32(const __m128i &a)
{
    uint16x4_t a_16x4 = vqmovn_u32(a);
    return vcombine_u16(a_16x4, a_16x4);
}

inline uint16_t _mm_max_epu16(const __m128i &a)
{
#ifdef __aarch64__
    return vmaxvq_u16(a);
#else
	//uint16x8_t a16x8 = vreinterpretq_u16_m128i(a);
	//uint16_t max_val = a16x8[0];
	//for (uint32_t i = 1; i < 8; ++i)
	//{
	//	max_val = a16x8[i] > max_val ? a16x8[i] : max_val;
	//}
	//return max_val;
	uint16x4_t tmp;
	tmp = vmax_u16(vget_low_u16(vreinterpretq_u16_m128i(a)),
		vget_high_u16(vreinterpretq_u16_m128i(a)));
	tmp = vpmax_u16(tmp, tmp);
	tmp = vpmax_u16(tmp, tmp);
	return vget_lane_u16(tmp, 0);
#endif
}

inline uint16_t _mm_max_epu16_even(const __m128i &a)
{
	return _mm_max_epu16(a);

#ifdef __aarch64__
	return vmaxvq_u16(a);
#else
	//armv7 code need to be tested to use
	uint32x2_t tmp;
	tmp = vmax_u32(vget_low_u32(vreinterpretq_u32_m128i(a)),
		vget_high_u32(vreinterpretq_u32_m128i(a)));
	tmp = vpmax_u32(tmp, tmp);
	return vget_lane_u16(tmp, 0);
#endif
}

inline uint16_t _mm_min_epu16(const __m128i &a)
{
#ifdef __aarch64__
    return vminvq_u16(a);
#else
    //uint16x8_t a16x8 = vreinterpretq_u16_m128i(a);
    //uint16_t min_val = a16x8[0];
    //for (uint32_t i = 1; i < 8; ++i)
    //{
    //    min_val = a16x8[i] < min_val ? a16x8[i] : min_val;
    //}
    //return min_val;
	
	//code reference https://github.com/DLTcollab/sse2neon/blob/master/sse2neon.h
	uint16x4_t tmp;
	tmp = vmin_u16(vget_low_u16(vreinterpretq_u16_m128i(a)),
		vget_high_u16(vreinterpretq_u16_m128i(a)));
	tmp = vpmin_u16(tmp, tmp);
	tmp = vpmin_u16(tmp, tmp);
	return vget_lane_u16(tmp, 0);
#endif
}

inline __m128i _mm_min_epu16(const __m128 &a, const __m128 &b)
{
    return vminq_u16(a, b);
}

inline __m128i _mm_max_epu16(const __m128i &a, const __m128i &b)
{
    return vmaxq_u16(a, b);
}

//added by soc
inline __m128i _mm_cmple_epu16_soc(const __m128i &a, const __m128i &b)
{
	return vcleq_u16(a, b);  //supported in v7/A32/A64
}
inline __m128i _mm_cmpge_epu16_soc(const __m128i &a, const __m128i &b)
{
	return vcgeq_u16(a, b);  //supported in v7/A32/A64
}

inline __m128i _mm_cmplt_epu16_soc(const __m128i &a, const __m128i &b)
{
	return vcltq_u16(a, b);  //supported in v7/A32/A64
}

inline __m128i _mm_cmplt_epu8_soc(const __m128i &a, const __m128i &b)
{
	return vcltq_u8(a, b);  //supported in v7/A32/A64
}

// Compares for less than abs
inline __m128 _mm_cmpAbslt_ps_soc(__m128 a, __m128 b)
{
	//reference https://www.cnblogs.com/fellow1988/p/12283792.html
	return vreinterpretq_m128_u32(vcaltq_f32(vreinterpretq_f32_m128(a), vreinterpretq_f32_m128(b)));
}


inline __m128i _mm_stream_load_si128(__m128i *mem_addr)
{
    return *mem_addr;
}

#define _MM_TRANSPOSE4_PS(row0, row1, row2, row3) \
    do                                            \
    {                                             \
        float32x4x2_t ROW01 = vtrnq_f32(row0, row1);              \
        float32x4x2_t ROW23 = vtrnq_f32(row2, row3);                      \
       row0 = vcombine_f32(vget_low_f32(ROW01.val[0]),                     \
				vget_low_f32(ROW23.val[0]));                  \
		row1 = vcombine_f32(vget_low_f32(ROW01.val[1]), \
			vget_low_f32(ROW23.val[1]));  \
		row2 = vcombine_f32(vget_high_f32(ROW01.val[0]), \
			vget_high_f32(ROW23.val[0])); \
		row3 = vcombine_f32(vget_high_f32(ROW01.val[1]), \
			vget_high_f32(ROW23.val[1])); \
    } while (0);


#define _MM_TRANSPOSE4_PS_43(row0, row1, row2, row3) \
    do                                            \
    {                                             \
        float32x4x2_t ROW01 = vtrnq_f32(row0, row1);              \
        float32x4x2_t ROW23 = vtrnq_f32(row2, row3);                      \
       row0 = vcombine_f32(vget_low_f32(ROW01.val[0]),                     \
				vget_low_f32(ROW23.val[0]));                  \
		row1 = vcombine_f32(vget_low_f32(ROW01.val[1]), \
			vget_low_f32(ROW23.val[1]));  \
		row2 = vcombine_f32(vget_high_f32(ROW01.val[0]), \
			vget_high_f32(ROW23.val[0])); \
    } while (0);


#define _MM_TRANSPOSE4_EPI32(row0, row1, row2, row3) \
    do                                            \
    {                                             \
        int32x4x2_t ROW01 = vtrnq_s32(row0, row1);              \
        int32x4x2_t ROW23 = vtrnq_s32(row2, row3);                      \
       row0 = vcombine_s32(vget_low_s32(ROW01.val[0]),                     \
				vget_low_s32(ROW23.val[0]));                  \
		row1 = vcombine_s32(vget_low_s32(ROW01.val[1]), \
			vget_low_s32(ROW23.val[1]));  \
		row2 = vcombine_s32(vget_high_s32(ROW01.val[0]), \
			vget_high_s32(ROW23.val[0])); \
		row3 = vcombine_s32(vget_high_s32(ROW01.val[1]), \
			vget_high_s32(ROW23.val[1])); \
    } while (0);


inline __m128 _mm_dp_ps(const __m128 &a, const __m128 &b, const __m128 &mask)
{
    __m128 m = _mm_mul_ps(_mm_mul_ps(a, mask), _mm_mul_ps(b, mask));
    __m128 t = _mm_add_ps(m, _mm_shuffle_ps(m, m, _MM_SHUFFLE(2, 3, 0, 1)));
    return _mm_add_ps(t, _mm_shuffle_ps(t, t, _MM_SHUFFLE(1, 0, 3, 2)));
}

inline __m128 _mm_dp_ps(const __m128 &a, const __m128 &b, const int imm8)
{
    if (imm8 == 0x7F)
    {
		__m128 m = _mm_mul_ps(a, b);
		return _mm_set_ps1(m[0] + m[1] + m[2]);
    }
    else if (imm8 == 0xFF)
    {
        __m128 m = _mm_mul_ps(a, b);
        return _mm_set_ps1(m[0] + m[1] + m[2] + m[3]);
    }

    __m128 mask = _mm_setr_ps((imm8 >> 4) & 1, (imm8 >> 5) & 1, (imm8 >> 6) & 1, (imm8 >> 7) & 1);
    __m128 m = _mm_mul_ps(_mm_mul_ps(a, mask), _mm_mul_ps(b, mask));
    __m128 t = _mm_add_ps(m, _mm_shuffle_ps(m, m, _MM_SHUFFLE(2, 3, 0, 1)));
    return _mm_add_ps(t, _mm_shuffle_ps(t, t, _MM_SHUFFLE(1, 0, 3, 2)));
}

inline float _mm_dp_ps_float_soc(const __m128 &a, const __m128 &b)
{
	__m128 m = _mm_mul_ps(a, b);
	return (m[0] + m[1] + m[2] + m[3]);
}

inline int _mm_extract_epi32(const __m128i &a, const int imm8)
{
    uint32x4_t &ia = *(uint32x4_t *)&a;
    return ia[imm8];
}

inline int _mm_testz_ps(const __m128 &a, const __m128 &b)
{
    __m128 tmp = _mm_and_ps(a, b);
    if (_mm_same_sign0(tmp))
    {
        return 1;
    }
    return 0;
}

inline __m128 _mm_set_w(const __m128 &a, float w)
{
    __m128 ret = a;
    ret[3] = w;
    return ret;
}

inline __m128 _mm_blendv_ps(const __m128 &a, const __m128 &b, const __m128 &mask)
{
    return vbslq_f32(mask, b, a);
}

// Multiply the low unsigned 32-bit integers from each packed 64-bit element in
// a and b, and store the unsigned 64-bit results in dst.
//
//   r0 :=  (a0 & 0xFFFFFFFF) * (b0 & 0xFFFFFFFF)
//   r1 :=  (a2 & 0xFFFFFFFF) * (b2 & 0xFFFFFFFF)
inline __m128i _mm_mul_epu32(__m128i a, __m128i b)
{
// vmull_u32 upcasts instead of masking, so we downcast.
uint32x2_t a_lo = vmovn_u64(vreinterpretq_u64_m128i(a));
uint32x2_t b_lo = vmovn_u64(vreinterpretq_u64_m128i(b));
return vreinterpretq_m128i_u64(vmull_u32(a_lo, b_lo));
}


inline __m128i _mm_unpacklo_epi8_soc(uint64_t b)
{
#if defined(__aarch64__)
	__m128i input = _mm_set_epi64x(0, b);
	return vreinterpretq_m128i_s8(
		vzip1q_s8(vdupq_n_s32(0), vreinterpretq_s8_m128i(input)));
#else
	int8x8_t a1 = vcreate_s8(0);
	int8x8_t b1 = vcreate_s8(b);
	int8x8x2_t result = vzip_s8(a1, b1);
	return vreinterpretq_m128i_s8(vcombine_s8(result.val[0], result.val[1]));
#endif
}

#if defined(__GNUC__) || defined(__clang__)
#pragma pop_macro("ALIGN_STRUCT")
#pragma pop_macro("inline")
#endif

#endif

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


#if defined(SDOC_NATIVE) && !defined(__aarch64__)
#pragma once
#include <immintrin.h>
#include <algorithm>
#pragma warning( disable : 4305  )
#pragma warning( disable : 4309  )
namespace 
{
	static const __m128i mask0 = _mm_setzero_si128();
	static const __m128i mask1 = _mm_cmpeq_epi16(mask0, mask0);
	static const __m128i mask8_32_even_odd = _mm_setr_epi8(0, 1, 4, 5, 8, 9, 12, 13, 2, 3, 6, 7, 10, 11, 14, 15);

	__m128i CvtU16(const __m128i& a)
	{
		__m128i res = _mm_cmpeq_epi16(_mm_srli_epi32(a, 16), mask0); // saturated 0
		res = _mm_or_si128(a, _mm_andnot_si128(res, mask1)); // saturated FFFF
		return _mm_shuffle_epi8(res, mask8_32_even_odd);
	}
} // namespace

#define _MM_TRANSPOSE4_EPI32(row0, row1, row2, row3) {              \
            __m128i _Tmp3, _Tmp2, _Tmp1, _Tmp0;                     \
                                                                    \
            _Tmp0   = _mm_unpacklo_epi32((row0), (row1));           \
            _Tmp1   = _mm_unpacklo_epi32((row2), (row3));           \
            _Tmp2   = _mm_unpackhi_epi32((row0), (row1));           \
            _Tmp3   = _mm_unpackhi_epi32((row2), (row3));           \
                                                                    \
            (row0) = _mm_unpacklo_epi64(_Tmp0, _Tmp1);              \
            (row1) = _mm_unpackhi_epi64(_Tmp0, _Tmp1);              \
            (row2) = _mm_unpacklo_epi64(_Tmp2, _Tmp3);              \
            (row3) = _mm_unpackhi_epi64(_Tmp2, _Tmp3);              \
        }


		

// Single float shuffle
inline __m128 _mm_shuffle_ps_single_index(__m128 a, int idx)
{
	switch (idx)
	{
	case 0:
		return _mm_shuffle_ps(a, a, _MM_SHUFFLE(0, 0, 0, 0));
	case 1:
		return _mm_shuffle_ps(a, a, _MM_SHUFFLE(1, 1, 1, 1));
	case 2:
		return _mm_shuffle_ps(a, a, _MM_SHUFFLE(2, 2, 2, 2));
	case 3:
		return _mm_shuffle_ps(a, a, _MM_SHUFFLE(3, 3, 3, 3));
	default:
		return _mm_set1_ps(0);
	}
}

inline __m128i _mm_cmple_epu16_soc(const __m128i &a, const __m128i &b)
{
	// Returns 0xFFFF where x <= y:
	//return  _mm_cmpeq_epi16(_mm_subs_epu16(a, b), _mm_setzero_si128());
	  return  _mm_cmpeq_epi16(_mm_max_epu16(a, b), b); //supported in v7/A32/A64
}


inline __m128i _mm_cmplt_epu16_soc(const __m128i &a, const __m128i &b)
{
	auto aleb =  _mm_cmpeq_epi16(_mm_max_epu16(a, b), a); //supported in v7/A32/A64,  a >= b
	return _mm_xor_si128(aleb, _mm_set1_epi16(0xFFFF));   //revert of a>=b, means a < b
}



inline int _mm_movemask_epi16_soc(__m128i a)
{
	__m128i lensAll = _mm_shuffle_epi8(a, _mm_setr_epi8(0, 2, 4, 6, 8, 10, 12, 14, -1, -1, -1, -1, -1, -1, -1, -1));
	return _mm_movemask_epi8(lensAll);
}

inline __m128 _mm_abs_ps_soc(__m128 &a) 
{
	__m128 minusZero = _mm_set1_ps(-0.0f);
	return _mm_andnot_ps(minusZero, a);
}
// negates each element in a vector.
inline __m128 _mm_negate_ps_soc(__m128 a)
{
	__m128 minusZero = _mm_set1_ps(-0.0f);
	return _mm_xor_ps(minusZero, a);
}
// Multiplies the four single-precision, floating-point values of a and float b
inline __m128 _mm_mul_ps_scalar_soc(__m128 a, float b)
{
	return _mm_mul_ps(a, _mm_set1_ps(b));
}
// Multiplies the four single-precision, floating-point values of a and float b
inline __m128 _mm_mul_ps_scalar_soc(__m128 a, __m128 b, int bIdx)
{
	return _mm_mul_ps(a, _mm_shuffle_ps_single_index(b, bIdx));
}
inline float _mm_dp_ps_float_soc(__m128 a, __m128 b)
{
	__m128 dist5 = _mm_dp_ps(a, b, 0xff);
	float r[4];
	_mm_store_ps(r, dist5);
	return r[0];
}

inline __m128 _mm_cmpAbslt_ps_soc(__m128 a, __m128 b)
{
	return _mm_cmplt_ps(_mm_abs_ps_soc(a), _mm_abs_ps_soc(b));
}
inline __m128 _mm_fmadd_ps_soc(const __m128 &a, float b, const __m128 &c)
{
	return  _mm_fmadd_ps(a, _mm_set1_ps(b), c);
}
inline __m128i _mm_cmple_epi32_soc(__m128i a, __m128i b) {
	return _mm_andnot_si128(_mm_cmpgt_epi32(a, b), _mm_set_epi32(~0, ~0, ~0, ~0));
}

inline __m128i _mm_cmpgt_epu8(__m128i x, __m128i y)
{
	// Returns 0xFF where x > y:
	return _mm_andnot_si128(
		_mm_cmpeq_epi8(x, y),
		_mm_cmpeq_epi8(_mm_max_epu8(x, y), x)
	);
}
inline __m128i _mm_cmplt_epu8_soc(__m128i x, __m128i y)
{
	// Returns 0xFF where x < y:
	return _mm_cmpgt_epu8(y, x);
}

inline __m128 _mm_shuffle_ps_1010_soc(__m128 a, __m128 b) {
	return _mm_shuffle_ps(a, b, (1<<2) + (1<<6) ); //01000100
}
inline __m128 _mm_shuffle_ps_3232_soc(__m128 a, __m128 b) {
	return _mm_shuffle_ps(a, b, 238); //11101110
}

inline __m128i _mm_srli_epi8(const __m128i &_A, int _Imm) {
	return _mm_and_si128(_mm_set1_epi8(0xFF >> _Imm), _mm_srli_epi32(_A, _Imm));
}
inline __m128i _mm_slli_epi8(const __m128i &_A, int _Imm) {
	return _mm_and_si128(_mm_set1_epi8(0xFF << _Imm), _mm_slli_epi32(_A, _Imm));
}


inline uint16_t _mm_max_epu16_even(const __m128i& a)
{
	__m128i b = _mm_srli_epi64(a, 32);
	b = _mm_max_epu16(a, b);
	uint16_t * data = (uint16_t*)& b;
	return std::max(data[0], data[4]);

////#if defined(SDOC_WIN)
////	__declspec(align(16)) uint16_t result[8] = { 0 };
////#elif defined(SDOC_OSX)
////	__attribute__((aligned(16))) uint16_t result[8] = { 0 };
////#endif
////
////	_mm_storeu_si128((__m128i *)result, a);
////	uint16_t r2 = *std::max_element(result, result + 8);
////	if (r1 != r2) std::cout << "ERROR " << r1 << " r2 " << r2 << std::endl;
////	return r1;
}

inline __m128i _mm_packus_epi32(const __m128i &a)
{
	__m128i low = CvtU16(a);
	return _mm_unpacklo_epi64(low, low);
}

inline __m128 _mm_hadd_ps(__m128 a)
{
	return _mm_hadd_ps(a, a);
}

inline uint16_t _mm_min_epu16(const __m128i &a)
{
////#if defined(SDOC_WIN)
////	__declspec(align(16)) uint16_t result[8] = { 0 };
////#elif defined(SDOC_OSX)
////    __attribute__((aligned(16))) uint16_t result[8] = { 0 };
////#endif
////	_mm_storeu_si128((__m128i *)result, a);
////	uint16_t x= *std::min_element(result, result + 8);

	__m128i c = _mm_srli_epi32(a, 16);
	c = _mm_min_epu16(a, c);

	__m128i b = _mm_srli_epi64(c, 32);
	b = _mm_min_epu16(c, b);
	uint16_t * data = (uint16_t*)& b;
	return std::min(data[0], data[4]);
}

inline bool _mm_same_sign0(__m128 a)
{
	int val = _mm_movemask_ps(a);
	
	return val == 0;
}

inline __m128 _mm_sum4_ps_soc(__m128 a)
{
	__m128 b = _mm_hadd_ps(a);
	return _mm_hadd_ps(b);
}
inline __m128 _mm_max4_ps_soc(__m128 a)
{
	__m128 maxExtent = _mm_max_ps(a, _mm_shuffle_ps(a, a, _MM_SHUFFLE(1, 0, 3, 2)));
	return _mm_max_ps(maxExtent, _mm_shuffle_ps(maxExtent, maxExtent, _MM_SHUFFLE(2, 3, 0, 1)));
}
//inline __m128 _mm_min_minXY_min_maxXY_ps_soc(__m128 minsXY, __m128 maxsXY)
//{
//	float r[4];
//	_mm_store_ps(r, minsXY);
//
//	r[0] = std::min(r[0], r[1]);
//	r[1] = std::min(r[2], r[3]);
//
//	float m[4];
//	_mm_store_ps(m, maxsXY);
//	r[2] = std::min(m[0], m[1]);
//	r[3] = std::min(m[2], m[3]);
//	
//	return _mm_load_ps(r);
//}


//inline bool _mm_same_sign0_firstThree_soc(__m128 a)
//{
//	int val = _mm_movemask_ps(a);
//	val &= 7;
//	return val == 0;
//}

//inline bool _mm_any_sign0_firstThree_soc(__m128 a)
//{
//	int val = _mm_movemask_ps(a);
//	val &= 7;
//	return val != 7;
//}

//inline bool _mm_any_sign0_soc(__m128 a)
//{
//	int val = _mm_movemask_ps(a);
//	val &= 15;
//	return val != 15;
//}

inline bool _mm_same_sign1_firstThree_soc(__m128 a)
{
	int val = _mm_movemask_ps(a);
	val &= 7;
	return val == 7;
}

inline bool _mm_same_sign1_soc(__m128 a)
{
	int val = _mm_movemask_ps(a);
	return val == 0xF;
}

inline __m128 _mm_set_w(const __m128 &a, float w)
{
	return _mm_blend_ps(a, _mm_set1_ps(w), 0b1000);
}
inline uint64_t _mm_getUint16Max8_soc(__m128i a)
{
	__m128i r16;
	r16 = _mm_srli_epi16(a, 8); //after right shift b>=1 unsigned var fits into signed range, so we could use _mm_packus_epi16 (signed 16 to unsigned 8)
	r16 = _mm_packus_epi16(r16, r16); //saturate and  narrow, use low 64 bits only

	uint64_t r[2];
	_mm_store_si128((__m128i*)r, r16);
	return r[0];
}



inline __m128i _mm_unpacklo_epi8_soc(uint64_t b)
{
	return  _mm_unpacklo_epi8(_mm_setzero_si128(), _mm_set_epi64x(0, b));
}
#endif
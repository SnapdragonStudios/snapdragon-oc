/*
** Vec3f from https://github.com/sp4cerat/Fast-Quadric-Mesh-Simplification/blob/master/src.gl/VecMath.h
** https://github.com/sp4cerat/Fast-Quadric-Mesh-Simplification/blob/master/README.md
**
Copyright © 2015-2019 Spacerat and contributors

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

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
	struct Vec3f
	{
		Vec3f() : X(0.0), Y(0.0), Z(0.0) {}
		Vec3f(float x, float y, float z) : X(x), Y(y), Z(z) {}
		Vec3f(const float *p) : X(p[0]), Y(p[1]), Z(p[2]) {}
		float X, Y, Z;
		Vec3f normalize() const
		{
			float len = length();
			if (len == 0.0f)
			{
				return Vec3f();
			}
			float invLen = 1.0f / len;
			return Vec3f(X*invLen, Y*invLen, Z*invLen);
		}
		Vec3f cross(const Vec3f &rhs) const
		{
			return Vec3f(Y * rhs.Z - Z * rhs.Y,
				Z * rhs.X - X * rhs.Z,
				X * rhs.Y - Y * rhs.X);
		}
		float dot(const Vec3f &rhs) const
		{
			return X * rhs.X + Y * rhs.Y + Z * rhs.Z;
		}

		float length() const
		{
			return std::sqrt(X * X + Y * Y + Z * Z);
		} 
		Vec3f operator-(const Vec3f &rhs) const
		{
			return Vec3f(this->X - rhs.X, this->Y - rhs.Y, this->Z - rhs.Z);
		}
	};
} // namespace common

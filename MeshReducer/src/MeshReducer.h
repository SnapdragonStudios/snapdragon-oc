/*
** https://github.com/sp4cerat/Fast-Quadric-Mesh-Simplification/blob/master/README.md
**
Copyright © 2015-2019 Spacerat and contributors

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

//============================================================================================================
//   DO NOT REMOVE THIS HEADER UNDER QUALCOMM PRODUCT KIT LICENSE AGREEMENT
//
//                  Copyright (c) 2023 QUALCOMM Technologies Inc.
//                              All Rights Reserved.
//
//                       Snapdragon™ Occlusion Culling
//                       Snapdragon™ OC
//
//                       Developed by Snapdragon Studios™
//               
//============================================================================================================

#pragma once

#include <vector>
#include <stdint.h>

#include <cmath>
#include <cstring>


#include <sys/types.h>
#include <sys/stat.h>
#include <string>
namespace common
{


	// double vector
	struct Vec3
	{
		Vec3() : X(0.0), Y(0.0), Z(0.0) {}
		Vec3(double x, double y, double z) : X(x), Y(y), Z(z) {}
		Vec3(double val) : X(val), Y(val), Z(val) {}
		double X, Y, Z;

		bool operator==(const Vec3& rhs) const
		{
			return (X == rhs.X && Y == rhs.Y && Z == rhs.Z);
		}

		Vec3 operator-(const Vec3& rhs) const
		{
			return Vec3(this->X - rhs.X, this->Y - rhs.Y, this->Z - rhs.Z);
		}

		Vec3 operator+(const Vec3& rhs) const
		{
			return Vec3(this->X + rhs.X, this->Y + rhs.Y, this->Z + rhs.Z);
		}

		Vec3 operator/(double div) const
		{
			return Vec3(this->X / div, this->Y / div, this->Z / div);
		}

		Vec3 operator*=(double scale)
		{
			this->X *= scale;
			this->Y *= scale;
			this->Z *= scale;
			return *this;
		}

		bool operator<(const Vec3& rhs) const
		{
			if (X < rhs.X)
			{
				return true;
			}
			else if (X > rhs.X)
			{
				return false;
			}

			// X == rhs.X
			if (Y < rhs.Y)
			{
				return true;
			}
			else if (Y > rhs.Y)
			{
				return false;
			}

			// Y == rhs.Y
			if (Z < rhs.Z)
			{
				return true;
			}
			else if (Z > rhs.Z)
			{
				return false;
			}

			return false;
		}

		Vec3 normalize() const
		{
			double len = length();
			if (len == 0.0)
			{
				return Vec3();
			}
			return Vec3(X / len, Y / len, Z / len);
		}

		double dot(const Vec3& rhs) const
		{
			return X * rhs.X + Y * rhs.Y + Z * rhs.Z;
		}

		Vec3 cross(const Vec3& rhs) const
		{
			return Vec3(Y * rhs.Z - Z * rhs.Y,
				Z * rhs.X - X * rhs.Z,
				X * rhs.Y - Y * rhs.X);
		}

		double length() const
		{
			return std::sqrt(X * X + Y * Y + Z * Z);
		}

		double squaredLength() const
		{
			return X * X + Y * Y + Z * Z;
		}


		static double squaredLength(const Vec3& v1, const Vec3& v2)
		{
			double x = v1.X - v2.X;
			double y = v1.Y - v2.Y;
			double z = v1.Z - v2.Z;
			return x * x + y * y + z * z;
		}

	};

	class SymetricMatrix
	{
	public:
		SymetricMatrix()
		{
			memset(m, 0, sizeof(m));
		}
		SymetricMatrix(bool lateInit)
		{
			if (lateInit) {}
			else
				memset(m, 0, sizeof(m));
		}

		explicit SymetricMatrix(
			double m11, double m12, double m13, double m14,
			double m22, double m23, double m24,
			double m33, double m34,
			double m44)
		{
			m[0] = m11;
			m[1] = m12;
			m[2] = m13;
			m[3] = m14;
			m[4] = m22;
			m[5] = m23;
			m[6] = m24;
			m[7] = m33;
			m[8] = m34;
			m[9] = m44;
		}
		void syncByNormalD(const Vec3& n, double d)
		{
			double a = n.X;
			double b = n.Y;
			double c = n.Z;

			m[0] = a * a;
			m[1] = a * b;
			m[2] = a * c;
			m[3] = a * d;
			m[4] = b * b;
			m[5] = b * c;
			m[6] = b * d;
			m[7] = c * c;
			m[8] = c * d;
			m[9] = d * d;
		}
		explicit SymetricMatrix(double a, double b, double c, double d)
		{
			m[0] = a * a;
			m[1] = a * b;
			m[2] = a * c;
			m[3] = a * d;
			m[4] = b * b;
			m[5] = b * c;
			m[6] = b * d;
			m[7] = c * c;
			m[8] = c * d;
			m[9] = d * d;
		}

		void reset()
		{
			memset(m, 0, sizeof(m));
		}

		double operator[](unsigned int index) const
		{
			return m[index];
		}

		bool solve(Vec3& result) const
		{
			double det = this->det(0, 1, 2, 1, 4, 5, 2, 5, 7);
			if (det == 0.0)
			{
				return false;
			}

			double c = 1 / det;
			result.X = -c * this->det(1, 2, 3, 4, 5, 6, 5, 7, 8);
			result.Y = c * this->det(0, 2, 3, 1, 5, 6, 2, 7, 8);
			result.Z = -c * this->det(0, 1, 3, 1, 4, 6, 2, 5, 8);

			return true;
		}

		bool solveForFace(Vec3& result, double& cost) const
		{
			double det = this->det(0, 1, 2, 1, 4, 5, 2, 5, 7);
			if (abs(det) < 1e-10)
			{
				return false;
			}

			double c = 1 / det;
			result.X = -c * this->det(1, 2, 3, 4, 5, 6, 5, 7, 8);
			result.Y = c * this->det(0, 2, 3, 1, 5, 6, 2, 7, 8);
			result.Z = -c * this->det(0, 1, 3, 1, 4, 6, 2, 5, 8);

			//this is based on the fact that 
			//	q11 q12 q13 q14		px	  0
			//	q21 q22 q23 q24	 *  py  = 0
			//	q31 q32 q33 q34		pz	  0
			//	0	0	0	1		1	  1 
			//	so the cost = p^t Q p = q41* px + q42*py +q43*pz+q44
			//=m[3] * result.X + m[6] * result.Y + m[8] * result.Z + m[9];


			//double row1 = m[0] * result.X + m[1] * result.Y + m[2] * result.Z + m[3];
			//double row2 = m[1] * result.X + m[4] * result.Y + m[5] * result.Z + m[6];
			//double row3 = m[2] * result.X + m[5] * result.Y + m[7] * result.Z + m[8];

			//if (abs(row1) + abs(row2) + abs(row3) > 200)
			cost = m[3] * result.X + m[6] * result.Y + m[8] * result.Z + m[9];
			return true;
		}

		double det(
			unsigned int a11, unsigned int a12, unsigned int a13,
			unsigned int a21, unsigned int a22, unsigned int a23,
			unsigned int a31, unsigned int a32, unsigned int a33) const
		{
			double det = m[a11] * (m[a22] * m[a33] - m[a23] * m[a32])
				+ m[a13] * (m[a21] * m[a32] - m[a22] * m[a31])
				+ m[a12] * (m[a23] * m[a31] - m[a21] * m[a33]);
			return det;

			////double det = m[a11] * m[a22] * m[a33] + m[a13] * m[a21] * m[a32] + m[a12] * m[a23] * m[a31] - m[a13] * m[a22] * m[a31] - m[a11] * m[a23] * m[a32] - m[a12] * m[a21] * m[a33];
			////return det;
		}

		const SymetricMatrix operator+(const SymetricMatrix& n) const
		{
			return SymetricMatrix(
				m[0] + n[0], m[1] + n[1], m[2] + n[2], m[3] + n[3],
				m[4] + n[4], m[5] + n[5], m[6] + n[6],
				m[7] + n[7], m[8] + n[8],
				m[9] + n[9]);
		}

		SymetricMatrix& operator+=(const SymetricMatrix& n)
		{
			m[0] += n[0];
			m[1] += n[1];
			m[2] += n[2];
			m[3] += n[3];
			m[4] += n[4];
			m[5] += n[5];
			m[6] += n[6];
			m[7] += n[7];
			m[8] += n[8];
			m[9] += n[9];
			return *this;
		}

		double m[10];
	};

class MeshReducer
{
public:
	static void release();
    static bool reduce(const float *vertices, const uint16_t *indices, unsigned int nVert, unsigned int nIdx,
		 float *reducedVertices, int &reducedVertNum, uint16_t* reducedIndices, int &reducedFaceNum, 
                       unsigned int nTarget);
};
} // namespace common

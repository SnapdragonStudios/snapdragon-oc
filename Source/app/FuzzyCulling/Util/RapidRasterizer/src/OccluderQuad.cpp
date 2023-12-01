//============================================================================================================
//
//
//                  Copyright (c) 2023, Qualcomm Innovation Center, Inc. All rights reserved.
//                              SPDX-License-Identifier: BSD-3-Clause
//
//============================================================================================================



#include "../OccluderQuad.h"
#include "MeshReducer.h"

#if defined(SDOC_WIN)
#pragma warning(disable : 4996)
#endif

#if defined(SDOC_NATIVE)
#include <assert.h>
#include <fstream>
#include <iostream>
#include <sstream>
#include <mutex>
#include "MathUtil.h"


#if defined(SDOC_WIN)
#include <direct.h>
#endif

#if defined(SDOC_NATIVE)
#include "Util/RapidRasterizer/MeshReducer.h"
#include "SOCUtil.h"
#endif
namespace util
{
	static int gDelayTerrainOccluderVertTh = 256;

	static float RightAngleUnitDotTh = 0; 
	static float ProjectLineMidAngleTh = 0; 
	static float PlaneQuadDotThreshold = 0; 
	static float QuadFourSameFaceTh = 0; 
	static float QuadTwoSameFaceTh = 0; 


	//set to 80, 10 for aggressive result
	static float  TerrainRectangleAngle = 88;      // [80, 89]
	static float  TerrainRectangleMergeAngle = 3;  // [1, 10]


	static constexpr float  ModelRectangleAngle = 89;
	static constexpr float  ModelRectangleMergeAngle = 1.1f;  // treat face normal angle smaller than 1.1 as plane quad


	static bool SaveSimplifyModel =  false;
	static std::string outputSaveDirectory = "D:\\TempSDOC";

	void OccluderQuad::ConfigControlParameters(bool isTerrain)
	{
		float degreeToRadian = 3.1415926f / 180.0f;
		if (isTerrain)
		{
			//relax parameters for terrain
			RightAngleUnitDotTh =  cos(TerrainRectangleAngle * degreeToRadian);
			ProjectLineMidAngleTh = cos(TerrainRectangleMergeAngle * degreeToRadian);
			PlaneQuadDotThreshold = ProjectLineMidAngleTh; // cos(5 degree)  //treat face normal angle smaller than 5 as plane quad
			QuadFourSameFaceTh = ProjectLineMidAngleTh; // cos(5 degree)  //treat face normal angle smaller than 5 as plane quad
			QuadTwoSameFaceTh = ProjectLineMidAngleTh;  //cos(3 degree)    //do not relax this constraints
		}
		else
		{
			//very strict controlling parameters
			RightAngleUnitDotTh = cos(ModelRectangleAngle * degreeToRadian);; // cos(89f * 3.1415926f / 180);
			ProjectLineMidAngleTh = cos(3 * degreeToRadian); //cos(3 degree)
			PlaneQuadDotThreshold = QuadFourSameFaceTh = QuadTwoSameFaceTh = cos(ModelRectangleMergeAngle * degreeToRadian); 

		}
	}

	static constexpr float QuadFourSameSquareAreaTh = 0.949f; //only merge roughly same area with threshold 0.94868329805

	enum TerrainOpMode {
		None,
		SafeMerging,
		AggressiveMerging,
	};
	static int gTerrainGridOptimization = TerrainOpMode::AggressiveMerging; //this bool would be able to configured by developer


	////terrain stats

	static constexpr int QuadFourMergeRound = 4; 


	static bool bSimplifyMesh = 1;

	static int DebugOccluderIdx = 0;

	//add here to dump model and simplified model
	bool OccluderQuad::storeModel(const std::string &file_path,
		const float* vertices, int nVert,
		const uint16_t *indices, int indicesNum)
	{
        FILE *fileWriter = fopen(file_path.c_str(), "w");
        if (fileWriter == nullptr)
		{
			return false;
		}

		unsigned int nFace = static_cast<unsigned int>(indicesNum / 3);

		fprintf(fileWriter, "OFF\n");
		fprintf(fileWriter, "%d %d 0\n", nVert, nFace);

		for (int i_vert = 0; i_vert < nVert; ++i_vert)
		{
			fprintf(fileWriter, "%f %f %f\n",
				vertices[i_vert * 3],
				vertices[(i_vert * 3) + 1],
				vertices[(i_vert * 3) + 2]);
		}

		for (unsigned int i_face = 0; i_face < nFace; ++i_face)
		{
			fprintf(fileWriter, "3 %d %d %d\n",
				indices[i_face * 3],
				indices[i_face * 3 + 1],
				indices[i_face * 3 + 2]);
		}
		fclose(fileWriter);

		return true;
	}

	static __m128 cross(__m128 a, __m128 b)
	{
		__m128 a_yzx = _mm_shuffle_ps(a, a, _MM_SHUFFLE(3, 0, 2, 1));
		__m128 b_yzx = _mm_shuffle_ps(b, b, _MM_SHUFFLE(3, 0, 2, 1));
		__m128 c =  _mm_fmsub_ps(a, b_yzx, _mm_mul_ps(a_yzx, b));
		return _mm_shuffle_ps(c, c, _MM_SHUFFLE(3, 0, 2, 1));
	}

	static __m128 normalize(__m128 v)
	{
		return _mm_mul_ps(v, _mm_rsqrt_ps(_mm_dp_ps(v, v, 0x7F)));
	}

	static float dotSum3(__m128 a, __m128 b)
	{
		__m128 d = _mm_mul_ps(a, b);
		float * df = (float*)&d;
		return df[0] + df[1] + df[2];
	}

	static void ProjectToNormalLine(__m128 unitAB, __m128 b, __m128 &target)
	{
		__m128 midB = _mm_sub_ps(target, b);
		float projLen = dotSum3(midB, unitAB);
		target = _mm_add_ps(b, _mm_mul_ps(unitAB, _mm_set1_ps(projLen)));
	}

	void  OccluderQuad::checkRectangleQuad(MeshQuad * q, __m128*vertices)
	{
		__m128 p0 = vertices[q->vIdx[0]];
		__m128 p1 = vertices[q->vIdx[1]];
		__m128 p2 = vertices[q->vIdx[2]];
		__m128 p3 = vertices[q->vIdx[3]];

		__m128 p1p0 = normalize(_mm_sub_ps(p1, p0));
		__m128 p2p1 = normalize(_mm_sub_ps(p2, p1));
		__m128 p3p2 = normalize(_mm_sub_ps(p3, p2));
		__m128 p0p3 = normalize(_mm_sub_ps(p0, p3));

		float dot0 = std::abs(dotSum3(p1p0, p2p1));
		float dot1 = std::abs(dotSum3(p3p2, p2p1));
		float dot2 = std::abs(dotSum3(p3p2, p0p3));
		float dot3 = std::abs(dotSum3(p1p0, p0p3));

		q->IsRectangle = dot0 < RightAngleUnitDotTh && dot1 < RightAngleUnitDotTh &&dot2 < RightAngleUnitDotTh && dot3 < RightAngleUnitDotTh;
	}


	static util::OccluderBakeBuffer* gBakeBuffer = new util::OccluderBakeBuffer();

	void OccluderQuad::OfflineTestClearBakeBuffer()
	{
		if (gBakeBuffer != nullptr) {
			delete gBakeBuffer;
			gBakeBuffer = nullptr;
		}
	}

	void OccluderQuad::HandleSquareTerrainInput(OccluderBakeBuffer* pBakeBuffer, const uint16_t* indices, unsigned int nIdx, const float* inputVertices, int nVert, QuadTriIndex & qti, float quadAngle, int squareTerrainXPoints)
	{
		std::vector< uint16_t > vertexIndices;
		vertexIndices.reserve(nIdx);
		
		//*********************************************************************
		float maxX = -std::numeric_limits<float>::infinity();
		float minX = std::numeric_limits<float>::infinity();
		float maxY = -std::numeric_limits<float>::infinity();
		float minY = std::numeric_limits<float>::infinity();
		for (int x = 1; x < nVert; x++) 
		{
			int x3 = x * 3;
			minX = std::min<float>(minX, inputVertices[x3]);
			maxX = std::max<float>(maxX, inputVertices[x3]);
			minY = std::min<float>(minY, inputVertices[x3 + 1]);
			maxY = std::max<float>(maxY, inputVertices[x3 + 1]);
		}

		int totalGrids = nIdx / 6;
		int gridWidth = (int)sqrt(totalGrids);

		float xUnit = (maxX - minX) / gridWidth;
		float yUnit = (maxY - minY) / gridWidth;
		if (xUnit == 0 || yUnit == 0) 
		{
			OccluderQuad::decomposeToQuad(pBakeBuffer, indices, nIdx, inputVertices, nVert, qti, quadAngle, nullptr, true);
			return;
		}

		//*********************************************************************
		//Find grid idx

		std::vector<uint16_t> TerrainGridX;
		std::vector<uint16_t> TerrainGridY;
		std::vector<uint16_t> GridToVert;
		TerrainGridX.resize(nVert);
		TerrainGridY.resize(nVert);

		int axisPoints = gridWidth + 1;
		if (squareTerrainXPoints > 0) {
			gridWidth = squareTerrainXPoints - 1;
			axisPoints = squareTerrainXPoints;
		}
		int gridPointNum = axisPoints * axisPoints;
		GridToVert.resize(gridPointNum);
		int maxGridIdx = gridPointNum - 1;

		uint16_t* gridMap = new uint16_t[gridPointNum];
		memset(gridMap, 0, sizeof(uint16_t) * gridPointNum);
		for (int idx = nVert - 1; idx >=0; idx--)
		{
			int x3 = idx * 3;
			TerrainGridX[idx] = (int)(0.01f + (inputVertices[x3] - minX) / xUnit);
			TerrainGridY[idx] = (int)(0.01f + (inputVertices[x3 + 1] - minY) / yUnit);

			int gridIdx = TerrainGridY[idx] * axisPoints + TerrainGridX[idx];
			if (gridIdx <= maxGridIdx)
			{
				gridMap[gridIdx] ++;
				GridToVert[gridIdx] = idx;
			}
		}


		int activeCount = 0;
		int grid2Count = 0;
		for (int idx = 0; idx < maxGridIdx; idx++) {
			activeCount += gridMap[idx] > 0;
			grid2Count += gridMap[idx] - 1;// >= 2;
		}

		bool IsTerrain = squareTerrainXPoints > 0;
		if (IsTerrain == false) {
			IsTerrain = activeCount == maxGridIdx && (gridWidth * gridWidth == totalGrids);
			IsTerrain &= grid2Count == 0 || (grid2Count == gridWidth * 2 + 3);

			if (IsTerrain) {
				//further check whether it is composited by 4 sub 
				if (grid2Count == gridWidth * 2 + 3) {
					bool pass = true;
					int offset = (gridWidth / 2) * axisPoints;
					for (int idx = 0; idx <= gridWidth; idx++) {
						int value = gridMap[idx + offset];
						pass &= value == 2 || value == 4;
					}
					int midValue = gridMap[(gridWidth / 2) + offset];
					pass &= midValue == 4;
					offset = (gridWidth / 2);
					for (int idx = 0; idx <= gridWidth; idx++) {
						int value = gridMap[idx*axisPoints + offset];
						pass &= value == 2 || value == 4;
					}
					IsTerrain &= pass;
				}
			}
		}
		if (IsTerrain) 
		{
			//only convert vertex if pass the verification
			for (unsigned int idx = 0; idx < nIdx; idx+=3)
			{
				int vertIdx0 = indices[idx];
				int gridIdx0 = TerrainGridY[vertIdx0] * axisPoints + TerrainGridX[vertIdx0];

				int vertIdx1 = indices[idx+1];
				int gridIdx1 = TerrainGridY[vertIdx1] * axisPoints + TerrainGridX[vertIdx1];

				int vertIdx2 = indices[idx+2];
				int gridIdx2 = TerrainGridY[vertIdx2] * axisPoints + TerrainGridX[vertIdx2];

				if (gridIdx0 != gridIdx1 && gridIdx1 != gridIdx2 && gridIdx0 != gridIdx2)
				{
					vertexIndices.push_back(GridToVert[gridIdx0]);
					vertexIndices.push_back(GridToVert[gridIdx1]);
					vertexIndices.push_back(GridToVert[gridIdx2]);
				}
			}
			TerrainInput ti;
			ti.terrainAxisSize = gridWidth + 1;
			ti.gGridX = &TerrainGridX[0];
			ti.gGridY = &TerrainGridY[0];
			ti.gGridToVertID = &GridToVert[0];
			//*********************************************************************

		//	storeModel("D:\\Temp\\" + std::to_string(DebugOccluderIdx++) + "Input.off", inputVertices, nVert, &vertexIndices[0], vertexIndices.size());

			OccluderQuad::decomposeToQuad(pBakeBuffer, &vertexIndices[0], (uint32_t) vertexIndices.size(), inputVertices, nVert, qti, quadAngle, &ti, true);
		}
		else {
			OccluderQuad::decomposeToQuad(pBakeBuffer, &vertexIndices[0], (uint32_t) vertexIndices.size(), inputVertices, nVert, qti, quadAngle, nullptr, true);
		}

		delete[]gridMap;

		return;
	}
	
	void OccluderQuad::SetOutputPath(std::string output)
	{
		outputSaveDirectory = output;
	}
	static void CreateOutputDir() {
#if defined(SDOC_WIN)
	_mkdir(outputSaveDirectory.c_str());
#elif
std::system(("mkdir " + outputSaveDirectory).c_str());
#endif
	}

#ifdef SUPPORT_ALL_FEATURE
	static void LoadObj(const char * path, float quadAngle, int squareTerrainXPoints)
	{
		std::vector< uint16_t > vertexIndices;
		std::vector< float > temp_vertices;

		FILE *file = fopen(path, "r");
		if (file == nullptr)
		{
			return ;
		}
		while (1) {
			char lineHeader[128];
			// read the first word of the line
			int res = fscanf(file, "%s", lineHeader);
			if (res == EOF)
				break; // EOF = End Of File. Quit the loop.
			if (strcmp(lineHeader, "v") == 0) {
				float x, y, z;
				fscanf(file, "%f %f %f\n", &x, &y, &z);
				temp_vertices.push_back(x);
				temp_vertices.push_back(y);
				temp_vertices.push_back(z);
			}
			else if (strcmp(lineHeader, "f") == 0) {
				int32_t x, y, z;
				fscanf(file, "%d %d %d\n", &x, &y, &z);
				vertexIndices.push_back(x - 1);
				vertexIndices.push_back(y - 1);
				vertexIndices.push_back(z - 1);
			}
		}
		int nVert = (uint32_t)temp_vertices.size() / 3;
		uint32_t nIdx = (uint32_t)vertexIndices.size();
		QuadTriIndex* qti = new QuadTriIndex();

		OccluderBakeBuffer* pbuffer = new OccluderBakeBuffer();
		if ( OccluderQuad::fitTerrainSquareProperty(nIdx, nVert, squareTerrainXPoints) || squareTerrainXPoints > 0)
		{
			OccluderQuad::HandleSquareTerrainInput(pbuffer, &vertexIndices[0], nIdx, &temp_vertices[0], nVert, *qti, quadAngle, squareTerrainXPoints);
		}
		else {
			OccluderQuad::decomposeToQuad(pbuffer, &vertexIndices[0], nIdx, &temp_vertices[0], nVert, *qti, quadAngle);
		}
		delete pbuffer;
		delete qti;
	}
	static void LoadOff(const char * path, float quadAngle, int squareTerrainXPoints)
	{
		std::vector< uint16_t > vertexIndices;
		std::vector< float > temp_vertices;

		FILE *file = fopen(path, "r");
		if (file == nullptr)
		{
			return;
		}
		while (1) {
			char lineHeader[128];
			// read the first word of the line
			int res = fscanf(file, "%s", lineHeader);
			if (res == EOF)
				break; // EOF = End Of File. Quit the loop.
			if (strcmp(lineHeader, "OFF") == 0) {
				int nVerts = 0;
				int nFaces = 0;
				int zero = 0;

				fscanf(file, "%d %d %d\n", &nVerts, &nFaces, &zero);
				temp_vertices.resize(nVerts * 3);
				float * pvert = &temp_vertices[0];
				for (int i = 0; i < nVerts; i++) 
				{
					fscanf(file, "%f %f %f\n", pvert, pvert+1, pvert+2);
					pvert += 3;
				}

				int three;
				vertexIndices.reserve(3 * nFaces);
				for (int i = 0; i < nFaces; i++) 
				{
					int a, b, c;
					fscanf(file, "%d %d %d %d\n", &three, &a, &b, &c);
					if (three == 3) {
						vertexIndices.push_back(a);
						vertexIndices.push_back(b);
						vertexIndices.push_back(c);
					}
				}

				QuadTriIndex* qti = new QuadTriIndex();
				OccluderBakeBuffer* pbuffer = new OccluderBakeBuffer();
				OccluderQuad::decomposeToQuad(pbuffer, &vertexIndices[0], nFaces*3, &temp_vertices[0], nVerts, *qti, quadAngle, 0);
				delete pbuffer;
				delete qti;
			}
		}
	}
#endif

	bool OccluderQuad::fitTerrainSquareProperty(uint32_t nIdx, uint32_t nVert, int terrainGridAxisPoint)
	{	
		if (terrainGridAxisPoint == 0) 
		{
			return false;
		}
		if (nVert >= (uint32_t)gDelayTerrainOccluderVertTh)
		{
			int totalGrids = nIdx / 3 / 2;
			int gridWidth = (int)sqrt(totalGrids);
			//it has square grid & verts
			if (gridWidth * gridWidth == totalGrids)
			{
				int maxGridPoint = (gridWidth + 1) * (gridWidth + 1);
				if (nVert == maxGridPoint ||
					nVert == maxGridPoint + gridWidth * 2 + 3)
				{
					return true;
				}
			}
		}
		return false;
	}


	static int gMaxVertNum = 1;
	static int gMaxIndexNum = 1;
	static int gMaxFaceNum = 1;
	void OccluderQuad::SetMaxVertIndexNum(int* value)
	{
		gMaxVertNum = std::max<int>(gMaxVertNum, value[0]);
		gMaxIndexNum = std::max<int>(gMaxIndexNum, value[1]);

		//****************************************************
		//to avoid dirty input
		gMaxVertNum &= 65535;
		gMaxIndexNum &= (65536 * 4 - 1);
		//****************************************************

		gMaxFaceNum = gMaxIndexNum / 3;
	}


	uint16_t* OccluderBakeBuffer::GetIndicesBySize(uint32_t size)
	{
		if (mIndices.size() < size)
		{
			uint32_t requestSize = std::max<int>(size, gMaxIndexNum);
			mIndices.resize(requestSize);
		}
		return &mIndices[0];
	}

	__m128* OccluderBakeBuffer::GetPointsBySize(uint32_t size)
	{
		if (mPoints.size() < size) {
			uint32_t requestSize = std::max<int>(size, gMaxVertNum);
			mPoints.resize(requestSize);
		}
		return &mPoints[0];
	}

	util::OccluderQuad::MeshFace* OccluderBakeBuffer::GetFaceBySize(uint32_t size)
	{
		if (mFaces.size() < size) {
			uint32_t requestSize = std::max<int>(size, gMaxFaceNum);
			mFaces.resize(requestSize);
		}
		for (uint32_t idx = 0; idx < size; idx++)
		{
			mFaces[idx].init(idx);
		}
		return &mFaces[0];
	}

	util::OccluderQuad::MeshQuad* OccluderBakeBuffer::GetQuadBySize(uint32_t size)
	{
		if (mQuads.size() < size)
		{
			uint32_t requestSize = std::max<int>(size, (gMaxFaceNum >> 1) + 1);
			mQuads.resize(requestSize);
		}
		return &mQuads[0];
	}

	util::OccluderQuad::MeshVertex* OccluderBakeBuffer::GetVertexBySize(uint32_t size)
	{
		if (mVertices.size() < size) {
			uint32_t requestSize = std::max<int>(size, gMaxVertNum);
			mVertices.resize(requestSize);

			for (uint32_t idx = 0; idx < requestSize; idx++)
			{
				mVertices[idx].VertID = idx;
			}
		}
		return &mVertices[0];
	}


	OccluderBakeBuffer::~OccluderBakeBuffer()
	{
		gMaxVertNum = 1;
		gMaxIndexNum = 1;
		gMaxFaceNum = 1;
	}

	const uint16_t* OccluderBakeBuffer::ReIndex(const uint16_t* indices, const float* inputVertices, int vertNum, unsigned int nIdx)
	{
		reIdx.resize(nIdx);
		remap.resize(vertNum);
		remap[0] = 0;
		for (int idx = 1; idx < vertNum; idx++)
		{
			remap[idx] = idx;

			for (int scan = 0; scan < idx; scan++)
			{
				int idx3 = idx * 3;
				int scan3 = scan * 3;
				if (inputVertices[idx3] == inputVertices[scan3] &&
					inputVertices[idx3 + 1] == inputVertices[scan3 + 1] &&
					inputVertices[idx3 + 2] == inputVertices[scan3 + 2])
				{
					remap[idx] = scan;

					break;
				}
			}
		}

		for (unsigned int idx = 0; idx < nIdx; idx++)
		{
			reIdx[idx] = remap[indices[idx]];
		}
		return reIdx.data();
	}

	void OccluderQuad::decomposeToQuad(OccluderBakeBuffer * pBakeBuffer, const uint16_t * indices, unsigned int nIdx, const float* inputVertices, int vertNum, QuadTriIndex & qti, float quadAngle, TerrainInput * terrain, bool checkedTerrain, int terrainGridAxisPoint)
	{
		bool isTerrain = terrain != nullptr;

		if (gTerrainGridOptimization != TerrainOpMode::None && checkedTerrain == false)
		{
			if (fitTerrainSquareProperty(nIdx, vertNum, terrainGridAxisPoint) || terrainGridAxisPoint > 0)
			{
				HandleSquareTerrainInput(pBakeBuffer, indices, nIdx, inputVertices, vertNum, qti, quadAngle, terrainGridAxisPoint);
				return;
			}
		}
		
		//this is used for possible later draw only
		qti.IsTerrain = terrainGridAxisPoint > 0;
		



		qti.IsTerrain |= isTerrain;

		ConfigControlParameters(isTerrain);

		if (vertNum <= 0 || nIdx < 3 || nIdx % 3 != 0)
		{
			return;//invalid input;
		}


		bool enableReIdx = true;

		//for non mobile platform, further check the duplication of vertex data
		if (enableReIdx && isTerrain == false)
		{
			indices = pBakeBuffer->ReIndex(indices, inputVertices, vertNum, nIdx);			
		}

		unsigned int nTri = nIdx / 3;

		quadAngle = std::min<float>(15, quadAngle);
		quadAngle = std::max<float>(1, quadAngle);

		float cosQuadAngle = cos(15.0f * 3.1415926f / 180); //default value, expect to be evaluated by compiler
		if (quadAngle != 15) 
		{
			cosQuadAngle = cos(quadAngle * 3.1415926f / 180);
		}


		__m128 refMin = _mm_set1_ps(std::numeric_limits<float>::infinity());
		__m128 refMax = _mm_set1_ps(-std::numeric_limits<float>::infinity());

		auto points = pBakeBuffer->GetPointsBySize(vertNum);
		for (int idx = 0; idx < vertNum; idx++)
		{
			const float *f = inputVertices + idx * 3;
			__m128 p = _mm_setr_ps(f[0], f[1], f[2], 1);
			refMin = _mm_min_ps(p, refMin);
			refMax = _mm_max_ps(p, refMax);
			points[idx] = p;
		}

		qti.verts = points;
		qti.refMax = refMax;
		qti.refMin = refMin;
		qti.nActiveVerts = vertNum;
		if (nTri > 65535 || nIdx <= 12 || quadAngle == 0 ) //not possible to form a Quad
		{
			qti.indices = nullptr;
			qti.nQuads = 0;
			qti.nTriangles = nIdx;
			return;
		}

		uint32_t nVert = static_cast<uint32_t>(vertNum);
		auto Vertices = pBakeBuffer->GetVertexBySize(nVert);

		for (uint32_t i_vert = 0; i_vert < nVert; i_vert++)
		{
			Vertices[i_vert].reset();
		}

		auto Faces = pBakeBuffer->GetFaceBySize(nTri);
		for (uint32_t i_tri = 0, i_vert = 0; i_tri < nTri; i_tri++, i_vert += 3)
		{
			auto v0 = &Vertices[indices[i_vert]];
			auto v1 = &Vertices[indices[i_vert + 1]];
			auto v2 = &Vertices[indices[i_vert + 2]];
			Faces[i_tri].init(v0, v1, v2);
		}


		for (uint32_t i_tri = 0; i_tri < nTri; ++i_tri)
		{
			MeshFace *f = &Faces[i_tri];

			auto edgeFace0 = f->Edges[0];
			__m128 p0 = points[f->nVertIdx[0]];
			__m128 p1 = points[edgeFace0.endVert];
			__m128 p2 = points[edgeFace0.apexVert];

			__m128 e1 = _mm_sub_ps(p1, p0);
			__m128 e2 = _mm_sub_ps(p2, p1);
			__m128 e3 = _mm_sub_ps(p2, p0);

			__m128 e1e3Cross = cross(e1, e3);

			__m128 area2 = _mm_sqrt_ps( _mm_dp_ps(e1e3Cross, e1e3Cross, 0x7F));
			f->UnitNormal = _mm_div_ps(e1e3Cross, area2);
			//real area is actually: 0.5 * areaT
			f->area = ((float*)&area2)[0];

			float len1 = (dotSum3(e1, e1)); //square length
			float len2 = (dotSum3(e2, e2));
			float len3 = (dotSum3(e3, e3));



			if (len1 >= len2 && len1 >= len3) f->LongestEdgeIdx = 0;
			else if (len2 >= len1 && len2 >= len3) f->LongestEdgeIdx = 1;
			else f->LongestEdgeIdx = 2;
		}

		int mergedQuad = 0;
		//match with smaller faceid, so start from face id 1
		for (uint32_t i_tri = 1; i_tri < nTri; ++i_tri)
		{
			MeshFace *f = &Faces[i_tri];

			{
				int longestEdgeIdx = f->LongestEdgeIdx;
				MeshEdgeFace *fe = &f->Edges[longestEdgeIdx];
				auto fv = f->nVertIdx[longestEdgeIdx];

				//purpose: find all the Smaller face that share the reverse of edge fe
				//ascending order linked list
				auto neighbors = Vertices[ fe->endVert].EdgeList;
				for (auto e = neighbors; e != nullptr; e = e->Next)
				{
					if (e->FaceId >= f->FaceID)
					{
						//neighbors are face id increasing linked list
						//we only test each face with the smaller face to avoid duplicate calculation
						//up to this stage, all the face would be larger than faceID, 
						//so use break, not continue

						break;
					}
					if (e->endVert == fv)
					{
						MeshFace * eface = &Faces[e->FaceId];
						if (eface->PairFace == nullptr)
						{
							float dotSum = dotSum3(f->UnitNormal, eface->UnitNormal);
							//allow SDOC Quad drawing
							if (dotSum >= cosQuadAngle)
							{
								mergedQuad++;

								eface->PairFace = f;
								f->PairFace = eface;

								f->MergedEdgeIdx = fe->edgeIdx;
								eface->MergedEdgeIdx = e->edgeIdx;
								
								

								f->IsPlanarQuad = eface->IsPlanarQuad = (dotSum >= PlaneQuadDotThreshold) && bSimplifyMesh;
							}
						}
						break;
					}
				}
			}
		}

		if (mergedQuad == 0) //not form a Quad
		{
			qti.indices = nullptr;
			qti.nQuads = 0;
			qti.nTriangles = nIdx;
			return;
		}
		
		if (SaveSimplifyModel) {
			CreateOutputDir();
			std::string file_path = outputSaveDirectory + "\\" + std::to_string(DebugOccluderIdx++) + "input.off";
			storeModel(file_path, inputVertices, nVert, indices, nIdx);
		}


		if ( bSimplifyMesh)
		{
			//quick check whether it is actually a simple quad
			__m128 refMin = qti.refMin;
			__m128 refMax = qti.refMax;

			__m128 extent = _mm_sub_ps(refMax, refMin);
			float * aabbf = (float*)&extent;
			float max = std::max<float>(aabbf[0], aabbf[1]);
			max = std::max<float>(aabbf[2], max);
			float planarTh = max * 0.000001f;

			int xFlat = aabbf[0] < planarTh;
			int yFlat = aabbf[1] < planarTh;
			int zFlat = aabbf[2] < planarTh;
			//Optimization: check whether it is a simple quad and output the quad if conditions allow
			if (xFlat + yFlat + zFlat == 1)
			{
				//check whether point to same direction
				bool allSameDirection = true;
				__m128 normal = Faces[0].UnitNormal;
				for (uint32_t i_tri = 1; i_tri < nTri; ++i_tri)
				{
					MeshFace *f = &Faces[i_tri];
					float dotSum = dotSum3(f->UnitNormal, normal);
					allSameDirection &= dotSum > 0.9f;
				}

				qti.IsPlanar = allSameDirection;
				if (allSameDirection)
				{
					float area = 0;
					if (xFlat) area = aabbf[1] * aabbf[2];
					else if (yFlat) area = aabbf[0] * aabbf[2];
					else  area = aabbf[0] * aabbf[1];
					float areaSum = 0;
					for (uint32_t i_tri = 0; i_tri < nTri; ++i_tri)
					{
						areaSum += Faces[i_tri].area;
					}
					areaSum *= 0.5f;
					float th = 0.01f;
					float areaThMin = area * (1 - th);
					float areaThMax = area * (1 + th);
					if (areaSum > areaThMin && areaSum < areaThMax)
					{
						points[0] = refMin;
						points[2] = refMax;
						__m128 mask;
						if (xFlat)
						{
							mask = _mm_castsi128_ps(_mm_setr_epi32(0, -1, 0, 0));
						}
						else {
							mask = _mm_castsi128_ps(_mm_setr_epi32(-1, 0, 0, 0));
						}
						points[1] = _mm_blendv_ps(refMin, refMax, mask);
						points[3] = _mm_blendv_ps(refMax, refMin, mask);

						__m128 e1 = _mm_sub_ps(points[1], points[0]);
						__m128 e3 = _mm_sub_ps(points[2], points[0]);

						__m128 e1e3Cross = cross(e1, e3);
						float dotSum = dotSum3(e1e3Cross, normal);
						if (dotSum < 0) {
							std::swap(points[1], points[3]);
						}
						qti.verts = points;

						qti.nQuads = 0;
						qti.nTriangles = 6;
						qti.indices = pBakeBuffer->GetIndicesBySize(6);
						qti.indices[0] = 0;
						qti.indices[1] = 1;
						qti.indices[2] = 2;
						qti.indices[3] = 0;
						qti.indices[4] = 2;
						qti.indices[5] = 3;
						qti.nActiveVerts = 4;

						if (SaveSimplifyModel) {
							CreateOutputDir();
							std::string file_path = outputSaveDirectory + "\\" + std::to_string(DebugOccluderIdx++) + "outQuad.off";

                            FILE *fileWriter = fopen(file_path.c_str(), "w");
                            if (fileWriter != nullptr)
							{
								fprintf(fileWriter, "OFF\n");
								fprintf(fileWriter, "%d %d 0\n", 4, 2);

								float * vertf = (float*)qti.verts;
								for (uint32_t i_vert = 0; i_vert < 4; ++i_vert)
								{
									fprintf(fileWriter, "%f %f %f\n",
										vertf[i_vert * 4],
										vertf[(i_vert * 4) + 1],
										vertf[(i_vert * 4) + 2]);
								}
								//save quads to two triangles
								fprintf(fileWriter, "3 0 1 2\n");
								fprintf(fileWriter, "3 0 2 3\n");
								fclose(fileWriter);
							}
						}

						return;
					}
				}
			}
		}

		int totalRequestQuads = mergedQuad;
		auto Quads = pBakeBuffer->GetQuadBySize(totalRequestQuads);
		memset(Quads, 0, totalRequestQuads * sizeof(MeshQuad)); //reset all to default 0 or null
		uint32_t quadIdx = 0;

		int totalRectangleQuad = 0;
		//assign all generate rect quads
		for (uint32_t i_tri = 0; i_tri < nTri; ++i_tri)
		{
			MeshFace *face1 = &Faces[i_tri];
			MeshFace *face2 = face1->PairFace;

			if (face2 != nullptr && i_tri < face2->FaceID)
			{
				//retrieve the quad info
				auto q = &Quads[quadIdx];
				q->qIdx = quadIdx;
				q->face1 = face1;
				q->face2 = face2;

				MeshEdgeFace &EdgeFace1 = face1->Edges[face1->MergedEdgeIdx];
				MeshEdgeFace &EdgeFace2 = face2->Edges[q->face2->MergedEdgeIdx];
				q->vIdx[0] = face1->nVertIdx[face1->MergedEdgeIdx];
				q->vIdx[1] = EdgeFace1.apexVert;
				q->vIdx[2] = EdgeFace1.endVert;
				q->vIdx[3] = EdgeFace2.apexVert;

				if (face1->IsPlanarQuad)
				{
					checkRectangleQuad(q, points);
					totalRectangleQuad += q->IsRectangle;
				}

				
				q->QuadKids = 1;

				quadIdx++;

			}
		}

		bool toSimplifyMesh = bSimplifyMesh;
		if (qti.IsPlanar == false) {
			toSimplifyMesh &= totalRectangleQuad > 1;
		}
		

		qti.nTriangles = 3 * (nTri - mergedQuad * 2);
		uint32_t quadNum = quadIdx;

		

		int quadFourMergeCount = 0;
		int quadTwoMergeCount = 0;
		if (toSimplifyMesh)
		{
			int neighborMergeCount = 0;
			bool quadMerge = 1;
			if (quadMerge) {
				int targetQuadIdx = 1;
				int mergeRound = QuadFourMergeRound + isTerrain;
				
				for (int round = 0; round < mergeRound; round++) {

					int terrainMergeIdx = 1 << round;
					int terrainCheckMask = terrainMergeIdx + terrainMergeIdx - 1;
					int lastQuadCount = quadFourMergeCount;

					int MaxORRectDegree = 0;
					SetRectangleDegreeToZero(Vertices, nVert);
					for (uint32_t i_quad = 0; i_quad < quadNum; ++i_quad)
					{
						//retrieve the quad info
						auto q = &Quads[i_quad];
						if (q->QuadKids == targetQuadIdx)
						{
							if (q->IsRectangle)
							{
								MaxORRectDegree |= Vertices[q->vIdx[0]].AddRectangle(i_quad);
								MaxORRectDegree |= Vertices[q->vIdx[1]].AddRectangle(i_quad);
								MaxORRectDegree |= Vertices[q->vIdx[2]].AddRectangle(i_quad);
								MaxORRectDegree |= Vertices[q->vIdx[3]].AddRectangle(i_quad);
							}
						}
					}
					if (MaxORRectDegree == 1) {
						toSimplifyMesh = false;
						break;
					}
					for (uint32_t idx = 0; idx < nVert; idx++)
					{
						MeshVertex* v = &Vertices[idx];
						if (isTerrain) 
						{
							bool okX = (terrain->gGridX[idx] & terrainCheckMask) == terrainMergeIdx;
							bool okY = (terrain->gGridY[idx] & terrainCheckMask) == terrainMergeIdx;
							if (okX == false || okY == false)
							{
								continue;
							}

							if (v->RectangleDegree == 2) {
								uint16_t p0 = v->quad[0];
								uint16_t p1 = v->quad[1];
								MeshQuad * q0 = &Quads[p0];
								MeshQuad * q1 = &Quads[p1];

								if (MergeQuad(q0, q1, points, Vertices, isTerrain, quadFourMergeCount)) continue;

							}
							else if (v->RectangleDegree == 3) {
								uint16_t p0 = v->quad[0];
								uint16_t p1 = v->quad[1];
								uint16_t p2 = v->quad[2];

								MeshQuad * q0 = &Quads[p0];
								MeshQuad * q1 = &Quads[p1];
								MeshQuad * q2 = &Quads[p2];


								if (MergeQuad(q0, q1, points, Vertices, isTerrain, quadFourMergeCount)) continue;
								if (MergeQuad(q0, q2, points, Vertices, isTerrain, quadFourMergeCount)) continue;
								if (MergeQuad(q1, q2, points, Vertices, isTerrain, quadFourMergeCount)) continue;

							}
						}
						if (v->RectangleDegree == 4) //pivot point
						{
							uint16_t p0 = v->quad[0];
							uint16_t p1 = v->quad[1];
							uint16_t p2 = v->quad[2];
							uint16_t p3 = v->quad[3];
							MeshQuad * q0 = &Quads[p0];
							MeshQuad * q1 = &Quads[p1];
							MeshQuad * q2 = &Quads[p2];
							MeshQuad * q3 = &Quads[p3];
							if (q0->QuadKids != targetQuadIdx || q1->QuadKids != targetQuadIdx || q2->QuadKids != targetQuadIdx || q3->QuadKids != targetQuadIdx)
							{
								continue;
							}

							float largestArea = std::max<float>(q0->face1->area,
								q1->face1->area);
							largestArea = std::max<float>(largestArea,
								q2->face1->area);
							largestArea = std::max<float>(largestArea,
								q3->face1->area);
							float areaTh = largestArea * QuadFourSameSquareAreaTh;
							if (q0->face1->area > areaTh &&
								q1->face1->area > areaTh &&
								q2->face1->area > areaTh &&
								q3->face1->area > areaTh
								) {
								float t1 = dotSum3(q0->face1->UnitNormal, q1->face1->UnitNormal);
								float t2 = dotSum3(q0->face1->UnitNormal, q2->face1->UnitNormal);
								float t3 = dotSum3(q0->face1->UnitNormal, q3->face1->UnitNormal);
								if (t1 >= QuadFourSameFaceTh && t2 >= QuadFourSameFaceTh && t3 >= QuadFourSameFaceTh) {
									if (q0->MergeWith(q1, points, Vertices, isTerrain, quadFourMergeCount))
									{
										if (q2->MergeWith(q3, points, Vertices, isTerrain, quadFourMergeCount))
											q0->MergeWith(q2, points, Vertices, isTerrain, quadFourMergeCount);
									}
									else if (q0->MergeWith(q2, points, Vertices, isTerrain, quadFourMergeCount))
									{
										if (q1->MergeWith(q3, points, Vertices, isTerrain, quadFourMergeCount))
											q0->MergeWith(q1, points, Vertices, isTerrain, quadFourMergeCount);
									}
									else if (q0->MergeWith(q3, points, Vertices, isTerrain, quadFourMergeCount))
									{
										if (q1->MergeWith(q2, points, Vertices, isTerrain, quadFourMergeCount))
											q0->MergeWith(q1, points, Vertices, isTerrain, quadFourMergeCount);
									}
									else if (q2->MergeWith(q3, points, Vertices, isTerrain, quadFourMergeCount)) {

									}
									else if (q2->MergeWith(q1, points, Vertices, isTerrain, quadFourMergeCount)) {

									}
								}
							}
						}
					}

					if (nIdx < 1000 || lastQuadCount == quadFourMergeCount)
					{
						break;
					}
					targetQuadIdx <<= 2; //* 4
					//during merging, some vertex still referring to dead quads
					//brute force update all
					SetRectangleDegreeToZero(Vertices, nVert);

					for (uint32_t i_quad = 0; i_quad < quadNum; ++i_quad)
					{
						//retrieve the quad info
						auto q = &Quads[i_quad];
						if (q->QuadKids == targetQuadIdx)
						{
							if (q->IsRectangle)
							{
								Vertices[q->vIdx[0]].AddRectangle(i_quad);
								Vertices[q->vIdx[1]].AddRectangle(i_quad);
								Vertices[q->vIdx[2]].AddRectangle(i_quad);
								Vertices[q->vIdx[3]].AddRectangle(i_quad);
							}
						}
					}
				}

				if (isTerrain && toSimplifyMesh)
				{
					SetRectangleDegreeToZero(Vertices, nVert);
					for (uint32_t i_quad = 0; i_quad < quadNum; ++i_quad)
					{
						//retrieve the quad info
						auto q = &Quads[i_quad];
						if (q->QuadKids > 0)
						{
							if (q->IsRectangle)
							{
								Vertices[q->vIdx[0]].AddRectangle(i_quad);
								Vertices[q->vIdx[1]].AddRectangle(i_quad);
								Vertices[q->vIdx[2]].AddRectangle(i_quad);
								Vertices[q->vIdx[3]].AddRectangle(i_quad);
							}
						}
					}
					//extra rounds of degree 2 merge.
					//only double expand to avoid extend T junction
					int degree2Merge = 1;
					while(degree2Merge > 0)
					{
						degree2Merge = 0;
						for (uint32_t idx = 0; idx < nVert; idx++)
						{
							MeshVertex* v = &Vertices[idx];
							if (v->RectangleDegree == 2)
							{
								uint16_t p0 = v->quad[0];
								uint16_t p1 = v->quad[1];
								MeshQuad * q0 = &Quads[p0];
								MeshQuad * q1 = &Quads[p1];
								if (q0->QuadKids == q1->QuadKids)//only allow double expand to avoid extend T
								{
									float t1 = dotSum3(q0->face1->UnitNormal, q1->face1->UnitNormal);
									if (t1 >= QuadTwoSameFaceTh)
									{
										//TerrainAggressiveMerging set to false by default
										//with aggressive on, the quad number would be reduced by another 10 % ~20 %
										//however, this would require extra handling during point correction
										//such as edge-joining to proper remove of T-junction
										if (gTerrainGridOptimization == TerrainOpMode::AggressiveMerging) 
										{											
											q0->MergeWith(q1, points, Vertices, isTerrain, degree2Merge);
										}
										else 
										{
											for (int i = 0; i < 4; i++)
											{
												for (int j = 0; j < 4; j++)
												{
													if (q0->vIdx[i] == q1->vIdx[j] && q0->vIdx[i] != idx)
													{
														uint16_t pair = q0->vIdx[i];
														MeshVertex* pairV = &Vertices[pair];
														if (pairV->RectangleDegree == 2)
														{
															q0->MergeWith(q1, points, Vertices, isTerrain, degree2Merge);
														}
														i = j = 4;
													}
												}
											}
										}
									}
								}

							}
						}
					}
				}
			}

			bool neighborMerge = 1;
			//terrain already handled in quad merge
			if (neighborMerge && isTerrain == false && toSimplifyMesh)
			{				

				int terminateTh = 5;
				bool planarFreeShapeMerge = false;
				do
				{
					neighborMergeCount = 0;
					SetRectangleDegreeToZero(Vertices, nVert);

					

					if (planarFreeShapeMerge == false) {
						for (uint32_t i_quad = 0; i_quad < quadNum; ++i_quad)
						{
							//retrieve the quad info
							auto q = &Quads[i_quad];
							if (q->QuadKids >= 1)
							{
								if (q->IsRectangle)
								{
									Vertices[q->vIdx[0]].AddRectangle(i_quad);
									Vertices[q->vIdx[1]].AddRectangle(i_quad);
									Vertices[q->vIdx[2]].AddRectangle(i_quad);
									Vertices[q->vIdx[3]].AddRectangle(i_quad);
								}
								else {
									//not allow to be pivot points
									Vertices[q->vIdx[0]].RectangleDegree |= 65536;
									Vertices[q->vIdx[1]].RectangleDegree |= 65536;
									Vertices[q->vIdx[2]].RectangleDegree |= 65536;
									Vertices[q->vIdx[3]].RectangleDegree |= 65536;
								}
							}
						}
					}
					else {
						for (uint32_t i_tri = 0; i_tri < quadNum; ++i_tri)
						{
							//retrieve the quad info
							auto q = &Quads[i_tri];
							if (q->QuadKids >= 1)
							{
								Vertices[q->vIdx[0]].AddRectangle(i_tri);
								Vertices[q->vIdx[1]].AddRectangle(i_tri);
								Vertices[q->vIdx[2]].AddRectangle(i_tri);
								Vertices[q->vIdx[3]].AddRectangle(i_tri);
							}
						}
					}
	
					if (planarFreeShapeMerge) {
						ProjectLineMidAngleTh = PlaneQuadDotThreshold; //apply strict constraint
						for (uint32_t idx = 0; idx < nVert; idx++)
						{
							MeshVertex* v = &Vertices[idx];
							if (v->RectangleDegree == 4)//pivot point
							{
								uint16_t p0 = v->quad[0];
								uint16_t p1 = v->quad[1];
								uint16_t p2 = v->quad[2];
								uint16_t p3 = v->quad[3];
								MeshQuad * q0 = &Quads[p0];
								MeshQuad * q1 = &Quads[p1];
								MeshQuad * q2 = &Quads[p2];
								MeshQuad * q3 = &Quads[p3];
								
								if (q0->MergeWith(q1, points, Vertices, false, neighborMergeCount)) 
								{
									q2->MergeWith(q3, points, Vertices, false, neighborMergeCount);
								}
								else if (q0->MergeWith(q2, points, Vertices, false, neighborMergeCount)) 
								{
									q1->MergeWith(q3, points, Vertices, false, neighborMergeCount);
								}
								else if (q0->MergeWith(q3, points, Vertices, false, neighborMergeCount)) 
								{
									q1->MergeWith(q3, points, Vertices, false, neighborMergeCount);
								}
							}
						}

						for (uint32_t idx = 0; idx < nVert; idx++)
						{
							MeshVertex* v = &Vertices[idx];
							if (v->RectangleDegree ==2)//pivot point
							{
								uint16_t p0 = v->quad[0];
								uint16_t p1 = v->quad[1];
							
								MeshQuad * q0 = &Quads[p0];
								MeshQuad * q1 = &Quads[p1];

								q0->MergeWith(q1, points, Vertices, false, neighborMergeCount);
							}
							else if (v->RectangleDegree == 3) {
								uint16_t p0 = v->quad[0];
								uint16_t p1 = v->quad[1];
								uint16_t p2 = v->quad[2];

								MeshQuad * q0 = &Quads[p0];
								MeshQuad * q1 = &Quads[p1];
								MeshQuad * q2 = &Quads[p2];

								if (q0->MergeWith(q1, points, Vertices, false, neighborMergeCount)) {

								}
								else if (q1->MergeWith(q2, points, Vertices, false, neighborMergeCount)) {

								}
								else if (q0->MergeWith(q2, points, Vertices, false, neighborMergeCount)) {

								}
							}
						}
						terminateTh = 2;
					//	neighborMergeCount = 0; //only one round 
					}
					else {
						for (uint32_t idx = 0; idx < nVert; idx++)
						{
							MeshVertex* v = &Vertices[idx];
							if (v->RectangleDegree == 2)//pivot point
							{
								uint16_t p0 = v->quad[0];
								uint16_t p1 = v->quad[1];
								MeshQuad * q0 = &Quads[p0];
								MeshQuad * q1 = &Quads[p1];

								if (q1->QuadKids == 0 || q0->QuadKids == 0)
								{
									continue;
								}
								if (qti.IsPlanar == false) {
									float largestArea = std::max<float>(q0->face1->area,
										q1->face1->area);
									float areaTh = largestArea * 0.7f;  //allow area ratio around  0.7

									if (((q1->QuadKids ==  q0->QuadKids  && q0->QuadKids > 0) )
										&&
										q0->face1->area > areaTh &&
										q1->face1->area > areaTh)
									{
										float t1 = dotSum3(q0->face1->UnitNormal, q1->face1->UnitNormal);
										float th = QuadTwoSameFaceTh;
										if (t1 >= th)
										{
											q0->MergeWith(q1, points, Vertices, isTerrain, neighborMergeCount);
										}
									}
								}
								else {
									q0->MergeWith(q1, points, Vertices, isTerrain, neighborMergeCount);
								}
							}
							else if (v->RectangleDegree == 4) {
								uint16_t p0 = v->quad[0];
								uint16_t p1 = v->quad[1];
								uint16_t p2 = v->quad[2];
								uint16_t p3 = v->quad[3];
								MeshQuad * q0 = &Quads[p0];
								MeshQuad * q1 = &Quads[p1];
								MeshQuad * q2 = &Quads[p2];
								MeshQuad * q3 = &Quads[p3];


								float t01 = dotSum3(q0->face1->UnitNormal, q1->face1->UnitNormal);

								if (t01 > QuadTwoSameFaceTh && q0->QuadKids == q1->QuadKids && q0->MergeWith(q1, points, Vertices, false, neighborMergeCount))
								{
									float t23 = dotSum3(q2->face1->UnitNormal, q3->face1->UnitNormal);
									if (t23 > QuadTwoSameFaceTh && q2->QuadKids == q3->QuadKids) {
										q2->MergeWith(q3, points, Vertices, false, neighborMergeCount);
									}
								}
								else {
									float t02 = dotSum3(q0->face1->UnitNormal, q2->face1->UnitNormal);
									if (t02 > QuadTwoSameFaceTh  && q0->QuadKids == q2->QuadKids && q0->MergeWith(q2, points, Vertices, false, neighborMergeCount))
									{
										float t13 = dotSum3(q1->face1->UnitNormal, q3->face1->UnitNormal);
										if (t13 > QuadTwoSameFaceTh && q1->QuadKids == q3->QuadKids) {
											q1->MergeWith(q3, points, Vertices, false, neighborMergeCount);
										}
									}
									else {

										float t03 = dotSum3(q0->face1->UnitNormal, q3->face1->UnitNormal);
										if (t03 > QuadTwoSameFaceTh  && q0->QuadKids == q3->QuadKids && q0->MergeWith(q3, points, Vertices, false, neighborMergeCount))
										{
											float t12 = dotSum3(q1->face1->UnitNormal, q2->face1->UnitNormal);
											if (t12 > QuadTwoSameFaceTh && q1->QuadKids == q2->QuadKids) {
												q1->MergeWith(q2, points, Vertices, false, neighborMergeCount);
											}
										}
									}
								}
							}
						}
					}

					quadTwoMergeCount += neighborMergeCount;
					//for planar quad, allow free shape merge
					if (neighborMergeCount < terminateTh && planarFreeShapeMerge == false && qti.IsPlanar)
					{
						neighborMergeCount += terminateTh+1;
						planarFreeShapeMerge = true;
					}
				} while (neighborMergeCount > terminateTh);
			}
			int validQuad = 0;
			for (uint32_t i_quad = 0; i_quad < quadNum; ++i_quad)
			{
				//retrieve the quad info
				MeshQuad* q = &Quads[i_quad];
				validQuad += q->QuadKids > 0;
			}
			qti.nQuads = validQuad << 2;

			if (isTerrain)
			{
				int mergedQuadNum = 0;
				for (uint32_t i_quad = 0; i_quad < quadNum; ++i_quad)
				{
					//retrieve the quad info
					MeshQuad* q = &Quads[i_quad];
					mergedQuadNum += q->QuadKids > 1;
				}

				if (mergedQuadNum > 0) 
				{
					std::vector<RectEdge*> mergedEdges; //sort edge
					mergedEdges.reserve(mergedQuadNum * 4);

					int totalLargeQuad = 0;
					for (uint32_t i_quad = 0; i_quad < quadNum; ++i_quad)
					{
						//retrieve the quad info
						MeshQuad* q = &Quads[i_quad];
						totalLargeQuad += q->QuadKids >= 2;
					}

					RectEdge * EdgePool = new RectEdge[totalLargeQuad * 4];
					int EdgePoolIdx = 0;

					bool validTerrain = true;
					for (uint32_t i_quad = 0; i_quad < quadNum && validTerrain; ++i_quad)
					{
						//retrieve the quad info
						MeshQuad* q = &Quads[i_quad];
						if (q->QuadKids >= 2)
						{
							for (int idx = 0; idx < 4 && validTerrain; idx++)
							{
								uint32_t startVert = q->vIdx[idx];
								uint32_t endVert = q->vIdx[(idx + 1) & 3];

								int startGridX = terrain->gGridX[startVert];
								int startGridY = terrain->gGridY[startVert];
								int endGridX = terrain->gGridX[endVert];
								int endGridY = terrain->gGridY[endVert];

								validTerrain &= ((startGridX == endGridX) || (startGridY == endGridY));
								if (validTerrain) 
								{
									int length = (endGridX - startGridX) + (endGridY - startGridY);

									if (length < 0)
									{
										length = -length;
										std::swap(startVert, endVert);
										std::swap(startGridX, endGridX);
										std::swap(startGridY, endGridY);
									}
									if (length > 1)
									{
										RectEdge *re = &EdgePool[EdgePoolIdx++];
										re->endGridX = endGridX;
										re->endGridY = endGridY;
										re->startGridX = startGridX;
										re->startGridY = startGridY;

										int StartGrid = re->startGridX + re->startGridY * terrain->terrainAxisSize;
										re->Key = StartGrid;
										re->Key <<= 16;
										re->Key |= 65535 - length;
										if (startGridX == endGridX)
										{
											re->Key = (((uint64_t)startGridX << 16) + re->startGridY) << 32;
											re->Key += 65535 - length;
										}
										mergedEdges.push_back(re);
									}
								}								
							}
						}
					}
					//validTerrain = false;
					if (validTerrain) {
						//time to seal holes. With square terrain assumption, we can derive all points on edge
						//sort in the order of edge-length/quadkids/smallGridIdx/endGridIdx
						//Warning: might not be able to converge if aggressiveness on
						std::sort(mergedEdges.begin(), mergedEdges.end(), RectEdge::AscendingSort);
						RectEdge* refEdge = mergedEdges[0];

						int validCount = 1;
						for (int idx = 1; idx < mergedEdges.size(); idx++)
						{
							RectEdge* edge = mergedEdges[idx];

							//std::cout << "Edge " << edge->startGridX << " " << edge->endGridX << " " << edge->startGridY << " " << edge->endGridY << std::endl;
							if (edge->MergeInto(refEdge))
							{
								edge->Key = 0;
							}
							else {
								refEdge = edge;
								validCount++;
							}
						}

						for (auto re : mergedEdges)
						{
							if (re->Key == 0) continue;

							uint32_t startGridX = re->startGridX;
							uint32_t startGridY = re->startGridY;
							uint32_t endGridX = re->endGridX;
							uint32_t endGridY = re->endGridY;
							int StartGrid = re->startGridX + re->startGridY * terrain->terrainAxisSize;
							int endGrid = re->endGridX + re->endGridY * terrain->terrainAxisSize;
							int length = (endGridX - startGridX) + (endGridY - startGridY);

							//length first, and then key
							uint64_t key = length << 16;
							key += (re->Key >> 32) & 65535;
							key <<= 32;
							key |= (StartGrid << 16) | endGrid;
							re->Key = key;
						}

						std::sort(mergedEdges.begin(), mergedEdges.end(), RectEdge::DescendingSort);
						//Two round to eliminate T junctions
						int round = 2;
						round += gTerrainGridOptimization == TerrainOpMode::AggressiveMerging; //one more round for aggressive mode
						for (int updateIdx = 0; updateIdx < round; updateIdx++)
						{
							for (int idx = 0; idx < validCount; idx++)
							{
								RectEdge * re = mergedEdges[idx];

								uint32_t startGridX = re->startGridX;
								uint32_t startGridY = re->startGridY;
								uint32_t endGridX = re->endGridX;
								uint32_t endGridY = re->endGridY;


								uint32_t startVert = terrain->gGridToVertID[startGridX + startGridY * terrain->terrainAxisSize];
								uint32_t endVert = terrain->gGridToVertID[endGridX + endGridY * terrain->terrainAxisSize];
								//assert(startGridY < endGridY || startGridX < endGridX);
								__m128 a = points[startVert];
								__m128 b = points[endVert];
								__m128 unitAB = normalize(_mm_sub_ps(a, b));

								int terrainAxisSize = terrain->terrainAxisSize;
								if (startGridX == endGridX)
								{
									//update Y
									for (uint32_t y = startGridY + 1; y < endGridY; y++) {
										int targetGrid = y * terrainAxisSize + startGridX;
										int targetVertx = terrain->gGridToVertID[targetGrid];
										ProjectToNormalLine(unitAB, b, points[targetVertx]);
									}
								}
								else// if (startGridY == endGridY) 
								{
									//update x
									for (uint32_t x = startGridX + 1; x < endGridX; x++)
									{
										int targetGrid = startGridY * terrainAxisSize + x;
										int targetVertx = terrain->gGridToVertID[targetGrid];
										ProjectToNormalLine(unitAB, b, points[targetVertx]);
									}
								}
							}
						}
					}

					mergedEdges.clear();
					delete[]EdgePool;
					if (validTerrain == false)
					{
						decomposeToQuad(pBakeBuffer, indices, nIdx, inputVertices, vertNum, qti, quadAngle, nullptr, checkedTerrain, -1);
					}
				}
			}
		}
		else
		{
			qti.nQuads = 4 * quadNum;
		}






		qti.indices = pBakeBuffer->GetIndicesBySize(qti.nQuads + qti.nTriangles);
		uint16_t *pIndices = qti.indices;

		//int totalFace = (qti.nQuads >> 2) + qti.nTriangles / 3;
		for (int i_tri = 0; i_tri < totalRequestQuads; ++i_tri)
		{
			MeshQuad *mq = &Quads[i_tri];
			if (mq->QuadKids > 0)
			{
				memcpy(pIndices, mq->vIdx, 4 * sizeof(uint16_t));
				pIndices += 4;
			}
		}

		////assign all isolated triangle
		for (uint32_t i_tri = 0; i_tri < nTri; ++i_tri)
		{
			MeshFace *face1 = &Faces[i_tri];

			// No quad found
			if (face1->PairFace == nullptr)
			{
				memcpy(pIndices, indices + (3 * i_tri), 3 * sizeof(uint16_t));
				pIndices += 3;// mq->PolygonPoint;
			}
		}


		uint32_t totalUseIdx = qti.nQuads + qti.nTriangles;
		//RectangleDegree used for merged vertex id
		for (uint32_t i = 0; i < nVert; i++) {
			Vertices[i].RectangleDegree = -1;
		}
		//Remove the not used vertices for strip triangle off file
		for (uint32_t i = 0; i < totalUseIdx; i++)
		{
			Vertices[qti.indices[i]].RectangleDegree = 0;
		}

		uint32_t nextVertID = 0;
		for (uint32_t i = 0; i < nVert; i++) {
			MeshVertex* v = &Vertices[i];
			if (v->RectangleDegree == 0)
			{
				v->RectangleDegree = nextVertID;
				qti.verts[nextVertID] = qti.verts[i];
				nextVertID++;
			}
		}
		for (uint32_t i = 0; i < totalUseIdx; i++)
		{
			qti.indices[i] = Vertices[qti.indices[i]].RectangleDegree;
		}
		qti.nActiveVerts = nextVertID;



		
		{
			if(SaveSimplifyModel)
			{
				inputVertices = (float*)qti.verts; //apply water tight fix to avoid any small holes
				
				std::string file_path;
				if (qti.nQuads > 0) {
					CreateOutputDir();
					if (isTerrain) {
						file_path = outputSaveDirectory + "\\" + std::to_string(DebugOccluderIdx++) + "TerrainQuad.off";
					}
					else {
						file_path = outputSaveDirectory + "\\" + std::to_string(DebugOccluderIdx++) + "QuadOut.off";
					}

					FILE *fileWriter = fopen(file_path.c_str(), "w");
					if (fileWriter != nullptr)
					{
						uint32_t nFace = (qti.nQuads >> 2) + qti.nTriangles / 3;


						fprintf(fileWriter, "OFF\n");
						fprintf(fileWriter, "%d %d 0\n", qti.nActiveVerts, nFace);

						for (uint32_t i_vert = 0; i_vert < qti.nActiveVerts; ++i_vert)
						{
							fprintf(fileWriter, "%f %f %f\n",
								inputVertices[i_vert * 4],
								inputVertices[(i_vert * 4) + 1],
								inputVertices[(i_vert * 4) + 2]);
						}
						for (uint32_t i = 0; i < qti.nQuads; i += 4)
						{
							fprintf(fileWriter, "4 %d %d %d %d\n",
								qti.indices[i],
								qti.indices[i + 1],
								qti.indices[i + 2],
								qti.indices[i + 3]);
						}
						for (uint32_t i = qti.nQuads; i < totalUseIdx; i += 3)
						{
							fprintf(fileWriter, "3 %d %d %d\n",
								qti.indices[i],
								qti.indices[i + 1],
								qti.indices[i + 2]);
						}

						fclose(fileWriter);
					}

				}
				
				

				//according to the triangle off file to store the simplified mesh if wish to 
				//push triangle mesh to SOC
				//save as triangle off file
				CreateOutputDir();
				file_path = outputSaveDirectory + "\\" + std::to_string(DebugOccluderIdx++) + "out.off";
				FILE * fileWriter = fopen(file_path.c_str(), "w");
                if (fileWriter != nullptr)
				{
					uint32_t nFace = (qti.nQuads>>1) + qti.nTriangles/3;

					int latestVertNum = nextVertID;
					fprintf(fileWriter, "OFF\n");
					fprintf(fileWriter, "%d %d 0\n", latestVertNum, nFace);

					for (uint32_t i_vert = 0; i_vert < qti.nActiveVerts; ++i_vert)
					{
						fprintf(fileWriter, "%f %f %f\n",
							inputVertices[i_vert * 4],
							inputVertices[(i_vert * 4) + 1],
							inputVertices[(i_vert * 4) + 2]);
					}
					for (uint32_t i = 0; i < qti.nQuads; i += 4)
					{
						//save quads to two triangles
						fprintf(fileWriter, "3 %d %d %d\n",
							qti.indices[i],
							qti.indices[i + 2],
							qti.indices[i + 1]);


						fprintf(fileWriter, "3 %d %d %d\n",
							qti.indices[i],
							qti.indices[i + 3],
							qti.indices[i + 2]);
					}
					for (uint32_t i = qti.nQuads; i < totalUseIdx; i += 3)
					{
						fprintf(fileWriter, "3 %d %d %d\n",
							qti.indices[i],
							qti.indices[i + 1],
							qti.indices[i + 2]);
					}

					fclose(fileWriter);
				}
			}



		}

	}


	bool OccluderQuad::MergeQuad(MeshQuad * q0, MeshQuad * q1, __m128* points, MeshVertex * Vertices, bool isTerrain, int &totalMerged)
	{
		if (q1->QuadKids == 0 || q0->QuadKids == 0)
		{
			return false;
		}

		float largestArea = std::max<float>(q0->face1->area,
			q1->face1->area);
		float areaTh = largestArea * 0.7f;  //allow area ratio around 0.7
		if (q1->QuadKids == q0->QuadKids 
			&& ((q0->IsRectangle && q1->IsRectangle)) &&
			q0->face1->area > areaTh &&
			q1->face1->area > areaTh)
		{
			float t1 = dotSum3(q0->face1->UnitNormal, q1->face1->UnitNormal);
			float th = QuadTwoSameFaceTh; //around 1 degree
			if (t1 >= th)
			{
				if (q0->MergeWith(q1, points, Vertices, isTerrain, totalMerged))
				{
					return true;
				}
			}
		}
		return false;
	}



	static bool ProjectToLine(__m128 a, __m128 b, __m128 mid, __m128 &projMid)
	{
		__m128 unitAB = normalize(_mm_sub_ps(a, b));
		__m128 midB = _mm_sub_ps(mid, b);
		__m128 unitMB = normalize(midB);
		float dotSum = dotSum3(unitAB, unitMB);
		if (dotSum > ProjectLineMidAngleTh) 
		{
			float projLen = dotSum3(midB, unitAB);

			projMid = _mm_add_ps(b, _mm_mul_ps(unitAB, _mm_set1_ps(projLen)));
			return true;
		}
		return false;
	}
	static bool MatchToLine(__m128 a, __m128 b, __m128 mid)
	{
		__m128 unitAB = normalize(_mm_sub_ps(a, b));
		__m128 midB = _mm_sub_ps(mid, b);
		__m128 unitMB = normalize(midB);
		float dotSum = dotSum3(unitAB, unitMB);
		if (dotSum > ProjectLineMidAngleTh)
		{
			return true;
		}
		return false;
	}
	bool OccluderQuad::MeshQuad::MergeWith(MeshQuad * q, __m128 * vertices, MeshVertex * MeshVertices, bool isTerrain, int &TotalMerged)
	{
		if (q->QuadKids == 0 || this->QuadKids == 0) {
			return false;
		}
		int SameVertex[8];
		int find = 0;
		uint16_t * target = q->vIdx;
		for (int i = 0; i < 4; i++)
		{
			for (int j = 0; j < 4; j++)
			{
				if (vIdx[i] == target[j])
				{
					SameVertex[find] = i;
					SameVertex[find + 1] = j;
					find += 2;
					break;
				}
			}
		}
		if (find == 4)
		{

			if (SameVertex[1] == 0 && SameVertex[3] == 3)
			{
				SameVertex[1] = 4;
			}
			else if ( SameVertex[1] == 3 && SameVertex[3] == 0)
			{
				SameVertex[3] = 4;
			}

			int Joint[2]; //for the point connect to the melting points
			Joint[0] = (SameVertex[0] * 2 - SameVertex[2] + 4) & 3;
			Joint[1] = (SameVertex[2] * 2 - SameVertex[0] + 4) & 3;
			//int v0First = 0;
			//assert(SameVertex[0] < SameVertex[2]);
			//{
			//	if (SameVertex[0] == 0 && SameVertex[2] == 3)
			//	{

			//	}
			//	else {
			//		v0First = 2;
			//	}
			//}
			int v0First = (SameVertex[2] - SameVertex[0]) ^ 3;
			//assert(v0First == v0First2);


			int v20First = 0;
			if (SameVertex[1] < SameVertex[3])
			{				
				v20First = 2;
			}
			//to enforce the two merge points are in reverse order of melting
			if (v0First != v20First) {

				uint16_t targetEx[8];
				memcpy(targetEx, target, 4 * sizeof(uint16_t));
				memcpy(targetEx + 4, target, 4 * sizeof(uint16_t));

				uint16_t idx0 = this->vIdx[SameVertex[0]];
				uint16_t idx0A = this->vIdx[Joint[0]];
				uint16_t idx0B = targetEx[SameVertex[1] + 3 - v0First];

				uint16_t idx2 = this->vIdx[SameVertex[2]];
				uint16_t idx2A = this->vIdx[Joint[1]];
				uint16_t idx2B = targetEx[SameVertex[1] + 2];

				if (isTerrain == false)
				{
					__m128 updatedPoints[2];
					if (ProjectToLine(vertices[idx0A], vertices[idx0B], vertices[idx0], updatedPoints[0]) &&
						ProjectToLine(vertices[idx2A], vertices[idx2B], vertices[idx2], updatedPoints[1]))
					{
						//take the projected new points
						vertices[idx0] = updatedPoints[0];
						vertices[idx2] = updatedPoints[1];

						this->vIdx[SameVertex[0]] = idx0B;
						this->vIdx[SameVertex[2]] = idx2B;

						MeshVertices[idx0].RectangleDegree -= 2;
						MeshVertices[idx2].RectangleDegree -= 2;

						this->QuadKids += q->QuadKids;
						q->QuadKids = 0;
						TotalMerged++;
						return true;
					}
				}
				else {
					if (MatchToLine(vertices[idx0A], vertices[idx0B], vertices[idx0]) &&
						MatchToLine(vertices[idx2A], vertices[idx2B], vertices[idx2]))
					{
						this->vIdx[SameVertex[0]] = idx0B;
						this->vIdx[SameVertex[2]] = idx2B;

						MeshVertices[idx0].RemoveQuad(this->qIdx, q->qIdx);
						MeshVertices[idx2].RemoveQuad(this->qIdx, q->qIdx);


						MeshVertices[idx0B].ReplaceQuad(q->qIdx, this->qIdx);
						MeshVertices[idx2B].ReplaceQuad(q->qIdx, this->qIdx);


						this->QuadKids += q->QuadKids;
						q->QuadKids = 0;
						TotalMerged++;
						return true;
					}
				}

				
			}
		}
		return false;
	}

	bool OccluderQuad::RectEdge::MergeInto(RectEdge* refEdge)
	{
		if (this->startGridX == this->endGridX) 
		{
			if (refEdge->startGridX == refEdge->endGridX) 
			{
				if (this->startGridX == refEdge->startGridX) 
				{
					if (refEdge->endGridY >= this->endGridY && refEdge->startGridY <= this->startGridY)
					{
						return true; //do nothing, refEdge contain current Edge
					}
					else if (refEdge->endGridY > this->startGridY) {
						if (refEdge->endGridY < this->endGridY) {
							//std::cout << "endY from " << refEdge->endGridY << " to " << this->endGridY << std::endl;
							refEdge->endGridY = this->endGridY;
							return true;
						}
					}
				}
			}
		}
		else if (this->startGridY == this->endGridY) 
		{
			if (refEdge->startGridY == refEdge->endGridY)
			{
				if (this->startGridY == refEdge->startGridY)
				{
					if (refEdge->endGridX >= this->endGridX && refEdge->startGridX <= this->startGridX) 
					{
						return true; //do nothing, refEdge contain current Edge
					}
					else if (refEdge->endGridX > this->startGridX) 
					{
						if (refEdge->endGridX < this->endGridX) {
							//std::cout << "endX from " << refEdge->endGridX << " to " << this->endGridX << std::endl;
							refEdge->endGridX = this->endGridX;
							return true;
						}
					}
				}
			}
		}
		return false;
	}

	//SDOC_SetTerrainRectangleAngle  [80, 89]
	void OccluderQuad::SetTerrainRectangleAngle(unsigned int configValue)
	{
		TerrainRectangleAngle = (float)configValue;
	}
	//SDOC_SetTerrainMergeAngle  [1, 10]
	void OccluderQuad::SetTerrainRectangleMergeAngle(unsigned int configValue)
	{
		TerrainRectangleMergeAngle = (float)configValue;
	}

	void OccluderQuad::SetSaveModel(bool v)
	{
		SaveSimplifyModel = v;
	}

	//SDOC_SimplifyMeshDuringBake
	void OccluderQuad::AllowPlanarQuadMerge(bool allow)
	{
		bSimplifyMesh = allow;
	}

	//SDOC_TerrainGridControlMergeDuringBake
	void OccluderQuad::SetTerrainGridOptimization(int value)
	{
		gTerrainGridOptimization = value;
	}

	void OccluderQuad::SetRectangleDegreeToZero(MeshVertex * Vertices, uint32_t nVert)
	{
		uint32_t nVert4 = (nVert >> 2) << 2;
		//group by 4 to reduce branch check
		for (uint32_t vidx = 0; vidx < nVert4; vidx+=4)
		{
			Vertices[vidx].RectangleDegree = 0;
			Vertices[vidx+1].RectangleDegree = 0;
			Vertices[vidx+2].RectangleDegree = 0;
			Vertices[vidx+3].RectangleDegree = 0;
		}
		for (uint32_t vidx = nVert4; vidx < nVert; vidx ++)
		{
			Vertices[vidx].RectangleDegree = 0;
		}
	}

#ifdef SUPPORT_ALL_FEATURE
	static bool testTerrain = false;
#endif
	void OccluderQuad::TestModel()
	{
#ifdef SUPPORT_ALL_FEATURE
		if (testTerrain == false) {
			testTerrain = true;
			//LoadObj("D:\\Landscape.obj", quadAngle, 63);
			LoadOff("D:\\40input.off", 15, 0);


			return;
		}
#endif
	}

	bool isInputModelValid(const unsigned short* indices, unsigned int nVert, unsigned int nIdx) {
		if (nVert <= 0 || nIdx < 3 || nIdx % 3 != 0)
		{
			std::cout << "Invalid Input model" << std::endl;
			return nullptr;//invalid input;
		}
		//use sse to verify whether the model indices are all smaller than vert number
		if (nVert < 65536)
		{
			__m128i nVert128 = _mm_set1_epi16(nVert);
			bool all_index_valid = true;
			int nIdx8 = (nIdx >> 3);
			__m128i* pIndices = (__m128i*) indices;
			__m128i true128 = _mm_set1_epi32(-1);
			for (uint32_t idx = 0; idx < nIdx8; idx++)
			{
				__m128i all_smaller = _mm_cmplt_epu16_soc(pIndices[idx], nVert128);
				true128 = _mm_and_si128(true128, all_smaller);
			}
			uint64_t* pTrue64 = (uint64_t*)&true128;
			uint64_t state = pTrue64[0] & pTrue64[1];
			bool smaller = (state == -1);
			for (uint32_t idx = nIdx8 << 3; idx < nIdx; idx++)
			{
				smaller &= indices[idx] < nVert;
			}
			if (smaller == false)
			{
				std::cout << "Invalid Input model: find invalid indices" << std::endl;
				return false; //this is invalid model input
			}
		}
		return true;
	}


	//due to usage of singleton and global shared result buffer, this function is not thread-safe
	//developer should add mutex lock at high level to make sdocMeshBake and result retrieving exclusive
	unsigned short* OccluderQuad::sdocMeshBake(int* outputCompressSize, const float *vertices, const unsigned short *indices, unsigned int nVert, unsigned int nIdx, float quadAngle, bool enableBackfaceCull, bool counterClockWise, int TerrainGridAxisPoint)
	{
		if (isInputModelValid(indices, nVert, nIdx) == false) {
			return nullptr;
		}

		if (gBakeBuffer == nullptr) {
			gBakeBuffer = new util::OccluderBakeBuffer();
		}
		//************************************************************************
		//decompose to quad
		//************************************************************************
		util::QuadTriIndex quadData;
		util::OccluderQuad::decomposeToQuad(gBakeBuffer, indices, nIdx, vertices, nVert, quadData, quadAngle, nullptr, false, TerrainGridAxisPoint);
		vertices = (float*)quadData.verts;



		int triangleFaceNum = quadData.nTriangles / 3;

		int triangleBatchNum = (triangleFaceNum + 3) >> 2;
		int quadBatchNum = (quadData.nQuads + 15) >> 4;


		if (quadBatchNum > 65535 || triangleBatchNum > 65535) {
			*outputCompressSize = 0;
			return nullptr;
		}

		////int batchNum = (triangleFaceNum + 3) >> 2;
		int tillQuadFinish = quadBatchNum * 6 + 2;
		int compressSize = tillQuadFinish + (triangleBatchNum * 5) - (triangleBatchNum >> 1);
		if (common::bEnableCompressMode) {
			compressSize = quadBatchNum * 32 + triangleBatchNum * 24 + quadData.nActiveVerts * 3 * 2 + 16 * 2;// +1000;
		}

		util::OccluderBakeBuffer* pBakeBuffer = gBakeBuffer;
		int faceBufferSize = (int)pBakeBuffer->mFaces.size() * sizeof(util::OccluderQuad::MeshFace);
		if (faceBufferSize < compressSize * 4) 
		{
			int faceNum = compressSize * 4 / sizeof(util::OccluderQuad::MeshFace) + 1;
			gBakeBuffer->mFaces.resize(faceNum);
		}
		int *outputBuffer = (int*)(&pBakeBuffer->mFaces[0]);

		if (common::bEnableCompressMode == false) {
			memset(outputBuffer, 0, tillQuadFinish * sizeof(__m128)); //reset all to zero,  Quad request input to be set to 0, triangle no need
		}
		//std::cout << " tillQuadFinish " << tillQuadFinish << " compressSize " << compressSize << std::endl;


		__m128 scalingXYZW = _mm_setr_ps(65535.0f, 65535.0f, 65535.0f, 0);

		__m128 extents = _mm_sub_ps(quadData.refMax, quadData.refMin);


		__m128 invExtents = _mm_div_ps(scalingXYZW, extents);
		//eliminate zero case
		__m128 positive = _mm_cmpgt_ps(extents, _mm_setzero_ps());
		invExtents = _mm_and_ps(invExtents, positive);

		__m128 minusZero = _mm_set1_ps(-0.0f);
		__m128 minusRefMinInvExtents = _mm_fmadd_ps(_mm_xor_ps(minusZero, invExtents), quadData.refMin, _mm_set1_ps(0.5f));

		__m128i * compressData = (__m128i *) outputBuffer;


		float *f = (float*)outputBuffer;
		f += 2; //first 64 bit used to store model description
		float *refMinf = (float*)& quadData.refMin;
		f[0] = refMinf[0];
		f[1] = refMinf[1];
		f[2] = refMinf[2];
		float *extentsf = (float*)& extents;
		f[3] = extentsf[0];
		f[4] = extentsf[1];
		f[5] = extentsf[2];
		uint16_t* pVint = (uint16_t*)compressData;
		//pVint += 3;
		pVint[0] = (int)enableBackfaceCull +( ((int)quadData.IsTerrain) << 5);
		pVint[1] = 0;// quadData.IsPlanar;
		pVint[2] = quadBatchNum;
		pVint[3] = triangleBatchNum;


		__m128i *pVertices = compressData + 2; //already store bbox quad patch and tri patch & backface cull

		//if (compressSize * 16 > nVert * 12 + nIdx * 2 && nVert <= 256) 
		if(common::bEnableCompressMode && quadData.nActiveVerts <= 65535 /3)
		{
			bool idxError = false;
			pVint[1] = quadData.nActiveVerts;
			const uint16_t* pIndexCurrent = quadData.indices;
			if (pIndexCurrent == nullptr) pIndexCurrent = indices;
			if (quadData.nActiveVerts <= common::SuperCompressVertNum)
			{
				uint8_t * pi8 = (uint8_t*)pVertices;

				int piIdx = 0;
				for (uint32_t qidx = 0; qidx < quadData.nQuads; qidx++)
				{
					pi8[piIdx++] = pIndexCurrent[qidx] * 3;
					idxError |= pIndexCurrent[qidx] >= quadData.nActiveVerts;
				}
				if ((quadData.nQuads & 15) > 0) {
					int quadFill = 16 - (quadData.nQuads & 15);
					for (int qidx = 0; qidx < quadFill; qidx++)
					{
						pi8[piIdx++] = 0;
					}
				}

				pIndexCurrent += quadData.nQuads;
				for (uint32_t tidx = 0; tidx < quadData.nTriangles; tidx++)
				{
					pi8[piIdx++] = pIndexCurrent[tidx] * 3;
					idxError |= pIndexCurrent[tidx] >= quadData.nActiveVerts;
				}

				if (idxError)
				{
					*outputCompressSize = 0;
					return nullptr;
				}


				if ((quadData.nTriangles % 12) > 0) {
					int triFill = 12 - (quadData.nTriangles % 12);
					for (int tidx = 0; tidx < triFill; tidx++)
					{
						pi8[piIdx++] = 0;
					}
				}

				uint16_t * pV16 = (uint16_t*)pVertices;
				pV16 += piIdx>>1;

				int pointIdx = 0;
				for (uint32_t idx = 0; idx < quadData.nActiveVerts; idx++)
				{
					__m128 v0 = _mm_fmadd_ps(quadData.verts[idx], invExtents, minusRefMinInvExtents);
					__m128i XYZ = _mm_cvttps_epi32(v0);
					__m128i max = _mm_set1_epi32(65535);
					XYZ = _mm_min_epi32(XYZ, max);
					__m128i min = _mm_set1_epi32(0);
					XYZ = _mm_max_epi32(XYZ, min);
					uint32_t * iXYZ = (uint32_t *)&XYZ;
					pV16[0] = iXYZ[0];
					pV16[1] = iXYZ[1];
					pV16[2] = iXYZ[2];
					pV16 += 3;
				}

				*outputCompressSize = (piIdx >> 1) +  quadData.nActiveVerts * 3 + 16;
			//	std::cout << "Byte8 Super Compressize Size " << *outputCompressSize * 2 << " Pidx " << piIdx << " Origin " << nVert * 12 + nIdx * 2 << " Now Point " << quadData.nActiveVerts << " OriginPoint " << nVert << std::endl;
				return (unsigned short*)outputBuffer;  //this is a temp buffer, the data inside should be retrieved immediately.
			}
			else {
				uint16_t * pi16 = (uint16_t*)pVertices;

				int piIdx = 0;
				for (uint32_t qidx = 0; qidx < quadData.nQuads; qidx++)
				{
					pi16[piIdx++] = pIndexCurrent[qidx] * 3;
					idxError |= pIndexCurrent[qidx] >= quadData.nActiveVerts;
				}
				if ((quadData.nQuads & 15) > 0) {
					int quadFill = 16 - (quadData.nQuads & 15);
					for (int qidx = 0; qidx < quadFill; qidx++)
					{
						pi16[piIdx++] = 0;
					}
				}

				pIndexCurrent += quadData.nQuads;
				for (uint32_t tidx = 0; tidx < quadData.nTriangles; tidx++)
				{
					pi16[piIdx++] = pIndexCurrent[tidx] * 3;
					idxError |= pIndexCurrent[tidx] >= quadData.nActiveVerts;
				}

				if (idxError)
				{
					*outputCompressSize = 0;
					return nullptr;
				}

				if ((quadData.nTriangles % 12) > 0) {
					int triFill = 12 - (quadData.nTriangles % 12);
					for (int tidx = 0; tidx < triFill; tidx++)
					{
						pi16[piIdx++] = 0;
					}
				}

				uint16_t * pV16 = (uint16_t*)pVertices  + piIdx;

				int pointIdx = 0;
				for (uint32_t idx = 0; idx < quadData.nActiveVerts; idx++)
				{
					__m128 v0 = _mm_fmadd_ps(quadData.verts[idx], invExtents, minusRefMinInvExtents);
					__m128i XYZ = _mm_cvttps_epi32(v0);
					__m128i max = _mm_set1_epi32(65535);
					XYZ = _mm_min_epi32(XYZ, max);
					__m128i min = _mm_set1_epi32(0);
					XYZ = _mm_max_epi32(XYZ, min);
					uint32_t * iXYZ = (uint32_t *)&XYZ;
					pV16[0] = iXYZ[0];
					pV16[1] = iXYZ[1];
					pV16[2] = iXYZ[2];
					pV16 += 3;
				}

				*outputCompressSize = piIdx + quadData.nActiveVerts * 3 + 16;
			//	std::cout << "16 Compressize Size " << *outputCompressSize * 2 << " Pidx " << piIdx << " Origin " << nVert * 12 + nIdx * 2 << " Now Point " << quadData.nActiveVerts << " OriginPoint " << nVert << std::endl;
				return (unsigned short*)outputBuffer;  //this is a temp buffer, the data inside should be retrieved immediately.
			}
			
		}
		if (common::bEnableCompressMode)
		{
			*outputCompressSize = 0;
			return nullptr;
		}

		

		const uint16_t* pIndexCurrent = quadData.indices;
		if (pIndexCurrent == nullptr) pIndexCurrent = indices;
		if (quadBatchNum > 0) {
			__m128 tempV[16];
			for (int batchIdx = 0; batchIdx < quadBatchNum; batchIdx++, pIndexCurrent += 16,
				pVertices += 6)
			{
				if (batchIdx == quadBatchNum - 1)
				{
					int left = quadData.nQuads & 15;
					if (left == 0) left = 16;
					for (int i = 0; i < left; i += 4) //per 4 triangle one batch
					{
						tempV[i] = quadData.verts[pIndexCurrent[i]];
						tempV[i + 1] = quadData.verts[pIndexCurrent[i + 1]];
						tempV[i + 2] = quadData.verts[pIndexCurrent[i + 2]];
						tempV[i + 3] = quadData.verts[pIndexCurrent[i + 3]];
					}
					if (left < 16)
					{
						__m128 defaultP = tempV[0];
						for (int i = left; i < 16; i += 4)
						{
							tempV[i] = defaultP;
							tempV[i + 1] = defaultP;
							tempV[i + 2] = defaultP;
							tempV[i + 3] = defaultP;
						}
					}

				}
				else
				{
					for (int i = 0; i < 16; i += 4) //per 4 triangle one batch
					{
						tempV[i] = quadData.verts[pIndexCurrent[i]];
						tempV[i + 1] = quadData.verts[pIndexCurrent[i + 1]];
						tempV[i + 2] = quadData.verts[pIndexCurrent[i + 2]];
						tempV[i + 3] = quadData.verts[pIndexCurrent[i + 3]];
					}
				}


				for (uint32_t j = 0; j < 4; ++j)
				{
					// Transform into [0,1] space relative to bounding box
					__m128 v0 = _mm_fmadd_ps(tempV[j + 0], invExtents, minusRefMinInvExtents);
					__m128 v1 = _mm_fmadd_ps(tempV[j + 4], invExtents, minusRefMinInvExtents);
					__m128 v2 = _mm_fmadd_ps(tempV[j + 8], invExtents, minusRefMinInvExtents);
					__m128 v3 = _mm_fmadd_ps(tempV[j + 12], invExtents, minusRefMinInvExtents);

					// Transpose into [xxxx][yyyy][zzzz][wwww]
					_MM_TRANSPOSE4_PS(v0, v1, v2, v3);


					__m128i X = _mm_cvttps_epi32(v0);
					__m128i Y = _mm_cvttps_epi32(v1);
					__m128i Z = _mm_cvttps_epi32(v2);

					//this is to force into the range[0, 0xFFFF]
					__m128i max = _mm_set1_epi32(65535);
					X = _mm_min_epi32(X, max);
					Y = _mm_min_epi32(Y, max);
					Z = _mm_min_epi32(Z, max);
					__m128i min = _mm_set1_epi32(0);
					X = _mm_max_epi32(X, min);
					Y = _mm_max_epi32(Y, min);
					Z = _mm_max_epi32(Z, min);

					if ((j & 1) == 0) {
						X = _mm_slli_epi32(X, 16);
						Y = _mm_slli_epi32(Y, 16);
						Z = _mm_slli_epi32(Z, 16);
					}

					int pIdx = (j >> 1);  //0 or 1
					pVertices[pIdx] = _mm_or_si128(pVertices[pIdx], X); //01 for X,  23 for Y, 45 for Z
					pVertices[pIdx + 2] = _mm_or_si128(pVertices[pIdx + 2], Y);
					pVertices[pIdx + 4] = _mm_or_si128(pVertices[pIdx + 4], Z);

				}
			}

		}

		int64_t used = pVertices - compressData;
		assert(used == 2 + quadBatchNum * 6);
		if (triangleBatchNum > 0)
		{
			__m128 tempV[12];

			pIndexCurrent = quadData.indices;
			if (pIndexCurrent == nullptr) pIndexCurrent = indices;
			pIndexCurrent += quadData.nQuads;
			__m128i ZTemp[3];
			for (int i = 0; i < triangleBatchNum; i++) //per 4 triangle one batch
			{
				uint32_t startFaceIdx = i << 2;
				uint32_t qIdxEnd = std::min<uint32_t>(4, triangleFaceNum - startFaceIdx) * 3;

				unsigned int qIdx3 = 0;
				for (; qIdx3 < qIdxEnd; pIndexCurrent += 3, qIdx3 += 3)
				{
					tempV[qIdx3] = quadData.verts[pIndexCurrent[0]];
					tempV[qIdx3 + 1] = quadData.verts[pIndexCurrent[2]];
					tempV[qIdx3 + 2] = quadData.verts[pIndexCurrent[1]];
				}

				while (qIdx3 < 12)
				{
					auto p = tempV[0];
					tempV[qIdx3] = p;
					tempV[qIdx3 + 1] = p;
					tempV[qIdx3 + 2] = p;
					qIdx3 += 3;
				}

				for (uint32_t j = 0; j < 3; ++j)
				{
					// Transform into [0,1] space relative to bounding box
					__m128 v0 = _mm_fmadd_ps(tempV[j + 0], invExtents, minusRefMinInvExtents);
					__m128 v1 = _mm_fmadd_ps(tempV[j + 3], invExtents, minusRefMinInvExtents);
					__m128 v2 = _mm_fmadd_ps(tempV[j + 6], invExtents, minusRefMinInvExtents);
					__m128 v3 = _mm_fmadd_ps(tempV[j + 9], invExtents, minusRefMinInvExtents);

					// Transpose into [xxxx][yyyy][zzzz][wwww]
					_MM_TRANSPOSE4_PS(v0, v1, v2, v3);


					__m128i X = _mm_cvttps_epi32(v0);
					__m128i Y = _mm_cvttps_epi32(v1);
					__m128i Z = _mm_cvttps_epi32(v2);

					//this is to force into the range[0, 0xFFFF]
					__m128i max = _mm_set1_epi32(65535);
					X = _mm_min_epi32(X, max);
					Y = _mm_min_epi32(Y, max);
					Z = _mm_min_epi32(Z, max);
					__m128i min = _mm_set1_epi32(0);
					X = _mm_max_epi32(X, min);
					Y = _mm_max_epi32(Y, min);
					Z = _mm_max_epi32(Z, min);

					__m128i XY = _mm_or_si128(_mm_slli_epi32(X, 16), Y);

					pVertices[j] = XY;
					ZTemp[j] = Z;
				}
				pVertices[3] = _mm_or_si128(_mm_slli_epi32(ZTemp[1], 16), ZTemp[2]);


				uint16_t* t16 = (uint16_t*)ZTemp;
				uint64_t t = t16[0];
				uint64_t t1 = t16[2];
				uint64_t t2 = t16[4];
				uint64_t t3 = t16[6];
				t |= t1 << 16;
				t |= t2 << 32;
				t |= t3 << 48;

				if ((i & 1) == 0)
				{
					uint64_t * r = (uint64_t*)(pVertices + 4);
					r[0] = t;
					pVertices += 5;
				}
				else {
					uint64_t * r = (uint64_t*)(pVertices);
					r = r - 1;
					r[0] = t;

					pVertices += 4;
				}
			}

		}
		used = pVertices - compressData;
		assert(used == compressSize);

		*outputCompressSize = (int)used * 8; 
		//if (((used * 16)  * 100.0 / (nVert * 12 + nIdx * 2)) > 90.0f) 
		//{
		//	std::cout << "Current Model save idx " << DebugOccluderIdx << std::endl;
		//}
		//std::cout << "***********ByteUsed " << (used * 16) << " Original " << (nVert * 12 + nIdx * 2) << " Ratio " << ((used * 16)  * 100.0 / (nVert * 12 + nIdx * 2)) << "%" << std::endl;

		return (unsigned short*)outputBuffer;  //this is a temp buffer, the data inside should be retrieved immediately.
	}




	bool OccluderQuad::sdocMeshSimplify(float* vertices, unsigned short* indices, unsigned int& nVert, unsigned int& nIdx, int modelId, unsigned int targetFaceNum, bool saveModel)
	{
		if (saveModel)
		{
			std::string savedFile = std::to_string(modelId) + std::string("_Input.OFF");
			LOGI("save: %s", SOCLogger::GetOutputDirectory() + savedFile.c_str());
			util::OccluderQuad::storeModel(SOCLogger::GetOutputDirectory() + savedFile, vertices, nVert, indices, nIdx);
		}

		int reducedFaceNum = 0;
		int reducedVertNum = 0;
		bool bValid = common::MeshReducer::reduce(vertices, indices, nVert, nIdx,
			vertices, reducedVertNum, indices, reducedFaceNum, targetFaceNum);

		if (bValid)
		{
			nVert = reducedVertNum;
			nIdx = reducedFaceNum * 3;
			if (saveModel)
			{
				std::string savedFile = std::to_string(modelId) + std::string("_LOD.OFF");
				LOGI("save: %s", SOCLogger::GetOutputDirectory() + savedFile.c_str());
				util::OccluderQuad::storeModel(SOCLogger::GetOutputDirectory() + savedFile, vertices, nVert, indices, nIdx);
			}
		}
		return bValid;
	}

	static int mModelID = 10000;
	unsigned short* OccluderQuad::sdocMeshLodBake(int* outputCompressSize, const float* vertices, const unsigned short* indices, unsigned int nVert, unsigned int nIdx, float quadAngle, bool enableBackfaceCull, bool counterClockWise, int TerrainGridAxisPoint)
	{

		bool simplifyMesh = false;
		if (simplifyMesh && nIdx > 3000)
		{
			std::vector<float> vp;
			std::vector<uint16_t> vi;
			vp.resize(nVert * 3);
			vi.resize(nIdx);
			memcpy(vp.data(), vertices, vp.size() * sizeof(float));
			memcpy(vi.data(), indices, vi.size() * sizeof(uint16_t));

			unsigned int reducedIdxNum = nIdx;
			unsigned int reducedVertNum = nVert;

			auto startTime = std::chrono::high_resolution_clock::now();

			bool bValid = sdocMeshSimplify(vp.data(), vi.data(), reducedVertNum, reducedIdxNum, mModelID++,	std::min<int>(3000, nIdx / 3 * 0.9), true);


			if (bValid)
			{
				std::cout << "Simplified from " << nIdx / 3 << "  " << reducedIdxNum << std::endl;
				unsigned short* data = util::OccluderQuad::sdocMeshBake(outputCompressSize, vp.data(), vi.data(), reducedVertNum, reducedIdxNum, quadAngle, enableBackfaceCull, counterClockWise, TerrainGridAxisPoint);				
				return data;
			}
		}
		unsigned short* data = util::OccluderQuad::sdocMeshBake(outputCompressSize, vertices, indices, nVert, nIdx, quadAngle, enableBackfaceCull, counterClockWise, TerrainGridAxisPoint);
		return data;
	}

}

#endif
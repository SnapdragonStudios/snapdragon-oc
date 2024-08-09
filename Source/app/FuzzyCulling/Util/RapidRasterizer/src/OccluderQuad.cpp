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
#include <unordered_map>
#endif
namespace util
{
	//***************************************
	//Mesh Baking Process
	//1. Duplicate vertices checking and removal
	//2. Quad formation such that the common edge is longest in Quad's two triangles
	//3. Quad formation as long as the quad is convex
	//4. Simple planar mesh checking and simplification, detect whether it is a simple rectangle or a simple general topological disk such as a filled circle
	//5. Quad merging for terrain
	//6. Quad merging for normal mesh round 1: merge rectangle quad
	//7. Quad merging for normal mesh round 2~5: merge general planar quad
	//8. Merge Triangle into Quad
	//9. Output baked data in following order
	//   Add baked data header
	//   Add baked data indices, compress the final mesh's indices array if vertices number is less than 85
	//   Add baked data's vertices
	
	static bool SaveSimplifyModel = false;
	static std::string outputSaveDirectory = "D:\\TempSDOC";
	//******************************************************************************
	static constexpr bool  AllowSecondRoundQuadMerge = true;  //set to false to disable second round quad formation
	static constexpr bool  AllowFreeMergeForMesh = true;
	//developer todo: tunning the ratio, approaching one would means smaller error
	static float GetSecondRoundCosQuadAngle(float cosQuadAngle) {
		if (AllowSecondRoundQuadMerge == false) return 2.0;
		float degreeToRadian = 3.1415926f / 180.0f;
		//return cos(15 * degreeToRadian);
		return cosQuadAngle;            //same condition with longest edge quad
	}
	static float GetSecondRoundQuadTwoFaceFreeMerge(float QuadTwoSameFaceTh) {
		float degreeToRadian = 3.1415926f / 180.0f;
		return cos(1.0f * degreeToRadian);
	}
	
	static float GetSecondRoundPlanarTh() {
		float degreeToRadian = 3.1415926f / 180.0f;
		return cos(1.0f * degreeToRadian);
	}
	static float GetSecondRoundQuadMergeLineLinkMerge(float lineLinkTh) {		
		float degreeToRadian = 3.1415926f / 180.0f;
		return cos(1.0f * degreeToRadian);
	}

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
			ProjectLineMidAngleTh = cos(3.0f * degreeToRadian); //cos(3 degree)
			 QuadFourSameFaceTh = QuadTwoSameFaceTh = cos(ModelRectangleMergeAngle * degreeToRadian);
			PlaneQuadDotThreshold = cos(0.2f * degreeToRadian);   //for mesh apply strict check

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
	static int DebugOccluderID = 0;
	static std::string GetDebugOccluderKey(QuadTriIndex& qti) {
		return std::to_string(DebugOccluderIdx++)+"_" +std::to_string(DebugOccluderID) +
			"_" + std::to_string(qti.nQuads / 4) + "_" + std::to_string(qti.nTriangles / 3) +"_" + std::to_string(qti.splitOneQuadToTriangles);
	}

	static void savePoint(FILE* fileWriter, const float* p)
	{
		fprintf(fileWriter, "%f %f %f\n",
			p[0], p[1], p[2]);
	}
	static void saveTriangle(FILE* fileWriter, const uint16_t* p)
	{
		fprintf(fileWriter, "3 %d %d %d\n",
			p[0], p[2], p[1]);
	}

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
			savePoint(fileWriter, vertices + i_vert * 3);
		}

		for (unsigned int i_face = 0; i_face < nFace; ++i_face)
		{
			saveTriangle(fileWriter, indices+i_face * 3);
		}
		fclose(fileWriter);

		return true;
	}

	static __m128 cross(__m128 a, __m128 b)
	{
		__m128 a_yzx = _mm_shuffle_ps(a, a, _MM_SHUFFLE(3, 0, 2, 1));
		__m128 b_yzx = _mm_shuffle_ps(b, b, _MM_SHUFFLE(3, 0, 2, 1));
		__m128 c = _mm_fmsub_ps_soc(a, b_yzx, _mm_mul_ps(a_yzx, b));
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

	void  OccluderQuad::checkRectangleQuad(MeshQuad* q, __m128* vertices)
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
#else
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



	uint16_t* OccluderBakeBuffer::GetIndicesBySize(uint32_t size)
	{
		if (size == 0) return nullptr;
		mIndices = new uint16_t[size];
		return mIndices;
	}

	__m128* OccluderBakeBuffer::GetPointsBySize(uint32_t size)
	{
		if (size == 0) return nullptr;
		mPoints = new __m128[size];
		return mPoints;
	}

	void OccluderBakeBuffer::RequestFaceBySize(uint32_t size)
	{
		if (size == 0) return;
		mFaces = new util::MeshFace[size];
	}

	void OccluderBakeBuffer::RequestQuadBySize(uint32_t size)
	{
		if (size == 0) return;
		mQuads = new util::MeshQuad[size];
		memset(mQuads, 0, size * sizeof(util::MeshQuad));		
	}

	void OccluderBakeBuffer::RequestVertexBySize(uint32_t size)
	{
		if (size == 0) return;
		mVertices = new util::MeshVertex[size];
		for (uint32_t idx = 0; idx < size; idx++)
		{
			mVertices[idx].VertID = idx;
		}
	}


	OccluderBakeBuffer::~OccluderBakeBuffer()
	{
		if (mParent != nullptr) delete mParent;
		if (mRequest != nullptr) delete mRequest;
		if (mReIdx != nullptr) delete[] mReIdx;
		if (mRemap != nullptr) delete[] mRemap;
		if (mIndices != nullptr) delete[] mIndices;
		if (mPoints != nullptr) delete[] mPoints;
		if (mFaces != nullptr) delete[] mFaces;
		if (mVertices != nullptr) delete[] mVertices;
		if (mQuads != nullptr) delete[] mQuads;
		}

	const uint16_t* OccluderBakeBuffer::ReIndex(const uint16_t* indices, const float* inputVertices, int vertNum, unsigned int nIdx)
	{
		mReIdx = new uint16_t[nIdx];
		mRemap = new uint16_t[vertNum];
		mRemap[0] = 0;
		for (int idx = 1; idx < vertNum; idx++)
		{
			mRemap[idx] = idx;

			for (int scan = 0; scan < idx; scan++)
			{
				int idx3 = idx * 3;
				int scan3 = scan * 3;
				if (inputVertices[idx3] == inputVertices[scan3] &&
					inputVertices[idx3 + 1] == inputVertices[scan3 + 1] &&
					inputVertices[idx3 + 2] == inputVertices[scan3 + 2])
				{
					mRemap[idx] = scan;

					break;
				}
			}
		}

		for (unsigned int idx = 0; idx < nIdx; idx++)
		{
			mReIdx[idx] = mRemap[indices[idx]];
		}
		return mReIdx;
	}

	static float getArea(__m128 p0, __m128 p1, __m128 p2) {
		__m128 e1 = _mm_sub_ps(p1, p0);
		__m128 e2 = _mm_sub_ps(p2, p1);
		__m128 e3 = _mm_sub_ps(p2, p0);
		__m128 e1e3Cross = cross(e1, e3);
		__m128 area2 = _mm_dp_ps(e1e3Cross, e1e3Cross, 0x7F);
		return  sqrt(((float*)&area2)[0]);
	}

	static __m128 getUnitNormalArea(__m128 p0, __m128 p1, __m128 p2, float& area) {
		__m128 e1 = _mm_sub_ps(p1, p0);
		__m128 e2 = _mm_sub_ps(p2, p1);
		__m128 e3 = _mm_sub_ps(p2, p0);
		__m128 e1e3Cross = cross(e1, e3);
		__m128 area2 = _mm_sqrt_ps(_mm_dp_ps(e1e3Cross, e1e3Cross, 0x7F));
		area = ((float*)&area2)[0];
		return _mm_div_ps(e1e3Cross, area2);
	}


	static void SplitQuadToTriangle(MeshQuad& mq, QuadTriIndex& qti)
	{
		if (mq.QuadKids > 0) {
			mq.face1->ValidSingleTriangle = true;
			mq.face2->ValidSingleTriangle = true;
			mq.face1->nVertIdx[0] = mq.vIdx[0];
			mq.face1->nVertIdx[1] = mq.vIdx[2];
			mq.face1->nVertIdx[2] = mq.vIdx[1];
			mq.face2->nVertIdx[0] = mq.vIdx[0];
			mq.face2->nVertIdx[1] = mq.vIdx[3];
			mq.face2->nVertIdx[2] = mq.vIdx[2];
			mq.QuadKids = 0; //disable quad
			qti.nQuads -= 4;
			qti.nTriangles += 6;
			qti.splitOneQuadToTriangles++;
		}
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
		qti.nTriangles = nIdx;
		//quadAngle = 0;
		if (nTri > 65535 ) //not possible to form a Quad
		{
			qti.indices = nullptr;
			qti.nQuads = 0;
			return;
		}

		uint32_t nVert = static_cast<uint32_t>(vertNum);
		pBakeBuffer->RequestVertexBySize(nVert);
		util::MeshVertex* Vertices = pBakeBuffer->mVertices;
		

		pBakeBuffer->RequestFaceBySize(nTri);
		auto Faces = pBakeBuffer->mFaces;
		std::unordered_map<uint64_t, uint8_t>* keyMap = new std::unordered_map<uint64_t, uint8_t>();
		for (uint32_t i_tri = 0, i_vert = 0; i_tri < nTri; i_tri++, i_vert += 3)
		{
			auto v0 = &Vertices[indices[i_vert]];
			auto v1 = &Vertices[indices[i_vert + 1]];
			auto v2 = &Vertices[indices[i_vert + 2]];
			Faces[i_tri].init(i_tri, v0, v1, v2);
			uint64_t key = Faces[i_tri].uniqueFaceID;
			if (keyMap->find(key) == keyMap->end()) {
				keyMap->insert({ key,1 });
			}
			else {
				Faces[i_tri].area = 0;
			}
		}
		delete keyMap;

		int zeroTriangle = 0; //triangles with zero area, duplicated or really zero area

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
			f->area *= ((float*)&area2)[0];

			float len1 = (dotSum3(e1, e1)); //square length
			float len2 = (dotSum3(e2, e2));
			float len3 = (dotSum3(e3, e3));

			if (f->area == 0) { //duplicated or really zero area triangles
				f->ValidSingleTriangle = false;
				zeroTriangle++;
			}


			if (len1 >= len2 && len1 >= len3) f->LongestEdgeIdx = 0;
			else if (len2 >= len1 && len2 >= len3) f->LongestEdgeIdx = 1;
			else f->LongestEdgeIdx = 2;
		}

		int mergedQuad = 0;
		//match with smaller faceid, so start from face id 1
		for (uint32_t i_tri = 1; i_tri < nTri; ++i_tri)
		{
			MeshFace *f = &Faces[i_tri];
			if (f->ValidSingleTriangle == false) continue;

			{
				int longestEdgeIdx = f->LongestEdgeIdx;
				MeshEdgeFace *fe = &f->Edges[longestEdgeIdx];
				auto fv = f->nVertIdx[longestEdgeIdx];

				//purpose: find all the Smaller face that share the reverse of edge fe
				//ascending order linked list
				auto neighbors = Vertices[fe->endVert].EdgeList;
				for (auto e = neighbors; e != nullptr; e = e->Next)
				{
					if (e->FaceIdx >= f->FaceIdx)
					{
						//neighbors are face id increasing linked list
						//we only test each face with the smaller face to avoid duplicate calculation
						//up to this stage, all the face would be larger than faceID, 
						//so use break, not continue

						break;
					}
					if (e->endVert == fv)
					{
						MeshFace * eface = &Faces[e->FaceIdx];
						if (eface->ValidSingleTriangle)
						{
							//first round longest edge to merge
							MeshEdgeFace* feOpposite = &eface->Edges[eface->LongestEdgeIdx];
							if (feOpposite->startVert != fe->endVert ||
								feOpposite->endVert != fe->startVert) {
								continue;
							}

							float dotSum = dotSum3(f->UnitNormal, eface->UnitNormal);
							//allow SDOC Quad drawing
							if (dotSum >= cosQuadAngle)
							{
								mergedQuad++;

								eface->PairFace = f;
								f->PairFace = eface;
								eface->ValidSingleTriangle = false;
								f->ValidSingleTriangle = false;

								f->MergedEdgeIdx = fe->edgeIdx;
								eface->MergedEdgeIdx = e->edgeIdx;
								
								

								f->IsPlanarQuad = eface->IsPlanarQuad = (dotSum >= PlaneQuadDotThreshold) && bSimplifyMesh;
								break;
							}		
						}
					}
				}
			}
		}

		int secondRoundQuad = 0;
		float cosQuadAngle2 = GetSecondRoundCosQuadAngle(cosQuadAngle);;  //apply more strict condition than largest edge merge
		float planarQuadTh = GetSecondRoundPlanarTh();  //apply more strict condition than largest edge merge
		for (uint32_t i_tri = 1; i_tri < nTri; ++i_tri)
		{
			MeshFace* f = &Faces[i_tri];
			if (f->ValidSingleTriangle == false) continue;

			for(int edgeIdx = 0; edgeIdx <=2; edgeIdx++)
			{
				MeshEdgeFace* fe = &f->Edges[edgeIdx];
				auto fv = f->nVertIdx[edgeIdx];

				//purpose: find all the Smaller face that share the reverse of edge fe
				//ascending order linked list
				auto neighbors = Vertices[fe->endVert].EdgeList;

				uint16_t startVert = fe->endVert;
				for (auto e = neighbors; e != nullptr; e = e->Next)
				{
					if (e->FaceIdx >= f->FaceIdx)
					{
						//neighbors are face id increasing linked list
						//we only test each face with the smaller face to avoid duplicate calculation
						//up to this stage, all the face would be larger than faceID, 
						//so use break, not continue

						break;
					}
					if (e->endVert == fv)
					{
						MeshFace* eface = &Faces[e->FaceIdx];
						if (eface->ValidSingleTriangle)
						{
							float dotSum = dotSum3(f->UnitNormal, eface->UnitNormal);
							//allow SDOC Quad drawing
							if (dotSum >= cosQuadAngle2)
							{
								uint16_t endVert = e->endVert;
								uint16_t apexVert = e->apexVert;
								uint16_t apexVert2 = 0;
								if (f->nVertIdx[0] != startVert && f->nVertIdx[0] != endVert) {
									apexVert2 = f->nVertIdx[0];
								}
								else if (f->nVertIdx[1] != startVert && f->nVertIdx[1] != endVert) {
									apexVert2 = f->nVertIdx[1];
								}
								else {
									apexVert2 = f->nVertIdx[2];
								}

								auto startPt = points[startVert];
								auto endPt = points[endVert];
								auto apexPt = points[apexVert];
								auto apexPt2 = points[apexVert2];
								//check convex property
								float combineArea = getArea(startPt, apexPt, apexPt2);
								float combineArea2 = getArea(endPt, apexPt, apexPt2);
								float AreaSum = eface->area + f->area;
								if (combineArea2 >= AreaSum || combineArea >= AreaSum) {
									continue;
								}
								mergedQuad++;

								eface->PairFace = f;
								f->PairFace = eface;
								eface->ValidSingleTriangle = false;
								f->ValidSingleTriangle = false;

								f->MergedEdgeIdx = fe->edgeIdx;
								eface->MergedEdgeIdx = e->edgeIdx;

								secondRoundQuad++;

								f->IsPlanarQuad = eface->IsPlanarQuad = (dotSum >= planarQuadTh) && bSimplifyMesh;
								edgeIdx = 3;
								break;
							}
						}
					}
				}
			}
		}
		if(SaveSimplifyModel && secondRoundQuad > 0)
			std::cout << "secondRoundQuad   " << secondRoundQuad << "  mergedQuad  " << mergedQuad << " Quad ratio " << (secondRoundQuad * 1.0 / mergedQuad) <<" Triangles " <<(nTri-mergedQuad*2) << std::endl;
		
		if (SaveSimplifyModel) {
			CreateOutputDir();
			std::string file_path = outputSaveDirectory + "\\" + GetDebugOccluderKey(qti)  + "input.off";
			storeModel(file_path, inputVertices, nVert, indices, nIdx);
		}

		if (mergedQuad == 0 && zeroTriangle == 0) //not form a Quad
		{
			qti.indices = nullptr;
			qti.nQuads = 0;
			qti.nTriangles = nIdx;
			return;
		}


		pBakeBuffer->mIsPlanarMesh = false;
		if ( bSimplifyMesh)
		{
			//quick check whether it is actually a simple quad
			__m128 extent = _mm_sub_ps(refMax, refMin);
			float * aabbf = (float*)&extent;
			float max = std::max<float>(aabbf[0], aabbf[1]);
			max = std::max<float>(aabbf[2], max);
			float planarTh = max * 0.0001f;

			int xFlat = aabbf[0] < planarTh;
			int yFlat = aabbf[1] < planarTh;
			int zFlat = aabbf[2] < planarTh;
			//Optimization: 
			// 1. check whether it is a simple quad and output the quad if conditions allow
			// 2. check whether it is a simple genus 0 shape could be represented by a center and boundary edges
			if (xFlat + yFlat + zFlat == 1 && pBakeBuffer->mParent == nullptr)
			{
				//check whether point to same direction
				bool allSameDirection = true;
				__m128 normal = Faces[0].UnitNormal;
				for (uint32_t i_tri = 1; i_tri < nTri; ++i_tri)
				{
					MeshFace *f = &Faces[i_tri];
					float dotSum = dotSum3(f->UnitNormal, normal);
					allSameDirection &= dotSum > 0.9999f;
				}

				pBakeBuffer->mIsPlanarMesh  = allSameDirection;
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

						qti.nQuads = 4;
						qti.nTriangles = 0;
						qti.indices = pBakeBuffer->GetIndicesBySize(6);
						qti.indices[0] = 3;
						qti.indices[1] = 2;
						qti.indices[2] = 1;
						qti.indices[3] = 0;
						qti.nActiveVerts = 4;

						if (SaveSimplifyModel) {
							CreateOutputDir();
							std::string file_path = outputSaveDirectory + "\\" + GetDebugOccluderKey(qti) + "outQuad.off";

                            FILE *fileWriter = fopen(file_path.c_str(), "w");
                            if (fileWriter != nullptr)
							{
								fprintf(fileWriter, "OFF\n");
								fprintf(fileWriter, "%d %d 0\n", 4, 1);

								float * vertf = (float*)qti.verts;
								for (uint32_t i_vert = 0; i_vert < 4; ++i_vert)
								{
									savePoint(fileWriter, vertf + i_vert * 4);
								}
								//save quads to two triangles
								fprintf(fileWriter, "4 3 2 1 0\n");
								fclose(fileWriter);
							}
						}

						return;
					}

					//now find edges have degree one and check whether those edge points could form a convex
					for (uint32_t i_tri = 0; i_tri < nTri; ++i_tri)
					{
						MeshFace* f = &Faces[i_tri];
						for (uint32_t edgeIdx = 0; edgeIdx < 3; edgeIdx++) {

							MeshEdgeFace* fe = &f->Edges[edgeIdx];
							auto fv = f->nVertIdx[edgeIdx];
							auto neighbors = Vertices[fe->endVert].EdgeList;

							uint16_t startVert = fe->endVert;
							for (auto e = neighbors; e != nullptr; e = e->Next)
							{
								if (e->endVert == fv) {
									fe->degree++;
								}
							}
						}
					}

					__m128 avgPt = _mm_set1_ps(0.0);
					float areaSumCenter = 0;
					float areaSumOriginal = 0;
					uint32_t totalDegreeOne = 0;
					int totalDegreeTwo = 0;
					for (uint32_t i_tri = 0; i_tri < nTri; ++i_tri)
					{
						MeshFace* f = &Faces[i_tri];
						for (uint32_t edgeIdx = 0; edgeIdx < 3; edgeIdx++) {

							MeshEdgeFace* fe = &f->Edges[edgeIdx];
							if (fe->degree == 1) {
								totalDegreeOne++;
								avgPt = _mm_add_ps(avgPt, points[fe->startVert]);
							}
							else if (fe->degree == 2)
								totalDegreeTwo++;
						}
					}
					if (totalDegreeOne > 0 && (totalDegreeOne + totalDegreeTwo) == nTri * 3) {
						avgPt = _mm_div_ps(avgPt, _mm_set1_ps((float)totalDegreeOne));
						bool sameDir = true;
						if (totalDegreeOne * 2 < nTri) {
							//now check the point fall on the same side of edge
							for (uint32_t i_tri = 0; i_tri < nTri; ++i_tri)
							{
								MeshFace* f = &Faces[i_tri];
								areaSumOriginal += f->area;
								for (uint32_t edgeIdx = 0; edgeIdx < 3; edgeIdx++) {

									MeshEdgeFace* fe = &f->Edges[edgeIdx];
									if (fe->degree == 1) {
										float faceArea = 0;
										__m128 unitNormal = getUnitNormalArea(points[fe->startVert], points[fe->endVert], avgPt, faceArea);
										areaSumCenter += faceArea;
										float dotSum = dotSum3(f->UnitNormal, unitNormal);
										sameDir &= dotSum > 0.99;
									}
								}
							}

							if (sameDir && abs(areaSumOriginal / areaSumCenter - 1.0) < 0.001) {
								float* pts = new float[3 *(vertNum + 1)];
								memcpy(pts, inputVertices, vertNum * 3 * sizeof(float));
								float *extraPt = (float*)&avgPt;
								memcpy(pts+vertNum * 3, extraPt, 3 * sizeof(float));
								uint16_t* indices2 = new uint16_t[totalDegreeOne * 3];
								int indicesIdx = 0;
								for (uint32_t i_tri = 0; i_tri < nTri; ++i_tri)
								{
									MeshFace* f = &Faces[i_tri];
									for (uint32_t edgeIdx = 0; edgeIdx < 3; edgeIdx++) {

										MeshEdgeFace* fe = &f->Edges[edgeIdx];
										if (fe->degree == 1) {
											indices2[indicesIdx++] = fe->startVert;
											indices2[indicesIdx++] = fe->endVert;											
											indices2[indicesIdx++] = vertNum;  //debug: change fe->apexVert for degree 2 triangles
										}
									}
								}
								pBakeBuffer->CreateNewBakeRequest(pts, vertNum + 1, indices2, totalDegreeOne * 3);

								if (SaveSimplifyModel) {
									CreateOutputDir();
									std::string file_path = outputSaveDirectory + "\\" + GetDebugOccluderKey(qti) + "inputEXTRA.off";
									storeModel(file_path, pts, vertNum + 1, indices2, totalDegreeOne * 3);
								}
								return ;
							}
						}

					}
				}
			}
		}

		int totalRequestQuads = mergedQuad;
		pBakeBuffer->RequestQuadBySize(totalRequestQuads);
		auto Quads = pBakeBuffer->mQuads;
		uint32_t quadIdx = 0;

		int totalRectangleQuad = 0;
		//assign all generate rect quads
		for (uint32_t i_tri = 0; i_tri < nTri; ++i_tri)
		{
			MeshFace *face1 = &Faces[i_tri];
			MeshFace *face2 = face1->PairFace;

			if (face2 != nullptr && i_tri < face2->FaceIdx)
			{
				//retrieve the quad info
				auto q = &Quads[quadIdx];
				q->qIdx = quadIdx;
				q->face1 = face1;
				q->face2 = face2;
				q->quad_area = face2->area + face1->area;

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
				q->IsPlanarQuad = face1->IsPlanarQuad;

				quadIdx++;

			}
		}

		bool toSimplifyMesh = quadIdx > 1;
		

		qti.nTriangles = 3 * (nTri - mergedQuad * 2 - zeroTriangle);
		uint32_t quadNum = quadIdx;

		

		int quadFourMergeCount = 0;
		int quadTwoMergeCount = 0;

		float lineLinkTH = ProjectLineMidAngleTh;
		if (toSimplifyMesh)
		{
			int neighborMergeCount = 0;
			bool quadMerge = 1;
			if (quadMerge) {
				bool allowDiffKidMerge = false;
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

								if (MergeQuad(q0, q1, points, Vertices, isTerrain, quadFourMergeCount, lineLinkTH, allowDiffKidMerge)) continue;

							}
							else if (v->RectangleDegree == 3) {
								uint16_t p0 = v->quad[0];
								uint16_t p1 = v->quad[1];
								uint16_t p2 = v->quad[2];

								MeshQuad * q0 = &Quads[p0];
								MeshQuad * q1 = &Quads[p1];
								MeshQuad * q2 = &Quads[p2];


								if (MergeQuad(q0, q1, points, Vertices, isTerrain, quadFourMergeCount, lineLinkTH, allowDiffKidMerge)) continue;
								if (MergeQuad(q0, q2, points, Vertices, isTerrain, quadFourMergeCount, lineLinkTH, allowDiffKidMerge)) continue;
								if (MergeQuad(q1, q2, points, Vertices, isTerrain, quadFourMergeCount, lineLinkTH, allowDiffKidMerge)) continue;

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
									if (q0->MergeWith(q1, QuadTwoSameFaceTh, points, Vertices, isTerrain, quadFourMergeCount, lineLinkTH))
									{
										if (q2->MergeWith(q3, QuadTwoSameFaceTh, points, Vertices, isTerrain, quadFourMergeCount, lineLinkTH))
											q0->MergeWith(q2, QuadTwoSameFaceTh, points, Vertices, isTerrain, quadFourMergeCount, lineLinkTH);
									}
									else if (q0->MergeWith(q2, QuadTwoSameFaceTh, points, Vertices, isTerrain, quadFourMergeCount, lineLinkTH))
									{
										if (q1->MergeWith(q3, QuadTwoSameFaceTh, points, Vertices, isTerrain, quadFourMergeCount, lineLinkTH))
											q0->MergeWith(q1, QuadTwoSameFaceTh, points, Vertices, isTerrain, quadFourMergeCount, lineLinkTH);
									}
									else if (q0->MergeWith(q3, QuadTwoSameFaceTh, points, Vertices, isTerrain, quadFourMergeCount, lineLinkTH))
									{
										if (q1->MergeWith(q2, QuadTwoSameFaceTh, points, Vertices, isTerrain, quadFourMergeCount, lineLinkTH))
											q0->MergeWith(q1, QuadTwoSameFaceTh, points, Vertices, isTerrain, quadFourMergeCount, lineLinkTH);
									}
									else if (q2->MergeWith(q3, QuadTwoSameFaceTh, points, Vertices, isTerrain, quadFourMergeCount, lineLinkTH)) {

									}
									else if (q2->MergeWith(q1, QuadTwoSameFaceTh, points, Vertices, isTerrain, quadFourMergeCount, lineLinkTH)) {

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
											q0->MergeWith(q1, QuadTwoSameFaceTh, points, Vertices, isTerrain, degree2Merge, lineLinkTH);
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
															q0->MergeWith(q1, QuadTwoSameFaceTh, points, Vertices, isTerrain, degree2Merge, lineLinkTH);
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

			int mergeRound = 0;
			bool neighborMerge = 1;
			//terrain already handled in quad merge
			if (neighborMerge && isTerrain == false && toSimplifyMesh)
			{				

				int terminateTh = 5;
				bool freeShapePlanarMerge = false;
				bool allowDiffKidMerge = false;
				do
				{
					neighborMergeCount = 0;
					SetRectangleDegreeToZero(Vertices, nVert);

					

					if (freeShapePlanarMerge == false) {
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
					if (freeShapePlanarMerge) {
						QuadTwoSameFaceTh = GetSecondRoundQuadTwoFaceFreeMerge(QuadTwoSameFaceTh); 
						
						lineLinkTH = GetSecondRoundQuadMergeLineLinkMerge(lineLinkTH);
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
								
								

								pBakeBuffer->QuickMerge(q0, q1, QuadTwoSameFaceTh, points, Vertices, neighborMergeCount, lineLinkTH, allowDiffKidMerge);
								pBakeBuffer->QuickMerge(q0, q2, QuadTwoSameFaceTh, points, Vertices, neighborMergeCount, lineLinkTH, allowDiffKidMerge);
								pBakeBuffer->QuickMerge(q0, q3, QuadTwoSameFaceTh, points, Vertices, neighborMergeCount, lineLinkTH, allowDiffKidMerge);
								pBakeBuffer->QuickMerge(q1, q2, QuadTwoSameFaceTh, points, Vertices, neighborMergeCount, lineLinkTH, allowDiffKidMerge);
								pBakeBuffer->QuickMerge(q1, q3, QuadTwoSameFaceTh, points, Vertices, neighborMergeCount, lineLinkTH, allowDiffKidMerge);
								pBakeBuffer->QuickMerge(q2, q3, QuadTwoSameFaceTh, points, Vertices, neighborMergeCount, lineLinkTH, allowDiffKidMerge);
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

								q0->MergeWith(q1, QuadTwoSameFaceTh, points, Vertices, false, neighborMergeCount, lineLinkTH);
							}
							else if (v->RectangleDegree == 3) {
								uint16_t p0 = v->quad[0];
								uint16_t p1 = v->quad[1];
								uint16_t p2 = v->quad[2];

								MeshQuad * q0 = &Quads[p0];
								MeshQuad * q1 = &Quads[p1];
								MeshQuad * q2 = &Quads[p2];

								if (q0->MergeWith(q1, QuadTwoSameFaceTh, points, Vertices, false, neighborMergeCount, lineLinkTH)) {
								}
								else if (q1->MergeWith(q2, QuadTwoSameFaceTh, points, Vertices, false, neighborMergeCount, lineLinkTH)) {
								}
								else if (q0->MergeWith(q2, QuadTwoSameFaceTh, points, Vertices, false, neighborMergeCount, lineLinkTH)) {
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
								
									float largestArea = std::max<float>(q0->face1->area,
										q1->face1->area);
									float areaTh = largestArea * 0.7f;  //allow area ratio around  0.7

									if (((q1->QuadKids ==  q0->QuadKids  && q0->QuadKids > 0) )
										&&
										q0->face1->area > areaTh &&
										q1->face1->area > areaTh)
									{
										q0->MergeWith(q1, QuadTwoSameFaceTh, points, Vertices, isTerrain, neighborMergeCount, lineLinkTH);
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


								pBakeBuffer->QuickMerge(q0, q1, QuadTwoSameFaceTh, points, Vertices, neighborMergeCount, lineLinkTH, allowDiffKidMerge);
								pBakeBuffer->QuickMerge(q0, q2, QuadTwoSameFaceTh, points, Vertices, neighborMergeCount, lineLinkTH, allowDiffKidMerge);
								pBakeBuffer->QuickMerge(q0, q3, QuadTwoSameFaceTh, points, Vertices, neighborMergeCount, lineLinkTH, allowDiffKidMerge);
								pBakeBuffer->QuickMerge(q1, q2, QuadTwoSameFaceTh, points, Vertices, neighborMergeCount, lineLinkTH, allowDiffKidMerge);
								pBakeBuffer->QuickMerge(q1, q3, QuadTwoSameFaceTh, points, Vertices, neighborMergeCount, lineLinkTH, allowDiffKidMerge);
								pBakeBuffer->QuickMerge(q2, q3, QuadTwoSameFaceTh, points, Vertices, neighborMergeCount, lineLinkTH, allowDiffKidMerge);
							}
							else if(false){
								int from = neighborMergeCount;
								uint16_t p0 = v->quad[0];
								uint16_t p1 = v->quad[1];
								uint16_t p2 = v->quad[2];
								uint16_t p3 = v->quad[3];
								MeshQuad* q0 = nullptr;
								MeshQuad* q1 = nullptr;
								MeshQuad* q2 = nullptr;
								MeshQuad* q3 = nullptr;
								if (p0 < totalRequestQuads) q0 = &Quads[p0];
								if (p1 < totalRequestQuads) q1 = &Quads[p1];
								if (p2 < totalRequestQuads) q2 = &Quads[p2];
								if (p3 < totalRequestQuads) q3 = &Quads[p3];


								pBakeBuffer->QuickMerge(q0, q1, QuadTwoSameFaceTh, points, Vertices, neighborMergeCount, lineLinkTH, allowDiffKidMerge);
								pBakeBuffer->QuickMerge(q0, q2, QuadTwoSameFaceTh, points, Vertices, neighborMergeCount, lineLinkTH, allowDiffKidMerge);
								pBakeBuffer->QuickMerge(q0, q3, QuadTwoSameFaceTh, points, Vertices, neighborMergeCount, lineLinkTH, allowDiffKidMerge);
								pBakeBuffer->QuickMerge(q1, q2, QuadTwoSameFaceTh, points, Vertices, neighborMergeCount, lineLinkTH, allowDiffKidMerge);
								pBakeBuffer->QuickMerge(q1, q3, QuadTwoSameFaceTh, points, Vertices, neighborMergeCount, lineLinkTH, allowDiffKidMerge);
								pBakeBuffer->QuickMerge(q2, q3, QuadTwoSameFaceTh, points, Vertices, neighborMergeCount, lineLinkTH, allowDiffKidMerge);
								
							}
						}
					}

					quadTwoMergeCount += neighborMergeCount;
					//for planar quad, allow free shape merge
					if (freeShapePlanarMerge == false && (AllowFreeMergeForMesh))
					{
						freeShapePlanarMerge = true;
					}
					if (SaveSimplifyModel) {
						std::cout << "MergeRound " << mergeRound << " TwoMergeCount " << neighborMergeCount << "   allowDiffKidMerge " << allowDiffKidMerge << " DebugOccluderID " << DebugOccluderID << std::endl;
					}
					mergeRound++;
					if (neighborMergeCount == 0 && mergeRound > 1 && mergeRound < 4) {
						mergeRound = 4;
					}
					allowDiffKidMerge = mergeRound == 4;
				} while (mergeRound < 5);
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


		//******************************************************
		//now try to merge triangle with neighboring quad
		if (qti.nTriangles > 0) {
			for (int loopIdx = 0; loopIdx < totalRequestQuads; ++loopIdx)
			{
				MeshQuad* mq = &Quads[loopIdx];
				if (mq->QuadKids > 0)
				{
					for (int qidx = 0; qidx < 4; qidx++) {
						uint16_t vIdx = mq->vIdx[qidx];
						util::MeshVertex& vert = Vertices[vIdx];
						vert.QuadIndices <<= 16;
						vert.QuadIndices |= loopIdx + 1; //store max four Quad Idx (+1) for vert
					}
				}
			}
			int mergedTriangle = 0;
			for (uint32_t i_tri = 0; i_tri < nTri; ++i_tri)
			{
				MeshFace* face = &Faces[i_tri];
				// No quad found
				if (face->ValidSingleTriangle)
				{
					for (int vidx = 0; vidx < 3; vidx++) {
						util::MeshVertex& vert = Vertices[face->nVertIdx[vidx]];
						uint64_t quadInfo = vert.QuadIndices;
						while (quadInfo > 0) {
							uint16_t vertexQuadIdx = (quadInfo & 65535);
							quadInfo >>= 16;
							if (vertexQuadIdx > 0) {
								MeshQuad* mq = &Quads[vertexQuadIdx - 1];
								if (MergeTriangleIntoQuad(face, mq, vert, points, Vertices)) {
									mergedTriangle++;
									vidx = 3;
									quadInfo = 0;
								}
							}
						}
					}
				}
			}
			if (mergedTriangle > 0 && SaveSimplifyModel) 
				std::cout << "*****************************************************EXTRA MERGE " << mergedTriangle << std::endl;
			qti.nTriangles -= mergedTriangle * 3;
		}
		
		//******************************************************

		if ((qti.nQuads & 15) == 4 && (qti.nTriangles % 12) <= 6 && (qti.nTriangles % 12) > 0) {
			//split the quad with minimal area
			float min_area = 3.402823E+38;
			int min_quad_idx = -1;
			for (int i_tri = totalRequestQuads - 1; i_tri >= 0; --i_tri)
			{
				MeshQuad* mq = &Quads[i_tri];
				if (mq->QuadKids > 0)
				{
					if (mq->quad_area < min_area) {
						min_area = mq->quad_area;
						min_quad_idx = i_tri;
					}
				}
			}
			if (min_quad_idx >= 0) {
				SplitQuadToTriangle(Quads[min_quad_idx], qti);
			}
		}
		else {
			__m128 extend = _mm_sub_ps(qti.refMax, qti.refMin);
			float* d = (float*)&extend;
			if (d[0] * d[1] * d[2] == 0 && qti.nQuads == 8
				&& qti.nTriangles == 0 
				&& nVert == 8 && nIdx == 36){ //AABB only two quad face left, split to 4 triangles
				for (int i_tri = 0; i_tri < totalRequestQuads; ++i_tri)
				{
					SplitQuadToTriangle(Quads[i_tri], qti);
				}
			}
		}

		if (qti.nTriangles == 0 && qti.nQuads <= 48 && (nVert%8==0))
		{
			float flatArea = 0;
			float neighborArea = 0;
			__m128 vertical = _mm_setr_ps(0, 0, 1, 0);
			for (int i_tri = 0; i_tri < totalRequestQuads; ++i_tri)
			{
				MeshQuad* mq = &Quads[i_tri];
				if (mq->QuadKids > 0)
				{
					float dotSum = dotSum3(mq->face1->UnitNormal, vertical);
					if (abs(dotSum) > 0.95f) {
						flatArea += mq->quad_area;
					}
					else {
						neighborArea += mq->quad_area;
					}
				}
			}
			if (neighborArea < flatArea * 0.001f) {
				qti.Superflat = 1;
			}
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
			if (face1->ValidSingleTriangle)
			{
				memcpy(pIndices, face1->nVertIdx, 3 * sizeof(uint16_t));
				pIndices += 3;// mq->PolygonPoint;
			}
		}


		uint32_t totalUseIdx = qti.nQuads + qti.nTriangles;
		//Remove the not used vertices for strip triangle off file
		for (uint32_t i = 0; i < totalUseIdx; i++)
		{
			Vertices[qti.indices[i]].NewVertexID = 0;
		}

		uint32_t nextVertID = 0;
		for (uint32_t i = 0; i < nVert; i++) {
			MeshVertex* v = &Vertices[i];
			if (v->NewVertexID == 0)
			{
				v->NewVertexID = nextVertID;
				qti.verts[nextVertID] = qti.verts[i];
				nextVertID++;
			}
		}
		for (uint32_t i = 0; i < totalUseIdx; i++)
		{
			qti.indices[i] = Vertices[qti.indices[i]].NewVertexID;
		}
		qti.nActiveVerts = nextVertID;



		
		{
			if(SaveSimplifyModel)
			{				
				std::string file_path;
				if (qti.nQuads > 0) {
					CreateOutputDir();
					if (isTerrain) {
						file_path = outputSaveDirectory + "\\" + GetDebugOccluderKey(qti) + "TerrainQuad.off";
					}
					else {
						file_path = outputSaveDirectory + "\\" + GetDebugOccluderKey(qti) + "QuadOut.off";
					}

					int saveQuad = 1; //
					uint32_t nFace = (qti.nQuads >> 2) * saveQuad + qti.nTriangles / 3;
					if (nFace > 0) {

						FILE* fileWriter = fopen(file_path.c_str(), "w");
						if (fileWriter != nullptr)
						{
							fprintf(fileWriter, "OFF\n");
							fprintf(fileWriter, "%d %d 0\n", qti.nActiveVerts, nFace);
							float* vertf = (float*)qti.verts;
							for (uint32_t i_vert = 0; i_vert < qti.nActiveVerts; ++i_vert)
							{
								savePoint(fileWriter, vertf + i_vert * 4);
							}
							if (saveQuad) {
								for (uint32_t i = 0; i < qti.nQuads; i += 4)
								{
									fprintf(fileWriter, "4 %d %d %d %d\n",
										qti.indices[i],
										qti.indices[i + 1],
										qti.indices[i + 2],
										qti.indices[i + 3]);
								}
							}
							for (uint32_t i = qti.nQuads; i < totalUseIdx; i += 3)
							{
								saveTriangle(fileWriter, qti.indices + i);
							}

							fclose(fileWriter);
						}
					}
				}
				
				

				//according to the triangle off file to store the simplified mesh if wish to 
				//push triangle mesh to SOC
				//save as triangle off file
				CreateOutputDir();
				file_path = outputSaveDirectory + "\\" + GetDebugOccluderKey(qti) + "out.off";
				FILE * fileWriter = fopen(file_path.c_str(), "w");
                if (fileWriter != nullptr)
				{
					uint32_t nFace = (qti.nQuads>>1) + qti.nTriangles/3;

					int latestVertNum = nextVertID;
					fprintf(fileWriter, "OFF\n");
					fprintf(fileWriter, "%d %d 0\n", latestVertNum, nFace);


					float* vertf = (float*)qti.verts;
					for (uint32_t i_vert = 0; i_vert < qti.nActiveVerts; ++i_vert)
					{
						savePoint(fileWriter, vertf + i_vert * 4);
					}
					for (uint32_t i = 0; i < qti.nQuads; i += 4)
					{
						fprintf(fileWriter, "3 %d %d %d\n",
							qti.indices[i],
							qti.indices[i + 1],
							qti.indices[i + 2]);
						fprintf(fileWriter, "3 %d %d %d\n",
							qti.indices[i],
							qti.indices[i + 2],
							qti.indices[i + 3]);
					}
					for (uint32_t i = qti.nQuads; i < totalUseIdx; i += 3)
					{
						saveTriangle(fileWriter, qti.indices + i);
					}

					fclose(fileWriter);
				}
			}
		}

	}


	bool OccluderQuad::MergeQuad(MeshQuad * q0, MeshQuad * q1, __m128* points, util::MeshVertex* Vertices, bool isTerrain, int &totalMerged, float lineLinkTH, bool allowDiffKidMerge)
	{
		if (q1->QuadKids == 0 || q0->QuadKids == 0)
		{
			return false;
		}

		float largestArea = std::max<float>(q0->face1->area,
			q1->face1->area);
		float areaTh = largestArea * 0.7f;  //allow area ratio around 0.7
		if ( (q1->QuadKids == q0->QuadKids|| allowDiffKidMerge)
			&& ((q0->IsRectangle && q1->IsRectangle)) &&
			q0->face1->area > areaTh &&
			q1->face1->area > areaTh)
		{
			if (q0->MergeWith(q1, QuadTwoSameFaceTh, points, Vertices, isTerrain, totalMerged, lineLinkTH))
			{
				return true;
			}
		}
		return false;
	}



	static bool ProjectToLine(__m128 a, __m128 b, __m128 mid, __m128 &projMid, float lineTh)
	{
		__m128 unitAB = normalize(_mm_sub_ps(a, b));
		__m128 midB = _mm_sub_ps(mid, b);
		__m128 unitMB = normalize(midB);
		float dotSum = dotSum3(unitAB, unitMB);
		if (dotSum > lineTh) 
		{
			float projLen = dotSum3(midB, unitAB);

			projMid = _mm_add_ps(b, _mm_mul_ps(unitAB, _mm_set1_ps(projLen)));
			return true;
		}
		return false;
	}
	static bool MatchToLine(__m128 a, __m128 b, __m128 mid, float lineTh)
	{
		__m128 unitAB = normalize(_mm_sub_ps(a, b));
		__m128 midB = _mm_sub_ps(mid, b);
		__m128 unitMB = normalize(midB);
		float dotSum = dotSum3(unitAB, unitMB);
		if (dotSum > lineTh)
		{
			return true;
		}
		return false;
	}
	bool MeshQuad::MergeWith(MeshQuad * q, float sameFaceTh, __m128 * vertices, util::MeshVertex* MeshVertices, bool isTerrain, int &TotalMerged, float lineTh)
	{
		float t1 = dotSum3(this->face1->UnitNormal, q->face1->UnitNormal);
		float th = QuadTwoSameFaceTh;
		if (t1 <= sameFaceTh)
		{
			return false;
		}

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
					if (ProjectToLine(vertices[idx0A], vertices[idx0B], vertices[idx0], updatedPoints[0], lineTh) &&
						ProjectToLine(vertices[idx2A], vertices[idx2B], vertices[idx2], updatedPoints[1], lineTh))
					{
						//take the projected new points
						vertices[idx0] = updatedPoints[0];
						vertices[idx2] = updatedPoints[1];

						this->vIdx[SameVertex[0]] = idx0B;
						this->vIdx[SameVertex[2]] = idx2B;

						MeshVertices[idx0].RectangleDegree -= 2;
						MeshVertices[idx2].RectangleDegree -= 2;

						this->QuadKids += q->QuadKids;
						this->quad_area += q->quad_area;
						q->QuadKids = 0;
						TotalMerged++;
						return true;
					}
				}
				else {
					if (MatchToLine(vertices[idx0A], vertices[idx0B], vertices[idx0], lineTh) &&
						MatchToLine(vertices[idx2A], vertices[idx2B], vertices[idx2], lineTh))
					{
						this->vIdx[SameVertex[0]] = idx0B;
						this->vIdx[SameVertex[2]] = idx2B;

						MeshVertices[idx0].RemoveQuad(this->qIdx, q->qIdx);
						MeshVertices[idx2].RemoveQuad(this->qIdx, q->qIdx);


						MeshVertices[idx0B].ReplaceQuad(q->qIdx, this->qIdx);
						MeshVertices[idx2B].ReplaceQuad(q->qIdx, this->qIdx);


						this->QuadKids += q->QuadKids;
						this->quad_area += q->quad_area;
						q->QuadKids = 0;
						TotalMerged++;
						return true;
					}
				}

				
			}
		}
		return false;
	}

	bool RectEdge::MergeInto(RectEdge* refEdge)
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

	void OccluderQuad::SetRectangleDegreeToZero(util::MeshVertex* Vertices, uint32_t nVert)
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
			LoadOff("D:\\14input.off", 15, 0);


			return;
		}
#endif
	}

	bool isInputModelValid(const unsigned short* indices, unsigned int nVert, unsigned int nIdx) {
		if (nVert <= 0 || nIdx < 3 || nIdx % 3 != 0)
		{
			std::cout << "Invalid Input model" << std::endl;
			return false;//invalid input;
		}
		//use sse to verify whether the model indices are all smaller than vert number
		if (nVert < 65536)
		{
			__m128i nVert128 = _mm_set1_epi16(nVert);
			bool all_index_valid = true;
			uint32_t nIdx8 = (nIdx >> 3);
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


	void OccluderQuad::Get_BakeData_QuadTriangleNum(uint16_t* value)
	{
		value[0] = value[4] * 4;
		value[1] = value[5] * 4;
	}


	void OccluderQuad::EnableMaxDepthToQueryOccludeeMesh(uint16_t* bakeData)
	{
		bakeData[0] |= 1 << 7;  //superflat bit
	}
	//due to usage of singleton and global shared result buffer, this function is not thread-safe
	//developer should add mutex lock at high level to make sdocMeshBake and result retrieving exclusive
	unsigned short* OccluderQuad::sdocMeshBake(int* outputCompressSize, const float *vertices, const unsigned short *indices, unsigned int nVert, unsigned int nIdx, float quadAngle, bool enableBackfaceCull, bool counterClockWise, int TerrainGridAxisPoint)
	{
		//TestModel();
		if (isInputModelValid(indices, nVert, nIdx) == false) {
			return nullptr;
		}

		util::OccluderBakeBuffer* pBakeBuffer = new util::OccluderBakeBuffer();
		//************************************************************************
		//decompose to quad
		//************************************************************************
		util::QuadTriIndex quadData;
		util::OccluderQuad::decomposeToQuad(pBakeBuffer, indices, nIdx, vertices, nVert, quadData, quadAngle, nullptr, false, TerrainGridAxisPoint);
		if (pBakeBuffer->mRequest != nullptr) { //one more round of baking
			util::BakeRequest* request = pBakeBuffer->mRequest;
			util::OccluderBakeBuffer* pBakeBuffer2 = new util::OccluderBakeBuffer();
			pBakeBuffer2->mParent = pBakeBuffer;
			util::OccluderQuad::decomposeToQuad(pBakeBuffer2, request->indices, request->nIdx, request->vertices, request->nVert, quadData, quadAngle, nullptr, false, TerrainGridAxisPoint);
			pBakeBuffer = pBakeBuffer2;
		}

		vertices = (float*)quadData.verts;



		int triangleFaceNum = quadData.nTriangles / 3;

		int triangleBatchNum = (triangleFaceNum + 3) >> 2;
		int quadBatchNum = (quadData.nQuads + 15) >> 4;


		if (quadBatchNum > 65535 || triangleBatchNum > 65535) {
			*outputCompressSize = 0;
			delete pBakeBuffer;
			return nullptr;
		}


		//if (compressSize * 16 > nVert * 12 + nIdx * 2 && nVert <= 256) 
		if(quadData.nActiveVerts <= 65535 /3)
		{
			int compressSize = quadBatchNum * 16 + triangleBatchNum * 12 + quadData.nActiveVerts * 3 * 2 + 8 * 4;// +1000;
		
	
	
			uint16_t* outputBuffer16 = new uint16_t[compressSize];

			int *outputBuffer = (int*)outputBuffer16;

			__m128 extents = _mm_sub_ps(quadData.refMax, quadData.refMin);
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
			uint16_t* pHead = (uint16_t*)compressData;

		
			pHead[0] = (int)enableBackfaceCull +( ((int)quadData.IsTerrain) << 5) ;
			pHead[1] = quadData.nActiveVerts;
			pHead[2] = quadBatchNum;
			pHead[3] = triangleBatchNum;

			__m128 InvExtents = _mm_div_ps(_mm_setr_ps(1.0f, 1.0f, 1.0f, 0), extents);
			//check whether any of BoundsRefinedExtents is zero
			__m128 positive = _mm_cmpgt_ps(extents, _mm_setzero_ps());
			__m128 invExtents = _mm_and_ps(InvExtents, positive);
			//temp set of rasterize required input
			compressData[2] = _mm_castps_si128(invExtents);
			compressData[3] = _mm_castps_si128(_mm_fmadd_ps_soc(_mm_negate_ps_soc(invExtents), quadData.refMin, _mm_setr_ps(0, 0, 0, 1)));
			
			__m128i* pVertices = compressData + 4; //already store bbox quad patch and tri patch & backface cull
			int piIdx = 0;
			const uint16_t* pIndexCurrent = quadData.indices;
			if (pIndexCurrent == nullptr) pIndexCurrent = indices;
			if (quadData.nActiveVerts <= common::SuperCompressVertNum)
			{
				uint8_t * pi8 = (uint8_t*)pVertices;

				int AABBMode = 0;
				if ((quadData.nQuads == 24 || quadData.nQuads == 48) && quadData.nTriangles == 0) {
					AABBMode = getAABBMode(quadData, pIndexCurrent);
					pHead[0] |= AABBMode << 8;
				}

				float dx = extentsf[0];
				float dy = extentsf[1];
				float dz = extentsf[2];
				float superFlatRatio = GetSuperFlatOccldueeRatio();
				if ((dz < std::min(dx, dy) * superFlatRatio) ||
					(dx < std::min(dy, dz) * superFlatRatio) ||
					(dy < std::min(dx, dz) * superFlatRatio) )
				{
					pHead[0] |= 1 << 7;  //superflat bit
				}
				pHead[0] |= quadData.Superflat << 7;

				if (AABBMode != 1) {
					for (uint32_t qidx = 0; qidx < quadData.nQuads; qidx++)
					{
						pi8[piIdx++] = pIndexCurrent[qidx] * 3;
					}

					int quadFill = 16 * quadBatchNum - quadData.nQuads;
					if (quadFill > 0) {
						memset(pi8 + piIdx, 0, quadFill * sizeof(uint8_t));
						piIdx += quadFill;
					}

					pIndexCurrent += quadData.nQuads;
					for (uint32_t tidx = 0; tidx < quadData.nTriangles; tidx++)
					{
						pi8[piIdx++] = pIndexCurrent[tidx] * 3;
					}
					int triFill = 12 * triangleBatchNum - quadData.nTriangles;
					if (triFill > 0) {
						memset(pi8 + piIdx, 0, triFill * sizeof(uint8_t));
						piIdx += triFill;
					}
					
					piIdx >>= 1; //as it use one byte to store index * 3, the total number of uint16_t is piIdx/2
					
					uint16_t* pV16 = (uint16_t*)pVertices + piIdx;
					float* fv = (float*)pV16;
					for (uint32_t idx = 0; idx < quadData.nActiveVerts; idx++)
					{
						memcpy(fv, &quadData.verts[idx], 3 * sizeof(float));
						fv += 3;
					}
				}
				else {
					quadData.nActiveVerts = 0; //store center and half scale
				}

			}
			else {
				uint16_t * pi16 = (uint16_t*)pVertices;

				for (uint32_t qidx = 0; qidx < quadData.nQuads; qidx++)
				{
					pi16[piIdx++] = pIndexCurrent[qidx] * 3;
				}

				int quadFill = 16 * quadBatchNum - quadData.nQuads;
				if (quadFill > 0) {
					memset(pi16 + piIdx, 0, quadFill * sizeof(uint16_t));
					piIdx += quadFill;
				}

				pIndexCurrent += quadData.nQuads;
				for (uint32_t tidx = 0; tidx < quadData.nTriangles; tidx++)
				{
					pi16[piIdx++] = pIndexCurrent[tidx] * 3;
				}


				int triFill = 12 * triangleBatchNum - quadData.nTriangles;
				if (triFill > 0) {
					memset(pi16 + piIdx, 0, triFill * sizeof(uint16_t));
					piIdx += triFill;
				}
				uint16_t* pV16 = (uint16_t*)pVertices + piIdx;
				float* fv = (float*)pV16;
				for (uint32_t idx = 0; idx < quadData.nActiveVerts; idx++)
				{
					memcpy(fv, &quadData.verts[idx], 3 * sizeof(float));
					fv += 3;
				}
			}

			*outputCompressSize = piIdx + quadData.nActiveVerts * 6 + 8 * 4;
			delete pBakeBuffer;
			return outputBuffer16;  //this is a temp buffer, the data inside should be retrieved immediately.
		}
		delete pBakeBuffer;
		*outputCompressSize = 0;
		return nullptr;
	}




	bool OccluderQuad::sdocMeshSimplify(float* vertices, unsigned short* indices, unsigned int& nVert, unsigned int& nIdx, int modelId, unsigned int targetFaceNum, bool saveModel)
	{
		if (saveModel)
		{
			std::string savedFile = std::to_string(modelId) + std::string("_Input.OFF");
			LOGI("save: %s", (SOCLogger::GetOutputDirectory() + savedFile).c_str());
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
				LOGI("save: %s", (SOCLogger::GetOutputDirectory() + savedFile).c_str());
				util::OccluderQuad::storeModel(SOCLogger::GetOutputDirectory() + savedFile, vertices, nVert, indices, nIdx);
			}
		}
		return bValid;
	}

	bool OccluderQuad::MergeTriangleIntoQuad(MeshFace* face, MeshQuad* mq, util::MeshVertex& vert, __m128* points, util::MeshVertex* vertices)
	{
		if (mq->IsPlanarQuad == false) return false;
		float t1 = dotSum3(face->UnitNormal, mq->face1->UnitNormal);
		float th = QuadTwoSameFaceTh;
		if (t1 <= 0.9999)
		{
			return false;
		}
		else {
			bool b1 = mq->containPoint(face->nVertIdx[0]);
			bool b2 = mq->containPoint(face->nVertIdx[1]);
			bool b3 = mq->containPoint(face->nVertIdx[2]);
			int count = (int)b1;
			count += (int)b2;
			count += (int)b3;
			if (count != 2) return false;
			int pivot = 0;
			if (b2 == false) pivot = 1;
			if (b3 == false) pivot = 2;

			bool quadMap[4];
			for (int idx = 0; idx < 4; idx++) {
				quadMap[idx] = face->containPoint(mq->vIdx[idx]);
			}
			int pivotIdx = face->nVertIdx[pivot];
			for (int idx = 0; idx < 4; idx++) {
				if (quadMap[idx] && quadMap[(idx + 1) & 3]) {
					uint16_t corner1 = mq->vIdx[(idx + 3) & 3];
					uint16_t corner2 = mq->vIdx[(idx + 2) & 3];
					bool line1 = sameLine(points, pivotIdx, mq->vIdx[idx], corner1);
					bool line2 = sameLine(points, pivotIdx, mq->vIdx[(idx + 1) & 3], corner2);
					if ((line1 && !line2) || (!line1 && line2) ){
						float combineArea = getArea(points[pivotIdx], points[corner1], points[corner2]);
						float AreaSum = face->area + mq->quad_area;
						if (combineArea < AreaSum) {
							int targetIdx = idx;
							if (!line1 && line2) {
								targetIdx = (idx + 1) & 3;
							}
							uint16_t backup = mq->vIdx[targetIdx];
							mq->vIdx[targetIdx] = pivotIdx;
							util::MeshVertex& mv = vertices[backup];
							mv.RemoveQuad(mq->qIdx);
							face->ValidSingleTriangle = false;
							mq->quad_area += face->area;
							return true;
						}
						else {
							//std::cout << "concave merge. reject!" << std::endl;
						}
					}
				}
			}
			
		}
		return false;
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

			bool bValid = sdocMeshSimplify(vp.data(), vi.data(), reducedVertNum, reducedIdxNum, mModelID++, std::min<int>(3000, (int)(nIdx / 3 * 0.9)), true);


			if (bValid)
			{
				std::cout << "Simplified from " << nIdx / 3 << "  " << reducedIdxNum << std::endl;
				unsigned short* data = util::OccluderQuad::sdocMeshBake(outputCompressSize, vp.data(), vi.data(), reducedVertNum, reducedIdxNum, quadAngle, enableBackfaceCull, counterClockWise, TerrainGridAxisPoint);
				return data;
			}
		}
		return util::OccluderQuad::sdocMeshBake(outputCompressSize, vertices, indices, nVert, nIdx, quadAngle, enableBackfaceCull, counterClockWise, TerrainGridAxisPoint);

	}

	bool OccluderQuad::sameLine(__m128* points, uint16_t a, uint16_t b, uint16_t c)
	{
		__m128 pa = points[a];
		__m128 pb = points[b];
		__m128 pc = points[c];

		float t1 = dotSum3(normalize(_mm_sub_ps(pa, pb)), normalize(_mm_sub_ps(pb, pc)));
		if (t1 >= 0.9998)
			return true;
		return false;
	}
	int OccluderQuad::getAABBMode(util::QuadTriIndex &quadData, const uint16_t* pIndexCurrent)
	{
		bool print = false;
		if (print) {
			uint16_t mBake2[48];
			mBake2[0] = 0;
			mBake2[1] = 6;
			mBake2[2] = 9;
			mBake2[3] = 3;
			mBake2[4] = 3;
			mBake2[5] = 9;
			mBake2[6] = 21;
			mBake2[7] = 15;
			mBake2[8] = 15;
			mBake2[9] = 21;
			mBake2[10] = 18;
			mBake2[11] = 12;
			mBake2[12] = 12;
			mBake2[13] = 18;
			mBake2[14] = 6;
			mBake2[15] = 0;
			mBake2[16] = 12;
			mBake2[17] = 0;
			mBake2[18] = 3;
			mBake2[19] = 15;
			mBake2[20] = 6;
			mBake2[21] = 18;
			mBake2[22] = 21;
			mBake2[23] = 9;
			mBake2[24] = 24;
			mBake2[25] = 30;
			mBake2[26] = 33;
			mBake2[27] = 27;
			mBake2[28] = 27;
			mBake2[29] = 33;
			mBake2[30] = 45;
			mBake2[31] = 39;
			mBake2[32] = 39;
			mBake2[33] = 45;
			mBake2[34] = 42;
			mBake2[35] = 36;
			mBake2[36] = 36;
			mBake2[37] = 42;
			mBake2[38] = 30;
			mBake2[39] = 24;
			mBake2[40] = 36;
			mBake2[41] = 24;
			mBake2[42] = 27;
			mBake2[43] = 39;
			mBake2[44] = 30;
			mBake2[45] = 42;
			mBake2[46] = 45;
			mBake2[47] = 33;
			uint64_t* bakeData3 = (uint64_t*)mBake2;
			for (int idx = 0; idx < 12; idx++) {
				std::cout << "bakeData[" << idx << "]=" << bakeData3[idx] / 3 << ";" << std::endl;
			}
		}

		uint64_t bakeData[6];
		bakeData[0] = 281487861743616;
		bakeData[1] = 1407404948520961;
		bakeData[2] = 1125925677105157;
		bakeData[3] = 8590327812;
		bakeData[4] = 1407379178520580;
		bakeData[5] = 844454995296258;
		uint64_t* inputData = (uint64_t*)pIndexCurrent;
		if (quadData.nQuads == 24 && quadData.nActiveVerts == 8) {
			for (int idx = 0; idx < 6; idx++) {
				if (bakeData[idx] != inputData[idx]) {
					return 0;
				}
			}
			__m128 verts[8];
			memcpy(verts, quadData.verts, 8 * sizeof(__m128));

			float* minf = (float*)&quadData.refMin;
			float* maxf = (float*)&quadData.refMax;
			float* vertf = (float*)quadData.verts;

			bool b1 = (vertf[0] == minf[0] && vertf[1] == minf[1] && vertf[2] == maxf[2]);
			bool b2 = (vertf[4] == maxf[0] && vertf[5] == minf[1] && vertf[6] == maxf[2]);
			bool b3 = (vertf[8] == minf[0] && vertf[9] == minf[1] && vertf[10] == minf[2]);
			bool b4 = (vertf[12] == maxf[0] && vertf[13] == minf[1] && vertf[14] == minf[2]);
			bool b5 = (vertf[16] == minf[0] && vertf[17] == maxf[1] && vertf[18] == maxf[2]);
			bool b6 = (vertf[20] == maxf[0] && vertf[21] == maxf[1] && vertf[22] == maxf[2]);
			bool b7 = (vertf[24] == minf[0] && vertf[25] == maxf[1] && vertf[26] == minf[2]);
			bool b8 = (vertf[28] == maxf[0] && vertf[29] == maxf[1] && vertf[30] == minf[2]);
			if (b1 && b2 && b3 && b4 && b5 && b6 && b7 && b8) {
				//std::cout << "----------------------------> min max -------------------------------------->" << std::endl;
				return 1;
			}
		}		
		return 0;
	}
	void OccluderQuad::ConfigDebugOccluder(int occluderID, uint16_t* CompactData)
	{
		DebugOccluderID = occluderID;
		if (CompactData != nullptr) {
			std::cout << "todo" << std::endl;
		}
	}

	void OccluderBakeBuffer::QuickMerge(MeshQuad* q0, MeshQuad* q1, float mergeQuadTh, __m128* points, MeshVertex*& Vertices, int& neighborMergeCount, float lineLinkTH, bool allowDiffKidMerge ){
		
		if ( (q0->QuadKids == q1->QuadKids && q0->QuadKids > 0) ||(allowDiffKidMerge && q0->QuadKids > 0 && q1->QuadKids > 0))		
		{
			if (q0->IsPlanarQuad && q1->IsPlanarQuad) {
				q0->MergeWith(q1, mergeQuadTh, points, Vertices, false, neighborMergeCount, lineLinkTH);
			}
		}
	}

	bool MeshFace::containPoint(uint16_t vIdx)
	{
		return this->nVertIdx[0] == vIdx || this->nVertIdx[1] == vIdx || this->nVertIdx[2] == vIdx;
	}

}

#endif
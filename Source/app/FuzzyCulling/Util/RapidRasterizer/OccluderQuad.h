//============================================================================================================
//
//
//                  Copyright (c) 2023, Qualcomm Innovation Center, Inc. All rights reserved.
//                              SPDX-License-Identifier: BSD-3-Clause
//
//============================================================================================================
/*
** Copyright (c) 2022 Qualcomm Technologies, Inc.
** All Rights Reserved.
** Qualcomm Technologies, Inc.
**
** Code from https://github.com/rawrunprotected/rasterizer/blob/master/SoftwareRasterizer/QuadDecomposition.cpp under CC0 1.0 Universal (CC0 1.0) Public Domain Dedication.
*/

#pragma once

#if defined(SDOC_NATIVE)

#include <algorithm>


#include <vector>
#include <string>
#include "Common/CompilerSpecificSIMD.h"

namespace SDOCUtil
{
	struct QuadTriIndex
	{
	public:
		bool IsTerrain = false;

		uint32_t nActiveVerts;
		uint32_t nQuads = 0;  //index number used for Quad
		uint32_t nTriangles = 0; //number of indices used for Triangles
		uint16_t* indices = nullptr; //Quad first, then triangle
		__m128 * verts = nullptr; //to store merged verts
		__m128 refMin;
		__m128 refMax;
		int splitOneQuadToTriangles = 0;
		int Superflat = 0;
	};

	struct TerrainInput {
	public:
		int terrainAxisSize = 0;
		uint16_t * gGridX = nullptr;
		uint16_t * gGridY = nullptr;
		uint16_t * gGridToVertID = nullptr;
	};

	//this edge is owned by vertex
	struct MeshEdgeFace
	{
		// vertices
		uint16_t edgeIdx = 0;
		uint16_t FaceIdx = 0;
		uint16_t endVert = 0;
		uint16_t apexVert = 0;

		uint16_t startVert = 0;
		uint16_t degree = 1;
		// a linked list to store other Edge owned by the same vertex
		MeshEdgeFace* Next = nullptr;
	};

	struct MeshVertex
	{

		void addVertexEdge(MeshEdgeFace* e)
		{
			if (EdgeList == nullptr)
			{
				EdgeListTail = EdgeList = e;
			}
			else
			{
				EdgeListTail->Next = e;
				EdgeListTail = e;
			}
		}

		// vertex ID
		uint16_t VertID = 0;

		int RectangleDegree = 0;  //also used for mergedVertex
		int NewVertexID = -1;  //also used for mergedVertex

		// edge linked list with start point this vertex
		MeshEdgeFace* EdgeList = nullptr;

		// edge linked list with start point this vertex.. might be dirty for a while
		MeshEdgeFace* EdgeListTail = nullptr;


		uint64_t QuadIndices = 0;
		void RemoveQuad(uint16_t qa) {
			for (int idx = 0; idx < 4; idx++) {
				int offset = idx * 16;
				uint64_t data = QuadIndices >> offset;
				if ((data & 65535) == qa) {
					uint64_t mask = 65535;
					mask <<= offset;
					uint64_t full = -1;
					full -= mask;
					QuadIndices &= full;
					return;
				}
			}
		}


		uint16_t  quad[4];
		void RemoveQuad(uint16_t qa, uint16_t qb) {
			if (RectangleDegree == 2) {
				RectangleDegree = 0;
			}
			else {
				for (int idx = 0; idx < RectangleDegree; idx++) {
					if (quad[idx] == qa)
					{
						quad[idx] = quad[RectangleDegree - 1];
						RectangleDegree--;
						break;
					}
				}
				for (int idx = 0; idx < RectangleDegree; idx++) {
					if (quad[idx] == qb)
					{
						quad[idx] = quad[RectangleDegree - 1];
						RectangleDegree--;
						break;
					}
				}
			}
		}
		int AddRectangle(uint16_t qidx)
		{
			if (RectangleDegree <= 3)
			{
				quad[RectangleDegree] = qidx;
			}
			RectangleDegree++;
			return RectangleDegree;
		}

		void ReplaceQuad(uint16_t from, uint16_t to)
		{
			for (int idx = 0; idx < RectangleDegree; idx++)
			{
				if (quad[idx] == from)
				{
					quad[idx] = to;
					break;
				}
			}
		}
	};


	struct MeshFace
	{
		void init(uint32_t triangeId, MeshVertex* v0, MeshVertex* v1, MeshVertex* v2)
		{
			FaceIdx = triangeId;
			Edges[0].edgeIdx = 0;
			Edges[1].edgeIdx = 1;
			Edges[2].edgeIdx = 2;

			nVertIdx[0] = v0->VertID;
			nVertIdx[1] = v1->VertID;
			nVertIdx[2] = v2->VertID;
			assignStartEndApex(&Edges[0], v0, v1, v2);
			assignStartEndApex(&Edges[1], v1, v2, v0);
			assignStartEndApex(&Edges[2], v2, v0, v1);

			area = 1.0;
			PairFace = nullptr;
			if (v0->VertID > v1->VertID && v0->VertID > v2->VertID) {
				uniqueFaceID = (uint64_t)v0->VertID << 32;
				uniqueFaceID |= (uint64_t)v1->VertID << 16;
				uniqueFaceID |= (uint64_t)v2->VertID;
			}
			else if (v1->VertID > v0->VertID && v1->VertID > v2->VertID) {
				uniqueFaceID = (uint64_t)v1->VertID << 32;
				uniqueFaceID |= (uint64_t)v2->VertID << 16;
				uniqueFaceID |= (uint64_t)v0->VertID;
			}
			else {
				uniqueFaceID = (uint64_t)v2->VertID << 32;
				uniqueFaceID |= (uint64_t)v0->VertID << 16;
				uniqueFaceID |= (uint64_t)v1->VertID;
			}
		}

		void assignStartEndApex(MeshEdgeFace* e, MeshVertex* v0, MeshVertex* v1, MeshVertex* v2)
		{
			e->FaceIdx = this->FaceIdx;
			e->startVert = v0->VertID;
			e->endVert = v1->VertID;
			e->apexVert = v2->VertID;


			e->Next = nullptr;
			v0->addVertexEdge(e);
		}

		bool ValidSingleTriangle = true;
		bool IsPlanarQuad = false;
		uint8_t LongestEdgeIdx = 0;
		uint16_t FaceIdx = 0;
		float area = 0;
		uint16_t MergedEdgeIdx = 0;
		uint16_t nVertIdx[3];
		uint64_t uniqueFaceID;
		__m128 UnitNormal;
		MeshEdgeFace Edges[3];

		MeshFace* PairFace = nullptr;
		bool containPoint(uint16_t vIdx);
	};

	struct RectEdge {
	public:
		uint16_t startGridX;
		uint16_t startGridY;
		uint16_t endGridX;
		uint16_t endGridY;

		uint64_t Key;

		static bool DescendingSort(RectEdge* e1, RectEdge* e2)
		{
			return e1->Key > e2->Key;
		}
		static bool AscendingSort(RectEdge* e1, RectEdge* e2)
		{
			return e1->Key < e2->Key;
		}
		bool MergeInto(RectEdge* refEdge);
	};
	struct MeshQuad
	{
		MeshFace* face1 = nullptr;
		MeshFace* face2 = nullptr;

		bool IsRectangle = false;

		uint16_t qIdx = 0;
		int QuadKids = 0;//only allow Quad merge in case that Quad Kids = 1
		uint16_t vIdx[4];

		bool MergeWith(MeshQuad* q, float sameFaceTh, __m128* vertices, SDOCUtil::MeshVertex* MeshVertices, bool IsTerrain, int& TotalMerged, float ProjectLineMidAngleTh);
		bool IsPlanarQuad = false;

		bool containPoint(uint16_t nVertIdx)
		{
			return nVertIdx == vIdx[0] || nVertIdx == vIdx[1] || nVertIdx == vIdx[2] || nVertIdx == vIdx[3];
		}

		float quad_area = 0;
	};

	struct BakeRequest
	{
	public:
		float* vertices = nullptr;
		int nVert;
		uint16_t* indices = nullptr;
		int nIdx;

		~BakeRequest() {
			if (vertices) delete[] vertices;
			if(indices)	delete[] indices;
		}
	};
	class OccluderBakeBuffer {
	private:
		 uint16_t* mIndices = nullptr;
		 __m128* mPoints = nullptr;
		 uint16_t* mReIdx = nullptr;
		 uint16_t* mRemap = nullptr;
	public:
		OccluderBakeBuffer* mParent = nullptr;
		BakeRequest* mRequest = nullptr;
		SDOCUtil::MeshFace* mFaces = nullptr;
		SDOCUtil::MeshQuad* mQuads = nullptr;
		SDOCUtil::MeshVertex* mVertices = nullptr;

		bool mIsPlanarMesh = false;
		void QuickMerge(MeshQuad* q0, MeshQuad* q1, float QuadTwoSameFaceTh, __m128* points, SDOCUtil::MeshVertex*& Vertices, int& neighborMergeCount, float lineLinkTH, bool allowDiffKidMerge);

		uint16_t* GetIndicesBySize(uint32_t size);


		__m128* GetPointsBySize(uint32_t size);


		void RequestFaceBySize(uint32_t size);

		void RequestQuadBySize(uint32_t size);

		void RequestVertexBySize(uint32_t size);
		~OccluderBakeBuffer();

		const uint16_t* ReIndex(const uint16_t* indices, const float* inputVertices, int vertNum, unsigned int nIdx);


		void CreateNewBakeRequest(float* pts, int vertNum, uint16_t* indices, int nIdx)
		{
			mRequest = new BakeRequest();
			mRequest->indices = indices;
			mRequest->vertices = pts;
			mRequest->nVert = vertNum;
			mRequest->nIdx = nIdx;
		}

	};

	//Quad-triangle Post-process feature requested by developers
	//after triangle merging to quads, developers could try to implement 'their own algorithm' to sort related quad/triangle to form 
	//group base on normal. It is preferred to pack similar normal primitives(quad/triangles) together
	typedef void (*PFPostQuadMergeCallback)(QuadTriIndex* MergedQuadTriInfo);

	class OccluderQuad {
	public:
		static float GetSuperFlatOccldueeRatio() {
			return 0.1f;
		}

		//SDOC_Get_BakeData_QuadTriangleNum: provide any array of uint16, first/second would be used to store Quad/Triangle number, 
		//the coming 4 uint16 would store first 4 elements of baked data
		static void Get_BakeData_QuadTriangleNum(uint16_t* value);
		//Extract Quad/Triangle batch number from bakedata, QuadSafeBatchNum, TriangleBatchIdxNum to store results
		static void Get_BakeData_QuadTriangleNum(uint16_t* BakedOccluderData, uint16_t* QuadSafeBatchNum, uint16_t* TriangleBatchIdxNum);
	public:

		static void checkRectangleQuad(MeshQuad* q, __m128* vertices);
		static bool fitTerrainSquareProperty(uint32_t nIdx, uint32_t nVert, int terrainGridAxisPoint);
		//in case of Close Wise input, triangle would be auto re-order to CCW
		static void decomposeToQuad(OccluderBakeBuffer* pBakeBuffer, const uint16_t* indices, unsigned int nIdx, const float* inputVertices, int vertNum, QuadTriIndex& qti, float quadAngle, TerrainInput* terrain = nullptr, bool checkedTerrain = false, int terrainGridAxisPoint = 0);


		static void AllowPlanarQuadMerge(bool allow);
		static void SetTerrainGridOptimization(int value);


		static void HandleSquareTerrainInput(SDOCUtil::OccluderBakeBuffer* pBakeBuffer, const uint16_t* indices, unsigned int nIdx, const float* inputVertices, int vertNum, QuadTriIndex& qti, float quadAngle, int terrainGridWidth);
		static void SetTerrainRectangleAngle(unsigned int configValue);
		static void SetTerrainRectangleMergeAngle(unsigned int configValue);
		static void SetSaveModel(bool saveModel);
		static void SetOutputPath(std::string output);

		static unsigned short* sdocMeshLodBake(int* outputCompressSize, const float* vertices, const unsigned short* indices, unsigned int nVert, unsigned int nIdx, float quadAngle, bool enableBackfaceCull, bool counterClockWise, int TerrainGridAxisPoint, PFPostQuadMergeCallback PostQuadMergeCallback = nullptr);
		static bool storeModel(const std::string& file_path,
			const float* vertices, int nVert,
			const uint16_t* indices, int indicesNum);
	private:
		static unsigned short* sdocMeshBake(int* outputCompressSize, const float* vertices, const unsigned short* indices, unsigned int nVert, unsigned int nIdx, float quadAngle, bool enableBackfaceCull, bool counterClockWise, int TerrainGridAxisPoint, PFPostQuadMergeCallback PostQuadMergeCallback);

		static bool MergeQuad(MeshQuad* q0, MeshQuad* q1, __m128* points, SDOCUtil::MeshVertex* Vertices, bool isTerrain, int& totalMerged, float lineLinkTH, bool allowDiffKidMerge);
		static void ConfigControlParameters(bool isTerrain);
		static void SetRectangleDegreeToZero(SDOCUtil::MeshVertex* Vertices, uint32_t nVert);
	public:
		static void TestModel();
#if defined(SDOC_NATIVE)
		static bool sdocMeshSimplify(float* vertices, unsigned short* indices, unsigned int& nVert, unsigned int& nIdx, int modelId, unsigned int targetFaceNum, bool saveModel);
#endif
	private:
		static bool MergeTriangleIntoQuad(MeshFace* face, MeshQuad* mq, SDOCUtil::MeshVertex& vert, __m128* points, SDOCUtil::MeshVertex* vertices);
		static bool sameLine(__m128* points, uint16_t a, uint16_t b, uint16_t c);
		static int getAABBMode(SDOCUtil::QuadTriIndex &quadData, const uint16_t* pIndexCurrent);
	public:
		static void ConfigDebugOccluder(int occluderID, uint16_t* CompactData);
		static void EnableMaxDepthToQueryOccludeeMesh(uint16_t* bakeData);
	};

} // namespace util

#endif
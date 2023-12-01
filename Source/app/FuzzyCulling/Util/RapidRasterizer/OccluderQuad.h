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

namespace util
{

	class OccluderBakeBuffer;
	struct QuadTriIndex
	{
	public:
		bool IsTerrain = false;
		bool IsPlanar = false;
		uint32_t nActiveVerts;
		uint32_t nQuads = 0;  //index number used for Quad
		uint32_t nTriangles = 0; //number of indices used for Triangles
		uint16_t* indices = nullptr; //Quad first, then triangle
		__m128 * verts = nullptr; //to store merged verts
		__m128 refMin;
		__m128 refMax;
	};

	struct TerrainInput {
	public:
		int terrainAxisSize = 0;
		uint16_t * gGridX = nullptr;
		uint16_t * gGridY = nullptr;
		uint16_t * gGridToVertID = nullptr;
	};
	class OccluderQuad {
	public:
		// data section
		struct MeshVertex;
		struct MeshEdgeFace;
		struct MeshQuad;
		struct MeshFace;

		//this edge is owned by vertex
		struct MeshEdgeFace
		{
			// vertices
			uint16_t edgeIdx = 0;
			uint16_t FaceId = 0;
			uint16_t endVert = 0;
			uint16_t apexVert = 0;

			// the face it belongs to
			//MeshFace * Face = nullptr;


			// a linked list to store other Edge owned by the same vertex
			MeshEdgeFace *Next = nullptr;
		};

		struct MeshVertex
		{
			MeshVertex() {}

			void addVertexEdge(MeshEdgeFace * e)
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

			// edge linked list with start point this vertex
			MeshEdgeFace *EdgeList = nullptr;

			// edge linked list with start point this vertex.. might be dirty for a while
			MeshEdgeFace *EdgeListTail = nullptr;

			void reset() {
				EdgeListTail = nullptr;
				EdgeList = nullptr;
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
			void init(MeshVertex* v0, MeshVertex* v1, MeshVertex* v2)
			{
				nVertIdx[0] = v0->VertID;
				nVertIdx[1] = v1->VertID;
				nVertIdx[2] = v2->VertID;
				assignStartEndApex(&Edges[0], v0, v1, v2);
				assignStartEndApex(&Edges[1], v1, v2, v0);
				assignStartEndApex(&Edges[2], v2, v0, v1);

				PairFace = nullptr;
			}

			void assignStartEndApex(MeshEdgeFace* e, MeshVertex* v0, MeshVertex* v1, MeshVertex* v2)
			{
				e->FaceId = this->FaceID;
				e->endVert = v1->VertID;
				e->apexVert = v2->VertID;


				e->Next = nullptr;
				v0->addVertexEdge(e);
			}

			void init(int id) 
			{
				FaceID = id;
				Edges[0].edgeIdx = 0;
				Edges[1].edgeIdx = 1;
				Edges[2].edgeIdx = 2;
			}

			bool IsPlanarQuad = false;
			uint8_t LongestEdgeIdx = 0;
			uint16_t FaceID = 0;
			float area = 0;
			uint16_t MergedEdgeIdx = 0;
			uint16_t nVertIdx[3];
			__m128 UnitNormal;
			MeshEdgeFace Edges[3];

			MeshFace * PairFace = nullptr;
		};

		struct RectEdge {
		public:
			uint16_t startGridX;
			uint16_t startGridY;
			uint16_t endGridX;
			uint16_t endGridY;

			uint64_t Key;

			static bool DescendingSort(RectEdge *e1, RectEdge *e2)
			{
				return e1->Key > e2->Key;
			}
			static bool AscendingSort(RectEdge *e1, RectEdge *e2)
			{
				return e1->Key < e2->Key;
			}
			bool MergeInto(RectEdge* refEdge);
		};
		 struct MeshQuad
		{
			MeshQuad() {}

			MeshFace * face1 = nullptr;
			MeshFace * face2 = nullptr;

			bool IsRectangle = false;
			bool IsPlanar = false;

			uint16_t qIdx = 0;
			int QuadKids = 0;//only allow Quad merge in case that Quad Kids = 1
			uint16_t vIdx[4];
			
			bool MergeWith(MeshQuad * q, __m128 * vertices, MeshVertex * MeshVertices,  bool IsTerrain, int &TotalMerged);
		};

	public:

		static void OfflineTestClearBakeBuffer();
		static void checkRectangleQuad(MeshQuad * q, __m128 * vertices);
		static bool fitTerrainSquareProperty(uint32_t nIdx, uint32_t nVert, int terrainGridAxisPoint);
		//in case of Close Wise input, triangle would be auto re-order to CCW
		static void decomposeToQuad(OccluderBakeBuffer* pBakeBuffer, const uint16_t* indices, unsigned int nIdx, const float*inputVertices, int vertNum, QuadTriIndex & qti, float quadAngle, TerrainInput * terrain = nullptr, bool checkedTerrain = false, int terrainGridAxisPoint = 0);


		static void AllowPlanarQuadMerge(bool allow);
		static void SetTerrainGridOptimization(int value);


		static void HandleSquareTerrainInput(util::OccluderBakeBuffer * pBakeBuffer, const uint16_t* indices, unsigned int nIdx, const float* inputVertices, int vertNum, QuadTriIndex & qti, float quadAngle, int terrainGridWidth);
		static void SetTerrainRectangleAngle(unsigned int configValue);
		static void SetTerrainRectangleMergeAngle(unsigned int configValue);
		static void SetSaveModel(bool param1);
		static void SetOutputPath(std::string output);
		static void SetMaxVertIndexNum(int* value);

		static unsigned short* sdocMeshLodBake(int* outputCompressSize, const float* vertices, const unsigned short* indices, unsigned int nVert, unsigned int nIdx, float quadAngle, bool enableBackfaceCull, bool counterClockWise, int TerrainGridAxisPoint);
		static bool storeModel(const std::string& file_path,
			const float* vertices, int nVert,
			const uint16_t* indices, int indicesNum);
private:
		static unsigned short* sdocMeshBake(int* outputCompressSize, const float* vertices, const unsigned short* indices, unsigned int nVert, unsigned int nIdx, float quadAngle, bool enableBackfaceCull, bool counterClockWise, int TerrainGridAxisPoint);

		static bool MergeQuad(MeshQuad * q0, MeshQuad * q1, __m128* points, MeshVertex *Vertices, bool isTerrain, int &totalMerged);
		static void ConfigControlParameters(bool isTerrain);
		static void SetRectangleDegreeToZero(util::OccluderQuad::MeshVertex * Vertices, uint32_t nVert);
public:
	static void TestModel();
#if defined(SDOC_NATIVE)
	static bool sdocMeshSimplify(float* vertices, unsigned short* indices, unsigned int& nVert, unsigned int& nIdx, int modelId, unsigned int targetFaceNum, bool saveModel);
#endif
	};


	class OccluderBakeBuffer {
	private:
		std::vector< uint16_t> mIndices;
		std::vector< __m128> mPoints;
		std::vector<util::OccluderQuad::MeshQuad> mQuads;
		std::vector<util::OccluderQuad::MeshVertex> mVertices;
	public:
		std::vector<util::OccluderQuad::MeshFace> mFaces;
		uint16_t* GetIndicesBySize(uint32_t size);


		__m128* GetPointsBySize(uint32_t size);


		util::OccluderQuad::MeshFace* GetFaceBySize(uint32_t size);

		util::OccluderQuad::MeshQuad* GetQuadBySize(uint32_t size);

		util::OccluderQuad::MeshVertex* GetVertexBySize(uint32_t size);
		~OccluderBakeBuffer();
		OccluderBakeBuffer() {
			//reserve to make access first element not crash
			mIndices.resize(1);
			mPoints.resize(1);
			mFaces.resize(1);
			mQuads.resize(1);
			mVertices.resize(1);
		}

		std::vector< uint16_t> reIdx;
		std::vector< uint16_t> remap;
		const uint16_t* ReIndex(const uint16_t* indices, const float* inputVertices, int vertNum, unsigned int nIdx);


	};
} // namespace util

#endif
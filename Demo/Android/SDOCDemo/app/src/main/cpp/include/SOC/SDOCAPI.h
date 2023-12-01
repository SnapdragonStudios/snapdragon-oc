//============================================================================================================
//
//
//                  Copyright (c) 2023, Qualcomm Innovation Center, Inc. All rights reserved.
//                              SPDX-License-Identifier: BSD-3-Clause
//
//============================================================================================================

#pragma once

#if defined(SDOC_LIB_EXPORT)
	#if defined(__ANDROID__) || defined(__APPLE__)
	#define VISIBLE_SYMBOL __attribute__((__visibility__("default")))
	#elif defined(_WIN64) || defined(_WIN32)
	#define VISIBLE_SYMBOL __declspec(dllexport)
	#endif
#else
	#if defined(_WIN32) || defined(_WIN64) || defined(__CYGWIN__) /** Windows */
	#if defined(__GNUC__)
	#define VISIBLE_SYMBOL __attribute__((dllimport))
	#else
	#define VISIBLE_SYMBOL __declspec(dllimport)
	#endif // __GNUC__
	#elif defined(__GNUC__)
	#if __GNUC__ >= 4 /** GCC 4.x has support for visibility options */
	#define VISIBLE_SYMBOL __attribute__((visibility("default")))
	#else
	#define VISIBLE_SYMBOL
	#endif // __GNUC__
	#else
	#error "Unknown case"
	#endif
#endif



#define SDOC_RenderMode_Full                                                                                             0
#define SDOC_RenderMode_Coherent                                                                                         1
#define SDOC_RenderMode_CoherentFast                                                                                     2
#define SDOC_RenderMode_ToggleRenderType                                                                                 12340

#define SDOC_SetCCW                                                                                                      20

#define SDOC_Get_IsSameCamera                                                                                            99
#define SDOC_Set_UsePrevDepthBuffer                                                                                      100
#define SDOC_RenderMode                                                                                                  101
#define SDOC_Set_UsePrevFrameOccluders                                                                                   102

#define SDOC_Get_Version                                                                                                 212

#define SDOC_Set_CoherentModeSmallRotateDotAngleThreshold                                                                213
#define SDOC_Set_CoherentModeLargeRotateDotAngleThreshold                                                                214
#define SDOC_Set_CoherentModeCameraDistanceNearThreshold                                                                 215
#define SDOC_Reset_DepthMapWidthAndHeight                                                                                220

#define SDOC_DestroySDOC                                                                                                 240

//debug settings
#define SDOC_Get_DepthBufferWidthHeight                                                                                  250
#define SDOC_Set_FrameCaptureOutputPath                                                                                  251
#define SDOC_Get_DepthMap                                                                                                252
#define SDOC_Save_DepthMap                                                                                               256
#define SDOC_Save_DepthMapPath                                                                                           257
#define SDOC_ShowCulled                                                                                                  260
#define SDOC_ShowOccludeeInDepthMap                                                                                      261
#define SDOC_Get_MemoryUsed                                                                                              270

#define SDOC_SetPrintLogInGame                                                                                           300
#define SDOC_Get_Log                                                                                                     301

#define SDOC_CaptureFrame                                                                                                400

#define SDOC_EnableOccluderPriorityQueue                                                                                 599
#define SDOC_BackFaceCullOffOccluderFirst                                                                                600
#define SDOC_FlushSubmittedOccluder                                                                                      602

#define SDOC_BakeMeshSimplifyConfig                                                                                      605 //Native Only
//r.sdoc.set 1700 to see bake vs non bake model usage
#define SDOC_DebugPrintActiveOccluder                                                                                    700
#define SDOC_GetOccluderPotentialVisibleSet                                                                              891
#define SDOC_BeforeQueryTreatTrueAsCulled                                                                                892
#define SDOC_SetQueryTreeData                                                                                            893



typedef void*(*PFN_sdocInit)(unsigned int width, unsigned int height, float nearPlane);
typedef bool(*PFN_sdocStartNewFrame)(void * pSDOC, const float *ViewPos, const float *ViewDir, const float *ViewProj);
typedef void(*PFN_sdocRenderOccluder)(void * pSDOC, const float *vertices, const unsigned short *indices, unsigned int nVert, unsigned int nIdx, const float *localToWorld, bool enableBackfaceCull);
typedef bool(*PFN_sdocSync)(void * pSDOC, unsigned int id, void *param);
typedef bool(*PFN_sdocSet)(void * pSDOC, unsigned int ID, unsigned int configValue);
typedef bool(*PFN_sdocQueryOccludees)(void * pSDOC, const float *bbox, unsigned int nMesh, bool *results);
typedef unsigned short* (*PFN_sdocMeshBake)(int* outputCompressSize, const float *vertices, const unsigned short *indices, unsigned int nVert, unsigned int nIdx,  float quadAngle, bool enableBackfaceCull, bool counterClockWise, int SquareTerrainAxisPoints);
typedef void(*PFN_sdocRenderBakedOccluder)(void * pSDOC, void *compressedModel, const float *localToWorld);



extern "C"
{
	/*******************************************************************************************************************************
	 *   sdocInit
	 *   @brief
	 *       Initialize SDOC.
	 *   @param width
	 *       Depth buffer width, must be divisible by 64, 64 is selected for max possible performance,
	 *       in the range of [64, 65535), suggest not larger than 1024
	 *   @param height
	 *       Depth buffer height, must be divisible by 8, in the range of [8, 65535), recommend 256
	 *   @param nearPlane
	 *       Near clip plane.
	 *   @return
	 *       TRUE if initialized successfully.
	*******************************************************************************************************************************/
	VISIBLE_SYMBOL void* sdocInit(unsigned int width, unsigned int height, float nearPlane);


	/*******************************************************************************************************************************
	 *   sdocStartNewFrame
	 *   @brief
	 *       Start new frame
	 *   @param ViewPos
	 *       Camera Position, an array of three floats.
	 *   @param ViewDir
	 *       Camera view direction, an array of three floats. if set to (0, 0, 0) this frame would be treated as critical frame and render in full mode
	 *   @param ViewProj
	 *       The view-projection matrix which points to a 4x4 matrix.
	 *       ViewProj equals to View * Projection.
	 *   @return
	 *       TRUE if SDOC starts a new frame.
	*******************************************************************************************************************************/
	VISIBLE_SYMBOL bool sdocStartNewFrame(void * pSDOC, const float *ViewPos, const float *ViewDir, const float *ViewProj);

	/*******************************************************************************************************************************
	 *   sdocRenderOccluder
	 *   @brief
	 *       Submit occluder to SDOC. 
	 *   @param vertices
	 *       Occluder vertices array, the vertex should be arranged in order XYZ.
	 *   @param indices
	 *       Occluder indices array. Each three indices make up a triangle.
	 *   @param nVert
	 *       Number of vertices.
	 *   @param nIdx
	 *       Number of indices.
	 *   @param localToWorld
	 *       A 4x4 matrix which transforms occluder from local coordinator to world coordinator.
	 *       No need to set if the occluder has already converted to the world coordinate.
	 *   @param enableBackfaceCull
	 *       true if backface enable, otherwise false
	 *   @return
	 *       void
	 *   Assumption: vertices != nullptr && indices != nullptr && nVert != 0 && nIdx != 0
	*******************************************************************************************************************************/
	VISIBLE_SYMBOL void sdocRenderOccluder(void * pSDOC, const float *vertices, const unsigned short *indices, unsigned int nVert, unsigned int nIdx,  const float *localToWorld, bool enableBackfaceCull);



	/*******************************************************************************************************************************
	 *   sdocQueryOccludees
	 *   @brief
	 *       Query occludees' visibility by world coordinate axis-aligned bounding box(AABB) collections.
	 *   @details
	 *       Batch query visibility of bounding box collections. The visibility results will be returned by results array.
	 *   @param bbox
	 *       Bounding box collections. The data of bbox should be arranged in the following order:
	 *       minX, minY, minZ, maxX, maxY, maxZ
	 *   @param nMesh
	 *       Number of Occludees.
	 *   @param results
	 *       Used to return visibility results.
	 *   @return
	 *       TRUE if queried successfully.
	 *   Assumption: bbox != nullptr && nMesh > 0 && results != nullptr
	*******************************************************************************************************************************/
	VISIBLE_SYMBOL bool sdocQueryOccludees(void * pSDOC, const float *bbox, unsigned int nMesh, bool *results);

	/*******************************************************************************************************************************
	 *   sdocSet
	 *   @brief
	 *       Notify SDOC to change setting according to the specified ID & Value
	 *   @param id
	 *       SDOC_SetCCW:                        configValue = 0 or 1. Default 1. 0 would means the model is clockwise
	 *       SDOC_Set_UsePrevDepthBuffer:        configValue = 1,0, default set to 0,  set 1 to enable
	 *       SDOC_Set_UsePrevFrameOccluders:     indicate the first N occluders are same with previous frame
	 *       SDOC_RenderMode:                    configValue = 0,1,2,12340    0 stands for full mode, 1 stands for coherent mode, 2 stands for coherent fast mode. Default set to coherent fast mode(2). 12340 would toggle render type from pure mesh to meshline, meshpoint, line, point iteratively
	 *       SDOC_ShowCulled                     configValue = 0 or 1, default 0, 1 to reverse the query result.
	 *       SDOC_ShowOccludeeInDepthMap         configValue = 0 or 1, default 0, hide/show the occludee info in depth map.
	 *       SDOC_CaptureFrame:                  configValue=1, once set, SDOC would capture 1 frame
	 *       SDOC_DestroySDOC:                   configValue=1, once the value is set to 1, Destroy SDOC instance
	 *       SDOC_EnableOccluderPriorityQueue    configValue = 1/0, default 0, means priority queue not enabled, once enable, sdoc would render the occluders according to 3 queues, backface cull off occluders, backface cull on occluders, terrain occluders
	 *       SDOC_BackFaceCullOffOccluderFirst configValue = 0/1, default 1. Backface cull off occluders would be rendered first.
	 *       SDOC_FlushSubmittedOccluder          config value = 1,  once called, force previous submitted occluders are rendered
	 *   @return
	 *       TRUE if set successfully.
	*******************************************************************************************************************************/
	VISIBLE_SYMBOL bool sdocSet(void * pSDOC, unsigned int ID, unsigned int configValue);




	/*******************************************************************************************************************************
	 *   sdocSync
	 *   @brief
	 *       Mainly used to get/set the info with specified ID and various input/output format
	 *   @param id
	 *       SDOC_Get_MemoryUsed to get the total memory used by SDOC, in the unit of KB, expect input param format unsigned int *
	 *       SDOC_Get_DepthBufferWidthHeight means get the width and height of occlusion buffer, expect input param format unsigned int *
	 *       SDOC_Get_DepthMap get occluder depth map of SDOC. expect input param format unsigned char*
	 *       SDOC_Save_DepthMapPath pre-set the depth map path going to be saved. Once saved the store path value would be reset to default.
	 *       SDOC_Save_DepthMap save depth map to a file, expect input param format unsigned char*
     *       SDOC_Get_Version    Get the SDOC version,  expect input param format unsigned int *
     *       SDOC_Set_CoherentModeSmallRotateDotAngleThreshold  expect input param format float *, if cross-frame view direction dot value smaller than this value,
	 *                                 camera is rotating small, default value: 0.9999f
	 *       SDOC_Set_CoherentModeLargeRotateDotAngleThreshold  expect input param format float *, if cross-frame view direction dot value smaller than this value,
	 *                                 camera is rotating large, default value: 0.9995f
	 *       SDOC_Reset_DepthMapWidthAndHeight  expect input param format unsigned int *, contain two value indicating width & height.
	 *       SDOC_SetPrintLogInGame, expect input param format unsigned int *, contain one value, if 1, store the msg else print in SDOC
	 *       SDOC_Get_Log  expect input param format char* with size of 256
	 *       SDOC_Set_FrameCaptureOutputPath expect input param format char* data
	 *       SDOC_Get_IsSameCamera expect input param format bool *, store true if the camera pose is same with previous frame
	 *       SDOC_BakeMeshSimplifyConfig  expect input an int array with 4 values to store
	 *		          SimplifyMeshDuringBake value= 0/others, 0 would disable mesh simplify and others would enable
	 *                TerrainGridControlMergeDuringBake config value = 0/1/2, default 2. 0/1/2 are corresponding to no grid guided merging/grid guided merging/grid guided aggressive merging.
	 *                TerrainRectangleAngle        configValue = [80, 89], default 88, for a quad if neighboring edge form angle not smaller than the configValue, the quad would be treated as rectangle
	 *                TerrainMergeAngle            configValue = [1, 10], default 3, if two rectangle have normal angle difference smaller than the configValue, the two rectangle could be merged
	 *       SDOC_GetOccluderPotentialVisibleSet expect an array of bool with occluder number + 1 to store whether the occluder is potentially visible and interleave or not.
	 *       SDOC_SetQueryTreeData: provide an array of uint16_t to SDOC to inform the tree information. If the occludee has sub occludees, the value would be sub number + 1; if the occludee is a sub occludee, the value should be 0; otherwise the value should be one
	 *   @return
	 *       TRUE if sync successfully.
	*******************************************************************************************************************************/
	VISIBLE_SYMBOL bool sdocSync(void * pSDOC, unsigned int id, void *param);




	/*******************************************************************************************************************************
	 *   sdocMeshBake
	 *   @brief
	 *       Submit model mesh to SDOC for compression. Compress the input mesh with uint16_t precision for each x/y/z. Native function only.
	 *   @param outputCompressSize
	 *       the number of int used to contain compressed data
	 *   @param vertices
	 *       Occluder vertices array, the vertex should be arranged in order XYZ.
	 *   @param indices
	 *       Occluder indices array. Every three indices make up a triangle.
	 *   @param nVert
	 *       Number of vertices.
	 *   @param nIdx
	 *       Number of indices.
	 *   @param quadAngle
	 *       if two neighboring plane normal angle is smaller than QuadAngle, they might form a quad and eligible for quad rasterization
	 *       Extremely large two triangles might introduce error. For model with super large triangles, this quadAngle should be set to a small value such as 1.
	 *       Valid quad angle range [0, 15], nonzero value would be clamped to [1, 15], recommended value 15. Set to 0 would disable quad drawing & mesh simplification.
	 *   @param enableBackfaceCull
	 *       true if backface enable, otherwise false
	 *   @param counterClockWise
	 *       True if model in counter clock wise order(UE default), otherwise false
	 *   @param SquareTerrainAxisPoints
	 *       If the input model is a NxN grid terrain where N means the number of point on X/Y axis, set SquareTerrainAxisPoints = N, otherwise 0
	 *   @return
	 *       the compressed data with outputCompressSize amount of int, the return bake buffer should be immediately copied as it would be reused for next mesh baking
	*******************************************************************************************************************************/
	VISIBLE_SYMBOL unsigned short* sdocMeshBake(int* outputCompressSize, const float *vertices, const unsigned short *indices, unsigned int nVert, unsigned int nIdx,  float quadAngle, bool enableBackfaceCull, bool counterClockWise, int SquareTerrainAxisPoints);

	/*******************************************************************************************************************************
	 *   sdocRenderOccluder
	 *   @brief
	 *       Submit compressed occluder to SDOC.
	 *   @param localToWorld
	 *       A 4x4 matrix which transforms occluder from local coordinator to world coordinator.
	 *       No need to set if the occluder has already converted to the world coordinate.
	 *   @return
	 *       void
	*******************************************************************************************************************************/
	VISIBLE_SYMBOL void sdocRenderBakedOccluder(void * pSDOC, unsigned short *compressedModel, const float *localToWorld);

}
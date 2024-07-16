//============================================================================================================
//
//
//                  Copyright (c) 2023, Qualcomm Innovation Center, Inc. All rights reserved.
//                              SPDX-License-Identifier: BSD-3-Clause
//
//============================================================================================================
#include "SOCPrivate.h"
#include "SDOCAPI.h"
#include <vector>
#if defined(SDOC_NATIVE)
#include <fstream>
#include <iostream>
#include <sstream>
#endif
#include "Common/MathUtil.h"
#include "Common/SOCUtil.h"


#if defined(SDOC_ANDROID)
#include <sys/stat.h>
#include <sys/system_properties.h>
#include <unistd.h>
#include <fstream>
#endif
#include "Util/RapidRasterizer/OccluderQuad.h"

#if defined(SDOC_NATIVE_DEBUG)&& defined(SDOC_NATIVE)
#include <thread>
using namespace std;
#endif

#if defined(SDOC_WIN)
#pragma warning( disable : 4996  )
#endif

using namespace common;
namespace
{

#if defined(SDOC_NATIVE_DEBUG)&& defined(SDOC_NATIVE)
	static constexpr int nOcludeeOBBSize = 18;   //Debug: change to 12 if input only 4 points, otherwise 18
#endif


static bool dumpOccluderOccludeeColorImage(const std::string &filename, unsigned char *input, unsigned int width, unsigned int height)
{
#ifdef SDOC_STRESS_TEST
	return true;
#endif
	int resolution = width * height;
	uint8_t *data = new uint8_t[resolution * 2];
	memcpy(data, input, resolution);
	memset(data + resolution, 0, resolution);
	uint64_t *metaData = reinterpret_cast<uint64_t*>(input + resolution);
	uint64_t totalSize = metaData[0]; metaData++;
	if (totalSize>0)
	{
		for (uint64_t idx = 0; idx < totalSize; idx+=2)
		{
			uint64_t pos = metaData[idx];
			uint64_t stateDepth = metaData[idx + 1];

			int minX = pos >> 48;
			int maxX = (pos >> 32) & 0xFFFF;
			int minY = (pos >> 16) & 0xFFFF;
			int maxY = pos & 0xFFFF;
			int visible = (int)(stateDepth& 1);
			uint8_t depth = (uint8_t)(stateDepth >> 48);


			uint8_t *dest = (uint8_t *)data + width * minY;
			for (int k = minX; k <= maxX; k++)
			{
				dest[k] = depth;
				dest[k + resolution] = 1 + visible;
			}
			dest = data + width * maxY;
			for (int k = minX; k <= maxX; k++)
			{
				dest[k] = depth;
				dest[k + resolution] = 1 + visible;
			}

			for (int k = minY + 1; k <= maxY - 1; k++)
			{
				dest = data + width * k;

				dest[minX] = depth;
				dest[minX + resolution] = 1 + visible;
				dest[maxX] = depth;
				dest[maxX + resolution] = 1 + visible;
			}
		}
	}
	uint8_t mask[12];
	mask[0] = mask[1] = mask[2] = 0xFF;
	mask[4] = 0xFF; mask[5] = mask[6] = 0;
	mask[8] = 0; mask[9] = 0xFF; mask[10] = 0;
	bool succeed = false;
	FILE *fileWriter = fopen(filename.c_str(), "w");
	if (fileWriter != nullptr) {
		fprintf(fileWriter, "P3\n%d %d\n255\n", width, height);
		for (unsigned int ny = 0; ny < height; ny++)
		{
			for (unsigned int nx = 0; nx < width; nx++)
			{
				int idx = nx + (height - 1 - ny) * width;
				uint8_t v = data[idx];
				uint8_t maskIdx = data[idx + resolution];
				uint8_t * pMask = mask + (maskIdx << 2);
				fprintf(fileWriter, "%d %d %d\n", v & pMask[0], v & pMask[1], v & pMask[2]);
				
			}
		}
		fclose(fileWriter);
		succeed = true;
	}
	delete[] data;
	return succeed;
}

#if defined(SDOC_NATIVE_DEBUG) && defined(SDOC_NATIVE)
static void dumpGrayImage(const std::string &filename, const unsigned char *data, unsigned int width, unsigned int height)
{
#ifdef SDOC_STRESS_TEST
	return ;
#endif
	std::ofstream ofs(filename, std::ios_base::out | std::ios_base::binary);
	if (ofs.bad()) {
		LOGI("fail to save %s", filename.c_str());
		return;
	}
	ofs << "P5\n" << width << " " << height << "\n255\n";
	for (unsigned int ny = 0; ny < height; ny++)
	{
		for (unsigned int nx = 0; nx < width; nx++)
		{
			int idx = nx +  (height - 1 - ny) * width;
			ofs << data[idx];
		}
	}
	ofs.close();
	LOGI("dumpGrayImage %s", filename.c_str());
}
static void dumpImageMOC(const std::string &filename, const unsigned char *data, unsigned int width, unsigned int height)
{


	FILE *fileWriter = fopen(filename.c_str(), "w");
	if (fileWriter != nullptr) {
		fprintf(fileWriter, "P3\n%d %d\n255\n", width, height);
		for (unsigned int ny = 0; ny < height; ny++)
		{
			for (unsigned int nx = 0; nx < width; nx++)
			{
				int idx = nx + ny * width;
				uint8_t v = data[idx];
				fprintf(fileWriter, "%d %d %d\n", v, v, v);
			}
		}
		fclose(fileWriter);
	}
}
#endif
} // namespace

namespace SOC
{

//PScene: 1152x96 -> 306 us
//		  1152x192-> 342 us
//		  576x192 -> 300 us
bool SOCPrivate::resize(unsigned int width, unsigned int height)
{

	width = width - (width & 7);
	height = height - (height & 7);
	// check whether width & height are the same as before.
	if (width == m_frameInfo->Width && height == m_frameInfo->Height)
	{
		return false;
	}
	if (width < 64 || height < 8)
	{
		return false;
	}
	


    m_rapidRasterizer->setResolution(width, height);


    // keep tracking in SOCFrameInfo
    m_frameInfo->Width = width;
	m_frameInfo->Height = height;

	this->mResolutionChanged = true;
	return true;
}

void SOCPrivate::setNearPlane(float nearPlane)
{

    // keep tracking in SOCFrameInfo
    m_frameInfo->NearPlane = nearPlane;
	m_rapidRasterizer->setNearPlane(nearPlane);
}


void SOCPrivate::startNewFrame(const float *CameraPos, const float *ViewDir, const float *ViewProj)
{
	common::Vec3f ViewDirV = common::Vec3f(ViewDir[0], ViewDir[1], ViewDir[2]);
	common::Vec3f lastDir = m_frameInfo->mCameraViewDir;
	m_frameInfo->mCameraViewDir = ViewDirV.normalize();
	float viewDot = lastDir.dot(m_frameInfo->mCameraViewDir);
	bool IsRotating = viewDot < this->SmallRotateDotAngleThreshold;
	bool IsRotatingLarge = viewDot < this->LargeRotateDotAngleThreshold;




    // cache the camera position and view-projection matrix
	float cameraPosSquareDis = common::FloatArray::calculateSquareDistance3(m_frameInfo->CameraPos, CameraPos);

	bool IsCameraDistanceNear = true;
	if(this->CameraNearDistanceThreshold > 0)
	{
		IsCameraDistanceNear = cameraPosSquareDis <= this->CameraNearDistanceThreshold * this->CameraNearDistanceThreshold;
	}



	m_frameInfo->startNewFrame();
    // set ViewProj
	bool sameVP = false;
	if (cameraPosSquareDis == 0) 
	{
		sameVP = common::FloatArray::containSameData16(m_frameInfo->ViewProjArray, ViewProj);
	}
    if (sameVP == false) 
	{
		memcpy(m_frameInfo->CameraPos, CameraPos, 3 * sizeof(float));
		memcpy(m_frameInfo->ViewProjArray, ViewProj, 16 * sizeof(float));

		m_frameInfo->IsSameCameraWithPrev = false;

		//update ViewProjT if VP is changed!
		m_frameInfo->m_rapidRasterizer->mViewProjT.updateTranspose(ViewProj);
	}
	else 
	{
		m_frameInfo->IsSameCameraWithPrev = !this->mResolutionChanged;
	}

	
	if (m_frameInfo->mIsRecording)
	{
		m_frameInfo->mIsRecording = false;
		m_frameInfo->StopFrameCapture();
	}
	else {
		if (m_frameInfo->mCaptureFrame)
		{
			m_frameInfo->mCaptureFrame = false;
			m_frameInfo->StartRecordFrame(CameraPos, ViewDir, ViewProj);
		}
	}

	bool criticalFrame = false;
	bool isRotating = false;
	{
		if (this->mRapidCoherentMode > 0)
		{
			if (IsRotatingLarge || IsCameraDistanceNear == false)
			{
				criticalFrame = true;
			}
			else
			{
				isRotating = IsRotating;
			}
		}

		criticalFrame |= ViewDir[0] == 0 && ViewDir[1] == 0 && ViewDir[2] == 0;
	}
	criticalFrame |= this->mResolutionChanged;
	this->mResolutionChanged = false;
	this->m_rapidRasterizer->onNewFrame(m_frameInfo->FrameCounter, criticalFrame, isRotating, CameraPos);

	
}

    bool SOCPrivate::batchQuery(const float *bbox, unsigned int nMesh, bool *results, bool obbMode) 
	{
		bool result = m_rapidRasterizer->batchQuery(bbox, nMesh, results, obbMode);


		if (m_frameInfo->mIsRecording)
		{
			if (results != nullptr) 
			{
				m_frameInfo->recordOccludee(bbox, nMesh, obbMode);
			}
		}

        return result;
    }
	
	void SOCPrivate::configPerformanceMode( unsigned int configValue)
	{
		if (configValue <= SDOC_RenderMode_CoherentFast)
		{
			this->mRapidCoherentMode = configValue;
			int octuple = 0;
			//if (configValue == SDOC_RenderMode_Full) //do nothing
			if (configValue == SDOC_RenderMode_CoherentFast)
			{
				octuple = 4;
			}
			else if (configValue == SDOC_RenderMode_Coherent)
			{
				octuple = 5;
			}

			this->m_rapidRasterizer->configCoherentMode(octuple); //no accurate 
		}
	}
    bool SOCPrivate::setConfig(unsigned int configTarget, unsigned int configValue)
    {
		bool handled = true;

		if (configTarget == SDOC_FlushSubmittedOccluder)
		{
			m_rapidRasterizer->OnRenderFinish();
			return true;
		}
		else if (configTarget == SDOC_RenderMode)
		{
			if (configValue <= SDOC_RenderMode_CoherentFast) 
			{
				this->configPerformanceMode(configValue);
				LOGI("SDOC set CoherentMode to %d", (int)this->mRapidCoherentMode);
			}

			if (configValue == SDOC_RenderMode_ToggleRenderType)
			{
				this->m_rapidRasterizer->setRenderType(-1);
			}
		}
		else if (configTarget == SDOC_CaptureFrame)
		{
			return this->onFrameCaptureSet(configValue);
		}
		else if (configTarget == SDOC_ShowCulled)
		{
			if (configValue != 0 && configValue != 1) return false;
			this->m_rapidRasterizer->mShowCulled = configValue == 1;
		}
		else if (configTarget == SDOC_SetCCW) 
		{
			bool IsModelCCW = configValue == 1;
			this->m_rapidRasterizer->SetCCW(IsModelCCW);
		}
		else if (configTarget == SDOC_ShowOccludeeInDepthMap) 
		{
			if (configValue != 0 && configValue != 1) return false;
			m_rapidRasterizer->ShowOccludeeInDepthmap(configValue);
		}
		else if (configTarget == SDOC_BackFaceCullOffOccluderFirst)
		{
			m_rapidRasterizer->mBackFaceCullOffFirst = configValue == 1;
			LOGI("SDOC set SDOC_BackFaceCullOffOccluderFirst to %d", (int)this->m_rapidRasterizer->mBackFaceCullOffFirst);
		}
		else if (configTarget == SDOC_EnableOccluderPriorityQueue) {
			this->m_rapidRasterizer->EnablePriorityQueue(configValue == 1);
			LOGI("SDOC EnablePriorityQueue to %d", (int)configValue == 1);
			return true;
		}
		else if (configTarget == SDOC_DebugPrintActiveOccluder) {
			if (configValue <= 1) {
				this->m_rapidRasterizer->PrintNumberOfOccluderOnce = configValue == 1;
				LOGI("SDOC set SDOC_DebugPrintActiveOccluder to %d", configValue);
			}
			return true;
		}
		else 
		{
			handled = false;
		}

		if (handled == false) 
		{
			LOGI("SDOC Config not supported. configTarget %d configValue %d", configTarget, configValue);
			return false;
		}
		
		return true;
    }


	SOCPrivate::SOCPrivate() 
	{
		// frame info
		m_frameInfo = new common::SOCFrameInfo();
        // create rapid rasterizer
        m_rapidRasterizer = new util::RapidRasterizer();



		this->m_frameInfo->m_rapidRasterizer = this->m_rapidRasterizer;



    	memset(m_frameInfo->CameraPos, 0, 3 * sizeof(float));

        setAlgoApproach(common::AlgoEnum::Rasterizer_FullTriangle);
	

		this->configPerformanceMode(SDOC_RenderMode_CoherentFast);

    }

#if defined(SDOC_NATIVE_DEBUG)&& defined(SDOC_NATIVE)

	static int compressedCount = 0;

	class SDOCLoader
	{
	public:
		int OccluderID = 0;

		struct OccluderData
		{
			int backfaceCull = 1; //0,1
			int occluderID = 0;
			OccluderData(int occId) 
			{
				this->occluderID = occId;
			}
			const float* Vertices = nullptr;
			unsigned int VerticesNum = 0;
			const uint16_t* Indices = nullptr;
			unsigned int nIdx = 0;
			float localToWorld[16];

			unsigned int CompactSize;
			uint16_t* CompactData = nullptr;

			
			~OccluderData()
			{
				if (Vertices != nullptr)	delete[] Vertices;
				if (Indices != nullptr)	delete[] Indices;
				if (CompactData != nullptr)	delete[] CompactData;
			}

			void CompressModel()
			{
				util::OccluderQuad::ConfigDebugOccluder(occluderID, CompactData);
				if (CompactData != nullptr) return;

				int outputCompressSize = 0;
				auto initStartTime = std::chrono::high_resolution_clock::now();
				CompactData = sdocMeshBake(&outputCompressSize, this->Vertices, this->Indices, this->VerticesNum, this->nIdx, 15, true, true, 0);
				
				if (CompactData != nullptr) {
					uint16_t queryQuadTriangle[6];
					memcpy(queryQuadTriangle + 2, CompactData, 4 * sizeof(uint16_t));
					util::OccluderQuad::Get_BakeData_QuadTriangleNum(queryQuadTriangle);

					//std::cout << "Baked Quad " << queryQuadTriangle[0] << " Triangle " << queryQuadTriangle[1] << " From original triangle " << (nIdx / 3) << std::endl;
				}
				compressedCount++;
			}


		};
		// for batch query
		struct OccludeeBatch
		{
			std::vector<float> data;
			uint32_t Number = 0;
			bool mObbQuery = false;
			void updateSize(unsigned int nOccludee, int perUnitSize)
			{
				this->mObbQuery = perUnitSize > 6;
				this->Number = nOccludee;
				this->data.resize(nOccludee * perUnitSize);
			}

		};
		struct CapturedFrameData
		{
			float CameraPos[3];
			float CameraDir[3];
			// ViewProj
			float ViewProj[16];

			std::vector<OccluderData*>  Occluders;
			std::vector<OccluderData*>  OccludeeMeshes;
			std::vector<OccludeeBatch*> Occludees;
			~CapturedFrameData()
			{
				for (int i = 0; i < Occluders.size(); i++) {
					OccluderData* occ = Occluders[i];
					delete occ;
				}

				for (int i = 0; i < Occludees.size(); i++) {
					OccludeeBatch* occ = Occludees[i];
					delete occ;
				}
				for (int i = 0; i < OccludeeMeshes.size(); i++) {
					delete OccludeeMeshes[i];
				}

				Occluders.clear();
				OccludeeMeshes.clear();
				Occludees.clear();
			}

		};


		~SDOCLoader()
		{
			if (frame != nullptr)
			{
				delete frame;
				frame = nullptr;
			}
		}


		void loadMatrix(std::ifstream& fin, float* matrix)
		{
			std::string line;
			for (unsigned int row = 0; row < 4; ++row)
			{
				fin >> matrix[row * 4 + 0] >> matrix[row * 4 + 1] >> matrix[row * 4 + 2] >> matrix[row * 4 + 3];
				std::getline(fin, line);
			}
		}


		bool getHeader(std::ifstream& fin, const std::string& header)
		{
			std::string line;
			while (std::getline(fin, line))
			{
				if (line.length() > 1)
				{
					if (line.find(header) != std::string::npos)
					{
						return true;
					}
				}
			}
			return false;
		}
		void loadBatchedOccludee(std::ifstream& fin, std::vector<OccludeeBatch*>& batches, int perUnitSize)
		{
			std::string line;
			// get the number

			OccludeeBatch* batch = new OccludeeBatch(); batches.push_back(batch);

			unsigned int nOccludee = 0;
			fin >> nOccludee;
			std::getline(fin, line);
			batch->updateSize(nOccludee, perUnitSize);
			float* arr = &batch->data[0];

			for (unsigned int i_box = 0; i_box < nOccludee; ++i_box, arr += perUnitSize)
			{

				fin >> arr[0] >> arr[1] >> arr[2];
				std::getline(fin, line);


				fin >> arr[3] >> arr[4] >> arr[5];
				std::getline(fin, line);

				if (perUnitSize >= 12) {
					fin >> arr[6] >> arr[7] >> arr[8];
					std::getline(fin, line);
					fin >> arr[9] >> arr[10] >> arr[11];
					std::getline(fin, line);
				}
				if (perUnitSize >= 18) {
					fin >> arr[12] >> arr[13] >> arr[14];
					std::getline(fin, line);
					fin >> arr[15] >> arr[16] >> arr[17];
					std::getline(fin, line);
				}
			}
			//std::cout << "nOcc " << nOccludee << std::endl;
		}
		SDOCLoader()
		{
			frame = new CapturedFrameData();
		}


		void loadCompactOccluder(std::ifstream& fin, std::vector<OccluderData*>& occluders, std::vector<OccluderData*>& occludeeMeshes, bool occludee)
		{
			std::string line;
			int n128;
			fin >> n128;
			std::getline(fin, line);

			OccluderData *occ = new OccluderData(this->OccluderID++);
			if (occludee)
				occludeeMeshes.push_back(occ);
			else
				occluders.push_back(occ);
			occ->CompactData = new uint16_t[n128 * 8];
			uint16_t* data = (uint16_t*)occ->CompactData;
			for (int i_vert = 0; i_vert < n128; ++i_vert, data += 8)
			{
				fin >> data[0] >> data[1] >> data[2] >> data[3] >> data[4] >> data[5] >> data[6] >> data[7];
				//for (int i = 0; i < 8; i++)
				//	std::cout << data[i] << " ";
				//std::cout << std::endl;
				std::getline(fin, line);
			}
			loadMatrix(fin, occ->localToWorld);
			return;
		}
		void loadOccluder(std::ifstream& fin, std::vector<OccluderData*>& occluders, std::vector<OccluderData*>& occludeeMeshes, bool occludee)
		{
			std::string line;
			// load number of vertices and number of faces
			OccluderData *occ = new OccluderData(this->OccluderID++);
			if(occludee)
				occludeeMeshes.push_back(occ);
			else
				occluders.push_back(occ);
			int nVert, nFace;
			fin >> nVert >> nFace;
			occ->backfaceCull = nVert < 10000000;
			if (occ->backfaceCull == false)
			{
				nVert -= 10000000;  //extract backface cull bit
			}

			std::getline(fin, line);
			occ->VerticesNum = nVert;
			occ->nIdx = nFace * 3;



			// allocation
			float* Vertices = new float[occ->VerticesNum * 3];
			for (int i_vert = 0; i_vert < nVert; ++i_vert)
			{
				fin >> Vertices[i_vert * 3 + 0] >> Vertices[i_vert * 3 + 1] >> Vertices[i_vert * 3 + 2];
				std::getline(fin, line);
			}
			occ->Vertices = Vertices;

			uint16_t* Indices = new uint16_t[occ->nIdx];
			for (int i_face = 0; i_face < nFace; ++i_face)
			{
				int i_face3 = i_face * 3;
				if (false) {  
					//use to debug a model could not build issue
					//copy the model .off file and insert to a simple capture file
					//search SaveSimplifyModel and set it to true to see the simplified model
					int test = 0;
					fin >> test >> Indices[i_face3 + 0] >> Indices[i_face3 + 1] >> Indices[i_face3 + 2];
					assert(test == 3);
				}
				else {
					fin >> Indices[i_face3 + 0] >> Indices[i_face3 + 1] >> Indices[i_face3 + 2];
				}

				std::getline(fin, line);
			}
			occ->Indices = Indices;

			// load LocalToWorld Matrix
			loadMatrix(fin, occ->localToWorld);
			occ->CompressModel();
		}

		bool load(const std::string& file_path)
		{
			if (file_path.empty())
			{
				std::cout << "File Path Empty" << std::endl;
				return false;
			}

			// open the file
			std::ifstream fin(file_path);
			if (!fin)
			{
				std::cout << "Fail to open Path" << std::endl;
				return false;
			}

			std::string line;


			////// get QCAP
			// load width, height & near plane
			if (!getHeader(fin, FB_SETTING_HEADER))
			{
				std::cout << "Fail to find FB_SETTING_HEADER" << std::endl;
				return false;
			}
			std::stringstream info;

			// get framebuffer settings
			fin >> Width >> Height >> NearPlane;
			int CloseWise = Height > 1000000;
			Height -= CloseWise * 1000000;
			this->CCW = 1 ^ CloseWise;
			std::getline(fin, line);


			//read frame 1
			std::getline(fin, line);
			if (line.find("Frame") == std::string::npos)
			{
				return true;
			}

			//frame 1
			int SaveFrameIndex = 0;
			fin >> SaveFrameIndex;
			std::getline(fin, line);

			//Camera PosDir
			std::getline(fin, line);

			CapturedFrameData& f = *frame;
			fin >> f.CameraPos[0] >> f.CameraPos[1] >> f.CameraPos[2] >> f.CameraDir[0] >> f.CameraDir[1] >> f.CameraDir[2];
			std::getline(fin, line);

			// View-Proj matrix
			if (!getHeader(fin, VIEW_PROJ_HEADER))
			{
				return false;
			}
			loadMatrix(fin, f.ViewProj);

			do
			{
				std::getline(fin, line);
				info.clear();
				info.str(line);

				if (line.find("CompactOccluder") != std::string::npos)
				{
					loadCompactOccluder(fin, f.Occluders, f.OccludeeMeshes, line.find("Query") != std::string::npos);
				}
				else if (line.find("Occluder") != std::string::npos)
				{
					loadOccluder(fin, f.Occluders, f.OccludeeMeshes, line.find("Query") != std::string::npos);
				}
				else if (line.find("Batched OccludeeOBB") != std::string::npos)
				{
					loadBatchedOccludee(fin, f.Occludees, nOcludeeOBBSize);
				}
				else if (line.find("Batched Occludee") != std::string::npos) 
				{
					loadBatchedOccludee(fin, f.Occludees, 6);
				}
			} while (fin.eof() == false);

			return true;
		}

		// width, height & near plane
		uint32_t Width = 0;
		uint32_t Height = 0;
		uint32_t CCW = 1;
		float NearPlane = 0.0f;

		CapturedFrameData* frame = nullptr;
	};

	static SDOCLoader::CapturedFrameData* CurrentDebugFrame = nullptr;
	static bool saveCheckerBoardImage = true;

	static int gTotalQueryNum = 0;
	static int gVisibleNum = 0;
	static SOCPrivate* Singleton = nullptr;
	static std::mutex printLock;
	static bool gCompressMode = false;
	static void QueryOccludees()
	{
		SDOCLoader::CapturedFrameData* frame = CurrentDebugFrame;

		int totalQueryNum = 0;
		int visibleNum = 0;
		for (int occIdx = 0; occIdx < frame->OccludeeMeshes.size(); occIdx++)
		{
			bool visible = false;
			totalQueryNum++;
			auto occ = frame->OccludeeMeshes[occIdx];
			if ((gCompressMode == false) || (occ->CompactData == nullptr))
			{
				{
					if (occ->Indices == nullptr) {
						visible = Singleton->m_frameInfo->queryOccludeeMesh((float*)(occ->CompactData), nullptr, 0, 0, occ->localToWorld, true);
					}
					else {
						visible = Singleton->m_frameInfo->queryOccludeeMesh(occ->Vertices, occ->Indices, occ->VerticesNum, occ->nIdx, occ->localToWorld, occ->backfaceCull);
					}
				}
				bool visible2 = false;
				if ( (occ->CompactData != nullptr)) {
					visible2 = Singleton->m_frameInfo->queryOccludeeMesh((float*)(occ->CompactData), nullptr, 0, 0, occ->localToWorld, true);
				}
				if (visible2 == false && visible == true) {
					std::cout << "*****************************ERROR in bake mode**********************************************************  " << occIdx<< std::endl;
				}

				if (visible2 == true && visible == false) {
					std::cout << "*****************************BakeCanSee**********************************************************  " << occIdx<< std::endl;
				}

			}
			else
			{
				visible = Singleton->m_frameInfo->queryOccludeeMesh((float*)(occ->CompactData), nullptr, 0, 0, occ->localToWorld, true);
			}
			if (visible == false) {
				//std::cout << "idx " << occIdx << " " << visible << std::endl;
			}
			visibleNum += visible;
		}

		
		for (int i = 0; i < frame->Occludees.size(); i++) {

			auto& batch = frame->Occludees[i];

			bool* allResults = new bool[batch->Number];
			Singleton->batchQuery(&batch->data[0], batch->Number, allResults, batch->mObbQuery);

			for (uint32_t idx = 0; idx < batch->Number; idx++)
			{
				visibleNum += (int)(allResults[idx] == true);
				totalQueryNum++;
				//if(allResults[idx] == false)
				//	std::cout << "Idx " << idx<<" " << batch->Number << " visible FAIL " << visibleNum << std::endl;
			}
			delete[]allResults;
		}
		gVisibleNum = visibleNum;
		gTotalQueryNum = totalQueryNum;
		printLock.lock();
		std::cout << "QUERY Result " << visibleNum << "/" << totalQueryNum <<"  ThreadID " << std::this_thread::get_id() << std::endl;
		printLock.unlock();
	}
    bool SOCPrivate::replay(const char *file_path, int config, int frameNum, uint64_t replaySetting, float * replayResult)
	{
		static SDOCLoader* loader = nullptr;
		LOGI("replay config %d", config);
       

		std::string inputPath = std::string(file_path);
		

        // create SDOCLoader if needed
        loader = new  SDOCLoader();


		// reset before replay
		// first load, then save
		if (!loader->load(file_path))
		{
			delete loader;
			loader = nullptr;
			LOGI( "Fail to load input capture file" );
			return false;
		}
		compressedCount = 0;


		this->resize(loader->Width, loader->Height);
		setConfig(SDOC_SetCCW, loader->CCW);
		std::cout << "Counter Clock Wise set to " << loader->CCW << std::endl;
		

		//disable frame skipper
		//if (replaySetting & 3)
		//{
		//	this->configPerformanceMode(SDOC_RenderMode_Coherent);
		//}
		//else {
		//	this->configPerformanceMode(SDOC_RenderMode_Full);
		//}
		this->configPerformanceMode(replaySetting & 3);

		LOGI( "Width %d Height %d",  loader->Width, loader->Height);

		this->setAlgoApproach((common::AlgoEnum) config);

		
		bool dumpPerDraw = (replaySetting & 8) > 0;
		LOGI("Replay setting %d  algo %d", replaySetting, this->algoApproachMask);

		int width = m_frameInfo->Width;
		int height = m_frameInfo->Height;
		unsigned char *image = new unsigned char[width * height * 2]; // clear
        m_frameInfo->FrameCounter = 0;
        bool printResult = false;
        auto end = std::chrono::high_resolution_clock::now();
        auto start = std::chrono::high_resolution_clock::now();
        int replayFrameIdx = 0;
		bool printedBug = false;


		int ReplayMaxFrame = frameNum;

		unsigned char *blockMasks = new unsigned char[512 * 1024];

		int allResultsLength = 1024;
		bool * allResults = new bool[allResultsLength];

		Singleton = this;

		auto initStartTime = std::chrono::high_resolution_clock::now();

		int totalOccluderNum = 0;

		int compressMode = (replaySetting >> 8) & 1;
		gCompressMode = compressMode;
		int renderMode = (replaySetting >> 9) & 7;
		this->setRenderType(renderMode);
		

		int roundNum = 65535 & (replaySetting >> 32);
		int focusDraw = (replaySetting << 32 >> 48) - 1;
		int loopCount = -1;
		
		start = std::chrono::high_resolution_clock::now();

		bool * occluderStates[2];
		int extraOffSet = (int)loader->frame->OccludeeMeshes.size(); //add extraOffset as might treat occludeeMesh as Occluders for debug
		int maxResultSize = (int)loader->frame->Occluders.size() + 1 + extraOffSet;
		occluderStates[0] = new bool[maxResultSize];
		memset(occluderStates[0], 0, sizeof(bool) * maxResultSize);
		occluderStates[1] = new bool[maxResultSize];
		memset(occluderStates[1], 0, sizeof(bool) * maxResultSize);

		while (m_frameInfo->FrameCounter < ReplayMaxFrame)
        {
			loopCount++;
			
			SDOCLoader::CapturedFrameData* frame = loader->frame;
			
			CurrentDebugFrame = frame;

			// start new frame
			float *c = frame->CameraDir;
			if (c[0] == 0 && c[1] == 0 && c[2] == 0) 
			{
				float p[3];
				p[0] = 1.0f;
				p[1] = p[2] = 0.0f;
				startNewFrame(frame->CameraPos, p, frame->ViewProj);
			}
			else
			{
				startNewFrame(frame->CameraPos, frame->CameraDir, frame->ViewProj);
			}
            // submit occluder
			
			if (loopCount > 0)
			{
				m_rapidRasterizer->UsePrevFrameOccluders((int) frame->Occluders.size());
				m_rapidRasterizer->OnRenderFinish();
			}
			else
			{
				//int count = 0;

				int maxAllow = 9999;
				

				if (dumpPerDraw) 
				{
					if (frame->Occluders.size() <= loopCount) 
					{
						return true;
					}
				}
				totalOccluderNum = 0;
				int drawIdx = 0;

				
				for (int occIdx = 0; occIdx < frame->Occluders.size(); occIdx++)
				{
					//std::cout << "occIdx " << occIdx << "  size " << frame->Occluders.size() << std::endl;
					auto occ = frame->Occluders[occIdx];
						bool submitDraw = true;
						if (dumpPerDraw) 
						{
							if (drawIdx > loopCount) 
							{
								submitDraw = false;
							}
						}

						if (focusDraw >= 0)
						{
							if (drawIdx != focusDraw) {
								submitDraw = false;
							}
						}

						//m_rapidRasterizer->EnableBackFaceCull(true);

						if (submitDraw) {
							if ( (compressMode == false) || (occ->CompactData == nullptr))
							{
								{
									if (occ->Indices == nullptr) {
										this->m_frameInfo->submitOccluder((float*)(occ->CompactData), nullptr, 0, 0,  occ->localToWorld, true);
									}
									else {
										this->m_frameInfo->submitOccluder(occ->Vertices, occ->Indices, occ->VerticesNum, occ->nIdx,  occ->localToWorld, occ->backfaceCull);

										//this->m_frameInfo->submitOccluder(occ->Vertices, indices, occ->VerticesNum, 12, occ->modelAABB, occ->localToWorld, occ->backfaceCull);
									}
								}
							}
							else 
							{
								this->m_frameInfo->submitOccluder((float*)(occ->CompactData), nullptr, 0, 0, occ->localToWorld, true);
							}
							totalOccluderNum++;
						}

						drawIdx++;
					
					maxAllow--;
					if (maxAllow == 0) {
						break;
					}
				}
				m_rapidRasterizer->OnRenderFinish();

			}





			{
				m_rapidRasterizer->SyncOccluderPVS(occluderStates[m_frameInfo->FrameCounter & 1]);
			}

			


			bool enableMtTest = false;
			if (enableMtTest) {
				thread t1(QueryOccludees);
				thread t2(QueryOccludees);
				thread t3(QueryOccludees);
				thread t4(QueryOccludees);
				thread t5(QueryOccludees);
				thread t6(QueryOccludees);
				thread t7(QueryOccludees);
				thread t8(QueryOccludees);

				// Wait for each thread to finish before continuing on.
				t1.join();
				t2.join();
				t3.join();
				t4.join();
				t5.join();
				t6.join();
				t7.join();
				t8.join();
			}
			else {
				QueryOccludees();
			}
		

				
			replayResult[1] = (float)gVisibleNum;
			replayResult[2] = (float)gTotalQueryNum;

			
			if (printResult)
			{
				LOGI("QueryResult: VisibleNum %d", gVisibleNum);
				int i = 0;
				for (; i + 4 <= gTotalQueryNum; i += 4)
				{
					int r1 = allResults[i];
					int r2 = allResults[i + 1];
					int r3 = allResults[i + 2];
					int r4 = allResults[i + 3];
					int result = (r1 << 3) + (r2 << 2) + (r3 << 1) + r4;

					std::cout << std::hex << result;
				}
				for (; i < gTotalQueryNum; i++)
				{
					auto v = allResults[i];
					if (v) LOGI( "1");
					else LOGI( "0");
				}
				std::cout << std::dec << "Frame: " << m_frameInfo->FrameCounter << std::endl;
			}


			if (dumpPerDraw) 
			{
				if (focusDraw >= 0 && loopCount != focusDraw) 
				{
					continue;
				}

				doDumpDepthMap(image, common::DumpImageMode::DumpFull);
				std::string inputFile = std::string(file_path);
				std::string result = inputFile.substr(0, inputFile.length() - 4) + "_" + std::to_string(config)+"_" +std::to_string(loopCount) + ".ppm";
				dumpOccluderOccludeeColorImage(result, image, width, height);
			}

			if (loopCount == 5 && false)
			{
				std::string inputFile = std::string(file_path);

				std::size_t found = inputFile.find_last_of("/");
				std::string fileName = inputFile.substr(found + 1);
				m_frameInfo->mOutputSaveCap = fileName;
				onFrameCaptureSet(1);
			}
		}	

		replayResult[3] = 0;
		{
			int culled = 0;
			for (int idx = 0; idx < loader->frame->Occluders.size(); idx++)
			{
				culled += !occluderStates[0][idx] && !occluderStates[1][idx];
			}
			std::cout << "****************** Occluder Culled " << culled << std::endl;
			replayResult[3] = (float) culled;
		}
		replayResult[4] =(float) loader->frame->Occluders.size();
		end = std::chrono::high_resolution_clock::now();
		int time = (int)std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();


		std::cout << "time " << time << std::endl;

		LOGI("Last Frame Query Result: VisibleNum  %d totalQueryNum %d TotalOccluderNum %d", gVisibleNum, gTotalQueryNum, totalOccluderNum);
		//token file_path to get the capture name
		std::string inputFile = std::string( file_path);

		std::size_t found = inputFile.find_last_of("/");
		std::string fileName = inputFile.substr(found + 1);
		std::string folderName = inputFile.substr(0, found + 1) + "/Output/";
#if defined(SDOC_WIN)
		_mkdir(folderName.c_str());
#elif defined(SDOC_ANDROID)
		mkdir(folderName.c_str(), 777);
#endif

         time = (int)std::chrono::duration_cast<std::chrono::microseconds>(end - initStartTime).count();
        std::cout << std::dec <<  "Total Time(seconds) used " << (time * 1.0 / 1000000) << std::endl;

		replayResult[0] = (float)(time * 1.0 / 1000000);

		//if(false)
		{
			doDumpDepthMap(blockMasks, common::DumpImageMode::DumpBlockMask);
			dumpGrayImage(folderName + "depth" + std::to_string(config) + "_BlockMask.pgm", blockMasks, 512, 1024);
			std::cout << "******************** " + folderName + "depth" + std::to_string(config) + "_BlockMask.pgm" << std::endl;
		}
		delete[] blockMasks;
		blockMasks = nullptr;


		delete[] occluderStates[0];
		delete[] occluderStates[1];



		{
			doDumpDepthMap(image, common::DumpImageMode::DumpFullMax);
			std::string result = inputFile.substr(0, inputFile.length()-4) + "_" + std::to_string(config)+"_r"+ std::to_string(roundNum)+ "Max.ppm";
			dumpOccluderOccludeeColorImage(result, image, width, height);


			doDumpDepthMap(image, common::DumpImageMode::DumpFullMin);
			 result = inputFile.substr(0, inputFile.length() - 4) + "_" + std::to_string(config) + "_r" + std::to_string(roundNum) + "Min.ppm";
			dumpOccluderOccludeeColorImage(result, image, width, height);

			LOGI("save %s", result.c_str());

			if (saveCheckerBoardImage) {
				saveCheckerBoardImage = false;
				int w = 256;
				uint8_t *cbi = new uint8_t[w * w];

				doDumpDepthMap(cbi, common::DumpImageMode::CheckerBoard_blackPattern);
				dumpGrayImage(folderName + "CheckerBoard_blackPattern.pgm", cbi, w, w);
				doDumpDepthMap(cbi, common::DumpImageMode::CheckerBoard_whitePattern);
				dumpGrayImage(folderName + "CheckerBoard_whitePattern.pgm", cbi, w, w);
				doDumpDepthMap(cbi, common::DumpImageMode::CheckerBoard_oddColumn);
				dumpGrayImage(folderName + "CheckerBoard_oddColumn.pgm", cbi, w, w);
				doDumpDepthMap(cbi, common::DumpImageMode::CheckerBoard_evenColumn);
				dumpGrayImage(folderName + "CheckerBoard_evenColumn.pgm", cbi, w, w);


				doDumpDepthMap(cbi, common::DumpImageMode::CheckerBoard_oddBlack);
				dumpGrayImage(folderName + "CheckerBoard_oddBlack.pgm", cbi, w, w);
				doDumpDepthMap(cbi, common::DumpImageMode::CheckerBoard_evenBlack);
				dumpGrayImage(folderName + "CheckerBoard_evenBlack.pgm", cbi, w, w);
				doDumpDepthMap(cbi, common::DumpImageMode::CheckerBoard_evenWhite);
				dumpGrayImage(folderName + "CheckerBoard_evenWhite.pgm", cbi, w, w);
				doDumpDepthMap(cbi, common::DumpImageMode::CheckerBoard_oddWhite);
				dumpGrayImage(folderName + "CheckerBoard_oddWhite.pgm", cbi, w, w);

				delete[]cbi;
			}

		}


        //delete[] image;
		LOGI("%d x %d", width, height);
		//image = new unsigned char[m_depthWidth * m_depthHeight / 8 / 8]; // clear
		if (false) {
			for (int idx = common::DumpImageMode::DumpHiz;
				idx <= common::DumpImageMode::DumpBlockMask; idx++)
			{

				doDumpDepthMap(image, (common::DumpImageMode) idx);

				if (idx == common::DumpImageMode::DumpHiz)
				{
					dumpGrayImage(folderName + "depth" + std::to_string(config) + "Hiz.pgm", image, width / 8, height / 8);
				}
				
			}
		}
		
		delete[] image;

		delete[] allResults;

		LOGI("Total Memory used %d", this->getMemoryByteUsage());

		if (loader != nullptr)
		{
			delete loader;
			loader = nullptr;
		}
        return true;
	}
#endif

    SOCPrivate::~SOCPrivate()
    {
		delete m_frameInfo;
		delete m_rapidRasterizer;

    }

    void SOCPrivate::setAlgoApproach(common::AlgoEnum config)
    {
		this->algoApproachMask = config;
    }

	

	bool SOCPrivate::onFrameCaptureSet(int configValue)
	{
		if (configValue == 0) return false;
		std::string outputDir = SOCLogger::GetOutputDirectory();
		if (outputDir == "")
		{
			LOGI("invalid output save path:%s", SOCLogger::Singleton.OutputDir.c_str());
			return false;
		}

		m_frameInfo->mCaptureFrame = true;
		return true;
	}

	bool SOCPrivate::doDumpDepthMap(unsigned char *data, common::DumpImageMode mode)
    {
        if (!data)
        {
            return false;
        }
		return m_rapidRasterizer->dumpDepthMap(data, mode);
    }

	bool SOCPrivate::saveColorImage(unsigned char * fullImgData, const char * path)
	{
		if (path != nullptr) 
		{
			std::string fileName = path;
			bool save = dumpOccluderOccludeeColorImage(fileName, fullImgData, m_frameInfo->Width, m_frameInfo->Height);
			if (save) {
				LOGI("Saved %s", fileName.c_str());
			}
			return save;
		}
		std::string outputDir = SOCLogger::Singleton.OutputDir;
		time_t now = time(0);
		tm* ltm = localtime(&now);
		int year = 1900 + ltm->tm_year;
		int month = 1 + ltm->tm_mon;
		int day = ltm->tm_mday;
		int hour = ltm->tm_hour;
		int minute = ltm->tm_min;
		int sec = ltm->tm_sec;
		std::string fileName = outputDir + "//SDOC"
			+ std::to_string(year) + "_"
			+ std::to_string(month) + "_"
			+ std::to_string(day) + "_"
			+ std::to_string(hour) + "_"
			+ std::to_string(minute) + "_"
			+ std::to_string(sec) + ".ppm";
		bool save = dumpOccluderOccludeeColorImage(fileName, fullImgData, m_frameInfo->Width, m_frameInfo->Height);
		if (save) {
			LOGI("Saved %s", fileName.c_str());
		}
		return save;
	}
	size_t SOCPrivate::getMemoryByteUsage()
	{
		//size_t minMemory = this->m_frameInfo->Width * this->m_frameInfo->Height * 2; //each pixel own at least 2 byte, use ushort to represent the depth

		size_t s = m_rapidRasterizer->getMemoryByteUsed();
		//s = std::max<size_t>(minMemory, s);
		s += sizeof(SOCPrivate) + sizeof(common::SOCFrameInfo);
		return s;
	}


	void SOCPrivate::setRenderType(int renderType)
	{
		this->m_rapidRasterizer->setRenderType(renderType);
	}


	void SOCPrivate::socDestroy()
	{
		delete this;
	}
	bool SOCPrivate::socConfig(unsigned int width, unsigned int height, float nearPlane)
	{
		// resize
		if (width < 64 || height < 8)
		{
			// width & height should be greater than 0
			return false;
		}
		else if ((width & 63) != 0 || (height & 7) != 0)
		{
			// width & height should be a multiple of 8
			return false;
		}
		else if (width > 65535 || height > 65535)
		{
			return false;
		}

		// nearplane
		if (nearPlane < 0.0f)
		{
			// nearplane should be greater than 0
			return false;
		}


		this->resize(width, height);
		this->setNearPlane(nearPlane);

		return true;
	}



} // namespace SOC

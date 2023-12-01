//============================================================================================================
//
//
//                  Copyright (c) 2023, Qualcomm Innovation Center, Inc. All rights reserved.
//                              SPDX-License-Identifier: BSD-3-Clause
//
//============================================================================================================

#include <string>
#include <fstream>
#include <iostream>
#include <sstream>
#include <vector>
#include <iosfwd>
#include <chrono>
#include <algorithm>
#include <chrono>
#include <thread>


#ifdef __ANDROID__
#include <jni.h>
#include "include/SOC/SDOCAPI.h"
#else
#include "API/SDOCAPI.h"
#include "SDOCReplayer.h"
#endif

#ifdef __ANDROID__
#include <android/log.h>
#define LOGI(...) __android_log_print(ANDROID_LOG_VERBOSE, "SDOC", __VA_ARGS__)
#elif SDOC_IOS
#include <CoreFoundation/CoreFoundation.h>
extern "C" {
	void NSLog(CFStringRef format, ...);
	void CLSLog(CFStringRef format, ...);
}
#define LOGI(format, ...) CLSLog(CFSTR(format), ##__VA_ARGS__)
#else
#include <iostream>
#include <stdarg.h>
static void LOGI(const char* format, ...)
{
	char message[512];
	va_list args;
	va_start(args, format);
	vsnprintf(message, 512, format, args);
	va_end(args);

	std::cout << message << std::endl;
}
#endif


struct OccludeeGroup
{
	// for batch query
	struct OccludeeBatch
	{
		OccludeeBatch(uint32_t num)
		{
			Number = num;
			data.resize(num * 6);
		}
		std::vector<float> data;
		uint32_t Number = 0;
	};
	std::vector<OccludeeBatch*> Batches;

	void clear()
	{
		for (auto b : Batches) delete b;
		Batches.clear();
	}
};
struct CapturedFrameData
{
	float CameraPos[3];
	float CameraDir[3];
	float ViewProj[16];

	struct OccludeeGroup Occludees;

	void clear()
	{
		// Camera pos
		for (unsigned int i = 0; i < 3; ++i)
		{
			CameraPos[i] = 0.0f;
		}

		// array
		for (unsigned int i = 0; i < 16; ++i)
		{
			ViewProj[i] = 0.0f;
		}

		Occludees.clear();
	}
};

static const std::string SDOC_FILE_HEAD = "SOC";
static const std::string FB_SETTING_HEADER = "Framebuffer Settings";
static const std::string CAM_POS_HEADER = "Camera PosDir";
static const std::string VIEW_PROJ_HEADER = "View Projection Matrix";
static const std::string BATCHED_OCE_HEADER = "Batched Occludee";
static const int BBOX_STRIDE = 6;

static void* pSDOC = nullptr;

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
		unsigned short* CompactData = nullptr;

		~OccluderData()
		{
			if (Vertices != nullptr)	delete[] Vertices;
			if (Indices != nullptr)	delete[] Indices;
			if (CompactData != nullptr)	delete[] CompactData;
		}

		void CompressModel()
		{
			if (CompactData != nullptr) return;

			int outputCompressSize = 0;
			auto initStartTime = std::chrono::high_resolution_clock::now();
			unsigned short * output = sdocMeshBake(&outputCompressSize, this->Vertices, this->Indices, this->VerticesNum, this->nIdx, 15, true, true, 0);
			if(output != nullptr)
			{
				CompactData = new unsigned short[outputCompressSize];
				memcpy(CompactData, output, outputCompressSize * sizeof(short));
			}
			auto time = (int)std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - initStartTime).count();
			LOGI("Total Bake Time(ms) used %f  Face  %d  VerticesNum %d", (time * 1.0 / 1000), nIdx/3, VerticesNum);
		}


	};
	// for batch query
	struct OccludeeBatch
	{
		std::vector<float> data;
		uint32_t Number = 0;
		void updateSize(unsigned int nOccludee)
		{
			this->Number = nOccludee;
			this->data.resize(nOccludee * 6);
		}

	};
	struct CapturedFrameData
	{
		float CameraPos[3];
		float CameraDir[3];
		// ViewProj
		float ViewProj[16];

		std::vector<OccluderData*>  Occluders;
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

			Occluders.clear();
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

	void loadBatchedOccludee(std::ifstream& fin, std::vector<OccludeeBatch*>& batches)
	{
		std::string line;
		// get the number

		OccludeeBatch* batch = new OccludeeBatch(); batches.push_back(batch);

		unsigned int nOccludee = 0;
		fin >> nOccludee;
		std::getline(fin, line);
		batch->updateSize(nOccludee);
		float* arr = &batch->data[0];

		for (unsigned int i_box = 0; i_box < nOccludee; ++i_box, arr += BBOX_STRIDE)
		{

			fin >> arr[0] >> arr[1] >> arr[2];
			std::getline(fin, line);


			fin >> arr[3] >> arr[4] >> arr[5];
			std::getline(fin, line);
		}
		std::cout << "nOcc " << nOccludee << std::endl;
	}
	SDOCLoader()
	{
		frame = new CapturedFrameData();
	}


	void loadCompactOccluder(std::ifstream& fin, std::vector<OccluderData*>& occluders)
	{
		std::string line;
		int n128;
		fin >> n128;
		std::getline(fin, line);

		//std::cout << "compact line " << n128 << std::endl;
		OccluderData *occ = new OccluderData(this->OccluderID++);
		occluders.push_back(occ);
		occ->CompactData = new unsigned short[n128 * 8];
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

	void loadOccluder(std::ifstream& fin, std::vector<OccluderData*>& occluders)
	{
		std::string line;
		// load number of vertices and number of faces
		OccluderData *occ = new OccluderData(this->OccluderID++);
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
			fin >> Indices[i_face3 + 0] >> Indices[i_face3 + 1] >> Indices[i_face3 + 2];
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
				loadCompactOccluder(fin, f.Occluders);
			}
			else if (line.find("Occluder") != std::string::npos)
			{
				loadOccluder(fin, f.Occluders);
			}
			else if (line.find("Batched Occludee") != std::string::npos)
			{
				loadBatchedOccludee(fin, f.Occludees);
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

static int ReplayMaxFrame = 0;
static std::string InputFolderPath = "";
static bool SleepBetweenFrames = false;
static bool QuickImageDumpMode = false;
void ReplayFrame(SDOCLoader* dataProvider, int mode, int frameNum, int saveFrameIdx)
{
#if defined(SDOC_ANDROID)
	// set thread affinity
	// common golden core: 4~6 in Snapdragon chips
	//attempt to bind to a golden core
	cpu_set_t mask;
	for (int i_cpu = 4; i_cpu <= 6; ++i_cpu)
	{
		CPU_ZERO(&mask);
		CPU_SET(i_cpu, &mask);
		if (sched_setaffinity(0, sizeof(mask), &mask) == 0)
		{
			break;
		}
	}
#endif

	unsigned int width = dataProvider->Width;
	unsigned int height = dataProvider->Height;

	pSDOC = sdocInit(width, height, 1.0f);
	sdocSet(pSDOC, SDOC_SetCCW, dataProvider->CCW);
	std::cout << "Counter Clock Wise set to " << dataProvider->CCW << std::endl;


	std::string outputPath = InputFolderPath + "/";
	sdocSync(pSDOC, SDOC_Set_FrameCaptureOutputPath, (void*)outputPath.c_str());

	int widthHeight[2];
	widthHeight[0] = width;
	widthHeight[1] = height;
	sdocSync(pSDOC, SDOC_Reset_DepthMapWidthAndHeight, widthHeight); //verify SDOC_Reset_DepthMapWidthAndHeight
	sdocSync(pSDOC, SDOC_Get_DepthBufferWidthHeight, widthHeight);	  //verify SDOC_Get_DepthBufferWidthHeight
	width = widthHeight[0];
	height = widthHeight[1];



	sdocSet(pSDOC, SDOC_RenderMode, mode); //set coherent mode
	sdocSet(pSDOC, SDOC_SetCCW, true); //treat as CCW

	std::vector<unsigned char> buffer;
	int bufferResolution = 0;
	if (saveFrameIdx >= 0)
	{
		buffer.resize(width * height * 2); //only save r,g channel and b = r&g
	}

	int visibleNum = 0;
	int totalQuery = 0;
	int occluderSubmitted = 0;


	int allResultsLength = 1024;
	bool* allResults = new bool[allResultsLength];

	ReplayMaxFrame = frameNum;
	if (QuickImageDumpMode && ReplayMaxFrame > frameNum)
	{
		ReplayMaxFrame = frameNum;
	}


#if defined(__aarch64__)
#else
	LOGI("Test only: Armv7 run compress data");
#endif
	auto end = std::chrono::high_resolution_clock::now();
	auto start = std::chrono::high_resolution_clock::now();

	int totalRenderTime = 0;
	for (int frameIdx = 0; frameIdx < ReplayMaxFrame; frameIdx++)
	{
		auto frame = dataProvider->frame;

		start = std::chrono::high_resolution_clock::now();
		// start new frame
		sdocStartNewFrame(pSDOC, frame->CameraPos, frame->CameraDir, frame->ViewProj);
		occluderSubmitted = 0;

		for (auto occluder : frame->Occluders)
		{
			//Test only: make armv7 cover compress mesh input, and armv8 normal mesh input
#if defined(__aarch64__)
			sdocRenderOccluder(pSDOC, occluder->Vertices, occluder->Indices, occluder->VerticesNum, occluder->nIdx, occluder->localToWorld, true);
#else			
			sdocRenderBakedOccluder(pSDOC, occluder->CompactData, occluder->localToWorld);
#endif

			occluderSubmitted++;
		}


		int maxQueryNum = 0;
		for (auto& batch : frame->Occludees) {
			maxQueryNum = std::max<int>(maxQueryNum, batch->Number);
		}
		if (maxQueryNum > allResultsLength)
		{
			delete[]allResults;
			allResultsLength = 2 * maxQueryNum;
			allResults = new bool[allResultsLength];
		}





		visibleNum = 0;
		int totalQueryNum = 0;

		for (auto& batch : frame->Occludees) {

			totalQueryNum += batch->Number;
			if (totalQueryNum > allResultsLength)
			{
				delete[]allResults;
				allResultsLength = 2 * totalQueryNum;
				allResults = new bool[allResultsLength];
			}
			sdocQueryOccludees(pSDOC, &batch->data[0], batch->Number, allResults);

			for (int idx = 0; idx < totalQueryNum; idx++)
			{
				visibleNum += (int)(allResults[idx] == true);
			}
		}
		totalQuery = totalQueryNum;

		end = std::chrono::high_resolution_clock::now();
		totalRenderTime += (int)std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();

		if (frameIdx <= saveFrameIdx)
		{
			sdocSync(pSDOC, SDOC_Get_DepthMap, &buffer[0]); //to get the occlusion depth map
		}

		if (SleepBetweenFrames)
		{
			std::this_thread::sleep_for(std::chrono::milliseconds(15)); // simulate, aim for 60fps
		}
	}


	if (saveFrameIdx >= 0)
	{
		//able to save ppm to show occludee status
		std::string file = InputFolderPath + "depthbuffer.ppm";
		const char* pChar =  file.c_str();		
		sdocSync(pSDOC, SDOC_Save_DepthMapPath, (void*)pChar);
		sdocSync(pSDOC, SDOC_Save_DepthMap, &buffer[0]);

		std::ofstream ofs(InputFolderPath + "depthbuffer.pgm", std::ios_base::out | std::ios_base::binary);
		ofs << "P5\n" << width << " " << height << "\n255\n";
		for (unsigned int ny = 0; ny < height; ny++)
		{
			for (unsigned int nx = 0; nx < width; nx++)
			{
				int idx = nx + (height - 1 - ny) * width;
				ofs << buffer[idx];
			}
		}
		ofs.close();
	}


	LOGI("OccluderNum %d QueryResult: VisibleNum %d TotalQuery %d ", occluderSubmitted, visibleNum, totalQuery);
	for (int i = 0; i < 10; i++) 
	{
		LOGI("*****Total Render Time %d ", totalRenderTime);
	}

	sdocSet(pSDOC, SDOC_DestroySDOC, 1);

	delete[]allResults;
}
std::string TestSDOC(std::string folderPath, std::string capFileName)
{
	InputFolderPath = folderPath;
	SDOCLoader* loader = new SDOCLoader();
	if (!loader->load(InputFolderPath + capFileName))
	{
		delete loader;
		return "Fail";
	}
	int totalFrame = 500;
	ReplayFrame(loader, SDOC_RenderMode_Coherent, totalFrame, totalFrame);

	delete loader;


	return "Success";
}


std::string TestSDOCQuick(std::string folderPath, std::string capFileName)
{
	InputFolderPath = folderPath;
	SDOCLoader* dataProvider = new SDOCLoader();
	if (!dataProvider->load(InputFolderPath + capFileName))
	{
		delete dataProvider;
		return "Fail";
	}


	int totalFrame = 500;
	QuickImageDumpMode = true;
	ReplayFrame(dataProvider, SDOC_RenderMode_CoherentFast, totalFrame, totalFrame);
	QuickImageDumpMode = false;
	delete dataProvider;


	return "Success";
}

#ifdef __ANDROID__
extern "C" JNIEXPORT jstring JNICALL
Java_com_qualcomm_sdocdemo_MainActivity_CallSDOCStressTestInJNI(
	JNIEnv * env,
	jobject /* this */) {

	SleepBetweenFrames = true;
	int totalRound = 99999;
	std::string result = "";
	for (int idx = 0; idx < totalRound; idx++)
	{
		result = TestSDOC("/sdcard/Android/data/com.qualcomm.sdocdemo/files/",
			"input.cap");
	}
	return env->NewStringUTF(result.c_str());
}

extern "C" JNIEXPORT jstring JNICALL
Java_com_qualcomm_sdocdemo_MainActivity_CallSDOCInJNIQuick(
	JNIEnv * env,
	jobject /* this */) {

	SleepBetweenFrames = false;
	std::string result = TestSDOCQuick("/sdcard/Android/data/com.qualcomm.sdocdemo/files/", "input.cap");

	return env->NewStringUTF(result.c_str());
}


extern "C" JNIEXPORT jstring JNICALL
Java_com_qualcomm_sdocdemo_MainActivity_GetPackageArmVersion(
	JNIEnv * env,
	jobject /* this */) {

#if defined(__aarch64__)
	return env->NewStringUTF("ArmV8 Normal Mesh");
#else
	return env->NewStringUTF("ArmV7 Compress Mesh");
#endif
}

#endif

//============================================================================================================
//
//
//                  Copyright (c) 2023, Qualcomm Innovation Center, Inc. All rights reserved.
//                              SPDX-License-Identifier: BSD-3-Clause
//
//============================================================================================================

#include <iostream>
#include "API/SDOCAPI.h"
#include <cassert>
#include "Util/RapidRasterizer/RapidRasterizer.h"
#include <fstream>
#include <iostream>

#include "SDOCReplayer.h"
//#include <Windows.h>

using namespace SDOCCommon;
using namespace SDOCUtil;



const char * getTextForEnum(int enumVal)
{
	for (int i = 24; i <= 26; i++) 
	{
		if (enumVal & (1 << i)) 
		{
			enumVal -= 1 << i;
		}
	}

	switch (enumVal)
	{

	case  SDOCCommon::AlgoEnum::Rasterizer_FullTriangle: return "Rasterizer_FullTriangle";
		

#ifdef SDOC_NATIVE_DEBUG

	case  SDOCCommon::AlgoEnum::Rasterizer_FullTriangle2: return "Rasterizer_FullTriangle2";
	case  SDOCCommon::AlgoEnum::Rasterizer_FullTriangle3: return "Rasterizer_FullTriangle3";
	case  SDOCCommon::AlgoEnum::Rasterizer_FullTriangle4: return "Rasterizer_FullTriangle4";
	case  SDOCCommon::AlgoEnum::Rasterizer_FullTriangle5: return "Rasterizer_FullTriangle5";

#endif
	default:
		break;
	}
	return "Undefined ";
}



bool socReplay(const char *file_path, float* result, int config = 0, int frameNum = 0, uint64_t settingConfig = 0)
{
//	Sleep(3000);
	void* pSDOC = sdocInit(1024, 512, 1.0f);
	sdocSync(pSDOC, SDOC_Test_Result, result);
	sdocSync(pSDOC, SDOC_Test_Config, &config);
	sdocSync(pSDOC, SDOC_Test_FrameNum, &frameNum);
	sdocSync(pSDOC, SDOC_Test_SettingConfig, &settingConfig);
	return sdocSync(pSDOC, SDOC_Test_ReplayAndDestroy, (void*)file_path);
}

static int frameCount = 500;
bool test(const char *file_path, float* result, int config = 0, int frameNum = 0, uint64_t settingConfig = 0)
{

#ifdef SDOC_NATIVE_DEBUG
	std::cout <<  getTextForEnum(config) << std::endl;

	bool output = socReplay(file_path, result, config, frameNum, settingConfig);


	std::cout << std::endl;
	return output;
#else
	std::cout << "ERROR: Please turn on SDOC_NATIVE_DEBUG" << std::endl;
#endif
	return false;

}
bool is_file_exist(std::string fileName)
{
	std::ifstream ifile;
	ifile.open(fileName);
	if (ifile) {
		return true;
	}
	else {
		return false;
	}
}



float decompressFloat(uint16_t depth)
{
    const float bias = 3.9623753e+28f; // 1.0f / floatCompressionBias

    union {
        uint32_t u;
        float f;
    } U = {uint32_t(depth) << 12};
    return  (U.f * bias);
}
static bool verify = true;

int main(int argc, char **argv)
{
	float lagestDelta = 0;
	for (int idx = 1; idx < 65535; idx++) {
		float d = decompressFloat(idx) - decompressFloat(idx - 1);
			if (d > lagestDelta) lagestDelta = d;
	}
	std::cout << "Lagest Delta " << lagestDelta << " revert " << 1 / lagestDelta << std::endl;
	std::cout << "check 65535 " << decompressFloat(65535) << std::endl;
	union w {
		int a;
		char b;
	}c;
	c.a = 1;
	if (c.b == 1) std::cout << "little" << std::endl;
	else std::cout << "big" << std::endl;


    
	
	frameCount = 1;


	std::vector<std::string> allCaptures;

	bool configManual = true;
	
    std::string developerGoldData = "../../GOLDEN_DATA/";
#ifdef SDOC_OSX
    developerGoldData = "../../../GOLDEN_DATA/";
#ifdef __aarch64__
    developerGoldData = "../../../GOLDEN_DATA/";
#endif
    
#endif
	bool quickCompare = true;
	if (configManual) {

		if (quickCompare)
		{
			developerGoldData += "all//";
			allCaptures.push_back("XYZDegenerate.cap");
			allCaptures.push_back("SuntempSlope.cap");
			allCaptures.push_back("SuntempleWindowBug.cap");
			allCaptures.push_back("SuntempleView.cap");
			allCaptures.push_back("SunTempleUnhandledMirrorBug.cap");
			allCaptures.push_back("SuntempleStatue.cap");
			allCaptures.push_back("SuntemplePackNearClipBugUnrollPartialQuad.cap");
			allCaptures.push_back("SuntempleOccludeeNeedMaxClampBug.cap");
			allCaptures.push_back("SuntempleLargeSlope.cap");
			allCaptures.push_back("SuntempleFloorBug.cap");
			allCaptures.push_back("SuntempleCeil.cap");
			allCaptures.push_back("SuntempleBug2.cap");
			allCaptures.push_back("Suntemple1.cap");
			allCaptures.push_back("SunPlane.cap");
			allCaptures.push_back("SunNearClip.cap");
			allCaptures.push_back("SunInterleaveBug.cap");
			allCaptures.push_back("SunBug.cap");
			allCaptures.push_back("SunBoundaryWall.cap");
			allCaptures.push_back("QSceneFull.cap");
			
		}
		else 
		{
			allCaptures.push_back("QSceneFull.cap");
		}

	}
	else 
	{
		developerGoldData = "";
		allCaptures.push_back(std::string(argv[1]));
	}

	bool runCompare = true;
	if(runCompare)
	{

		int round = 1;
#ifdef SDOC_STRESS_TEST
		round = 200000;
#endif
		std::vector<int> algos;
		if (quickCompare) 
		{
#ifdef SDOC_OSX
        //    algos.push_back(2);
#else
#endif            
			//algos.push_back(2);
			//algos.push_back(3);
			int compressMode = 0;
			//algos.push_back(compressMode * 16 + 3); //pure triangle approach
			int renderMode = 0;
			int interleave = 0;

			//algos.push_back( (interleave << 8) + renderMode * 2048 + compressMode * 16 + 2);
			compressMode = 0;
			algos.push_back((interleave << 8) + renderMode * 2048 + compressMode * 16 + 3);
			compressMode = 1;
			algos.push_back( (interleave << 8) + renderMode * 2048 + compressMode * 16 + 4);


		}
		
		std::vector<std::string> allResults;
		std::vector<float> allTimes;
		float results[10];
		int totalAlgo = (int)algos.size();

		for (uint64_t roundIdx = 0; roundIdx < round; roundIdx++)
		{
			uint64_t rIdx = 1;
			for (auto cap : allCaptures)
			{
				std::cout << "Round " << rIdx << std::endl;
				std::cout << "Replay Target: " << cap.c_str() << std::endl;

				for(auto inputConfig: algos)
				{
					auto inputCap = developerGoldData + cap;
                    std::cout<<inputCap<<std::endl;
					if(quickCompare)
					{
					//	quickVerify(inputCap.c_str());
					}
					bool useReplayer = false;
#if !defined(SDOC_NATIVE_DEBUG)
					useReplayer = true;
#endif
					uint64_t replaySetting = 0; 
					//SOC settings ******************************************
					// on off interleave mode, 2 is coherent fast, 1 is coherent

					uint64_t dumpDrawCall = 0; //set to 1 to dump per draw depth map
					uint64_t focusDraw = -1;   //ignore all other draw calls, only submit the selected draw calls
					
					int approach = inputConfig & 7;
					//focusDraw = 13;
					if (focusDraw >= 0) {
						dumpDrawCall = 0;
					}

					uint64_t compressMode = (inputConfig >> 4) & 1;
					uint64_t renderMode = (inputConfig >> 11) & 7;

					std::cout << "RenderMode " << renderMode << std::endl;
					uint64_t interleave = (inputConfig >> 8) & 3;;   // on off interleave mode

					uint64_t CW = (inputConfig & 1024) / 1024;
					replaySetting = ((focusDraw+1) << 16) 
						| (CW << 60)
						| (renderMode << 9)
						| (compressMode << 8)
						|(dumpDrawCall << 3)  | interleave;
					replaySetting |= rIdx << 32;
					if (useReplayer) {
						std::cout << "Android Demo Replay approach "  << std::endl;
						auto result = TestSDOC(developerGoldData, cap);
						std::cout << "Android Demo Replay approach " << result.c_str() << std::endl;
						break;
					}
					else if (test(inputCap.c_str(), results, 1 << approach, frameCount, replaySetting))
					{
						if (round == 1) {
							float currentTime = results[0];
							allTimes.push_back(currentTime);
							std::string output = inputCap + " " + getTextForEnum(1 << approach);
							output += "    Time " + std::to_string(results[0]) + " Query " + std::to_string((int)results[1]) + "/" + std::to_string((int)results[2]) + " occluderCulled " + std::to_string((int)results[3]) + "/" + std::to_string((int)results[4]);
							//stress memory test, no need to store the results
						
								allResults.push_back(output);
							if (approach <=6) {

								std::cout << " Compare with "<< getTextForEnum(1 << (algos[0] & 7)) <<" Ratio " << allTimes[(allTimes.size()-1) / totalAlgo * totalAlgo] / currentTime << std::endl;
								std::cout << std::endl;
								std::cout << std::endl;
							}
						}
					}
					else {

						std::cout << "Fail. Please Check " << std::endl;	
						return -1;
					}
					if (dumpDrawCall > 0) 
					{
#if defined(SDOC_WIN)
						system("pause");
#endif
						return 0;
					}
				}
			}
			if (quickCompare) {
				int idx = 0;
				for (auto s : allResults)
				{
					if (idx % totalAlgo == 0) {

						std::cout << s << std::endl;
					}
					else
					{
						float current = allTimes[idx];
						float ref = allTimes[idx / totalAlgo * totalAlgo];
						std::cout << s << " Ratio " << ref / current << std::endl;
					}
					idx++;
				}

				allResults.clear();
			}
		}
	}

#if defined(SDOC_WIN)
	system("pause");
#endif
	return 0;
}

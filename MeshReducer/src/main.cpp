//============================================================================================================
//
//
//                  Copyright (c) 2023, Qualcomm Innovation Center, Inc. All rights reserved.
//                              SPDX-License-Identifier: BSD-3-Clause
//
//============================================================================================================



#include <algorithm>
#include <vector>
#include <string>
#include "MeshReducer.h"
#include <chrono>
#include <iostream>

#include <Windows.h>
#include <string>
#include <io.h>

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <ctime>
#include <iostream>
#include <chrono>




std::string GetCurrentDirectory()
{
	char buffer[MAX_PATH];
	GetModuleFileNameA(NULL, buffer, MAX_PATH);
	std::string::size_type pos = std::string(buffer).find_last_of("\\/");

	return std::string(buffer).substr(0, pos);
}


//add here to dump model and simplified model
static bool storeModel(const std::string& file_path,
	const float* vertices, int nVert,
	const uint16_t* indices, int indicesNum)
{

	//LOGI("save model %s", file_path.c_str());
	FILE* fileWriter = fopen(file_path.c_str(), "w");
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


bool ReduceMesh(float* vertices, unsigned short* indices, unsigned int& nVert, unsigned int& nIdx, unsigned int targetFaceNum,  bool saveModel, std::string outFile)
{

	int reducedFaceNum = 0;
	int reducedVertNum = 0;

	auto startTime = std::chrono::high_resolution_clock::now();

	bool bValid = common::MeshReducer::reduce(vertices, indices, nVert, nIdx,
		vertices, reducedVertNum, indices, reducedFaceNum, targetFaceNum);
	std::cout << "Time(Millisecond) " << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - startTime).count() << std::endl;

	if (bValid)
	{
		nVert = reducedVertNum;
		nIdx = reducedFaceNum * 3;
		if (saveModel)
		{
			storeModel(outFile, vertices, nVert, indices, nIdx);
		}
	}
	return bValid;
}


static void SimplifyObj(std::string path, std::string pathOut)
{
	std::vector< uint16_t > vertexIndices;
	std::vector< float > temp_vertices;

	FILE* file = fopen(path.c_str(), "r");
	if (file == nullptr)
	{
		return;
	}


	char lineHeader[128];
	// read the first word of the line
	int res = fscanf(file, "%s", lineHeader);
	unsigned int nVert;
	int nFaces;
	int mode;
	fscanf(file, "%d %d %d\n", &nVert, &nFaces, &mode);
	temp_vertices.reserve(3 * nVert);
	for (int idx = 0; idx < nVert; idx++) {
		float x, y, z;
		fscanf(file, "%f %f %f\n", &x, &y, &z);
		temp_vertices.push_back(x);
		temp_vertices.push_back(y);
		temp_vertices.push_back(z);
	}

	for (int idx = 0; idx < nFaces; idx++) {
		int a, x, y, z;
		fscanf(file, "%d %d %d %d\n", &a, &x, &y, &z);
		vertexIndices.push_back(x);
		vertexIndices.push_back(y);
		vertexIndices.push_back(z);
	}
	
	uint32_t nIdx = (uint32_t)vertexIndices.size();
	if (nIdx > 6000) // > 2000 faces 
	{
		ReduceMesh(temp_vertices.data(), vertexIndices.data(), nVert, nIdx, nIdx / 8, true, pathOut);
	}
	
}



int main() 
{
	SimplifyObj(GetCurrentDirectory() + "\\..\\TestData\\SunTempleStatue.OFF", GetCurrentDirectory() + "\\..\\TestData\\SunTempleStatueOut.OFF");
	
	return 0;
}
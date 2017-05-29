// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
  * Copyright (C)2017  Department of Neuroscience - University of Lethbridge
  * Author:Matt Tata, Marko Ilievski
  * email: m.ilievski@uleth.ca, matthew.tata@uleth.ca, francesco.rea@iit.it
  * Permission is granted to copy, distribute, and/or modify this program
  * under the terms of the GNU General Public License, version 2 or any
  * later version published by the Free Software Foundation.
  *
  * A copy of the license can be found at
  * http://www.robotcub.org/icub/license/gpl.txt
  *
  * This program is distributed in the hope that it will be useful, but
  * WITHOUT ANY WARRANTY; without even the implied warranty of
  * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
  * Public License for more details
*/
/**
 * @file AudioPreprocesserModule.cpp
 * @brief Implementation of the processing module
 */

#include "AudioPreprocesserModule.h"
#define NUM_FRAME_SAMPLES 4096 //get this into resource finder!!!!



using namespace yarp::os;

AudioPreprocesserModule::AudioPreprocesserModule()
{
    yDebug("AudioProcesserModule");
    //Colouring to match the colour code of yarp
	myerror = "\033[0;31m";
	myinfo = "\033[0;32m";
	mywarn = "\033[0;33m";
	myreset = "\033[0m";







}

AudioPreprocesserModule::~AudioPreprocesserModule()
{
	delete gammatoneAudioFilter;
	delete beamForm;
	delete rawAudio;

}

bool AudioPreprocesserModule::configure(yarp::os::ResourceFinder &rf)
{
    yInfo("Configuring the module");
	inPort = new yarp::os::BufferedPort<yarp::sig::Sound>();
	//inPort = new yarp::os::BufferedPort<audio::Sound>();
	inPort->open("/iCubAudioAttention/Preprocesser:i");

	outPort = new yarp::os::Port();
	outPort->open("/iCubAudioAttention/Preprocesser:o");

	outGammaToneFilteredAudioPort = new yarp::os::Port();
	outGammaToneFilteredAudioPort->open("/iCubAudioAttention/GammaToneFilteredAudio:o");

	if (yarp::os::Network::exists("/iCubAudioAttention/Preprocesser:i"))
	{
		if (yarp::os::Network::connect("/sender", "/iCubAudioAttention/Preprocesser:i") == false)
		{
			std::cout << myerror << "[ERROR]" << myreset << " Could not make connection to /sender. Exiting.\n";
			yError("Could not make connection to /sender. Exiting");
			return false;
		}
	}
	else
	{
		return false;
	}


   	//Set the file which the module uses to grab the config information
    yInfo("loading configuration file");
	fileName = "../../src/Configuration/loadFile.xml";
	//calls the parser and the config file to configure the needed variables in this class
	loadFile(rf);
    yInfo("file successfully load");
    
    //preparing GammatoneFilter and beamForming
	gammatoneAudioFilter = new GammatoneFilter(samplingRate, lowCf, highCf, nBands, frameSamples, nMics, false, false);
	beamForm = new BeamFormer(nBands, frameSamples, nMics, nBeamsPerHemi);

    // preparing other memory structures
    rawAudio = new float[(frameSamples * nMics)];
	oldtime = 0;

	for (int i = 0; i < interpellateNSamples * 2; i++) {
		std::vector<double> tempvector;
		for (int j = 0; j < nBands; j++) {
			tempvector.push_back(0);
		}
		highResolutionAudioMap.push_back(tempvector);
	}

	outAudioMap = new yarp::sig::Matrix(nBands, interpellateNSamples * 2);
	outGammaToneFilteredAudioMap = new yarp::sig::Matrix(nBands*2, frameSamples);
	createMemoryMappedFile();
    
    /* create the thread and pass pointers to the module parameters */
	apr = new AudioPreprocesserRatethread(robotName, configFile);
	apr->setName(getName().c_str());
    //rThread->setInputPortName(inputPortName.c_str());
    
    /* now start the thread to do the work */
    //apr->start(); // this calls threadInit() and it if returns true, it then calls run()
    
	return true;
}

double AudioPreprocesserModule::getPeriod()
{
	// TODO Should this all ways stay this low
	return 0.05;
}

bool AudioPreprocesserModule::updateModule()
{

	s = inPort->read(true);

	inPort->getEnvelope(ts);

	if (ts.getCount() != lastframe + 1) {
		
	}

	int row = 0;
	for (int col = 0 ; col < frameSamples; col++) {
		yarp::os::NetInt32 temp_c = (yarp::os::NetInt32) s->get(col, 0);
		yarp::os::NetInt32 temp_d = (yarp::os::NetInt32) s->get(col, 1);
		rawAudio[row]   = (float) temp_c / normDivid;
		rawAudio[row + 1]	= (float) temp_d / normDivid;
		//printf("%d   %f",temp_c,rawAudio[row]);
		row += 2;
	}

	memoryMapperRawAudio();

	gammatoneAudioFilter->gammatoneFilterBank(rawAudio);
	beamForm->inputAudio(gammatoneAudioFilter->getFilteredAudio());

	memoryMapperGammaToneFilteredAudio(gammatoneAudioFilter->getFilteredAudio());
	sendGammatoneFilteredAudio(gammatoneAudioFilter->getFilteredAudio());
	outGammaToneFilteredAudioPort->setEnvelope(ts);
	outGammaToneFilteredAudioPort->write(*outGammaToneFilteredAudioMap);

	reducedBeamFormedAudioVector = beamForm->getReducedBeamAudio();
	
	spineInterp();

	memoryMapper();
	sendAudioMap();
	outPort->setEnvelope(ts);
	outPort->write(*outAudioMap);

	//Timing how long the module took
	lastframe = ts.getCount();
	stopTime=Time::now();
	yInfo("elapsed time = %f\n",stopTime-startTime);
	startTime=stopTime;
	return true;
}

bool AudioPreprocesserModule::interruptModule()
{
	fprintf(stderr, "[WARN] Interrupting\n");
	return true;
}

bool AudioPreprocesserModule::close()
{
	fprintf(stderr, "[INFO] Calling close\n");
	return true;
}

void AudioPreprocesserModule::createMemoryMappedFile()
{
	int memoryMapSize = ((nBands * (interpellateNSamples + 1)) * 2 + 2);
	double initializationArray [memoryMapSize];
	fid = fopen("/tmp/preprocessedAudioMap.tmp", "w");
	fwrite(initializationArray, sizeof(double), sizeof(initializationArray), fid);
	fclose(fid);
	mappedFileID = open("/tmp/preprocessedAudioMap.tmp", O_RDWR);
	mappedAudioData = (double *)mmap(0, (sizeof(initializationArray)), PROT_WRITE, MAP_SHARED , mappedFileID, 0);

	int memoryMapRawAudio = (4*NUM_FRAME_SAMPLES);
	double initializationRawAudioArray[4*NUM_FRAME_SAMPLES];
	rawFid = fopen("/tmp/preprocessedRawAudio.tmp", "w");
	fwrite(initializationRawAudioArray, sizeof(double), sizeof(initializationRawAudioArray), rawFid);
	fclose(rawFid);
	mappedRawAduioFileID = open("/tmp/preprocessedRawAudio.tmp", O_RDWR);
	mappedRawAduioData = (double *)mmap(0, (sizeof(initializationRawAudioArray)), PROT_WRITE, MAP_SHARED , mappedRawAduioFileID, 0);

	int memoryMapGammaToneFilteredAudio = ((nBands*2)*NUM_FRAME_SAMPLES);
	double *initializationGammaToneFilteredAudioArray = new double[memoryMapGammaToneFilteredAudio];
	gammaToneFilteredFid = fopen("/tmp/GammaToneFilteredAudio.tmp", "w");
	std::cerr << memoryMapGammaToneFilteredAudio << std::endl;
	fwrite(initializationGammaToneFilteredAudioArray, sizeof(double), memoryMapGammaToneFilteredAudio*8, gammaToneFilteredFid);
	fclose(gammaToneFilteredFid);
	mappedGammaToneFilteredAduioFileID = open("/tmp/GammaToneFilteredAudio.tmp", O_RDWR);
	mappedGammaToneFilteredAduioData = (double *)mmap(0, memoryMapGammaToneFilteredAudio*8, PROT_WRITE, MAP_SHARED , mappedGammaToneFilteredAduioFileID, 0);
}

void AudioPreprocesserModule::loadFile(yarp::os::ResourceFinder &rf)
{
	ConfigParser *confPars;
	try {
		confPars = ConfigParser::getInstance(fileName);
		Config pars = (confPars->getConfig("default"));
		frameSamples = pars.getFrameSamples();
		nBands = pars.getNBands();
		nMics = pars.getNMics();
		interpellateNSamples = pars.getInterpellateNSamples();
		totalBeams = pars.getNBeamsPerHemifield() * 2 + 1;

        int _frameSamples  = rf.check("frameSamples", 
                           Value("4096"), 
                           "frame samples (int)").asInt();
        int _nBands  = rf.check("nBands", 
                           Value("128"), 
                           "numberBands (int)").asInt();
        int _nMics  = rf.check("nMics", 
                           Value("2"), 
                           "number mics (int)").asInt();
        int _interpellateNSamples  = rf.check("interpellateNSamples", 
                           Value("180"), 
                           "interpellate N samples (int)").asInt();
        double _micDistance = rf.check("micDistance", 
                           Value("0.145"), 
                           "micDistance (double)").asDouble();
        int _C = rf.check("C", 
                           Value("338"), 
                           "C speed of sound (int)").asInt();
        samplingRate = rf.find("samplingRate").asInt();
        lowCf = rf.find("lowCf").asInt();
		highCf = rf.find("highCf").asInt();
		longBufferSize = rf.find("longBufferSize").asInt();

        nBeamsPerHemi  = (int)((_micDistance / _C) * samplingRate) - 1;
        yInfo("_beamsPerHemi %d = %f / %d * %d", nBeamsPerHemi, _micDistance, _C, samplingRate);
        int _totalBeams = nBeamsPerHemi * 2 + 1;
        yInfo("frameSamples = %d, %d", frameSamples, _frameSamples);
        yInfo("nBands = %d, %d", nBands, _nBands);
        yInfo("nMics = %d, %d", nMics, _nMics);
        yInfo("interpellateNSamples = %d, %d", interpellateNSamples, _interpellateNSamples);
		yInfo("total beams = %d, %d",totalBeams, _totalBeams);
        //Time::delay(5.0);
	}
	catch (int a) {
        yError("Error in the loading of file");
	}


}

void AudioPreprocesserModule::memoryMapper()
{
	mappedAudioData[0] = ts.getCount();
	mappedAudioData[1] = ts.getTime();
	int count = 0;
	//printf("inpterellateNSamples = %d",interpellateNSamples);
	for (int i = 0; i < interpellateNSamples * 2; i++)
	{
		for (int j = 0; j < nBands; j++)
		{
			mappedAudioData[(count++) + 2] = highResolutionAudioMap[i][j];
		}
	}

}
void AudioPreprocesserModule::memoryMapperRawAudio()
{
	int currentCounter = ts.getCount();
	double currentTime = ts.getTime();
	int row = 0;
	int j = 0;
  	for (int col = 0 ; col < frameSamples; col+=1) {

		mappedRawAduioData[j] = 	(double)rawAudio[row];
		mappedRawAduioData[j+1] = 	(double)rawAudio[row+1];
		mappedRawAduioData[j+2] = 	(double)(currentCounter * frameSamples) + col;
    	mappedRawAduioData[j+3]	= 	(double)(currentTime + col * (1.0 / 48000));
    	row += 2;
    	j +=4;
  	}

}


void AudioPreprocesserModule::memoryMapperGammaToneFilteredAudio(const std::vector<float*> gammatoneAudio){

	for (int i = 0; i < nBands; i++)
	{
		for (int j = 0; j < frameSamples; j++)
		{
			mappedGammaToneFilteredAduioData[(i*frameSamples)+j] = (double)gammatoneAudio[i][j];
		}
	}
	int visted = (nBands*frameSamples);
	for (int i = 0; i < nBands; i++)
	{
		for (int j = 0; j < frameSamples; j++)
		{
			mappedGammaToneFilteredAduioData[(i*frameSamples)+j+visted] = (double)gammatoneAudio[i + nBands][j];
		}
	}


}

void AudioPreprocesserModule::sendGammatoneFilteredAudio(const std::vector<float*> &gammatoneAudio){
	for (int i = 0; i < nBands; i++)
	{
		yarp::sig::Vector tempV(frameSamples);
		for (int j = 0; j < frameSamples; j++)
		{
			tempV[j] = gammatoneAudio[i][j];
		}
		outGammaToneFilteredAudioMap->setRow(i, tempV);
	}
	for (int i = 0; i < nBands; i++)
	{
		yarp::sig::Vector tempV(frameSamples);
		for (int j = 0; j < frameSamples; j++)
		{
			tempV[j] = gammatoneAudio[i+nBands][j];
		}
		outGammaToneFilteredAudioMap->setRow(i+nBands, tempV);
	}

}

void AudioPreprocesserModule::sendAudioMap()
{
	for (int i = 0; i < nBands; i++)
	{
		yarp::sig::Vector tempV(interpellateNSamples * 2);
		for (int j = 0; j < interpellateNSamples * 2; j++)
		{
			tempV[j] = highResolutionAudioMap[j][i];
		}
		outAudioMap->setRow(i, tempV);
	}

}

void AudioPreprocesserModule::spineInterp()
{
	double offset = (interpellateNSamples / (double)totalBeams);
	for (int i = 0; i < nBands; i++)
	{
		int k = 0;
		double curroffset = 0;
		for (int j = 0; j < interpellateNSamples; j++)
		{
			if (j == (int)curroffset && k < (totalBeams - 1))
			{
				curroffset += offset;
				k++;
			}
			highResolutionAudioMap[j][i] = interpl(j, curroffset - offset, curroffset , reducedBeamFormedAudioVector[k - 1][i], reducedBeamFormedAudioVector[k][i]);
		}
		curroffset = interpellateNSamples;
		for (int j = interpellateNSamples; j < interpellateNSamples * 2; j++)
		{
			if (j == (int)curroffset && k > 1)
			{
				if (j != interpellateNSamples)
					k--;
				curroffset += offset;
			}
			highResolutionAudioMap[j][i] = interpl(j, curroffset - offset, curroffset, reducedBeamFormedAudioVector[k][i], reducedBeamFormedAudioVector[k - 1][i]);
		}
	}

}

double AudioPreprocesserModule::interpl(int x, int x1, int x2, double y1, double y2)
{
	return y1 + ((y2 - y1) * (x - x1)) / (x2 - x1);
}

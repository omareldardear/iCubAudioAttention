// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2018 Department of Neuroscience - University of Lethbridge
 * Author: Austin Kothig, Francesco Rea, Marko Ilievski, Matt Tata
 * email: kothiga@uleth.ca, francesco.reak@iit.it, marko.ilievski@uwaterloo.ca, matthew.tata@uleth.ca
 * 
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

/* ===========================================================================
 * @file  gammatoneFilterBank.h
 * @brief Object for processing raw audio into a bank of frequency 
 *         bands using a 4th order gammatone filter.
 * =========================================================================== */

#ifndef _GAMMATONE_FILTER_BANK_H_
#define _GAMMATONE_FILTER_BANK_H_

#define BW_CORRECTION      1.0190
#define VERY_SMALL_NUMBER  1e-100

#include <yarp/math/Math.h>
#include <yarp/sig/all.h>
#include <yarp/os/all.h>

#ifdef WITH_OMP
#include <omp.h>
#endif

class GammatoneFilterBank { 

  private:
    
	/* ===========================================================================
	 *  Variables set in the constructor. Should not be changed for life of object.
	 * =========================================================================== */
    int    numMics;
	int    samplingRate;
	int    numFrameSamples;
	int    numBands;
	double lowCf;
	double highCf;
	bool   halfRec;
	bool   erbSpaced;

	yarp::sig::Vector cfs;
	yarp::sig::Matrix _proxyBasilarMembrane;
	yarp::sig::Matrix _proxyEnvelope;
	yarp::sig::Matrix _proxyPhase;
	double            tpt;                  //-- 2 pi / samp rate.
	const double      _pi  = 2 * acos(0.0); //-- High precision pi.
	const double      _ln2 = log(2);        //-- High precision natural log of 2.


  public:

	/* ===========================================================================
	 *  Main Constructor.
	 * 
	 * @param mics    : Number of Microphones.
	 * @param samples : Number of Samples Recorded per Second.
	 * @param frames  : Number of Samples in incoming Frame.
     * @param bands   : Number of Frequency Bands.
     * @param lcf     : Lowest Center Frequency.
     * @param hcf     : Highest Center Frequency.
     * @param hrec    : Enable Half-Wave Rectifying.
     * @param erbs    : Enable ERB Spaced Center Frequencies.
	 * =========================================================================== */
	GammatoneFilterBank(int mics, int samples, int frames, int bands, double lcf, double hcf, bool hrec, bool erbs);


	/* ===========================================================================
	 *  Destructor.
	 * =========================================================================== */
	~GammatoneFilterBank();


    /* ===========================================================================
	 *  Apply a 4th order gammatone filter onto the provided raw audio. 
	 *  Including additional matracies in the input will call similar 
	 *    implimentations to return the Envelope, as well as the Phase
	 *    for each band in the bank.
	 * 
	 * @param RawAudio     : Raw audio (number of mics, number of samples).
     * @param FilterBank   : Target for the filtered audio (number of mics * number of bands, number of samples).
	 * @param EnvelopeBank : [OPTIONAL] Target for a Hilbert Envelope of each band (number of mics * number of bands, number of samples).
	 * @param PhaseBank    : [OPTIONAL] Target for the phase of each band (number of mics * number of bands, number of samples).
	 * =========================================================================== */
    void getGammatoneFilteredAudio(const yarp::sig::Matrix& RawAudio, yarp::sig::Matrix& FilterBank);
	void getGammatoneFilteredAudio(const yarp::sig::Matrix& RawAudio, yarp::sig::Matrix& FilterBank, yarp::sig::Matrix& EnvelopeBank);
	void getGammatoneFilteredAudio(const yarp::sig::Matrix& RawAudio, yarp::sig::Matrix& FilterBank, yarp::sig::Matrix& EnvelopeBank, yarp::sig::Matrix& PhaseBank);


    /* ===========================================================================
	 *  Find the total RMS power for each band across samples.
	 * 
	 * @param FilterBank : The filtered audio (number of mics * number of bands, number of samples).
	 * @param BankPower  : Target for the power of each band (number of bands, number of mics).
	 * =========================================================================== */
    void getGammatoneFilteredPower(const yarp::sig::Matrix& FilterBank, yarp::sig::Matrix& BankPower);


	/* ===========================================================================
	 *  Apply a band pass on a bank of audio, for some specified frequency band.
	 * 
	 * @param AudioBank      : Source for the audio to be filtered (number of mics * number of bands, number of samples).
	 * @param BandPassedBank : Target for the band pass (number of mics * number of bands, number of samples).
	 * @param BandFreq       : The desired frequency band to be isolated.
	 * =========================================================================== */
	void getBandPassedAudio(const yarp::sig::Matrix& AudioBank, yarp::sig::Matrix& BandPassedBank, const double CenterFrequency);


  private:
	
	/* ===========================================================================
	 *  Apply a single 4th order gammatone filter onto the raw audio for the 
	 *    specified center frequency. If enabled will also return the 
	 *    Hilbert Envelope, and the Instantaneous Phase of the band.
	 * 
	 * @param RawAudio        : Source for the input audio (number of samples).
	 * @param BasilarMembrane : Target for the Basilar Membrane Response (number of samples).
	 * @param Envelope        : Target for the Hilbert Envelope (number of samples).
	 * @param Phase           : Target for the Instantaneous Phase (number of samples).
	 * @param CenterFrequency : The Frequency that will be filtered for.
	 * @param IncludeEnvelope : Boolean value for if the envelope should be calculated.
	 * @param IncludePhase    : Boolean value for if the phase should be calculated.
	 * =========================================================================== */
	void singleGammatoneFilter(const double* RawAudio, double* BasilarMembrane, double* Envelope, const double CenterFrequency, const bool IncludeEnvelope);
	

	/* ===========================================================================
	 *  Apply a single band pass filter onto the provided audio for the
	 *    specified band frequency.
	 * 
	 * @param Audio    : Source for the input audio (number of samples).
	 * @param BandPass : Target for the Band Passed audio (number of samples).
	 * @param BandFreq : The desired frequency band to be isolated.
	 * =========================================================================== */
	void singleBandPass(const double* Audio, double* BandPass, const double CenterFrequency);


	/* ===========================================================================
	 *  Fill the vector cfs with erb spaced center frequencies.
	 * =========================================================================== */
	void makeErbCFs();


	/* ===========================================================================
	 *  Fill the vector cfs with linear spaced center frequencies.
	 * =========================================================================== */
	void makeLinearCFs();

	
	/* ===========================================================================
	 *  Macro for converting Frequency to Equivalent Rectangular Bandwidth.
	 * =========================================================================== */
	double HzToErb(double Hz);


	/* ===========================================================================
	 *  Macro for converting Frequency to Equivalent Rectangular Bandwidth Rate.
	 * =========================================================================== */
	double HzToErbRate(double Hz);


	/* ===========================================================================
	 *  Macro for converting Equivalent Rectangular Bandwidth Rate to Frequency.
	 * =========================================================================== */
	double ErbRateToHz(double Erb);	
};

#endif //_GAMMATONE_FILTER_BANK_H_

//----- end-of-file --- ( next line intentionally left blank ) ------------------

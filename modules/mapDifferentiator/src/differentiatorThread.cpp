// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
/*
  * Copyright (C) 2021  Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
  * Author: Omar Eldardeer
  * email: omar.eldardeer@iit.it
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

#include "iCub/differentiatorThread.h"

#define THPERIOD 0.01

differentiatorThread::differentiatorThread(string moduleName):PeriodicThread(THPERIOD){

    //initialize names
    this->moduleName = moduleName;
    inputCurrentMatrixName = getName("/inputCurrentMatrix:i");
    outputDifferentiatedMatrixPortName = getName("/outputDifferentiatedMatrix:o");
    matInitialized = false;

}

differentiatorThread::~differentiatorThread(){

}


bool differentiatorThread::configure(yarp::os::ResourceFinder &rf){

    return true;
}


bool differentiatorThread::threadInit() {

    /* ===========================================================================
	 *  Initialize all ports. If any fail to open, return false to
	 *    let RFModule know initialization was unsuccessful.
	 * =========================================================================== */

    if (!inputCurrentMatrixPort.open(inputCurrentMatrixName)) {
        yError("Unable to open %s port ",inputCurrentMatrixName.c_str());
        return false;
    }
    if (!outputDifferentiatedMatrixPort.open(outputDifferentiatedMatrixPortName)) {
        yError("Unable to open %s port ",outputDifferentiatedMatrixPortName.c_str());
        return false;
    }

    yInfo("Initialization of the processing thread correctly ended.");

    return true;
}
void differentiatorThread::run() {

    if (inputCurrentMatrixPort.getInputCount()) {


        //-- Get Input.
        currentMatrix = *inputCurrentMatrixPort.read(true);

        if(!matInitialized){
            oldMatrix = currentMatrix;
            matInitialized = true;
            differentiatedMatrix.resize(oldMatrix.rows(), oldMatrix.cols());
            differentiatedMatrix.zero();
        }
        bool result = performTheSubtraction();

        if(result){
            publishPorts();
        }
   }

}

void differentiatorThread::threadRelease() {

    //-- Stop all threads.
    inputCurrentMatrixPort.interrupt();
    outputDifferentiatedMatrixPort.interrupt();

    //-- Close the threads.
    inputCurrentMatrixPort.close();
    outputDifferentiatedMatrixPort.close();


}


string differentiatorThread::getName(const char* p) const{
    string str(moduleName);
    str.append(p);
    return str;
}

bool differentiatorThread::performTheSubtraction() {
    if(!(oldMatrix.rows() == currentMatrix.rows() && oldMatrix.cols() == currentMatrix.cols())){
        yError(" Un equal matrices size ");
        return false;
    }

    for (int row = 0; row < oldMatrix.rows(); row++) {
        for (int col = 0; col < oldMatrix.cols(); col++) {
            differentiatedMatrix[row][col] = currentMatrix[row][col] - oldMatrix[row][col];
        }
    }
    return true;
}

void differentiatorThread::publishPorts() {
    if (outputDifferentiatedMatrixPort.getOutputCount()) {
        outputDifferentiatedMatrixPort.prepare() = differentiatedMatrix;
        outputDifferentiatedMatrixPort.write();
    }

}



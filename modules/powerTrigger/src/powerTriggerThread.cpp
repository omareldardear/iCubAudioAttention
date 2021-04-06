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

#include "iCub/powerTriggerThread.h"
#define THPERIOD 0.01   
powerTriggerThread::powerTriggerThread(string moduleName):PeriodicThread(THPERIOD){

    /* ===========================================================================
     *  initialize the names
     * =========================================================================== */
    this->moduleName = moduleName;

    rawPowerInPortName =  getName("/rawPower:i");
    instantStateOutPortName =  getName("/instantState:o");
    bufferedStateOutPortName =  getName("/bufferedState:o");
    eventStateOutPortName =  getName("/eventState:o");
    stateVisualizerPortName = getName("/visualizer:o");

    /* ===========================================================================
	 *  initialize the variables
	 * =========================================================================== */
    rawPowerCurrentTotal = 0;
    currentState = false;
    previousState = false;
    bufferedState = false;
    previousBufferedState = false;

    resetBufferTime = 0;
}

powerTriggerThread::~powerTriggerThread(){

}


string powerTriggerThread::getName(const char* p) const{
    string str(moduleName);
    str.append(p);
    return str;
}


bool powerTriggerThread::configure(yarp::os::ResourceFinder &rf){

    positiveThreshold = rf.findGroup("powerRanges").check("positiveThreshold",yarp::os::Value(true),"positive audio motor action threshold  (boolean)").asFloat64();
    negativeThreshold = rf.findGroup("powerRanges").check("negativeThreshold",yarp::os::Value(false),"negative audio motor action threshold  (boolean)").asFloat64();
    onPowerThreshold  = rf.findGroup("DecisionMaking").check("audioPowerThreshold", yarp::os::Value(1.25), "threshold of the power of the audio for triggering the action(float)").asFloat64();


    yInfo( "\t                [powerRanges]                 "                               );
    yInfo( "\t ============================================ "                               );
    yInfo( "\t positive audio action threshold : %.3f",       positiveThreshold             );
    yInfo( "\t negative audio action threshold : %.3f",       negativeThreshold             );
    yInfo("\t audio on threshold              : %.3f",        onPowerThreshold              );
    yInfo( " " );



    triggerResetDelay = rf.findGroup("bufferedParams").check("triggerResetDelay",    yarp::os::Value(0.5), "trigger Reset Delay from the decreasing point (float)").asFloat64();
    yInfo( "\t              [bufferedParams]                "                                    );
    yInfo( "\t ============================================ "                                    );
    yInfo( "\t triggerResetDelay              : %.4f seconds",            triggerResetDelay      );


    return true;
}

void powerTriggerThread::run() {

    if (rawPowerInPort.getInputCount()) {
        //-- Get Input.
        rawPowerMatrix = *rawPowerInPort.read(true);
        rawPowerCurrentTotal = rawPowerMatrix[0][0] +  rawPowerMatrix[1][0];
    }

    if(rawPowerCurrentTotal >=  onPowerThreshold){
        currentState = true;
        bufferedState = true;
        resetBufferTime =  Time::now() + triggerResetDelay;
    }
    else{
        // reset state
        currentState = false;
    }
    // resetting buffered state
    if(bufferedState && resetBufferTime < Time::now()){
        bufferedState = false;
        yInfo("Reset Trigger ");
    }

    publishStateOnPorts();
    publishEventsOnPorts();
    publishVisualzation();

    previousBufferedState = bufferedState;
    previousState = currentState;

}

void powerTriggerThread::threadRelease() {

    rawPowerInPort.interrupt();
    instantStateOutPort.interrupt();
    bufferedStateOutPort.interrupt();
    eventStateOutPort.interrupt();
    stateVisualizerOutPort.interrupt();

    rawPowerInPort.close();
    instantStateOutPort.close();
    bufferedStateOutPort.close();
    eventStateOutPort.close();
    stateVisualizerOutPort.close();
 
}


bool powerTriggerThread::threadInit() {

    /* ===========================================================================
	 *  Initialize all ports. If any fail to open, return false to
	 *    let RFModule know initialization was unsuccessful.
	 * =========================================================================== */

    if (!rawPowerInPort.open(rawPowerInPortName)) {
        yError("Unable to open %s port ",rawPowerInPortName.c_str());
        return false;
    }

    if (!instantStateOutPort.open(instantStateOutPortName)) {
        yError("Unable to open %s port ",instantStateOutPortName.c_str());
        return false;
    }

    if (!bufferedStateOutPort.open(bufferedStateOutPortName)) {
        yError("Unable to open %s port ",bufferedStateOutPortName.c_str());
        return false;
    }

    if (!eventStateOutPort.open(eventStateOutPortName)) {
        yError("Unable to open %s port ", eventStateOutPortName.c_str());
        return false;
    }

    if (!stateVisualizerOutPort.open(stateVisualizerPortName)) {
        yError("Unable to open %s port ",stateVisualizerPortName.c_str());
        return false;
    }

    yInfo("Initialization of the processing thread correctly ended.");

    return true;
}

void powerTriggerThread::publishStateOnPorts() {
    if(instantStateOutPort.getOutputCount()){
        Bottle tmp;
        tmp.addInt(currentState);
        instantStateOutPort.prepare() = tmp;
        instantStateOutPort.write();
    }
    if(bufferedStateOutPort.getOutputCount()){
        Bottle tmp;
        tmp.addInt(bufferedState);
        bufferedStateOutPort.prepare() = tmp;
        bufferedStateOutPort.write();
    }
}

void powerTriggerThread::publishEventsOnPorts() {
    bool isEvent = false;
    int event = 0;
    if((currentState) && (bufferedState) && (!previousState) && (!previousBufferedState)) {
        event = TRIGGER_EVENTS(ON);
        isEvent = true;
        yInfo("ON EVENT %d",event);
    }
    else if((!currentState) && (bufferedState) && (previousState) && (previousBufferedState)) {
        event = TRIGGER_EVENTS(OFF);
        isEvent = true;
        yInfo("OFF EVENT %d",event);
    }
    else if((currentState) && (bufferedState) && (!previousState) && (previousBufferedState)) {
        event = TRIGGER_EVENTS(RETURN_ON);
        isEvent = true;
        yInfo("RETURN ON EVENT %d",event);
    }
    else if((!currentState) && (!bufferedState) && (!previousState) && (previousBufferedState)) {
        event = TRIGGER_EVENTS(OFF_BUFFERED);
        isEvent = true;
        yInfo("OFF BUFFERED EVENT %d",event);
    }

    if(isEvent && eventStateOutPort.getOutputCount()){
        Bottle tmp;
        tmp.addInt(event);
        eventStateOutPort.prepare() = tmp;
        eventStateOutPort.write();

    }

}

void powerTriggerThread::publishVisualzation() {
    if (stateVisualizerOutPort.getOutputCount()){
        stateVisualizerImg.resize(400,200);
        stateVisualizerImg.zero();
        if(currentState){
            draw::addCircle(stateVisualizerImg,
                            PixelRgb(0,255,0),
                            (int) stateVisualizerImg.width()/4,
                            (int) stateVisualizerImg.height()/2,
                            (int) stateVisualizerImg.height()/4);

        }
        else{
            draw::addCircle(stateVisualizerImg,
                            PixelRgb(255,0,0),
                            (int) stateVisualizerImg.width()/4,
                            (int) stateVisualizerImg.height()/2,
                            (int) stateVisualizerImg.height()/4);
        }


        if(bufferedState){
            draw::addCircle(stateVisualizerImg,
                            PixelRgb(0,255,0),
                            (int) stateVisualizerImg.width()/4*3,
                            (int) stateVisualizerImg.height()/2,
                            (int) stateVisualizerImg.height()/4);

        }
        else{
            draw::addCircle(stateVisualizerImg,
                            PixelRgb(255,0,0),
                            (int) stateVisualizerImg.width()/4*3,
                            (int) stateVisualizerImg.height()/2,
                            (int) stateVisualizerImg.height()/4);

        }

        yImgPixelRgb & publishedImage = stateVisualizerOutPort.prepare();
        publishedImage.copy(stateVisualizerImg);
        stateVisualizerOutPort.write();

    }

}



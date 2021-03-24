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

#ifndef ICUBAUDIOATTENTION_POWERTRIGGERTHREAD_H
#define ICUBAUDIOATTENTION_POWERTRIGGERTHREAD_H


#include <string>
#include <yarp/os/PeriodicThread.h>
#include <yarp/os/Time.h>
#include <yarp/sig/all.h>
#include <yarp/os/Thread.h>
#include <yarp/os/Network.h>
#include <yarp/os/all.h>

using namespace yarp::os;
using namespace std;

typedef yarp::sig::Matrix yMatrix;
typedef yarp::os::BufferedPort< yMatrix > yMatrixBuffer;


enum TRIGGER_EVENTS{
    ON = 0,
    OFF = 1,
    RETURN_ON = 2,
    OFF_BUFFERED = 3,
}
    
class powerTriggerThread : public PeriodicThread {

public:
    powerTriggerThread(string moduleName = "powerTrigger");
    ~powerTriggerThread();
    bool configure(yarp::os::ResourceFinder &rf);
    bool threadInit();
    void run();
    void threadRelease();
    string getName(const char* p) const;


private:


    /* ===========================================================================
	 *  Names
	 * =========================================================================== */

    string moduleName;


    string rawPowerInPortName;
    string instantStateOutPortName;
    string bufferedStateOutPortName;
    string eventStateCmdPortName;

    /* ===========================================================================
	 *  Params
	 * =========================================================================== */

    //powerRanges
    float positiveThreshold;
    float negativeThreshold;
    float onPowerThreshold;

    //bufferedParams
    float triggerResetDelay;


    /* ===========================================================================
	 *  Ports
	 * =========================================================================== */

    
    yMatrixBuffer rawPowerInPort;
    BufferedPort<Bottle> instantStateOutPort;
    BufferedPort<Bottle> bufferedStateOutPort;
    RpcClient eventStateCmdPort;


    /* ===========================================================================
	 *  Data
	 * =========================================================================== */

    yMatrix rawPowerMatrix;


    /* ===========================================================================
	 *  Variables
	 * =========================================================================== */

    double rawPowerCurrentTotal;
    bool currentState;
    bool previousState;
    bool bufferedState;
    bool previousBufferedState;

    double resetBufferTime;


    void publishStateOnPorts();
    void publishEventsOnPorts();


};



#endif //POWERTRIGGER_POWERTRIGGERTHREAD_H 


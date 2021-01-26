//
// Created by omar on 25/01/21.
//

#ifndef AUDIOATTENTION_DIFFERENTIATORTHREAD_H
#define AUDIOATTENTION_DIFFERENTIATORTHREAD_H

#include <yarp/os/all.h>
#include <yarp/sig/all.h>

typedef yarp::sig::Matrix                           yMatrix;
typedef yarp::os::BufferedPort< yMatrix           > yMatrixBuffer;

using namespace yarp::os;
using namespace std;
class differentiatorThread  : public PeriodicThread {

public:
    differentiatorThread(string moduleName = "mapDifferentiator");
    ~differentiatorThread();
    bool configure(yarp::os::ResourceFinder &rf);
    bool threadInit();
    void run();
    void threadRelease();
    string getName(const char* p) const;



private:
    //Names
    string moduleName;
    string inputCurrentMatrixName;
    string outputDifferentiatedMatrixPortName;

    // Input Ports
    yMatrixBuffer inputCurrentMatrixPort;


    // Output Ports
    yMatrixBuffer outputDifferentiatedMatrixPort;


    // Variables
    yMatrix oldMatrix;
    yMatrix currentMatrix;
    yMatrix differentiatedMatrix;

    bool matInitialized;

    bool performTheSubtraction();
    void publishPorts();



};


#endif //AUDIOATTENTION_DIFFERENTIATORTHREAD_H

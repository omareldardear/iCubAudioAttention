



%open a yarp port and read data continuosly
LoadYarp;

import yarp.Port;
import yarp.Bottle;
import yarp.BufferedPortSound;
import yarp.Sound;

bufferPort = BufferedPortSound;
bufferPort.open('/matlab/bufferListener');

port=Port;
%first close the port just in case
port.close;

port.open('/matlab/audioListner');

%yarp.connect('/sender','/matlab/listner');


b = Bottle;
s = Sound;

done=0;
for i=1:3    
    display('running');
    s = bufferPort.read(true);
    
%     s=port.read(b,true);
%    display(b.toString_c());    
 %   pause(2);
    
end


To use the code, install Platform IO on VSCode, then install the ESP32-Arduino dependencies. For guide, please refer to https://www.youtube.com/watch?v=5edPOlQQKmo.

Before using the code, please change the MAC address specifications in FlightTestOptitrack_Drone, FlightTestOptitrack_Syncer and Launcher_new.
There's a small piece of code there that you can uncomment and then read out the MAC address of the devices through serial port.

If you are using a different tracking system other than Optitrack, you only need to modify FlightTestOptitrack_Server.
If your Optitrack system cannot running at 360Hz, contact me or use FlightTestOptitrack_Server\FIRFilterDesign.m in MATLAB to design a new filter compatible with your setup.
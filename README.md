# Using PythonAPI CoppeliaSim
* We can do many simulation tasks in CoppeliaSim, but the biggest turn down is that we will have to learn a whole new language called Lua. To overcome this CoppeliaSim provides us with an API with can connect CoppeliaSim to many languages including Python, C, C++. In this blog we will be focusing on Python API. I will be showing you some basic tasks you can do using Python API.
In this blog we will be covering the basic functions of Python API and their use in simulation</br>
1)Setting Up the API</br>
2)Establishing Communication with V-Rep</br>
3)Retrieving Object Handles in python</br>
4)Setting Actuator Velocities</br>
5)Retrieving Image data from Vrep to python</br>

 
## Setting Up the Files
For Setting up the API, we should add the necessary files to our working directory that are needed for the API which can be found in the CoppeliaSim installation Folder
1)      Navigate to `CoppeliaRobotics\CoppeliaSimEdu\programming\remoteApiBindings\python\python` and copy all the .py files into the working directory
2)      Navigate to `CoppeliaRobotics\CoppeliaSimEdu\programming\remoteApiBindings\lib\lib`
        Depending on the System your are on you can select the folder (Windows, Ubuntu 16/18, MacOS)
        copy the .dll, .so or the .dylib file respectively.
 
3) The next thing we want to do is that we have to create a threaded script in any component.
4) In the Script we should add the following statement in the sysCall_threadmain() function

```python
simRemoteApi.start(19999)
```

Note : 199999 is the port that will be used for API communications, this can be defined by you.

Now you are all set for using the PythonAPI in CoppeliaSim.
## Establishing Communication
* For Establishing Communication we need to follow the following steps :
1)      import the `sim` library in the code
2)      Add the below statements to the code
```python
sim.simxFinish(-1)

clientID = sim.simxStart('127.0.0.1',19999,True,True,5000,5)
```
Note : The port being used in the statement should match the port number specified while setting up.

* For Testing the establishment of communication you can run the following code after starting the simulation in CoppeliaSim :
```python
import sim
import sys

sim.simxFinish(-1)

clientID=sim.simxStart('127.0.0.1',19999,True,True,5000,5)

if clientID != -1:
    print("Connected to the remote API server")
else:
    print("Connection not successful")
    sys.exit("Could not connect")
```
## Retrieving Object Handles in python
* Object handles can be considered as a key or an ID which a component posses. It is used to provide commands to a component(For eg. joint Velocity to a motor). PythonAPI also has an inbuild function for that:

```python
sim.simxGetObjectHandle()
```

| Parameters : | clientID: the client ID () </br> objectName: name of the object. </br>operationMode: a remote API function operation mode. Recommended operation mode for this function is `sim.simx_opmode_blocking`  |
| :--- | :--- |
|Return Values| returnCode: a remote API function return code </br> handle: the Object handle|

An Example of using this Function is :
```python
error_code,motor_handle = sim.simxGetObjectHandle(clientID,"Object Name in CoppeliaSim", sim.simx_opmode_oneshot_wait)
```
## Setting Actuator Velocities
* Setting actuator velocities is a really simple task. The following function can be used for it:

```python
sim.simxSetJointTargetVelocity()
```
| Parameters : | clientID: the client ID. refer to simxStart.</br>jointHandle: handle of the joint</br>targetVelocity: target velocity of the joint (linear or angular velocity depending on the joint-type)</br>operationMode: a remote API function operation mode. Recommended operation modes for this function are `simx_opmode_oneshot` or `simx_opmode_streaming`   |
| :--- | :--- |
|Return Values| returnCode: a remote API function return code |

An Example of using this Function is :
```python
error_Code = sim.simxSetJointTargetVelocity(clientID,motor_handle,10,sim.simx_opmode_streaming)
```
## Retrieving Image data 
* We can also retrieve image data from CoppeliaSim using the following Function:

```python
sim.simxGetVisionSensorImage()
```
| Parameters : | clientID: the client ID. refer to simxStart.</br>sensorHandle: handle of the vision sensor</br>options: image options, bit-coded:bit0 set: each image pixel is a byte (greyscale image), otherwise each image pixel is a rgb byte-triplet</br>operationMode: a remote API function operation mode. Recommended operation modes for this function are `sim.simx_opmode_streaming` |
| :--- | :--- |
|Return Values| returnCode: a remote API function return code</br>resolution: the resolution of the image (x,y)<br/>image: the image data.  |


The image array returned by the function is a 1D Array. So for the image to be displayable we first resize the image and convert it to RGB formate using `numpy`


```python
errorCode,resolution,image=vrep.simxGetVisionSensorImage(clientID,cam_handle,0,vrep.simx_opmode_streaming)
img = np.array(image,dtype = np.uint8)
img.resize = (resolution[0],resolution[1],3)
```
## Finding the rest of the Functions 
* You can find the rest of the equivalent functions from the [PythonAPI Functions list](https://www.coppeliarobotics.com/helpFiles/en/remoteApiFunctionsPython.htm). You just have to just search the lua function's name and you will find the Python equivalent function and its description. 

## References Used
* [A video tutorial via a small project by Nikolai](https://youtu.be/SQont-mTnfM)</br>
* [The official API documentation and Functions List provided by CoppeliaSim](https://www.coppeliarobotics.com/helpFiles/en/remoteApiFunctionsPython.htm)</br>

To use the remote API functionality in your Python script, you will need following 3 items:
  sim.py
  simConst.py
  remoteApi.dll, remoteApi.dylib or remoteApi.so (depending on your target platform)
Above files are located in CoppeliaSim's installation directory, under programming/remoteApiBindings/python. You might have to build the remoteApi shared library yourself (using remoteApiSharedLib.vcproj or makefile) if not already built. In that case, make sure you have defined NON_MATLAB_PARSING and MAX_EXT_API_CONNECTIONS=255 (and optionally DO_NOT_USE_SHARED_MEMORY) as a preprocessor definition.
Once you have above elements in a directory known to Python, call import sim to load the library. To enable the remote API on the client side (i.e. your application), call sim.simxStart. See the simpleTest.py script in the programming/remoteApiBindings/python directory for an example. This page lists and describes all supported Python remote API functions. CoppeliaSim remote API functions can easily be recognized from their "simx"-prefix.

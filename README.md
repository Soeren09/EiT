# EiT
Practial project for Expert in Teams

# Folder structure
```bash
.Root
+-- Device_Tryouts
|   +-- # Contains Scene .xml file and collision setup
|   |
|   +-- components
|       +-- # Contains object files in .stl format
|
+-- PrototypeControl
|   +-- # Robot control
|
+-- STOMPA
|   +-- 3D Print
|   |   +-- # Contains all 3D printed components
|   +-- LaserCut
|   |   +-- # Containt all laser cutter drawings 
|   +-- PDF Work Drawings
|   |   +-- # Containt all Work Drawings for the mechanical components
|   +-- STOMPA 3D files
|       +-- # Containt the Invertor files for the CAD model
| 
+-- Workcell
|   +-- # Examplecode for RobworkStudio scene
|
+-- PluginUIapp
    +-- # Contain code for simulating the STOMPA in RobworkStudio
    
```


To rebuild the pluginUIapp.so:
Local paths are stored in: pluginUIapp/src/AbsolutePaths.h
which is ignored by git so you will have to create it. Mine looks like this:

#ifndef PLUGINUIAPP_ABSOLUTEPATHS_H
#define PLUGINUIAPP_ABSOLUTEPATHS_H
#include <string>

//Malte:
const std::string URFilePath = "/home/maltenj/EiT/Workcell/Scene.wc.xml";
const std::string GantryFilePath = "/home/maltenj/EiT/Device_Tryouts/STOMPAScene.xml";
const std::string PlaybackFilePath = "/home/maltenj/EiT/pluginUIapp/src/";
const char* GoogleLogPath =  "/home/maltenj/EiT/pluginUIapp/src/";

#endif

Insert your own paths. 

Also we linked ceres, so you might have to install it. see http://ceres-solver.org/
Alternatively you can delete all references to it. We don't use it currently. 

To build run:
$ cd /<PathToGit>/Eit/Practical/EiT/pluginUIapp/build
$ cmake ..
$ make -j4

And pray it links correctly...

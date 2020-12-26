# EiT
Practial project for Expert in Teams




To rebuild the pluginUIapp.so:
$ cd /<PathToGit>/Eit/Practical/EiT/pluginUIapp/build
$ cmake ..
$ make -j4




Inverse kinematics is dependant on ceres. 


# Folder structure
```bash
.Root
+-- Device_Tryouts
| |
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
| Workcell
| - 
| 
| PluginUIapp
|
```

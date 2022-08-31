## the problem in data capture

Avoid weak textures, such as pure color walls and tables without textures. Weak texture scenes will make the images lack of features, which can be avoided by arranging the scene.

Avoid overexposure of images. Similar to weak textures, overexposure and highlights also make images difficult to match, which needs to be avoided in scene capture.

Avoid similar scenes. Similar scenes can easily lead to wrong image correlation and wrong reconstruction results.

Avoid lack of motion. It is necessary to keep the camera moving during the acquisition process. Lack of motion may lead to failure of scene depth estimation.



## the problem in matching stage 

For large data sets, ensure that there is enough memory and space to store the feature extraction and matching results.
The matching of 1W images may require about 20GB of memory and hard disk space.

## the problem in scale estimation 

The side length of apriltag (in A4 paper) should be 0.113 M. 
if markers of other sizes are used, this parameter should be changed in [scale estimation](../../src/estimate_scale).

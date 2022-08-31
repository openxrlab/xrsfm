# APIS
## run_matching
input: images, retrivel results, matching strategy\
purpose: extract feature from images and propose feature matching 

## run_reconstruction
input: matching results, camera intrinsic parameters \
purpose: reconstruct the scene for sequential images

## estimate_scale
input: images, reconstruction results\
purpose: estimate the true scale of the reconstruction 

## unpack_collect_data
input: binary file recorded by RGBCaptureTool \
purpose: parse recorded binary files to generate image data and camera internal parameters

# Data Format

## ios camera file
Camera intrisic parameters acquired by RGBCaptureTool

image_name model_name fx fy cx cy distortion_param

## camera intrisic file (for unordered data)
Camera intrisic parameters for unordered dataset

image_name model_name width height fx(fy) cx cy distortion_param

## retrieval file
The retrieval file is a text file, and each line record a pair of image names.
For each image， a fixed number of similar images are retrieved and arranged in order.
```
image1 image1_similar1
image1 image1_similar2
...
image1 image1_similarK
image2 image2_similar1
...
image2 image2_similarK
...
imageN imageN_similarK
...
```
## feature file
The feature file is a binary file that stores the keypoints and descriptors of each image.


## frame pair file
The frame pair file is a binary file that stores the matching results and epipolar geometry information of several image pairs.

## map files
Map data consists of three binary files, which are the same as the format in colmap. You can run colmap gui to observe the map data.


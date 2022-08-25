# APIS
## run_matching
input: images, retrivel results, matching strategy\
purpose: extract feature from images and propose feature matching 

## rec_sequential
input: images, matching results, camera intrinsic parameters,initial frame\
purpose: reconstruct the scene for sequential images

## recover_scale
input: images, reconstruction results\
purpose: recover the true scale of the reconstruction 

# Data Format

## ios camera file
ios录制的相机内参文件
image_name model_name fx fy cx cy distortion_param

## cameras file
无序数据集的相机内参文件
image_name model_name width height fx(fy) cx cy distortion_param

## retrieval file
检索文件是一个txt文件，每行存储一对图像的名字。
对每个图像检索固定的数目的相似图像，并且按顺序排列。

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

## features file
特征文件是一个二进制文件，存储了每张图像的特征的关键点和描述子。

## frame pairs file
匹配文件是一个二进制文件，存储了若干个图像对的匹配结果和对极几何信息。

## map files
地图数据由三个二进制文件组成，他们与COLMAP中的设置相同，你可以运行colmap gui来观察生成的地图。
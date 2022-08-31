# APIS
## run_matching
输入: images, retrivel results, matching strategy \
功能: 从图像中提取特征并进行特征匹配

## run_reconstruction
输入: matching results, camera intrinsic parameters \
功能: 为顺序图像集重建场景

## estimate_scale
输入: images, reconstruction results \
功能: 估计重建结果的真实尺度

## unpack_collect_data
输入：RGBCaptureTool录制的二进制文件 \
功能：解析录制的二进制文件，生成图片数据和相机内参

# Data Format

## ios camera file
RGBCaptureTool录制的相机内参文件
image_name model_name fx fy cx cy distortion_param

## cameras file
无序数据集的相机内参文件
image_name model_name width height fx(fy) cx cy distortion_param

## retrieval file
检索文件是一个文本文件，每行存储一对图像的名字。
对每个图像检索固定的数目的相似图像，并且按顺序排列。
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

## features file
特征文件是一个二进制文件，存储了每张图像的关键点和描述子。

## frame pairs file
匹配文件是一个二进制文件，存储了若干个图像对的匹配结果和对极几何信息。

## map files
地图数据由三个二进制文件组成（cameras.bin, images.bin, points.bin），他们与COLMAP中的设置相同，你可以运行colmap gui来观察生成的地图。
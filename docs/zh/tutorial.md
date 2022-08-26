 
# 教程 

## 运动恢复结构

运动恢复结构（Structure-from-Motion, SfM）从不同角度拍摄的图像中恢复场景的三维结构。
其输入是一组图像，输出是图像所观测到的场景的三维重建，重建结果一般由三维点集合表示，也被成为点云。
通常，运动恢复结构系统可以分为两阶段，匹配阶段和重建阶段。
匹配阶段通过特征匹配算法寻找图像的像素关联关系;
重建阶段通过二维的观测估计相机的位姿和三维结构。
基于单目图像的重建会缺失真实的尺度，因此XRSfM额外提供了估计尺度功能。
XRSfM的估计尺度功能依赖于场景中预先放置的人工标志物，具有真实尺度的三维结构能够很好的支持面向AR的定位应用。


### 1.匹配阶段
输入:图像文件夹，检索文件，匹配策略类型

输出:特征提取结果，特征匹配结果

通过下列命令行运行匹配阶段
```
./bin/run_matching images_path retrival_path matching_type output_path
```

"retrival_path" 指向检索文件的路径。
检索文件是一个文本文件存储着图像检索的结果。
目前图像检索功能被封装在XRLocation, 具体的生成方式可以参考（url）。
你也可以通过其他检索方法生成该文件，只需确保文件的格式符合要求（url）。

"matching_type" 指定了匹配策略。
XRSfM支持顺序匹配("sequential"), 基于检索的匹配("retrival")，和基于共视性的匹配("covisibility")。
基于共视性的匹配具有较高的匹配效率，其实现参考了ICRA2020论文“Efficient Covisibility-Based Image Matching for Large-Scale SfM”。


### 2.重建阶段
Input:特征提取结果，特征匹配结果，相机内参文件

Output:重建结果

通过下列命令行运行重建阶段 
```
./bin/sequential_reconstruction bin_path images_path camera_path output_path init_id1 init_id2
```

"bin_path" 是一个文件夹路径，包含匹配阶段生成的特征和匹配结果的二进制文件。

"camera_path" 是文件camera.txt的路径，该文件存储摄像机内部参数信息。

"init_id1" and "init_id2" 是可选的输入参数，指定重建阶段的初始帧，当不输入该参数时，程序将会自行选择初始帧。

输出的重建结果由三部分组成“cameras.bin”,“images.bin”,"points.bin"。
文件格式与知名的开源工作COLMAP相同，你也可以使用COLMAP的图像用户界面查看结果。

### 3.尺度估计
Input:图像文件夹，重建结果

Output:恢复尺度的重建结果


通过下列命令行运行重建阶段 
```
./bin/tag_refine images_dir map_dir
```

"map_dir" 指向重建结果的文件夹 (cameras.bin, images.bin, points.bin).

该步骤旨在恢复重建结果的真实比例。
首先，确保输入的重建结果正确，否则整个过程无法正常运行。
该程序将从图像中提取apriltag以计算比例，应确保每个ID的apriltag在场景中是唯一的。

## 运行你自己的数据

### 数据采集
我们推荐使用采集工具拍摄图像，它会同时获取一个准确的相机内参。
用户也可以使用其他来源的图像，但鉴于当前xrsfm并不支持相机自标定，用户需要给出相机内参，这可以由标定得到。
更多的相机模型支持和相机自标定功能将在后续的版本中支持。

### 数据准备
除了上述的图像数据和相机内参外，还需要准备图像的检索结果，这部分功能目前被封装在XRLocation中，详细参见（url）
在运行重建前，你应该确保有以下数据：
存储着图像数据的文件夹（images_path） ，
检索文件（retrival_path），
相机内参文件（camera_path）。
运行结果将需要两个文件夹存储，存储初始重建结果的文件夹（results_path）和存储尺度恢复后结果的文件夹（refined_results_path）。

### 运行重建
运行下列命令行以进行重建。
```
./bin/run_matching ${images_path}$ ${retrival_path}$ sequential ${results_path}$
./bin/sequential_reconstruction ${results_path}$ ${images_path}$ ${camera_path}$ ${results_path}$
./bin/tag_refine ${images_path}$ ${results_path}$ ${refined_results_path}$
```
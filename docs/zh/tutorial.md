
# 教程

## 运动恢复结构

运动恢复结构（Structure-from-Motion, SfM）从不同角度拍摄的图像中恢复场景的三维结构。
其输入是一组图像，输出是图像所观测到的场景的三维重建，重建结果一般由三维点集合表示，也被称为点云。
通常，运动恢复结构系统可以分为两阶段，匹配阶段和重建阶段。
匹配阶段通过特征匹配算法寻找图像的像素关联关系;
重建阶段通过图像的二维观测估计相机的位姿和场景的三维结构。
基于单目图像的重建会缺失真实的尺度，因此XRSfM额外提供了尺度估计功能。
XRSfM的尺度估计功能依赖于场景中预先放置的已知尺度的人工标志物，具有真实尺度的重建结果能够很好的支持面向AR的定位应用。


### 1. 匹配阶段
输入: 图像文件夹，检索文件，匹配策略类型

输出: 特征提取结果，特征匹配结果

通过下列命令行运行匹配阶段
```
./bin/run_matching images_path retrieval_path matching_type output_path
```

"retrieval_path" 指向检索文件的路径。
检索文件是一个文本文件存储着图像检索的结果。
目前图像检索功能被封装在XRLocalization, 具体的生成方式可以参考[链接](https://github.com/openxrlab/xrlocalization/tree/main/docs/en/tutorials/generate_image_pairs.md)。
你也可以通过其他检索方法生成该文件，具体的文件格式可以参考[introduction.md](./introduction.md)。

"matching_type" 指定了匹配策略。
XRSfM支持顺序匹配("sequential")、基于检索的匹配("retrieval")和基于共视性的匹配("covisibility")。
基于共视性的匹配策略在无序图像集上具有较高的匹配效率，其实现参考了ICRA2020论文“Efficient Covisibility-Based Image Matching for Large-Scale SfM”。


### 2. 重建阶段
输入: 特征提取结果，特征匹配结果，相机内参文件

输出: 重建结果

通过下列命令行运行重建阶段
```
./bin/run_reconstruction bin_path camera_path output_path init_id1 init_id2
```

"bin_path" 是一个文件夹路径，包含匹配阶段生成的特征和匹配结果的二进制文件。

"camera_path" 是文件camera.txt的路径，该文件存储摄像机内部参数信息。

"init_id1" and "init_id2" 是可选的输入参数，指定重建阶段的初始帧，当不输入该参数时，程序将会自行选择初始帧。

输出的重建结果由三部分组成“cameras.bin”,“images.bin”,"points.bin"。
文件格式与知名的开源工作COLMAP相同，用户可以使用COLMAP的图像用户界面查看本项目的重建结果。

### 3. 尺度估计（可选）
输入: 图像文件夹，重建结果

输出: 恢复尺度的重建结果


通过下列命令行运行尺度估计
```
./bin/estimate_scale images_dir model_dir
```

“model_dir”存储二进制文件（camera.bin、images.bin、points.bin）。

该步骤旨在恢复重建结果的真实比例。
首先，确保输入的重建结果正确，否则整个过程无法正常运行。
该程序将从图像中提取apriltag以计算比例，应确保每个ID的apriltag在场景中是唯一的。

### 4. 生成彩色点云（可选）

输入： 图像、重建结果



输出： 彩色点云



使用以下命令行运行彩色点云生成

```

python3 ./scripts/pointcloud_color_calculator.py --images_dir=/path/to/images/ --model_dir=/path/to/model/

```



“model_dir”存储二进制文件（camera.bin、images.bin、points.bin）。



此步骤旨在计算每个3D点的颜色。为了提高效率，我们跳过了重建阶段的颜色计算，原始点云是黑色的。此步骤将基于图像颜色生成彩色点云。建议在执行此步骤之前保存原始重建结果。

## 运行你自己的数据

### 数据采集
我们推荐使用[采集工具](http://doc.openxrlab.org.cn/openxrlab_document/ARDemo/ARdemo.html#data-capturer-on-your-phone)拍摄图像，它会同时获取一个准确的相机内参。
用户也可以使用其他来源的图像，但鉴于当前版本不支持相机自标定，用户需要给出相机内参，这可以由标定得到。

### 数据准备
除了上述的图像数据和相机内参外，还需要准备图像的检索结果，这部分功能目前被封装在XRLocation中，详细参见[XRLocalization](https://github.com/openxrlab/xrlocalization/tree/main/docs/en/tutorials/generate_image_pairs.md)。
用户也可以直接下载[测试数据](https://openxrlab-share-mainland.oss-cn-hangzhou.aliyuncs.com/xrsfm/test_data.zip?versionId=CAEQQBiBgMCi_6mllxgiIGI2ZjM1YjE1NjBmNTRmYjc5NzZlMzZkNWY1ZTk1YWFj)来运行程序。
在运行重建前，你应该确保有以下数据：
存储着图像数据的文件夹（images_path） ，
检索文件（retrieval_path），
相机内参文件（camera_path）。
运行结果将需要两个文件夹存储，存储初始重建结果的文件夹（results_path）和存储尺度恢复后结果的文件夹（refined_results_path）。

### 运行重建

运行下列命令行以进行重建。
```
./bin/run_matching ${images_path}$ ${retrieval_path}$ sequential ${results_path}$
./bin/run_reconstruction ${results_path}$ ${camera_path}$ ${results_path}$
./bin/estimate_scale ${images_path}$ ${results_path}$ ${refined_results_path}$
```
你也可以创建工作空间文件夹，将相关数据按以下格式排列。
```
workspace
-images
--image1.png
--image2.png
...
--imagen.png
-camera.txt
-retrieval.txt
```
然后通过以下脚本达到相同的效果。
```
python3 ./scripts/auto_reconstruction.py --workspace_path ${workspace_path}$ --estimate_scale
```

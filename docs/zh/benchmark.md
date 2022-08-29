
# Benchmark
---------------------

我们提供了一些脚本来帮助用户轻松运行公开数据集上的评估。
支持的数据集包括顺序数据集KITTI（url）和无序数据集1DSfM（url）

## 在KITTI数据集上进行评估

### 数据准备

KITTI数据集可以在（url）中下载。
作为必要的输入，可以从（url）下载KITTI数据集的图像检索结果。

### 匹配阶段

运行下列python脚本来进行匹配阶段
```
python3 ./scripts/run_kitti_matching.py --data_path ${dataset_path}$ --retrival_dir_path ${retrival_dir_path}$ --output_dir_path ${output_dir_path}$
```

### 重建阶段

运行下列python脚本来进行重建阶段
```
python3 ./scripts/run_kitti_reconstruction.py --data_path ${dataset_path}$ --bin_dir_path ${bin_dir_path}$
```

## 在1DSfM数据集上进行评估

### 数据准备

1DSfM数据集可以在[项目网页](https://www.cs.cornell.edu/projects/1dsfm/)中下载。
作为必要的输入，可以从（url）下载KITTI数据集的图像检索结果。

### 匹配阶段

运行下列python脚本来进行匹配阶段
```
python3 ./scripts/run_1dsfm_matching.py --data_path ${dataset_path}$ --retrival_dir_path ${retrival_dir_path}$ --output_dir_path ${output_dir_path}$
```

### 重建阶段

运行下列python脚本来进行重建阶段
```
python3 ./scripts/run_1dsfm_reconstruction.py --data_path ${dataset_path}$ --bin_dir_path ${bin_dir_path}$
```

## 评估结果

### 基于NetVLAD50的匹配结果
|  | 注册帧数 | 匹配耗时 |
|:------:|:-------:|:--------:| 
|Alamo	            |862|	1405 |
|Ellis Island	    |351|	988  |
|Gendarmenmarkt	    |1020|	596  |
|Madrid Metropolis  |435|	430  |
|Montreal Notre Dame|564|	972  |
|NYC Library	    |574|	975  |
|Piazza del Popolo  |951|	872  |
|Piccadilly	        |2988|	2717 |
|Roman Forum	    |1599|	1226 |
|Tower of London    |699|	732  |
|Trafalgar	        |7725|	6396 |
|Union Square 	    |1070|	2313 |
|Vienna Cathedral   |1119|	3328 |
|Yorkminster	    |927|	1620 |

### 基于共视性的匹配结果
|  | 注册帧数 | 匹配耗时 |
|:------:|:-------:|:--------:| 
|Alamo	            |787 |	120 |
|Ellis Island	    |339 |	97  |
|Gendarmenmarkt	    |970 |	222 |
|Madrid Metropolis  |447 |	81  |
|Montreal Notre Dame|482 |	79  |
|NYC Library	    |581 |	97  |
|Piazza del Popolo  |926 |	161 |
|Piccadilly	        |3014|	740 |
|Roman Forum	    |1580|	327 |
|Tower of London    |641 |	118 |
|Trafalgar	        |7468|	2076|
|Union Square 	    |1034|	    |
|Vienna Cathedral   |969 |	117 |
|Yorkminster	    |997 |	211 |

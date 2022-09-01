
# Benchmark
---------------------

We provide some scripts to help users can run benchmark easily.
The supported datasets include the sequential dataset [KITTI](http://www.cvlibs.net/datasets/kitti/index.php) and the unordered dataset [1DSfM](https://www.cs.cornell.edu/projects/1dsfm/).


## Evaluation on the KITTI dataset

### Data preparation

The KITTI dataset can be download in [link](https://s3.eu-central-1.amazonaws.com/avg-kitti/data_odometry_gray.zip).
As a necessary input, the image retrieval results can be downloaded from  [link](https://openxrlab-share.oss-cn-hongkong.aliyuncs.com/xrsfm/KITTI.zip?versionId=CAEQQBiBgMCu.KallxgiIGM4MTk2MmJmNDU1YTQzYjBhYTJjZmIyYzQ3YzM2ODIx).

### Run matching stage

Run the following python script to process the matching stage
```
python3 ./scripts/run_kitti_matching.py --data_path ${dataset_path}$ --retrieval_dir_path ${retrieval_dir_path}$  --output_dir_path ${output_dir_path}$
```

### Run reconstruction stage


Run the following python script to process the reconstruction stage
```
python3 ./scripts/run_kitti_reconstruction.py --data_path ${dataset_path}$ --bin_dir_path ${bin_dir_path}$
```

### Run example

Download the data set and the retrieval data, unzip them and place them into '/path'
```
/path
-dataset
--sequences
---00
...
---10
-KITTI
--00
...
---10
```

Run the following scripts:
```
python3 ./scripts/run_kitti_matching.py --data_path /path/dataset/sequences/ --retrieval_dir_path /path/KITTI/ --output_dir_path /path/KITTI/
python3 ./scripts/run_kitti_reconstruction.py --data_path /path/dataset/sequences/ --bin_dir_path /path/KITTI/
```



## Evaluation was performed on the 1DSfM dataset

### Data preparation

The 1DSfM dataset can be download from [project web page](https://www.cs.cornell.edu/projects/1dsfm/).
The image retrieval results and camera intrinsic parameters can be downloaded from the [link](https://openxrlab-share.oss-cn-hongkong.aliyuncs.com/xrsfm/1DSfM.zip?versionId=CAEQQBiBgIDF.KallxgiIDcyNDJmNTM4OWJhNzRlYzdhNDhkZmNjMjQ0YWU0ODA3).

### Run matching

Run the following python script to process the matching stage
```
python3 ./scripts/run_1dsfm_matching.py --data_path ${dataset_path}$ --retrieval_dir_path ${retrieval_dir_path}$ --output_dir_path ${output_dir_path}$
```

### Run reconstruction


Run the following python script to process the reconstruction stage
```
python3 ./scripts/run_1dsfm_reconstruction.py --data_path ${dataset_path}$ --bin_dir_path ${bin_dir_path}$
```

### Results 


|  | Registered Frames | Matching Time |
|:------:|:-------:|:--------:|
| - | - | -|
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

|  | Registered Frames | Matching Time |
|:------:|:-------:|:--------:|
| - | - | -|
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
|Union Square 	    |1034|  721 |
|Vienna Cathedral   |969 |	117 |
|Yorkminster	    |997 |	211 |

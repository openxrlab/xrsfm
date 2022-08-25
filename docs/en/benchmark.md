
# Benchmark
---------------------

We provide some scripts to help users can run benchmark easily.
The supported datasets include the sequential dataset KITTI(url) and the unordered dataset 1DSfM(url)


## Run KITTI

### Data preparation

The KITTI dataset can be download in (url).
As a necessary input, the image retrieval results can be downloaded from the url.

### Run matching

```
python3 ./scripts/run_kitti_matching.py --data_path dataset_path --retrival_dir_path download_path --output_dir_path output_path
```

### Run reconstruction

```
python3 ./scripts/run_kitti_reconstruction.py --data_path dataset_path --bin_dir_path download_path
```

## Run 1DSfM

### Data preparation

The 1DSfM dataset can be download in (url).
The image retrieval results and camera intrinsic parameters can be downloaded from the url.

### Run matching

```
python3 ./scripts/run_1dsfm_matching.py --data_path dataset_path --retrival_dir_path download_path --output_dir_path output_path
```

### Run reconstruction

```
python3 ./scripts/run_1dsfm_reconstruction.py --data_path dataset_path --bin_dir_path download_path
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
|Union Square 	    |1034|	    |
|Vienna Cathedral   |969 |	117 |
|Yorkminster	    |997 |	211 |


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


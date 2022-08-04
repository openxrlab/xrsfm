import os

seq_name_list = [
    "00",
    "01",
    "02",
    "03",
    "04",
    "05",
    "06",
    "07",
    "08",
    "09",
    "10"
]
 
  
data_path = "/data/ECIM/SfM/KITTI/"

for seq_name in seq_name_list:
    print("Begin "+seq_name)
    os.system("evo_ape tum /data/ECIM/SfM/KITTI/gt/"+seq_name+"_tum.txt /data/ECIM/SfM/KITTI/"+seq_name+"_test.tum  -vsa")
    print("End "+seq_name)

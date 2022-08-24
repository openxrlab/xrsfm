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
output_path = "/data/OpenData/" 

for seq_name in seq_name_list:
    print("Begin "+seq_name)
    os.system("evo_ape tum "+data_path+"gt/"+seq_name+"_tum.txt "+output_path+seq_name+"/"+seq_name+"_test.tum  -vsa")
    print("End "+seq_name)

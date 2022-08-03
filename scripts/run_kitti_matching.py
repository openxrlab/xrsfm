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

config_path = "../config_open.json"

for seq_name in seq_name_list:
    print("Begin "+seq_name)
    fo = open(config_path, "w")
    str = "{\n"
    str += "\t\"image_dir_path\": \"/data/ECIM/SfM/KITTI/"+seq_name+"/image_0/\",\n"
    str += "\t\"retrival_path\": \"/data/ECIM/Mat/" + \
        seq_name+"/netvlad_image_pairs/nv100_all.txt\",\n"
    str += "\t\"matching_type\": \"sequential\",\n"
    str += "\t\"output_path\": \"/data/ECIM/SfM/KITTI/"+seq_name+"/open/nv50\"\n"
    str += "}"
    fo.write(str)
    fo.close()


import os



seq_name_list = [
    "00",
    "01",
    "02",
    "03",
    "04",
    "05",
    "06",
    "09",
    "10"
]
 
config_path = "/home/yzc/Projects/xrsfm/config_kitti.json"

for seq_name in seq_name_list:
    print("Begin "+seq_name)
    fo = open(config_path, "w")
    str = "{\n"
    str += "\t\"data_path\": \"/data/ECIM/SfM/KITTI/\",\n"
    str += "\t\"seq_name\": \""+seq_name+"\",\n" 
    str += "\t\"output_path\": \"/data/ECIM/SfM/KITTI/\",\n"
    str += "\t\"camera_size\": 0.1,\n"
    str += "\t\"init_id1\": 0,\n"
    str += "\t\"init_id2\": 10,\n"
    str += "\t\"debug\": 0\n"
    str += "}"
    fo.write(str)
    fo.close()
    os.system("/home/yzc/Projects/xrsfm/bin/run_kitti")
    print("End "+seq_name)

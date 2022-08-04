import os

# seq_name_list = [
#     "00",
#     "01",
#     "02",
#     "03",
#     "04",
#     "05",
#     "06",
#     "07",
#     "08",
#     "09",
#     "10"
# ]
seq_name_list = [ 
    "01", 
    "05"
]
 
config_path = "/home/yzc/Projects/xrsfm/config_kitti.json"
data_path = "/data/ECIM/SfM/KITTI/"

for seq_name in seq_name_list:
    print("Begin "+seq_name)

    init_id1 = 0
    if seq_name == '01':
        init_id1 = 400
    if seq_name == '06':
        init_id1 = 200
    init_id2 = init_id1+10

    fo = open(config_path, "w")
    str_config = "{\n"
    str_config += "\t\"data_path\": \"/data/ECIM/SfM/KITTI/\",\n"
    str_config += "\t\"seq_name\": \""+seq_name+"\",\n" 
    str_config += "\t\"output_path\": \"/data/ECIM/SfM/KITTI/\",\n"
    str_config += "\t\"camera_size\": 0.1,\n"
    str_config += "\t\"init_id1\": "+str(init_id1)+",\n"
    str_config += "\t\"init_id2\": "+str(init_id2)+",\n"
    str_config += "\t\"debug\": 0\n"
    str_config += "}"
    fo.write(str_config)
    fo.close()
    os.system("/home/yzc/Projects/xrsfm/bin/rec_kitti > /data/ECIM/SfM/KITTI/log"+seq_name+".txt")
    print("End "+seq_name)

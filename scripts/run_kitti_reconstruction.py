import os
from argparse import ArgumentParser

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

# config_path = "/home/yzc/Projects/xrsfm/config_kitti.json"
# data_path = "/data/ECIM/SfM/KITTI/"
# bin_dir_path = "/data/OpenData/"


# for seq_name in seq_name_list:
#     print("Begin "+seq_name)

#     init_id1 = 0
#     if seq_name == '01':
#         init_id1 = 500
#     if seq_name == '06':
#         init_id1 = 200
#     init_id2 = init_id1+10

#     bin_path = bin_dir_path+seq_name+'/'
    
#     fo = open(config_path, "w")
#     str_config = "{\n"
#     str_config += "\t\"data_path\": \""+data_path+"\",\n"
#     str_config += "\t\"seq_name\": \""+seq_name+"\",\n" 
#     str_config += "\t\"bin_path\": \""+bin_path+"\",\n"
#     str_config += "\t\"output_path\": \""+bin_path+"\",\n"
#     str_config += "\t\"init_id1\": "+str(init_id1)+",\n"
#     str_config += "\t\"init_id2\": "+str(init_id2)+"\n"
#     str_config += "}"
#     fo.write(str_config)
#     fo.close()
#     os.system("/home/yzc/Projects/xrsfm/bin/rec_kitti > "+bin_path+"rec.txt")
#     print("End "+seq_name)

def get_opts():
    parser = ArgumentParser()

    parser.add_argument('--data_path', type=str, required=True,
                        help='data_path') 
    parser.add_argument('--bin_dir_path', type=str, required=True,
                        help='bin_dir_path')
    return parser.parse_args()


if __name__ == "__main__":
    args = get_opts()

    exe = "./bin/rec_kitti" 
    data_path = args.data_path 
    bin_dir_path = args.bin_dir_path
    
    for seq_name in seq_name_list:
        init_id1 = 0
        if seq_name == '01':
            init_id1 = 500
        if seq_name == '06':
            init_id1 = 200
        init_id2 = init_id1+10

        print("Begin "+seq_name)
        bin_path = bin_dir_path+seq_name+'/'
        os.system(exe+' '+bin_path+' '+data_path+' '+seq_name+' '+str(init_id1)+' '+str(init_id2))
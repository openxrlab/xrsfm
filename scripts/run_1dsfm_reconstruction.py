import os

seq_name_list = ['Alamo', 'Ellis_Island', 'Gendarmenmarkt', 'Madrid_Metropolis', 'Montreal_Notre_Dame', 'NYC_Library', 'Piazza_del_Popolo',
 'Piccadilly', 'Roman_Forum', 'Tower_of_London', 'Trafalgar', 'Union_Square', 'Vienna_Cathedral', 'Yorkminster']
  
seq_name_list = ['Alamo', 'Ellis_Island', 'Gendarmenmarkt', 'Madrid_Metropolis', 'Montreal_Notre_Dame', 'NYC_Library', 'Piazza_del_Popolo',
 'Piccadilly', 'Roman_Forum', 'Tower_of_London', 'Union_Square', 'Vienna_Cathedral', 'Yorkminster']

config_path = "/home/yzc/Projects/xrsfm/config_uno.json"
# data_path = "/data/ECIM/SfM/ECIM_1DSFM/"
data_path = "/data/OpenData/"


for seq_name in seq_name_list:
    print("Begin "+seq_name)
    image_dir_path = '/data/ECIM/SfM/ECIM_1DSFM/'+seq_name+'/images/'
    bin_dir_path = data_path+seq_name+"/"
    os.system("cp /data/ECIM/SfM/ECIM_1DSFM/"+seq_name+'/open/camera_info.txt '+bin_dir_path)
    fo = open(config_path, "w")
    str = "{\n"
    str += "\t\"image_dir_path\": \""+image_dir_path+"\",\n"
    str += "\t\"bin_dir_path\": \""+bin_dir_path+"/\",\n"
    str += "\t\"output_path\": \""+bin_dir_path+"\",\n"
    str += "\t\"camera_size\": 0.5,\n"
    str += "\t\"debug\": 0\n"
    str += "}"
    fo.write(str)
    fo.close() 
    # os.system("mkdir "+output_path)
    os.system("/home/yzc/Projects/xrsfm/bin/rec_1dsfm > "+data_path+seq_name+"/log_our.txt")
    print("End "+seq_name)

import os

seq_name_list = ['Alamo', 'Ellis_Island', 'Gendarmenmarkt', 'Madrid_Metropolis', 'Montreal_Notre_Dame', 'NYC_Library', 'Piazza_del_Popolo',
 'Piccadilly', 'Roman_Forum', 'Tower_of_London', 'Trafalgar', 'Piazza_del_Popolo', 'Union_Square', 'Vienna_Cathedral', 'Yorkminster']
  

config_path = "/home/yzc/Projects/xrsfm/config_uno.json"
data_path = "/data/ECIM/SfM/ECIM_1DSFM/"


for seq_name in seq_name_list:
    print("Begin "+seq_name)
    output_path = data_path+seq_name+"/open/ours1/"
    fo = open(config_path, "w")
    str = "{\n"
    str += "\t\"data_path\": \""+data_path+"\",\n"
    str += "\t\"seq_name\": \""+seq_name+"/\",\n"
    str += "\t\"output_path\": \""+output_path+"\",\n"
    str += "\t\"camera_size\": 0.5,\n"
    str += "\t\"debug\": 0\n"
    str += "}"
    fo.write(str)
    fo.close() 
    os.system("mkdir "+output_path)
    os.system("/home/yzc/Projects/xrsfm/bin/rec_1dsfm > "+data_path+seq_name+"/open/log_our3.txt")
    print("End "+seq_name)

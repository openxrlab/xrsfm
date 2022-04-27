import os

seq_name_list = ['Alamo', 'Ellis_Island', 'Gendarmenmarkt', 'Madrid_Metropolis', 'Montreal_Notre_Dame', 'NYC_Library', 'Piazza_del_Popolo',
 'Piccadilly', 'Roman_Forum', 'Tower_of_London', 'Trafalgar', 'Piazza_del_Popolo', 'Union_Square', 'Vienna_Cathedral', 'Yorkminster']
 
seq_name_list = ['Union_Square']

config_path = "/home/yzc/Projects/TIP_ECSfM/ECIM/config_uno.json"

for seq_name in seq_name_list:
    print("Begin "+seq_name)
    output_path = "/data/ECIM/SfM/ECIM_1DSFM/"+seq_name+"/open/ours/"
    fo = open(config_path, "w")
    str = "{\n"
    str += "\t\"data_path\": \"/data/ECIM/SfM/ECIM_1DSFM/\",\n"
    str += "\t\"seq_name\": \""+seq_name+"/\",\n"
    str += "\t\"output_path\": \""+output_path+"\",\n"
    str += "\t\"camera_size\": 0.5,\n"
    str += "\t\"debug\": 0\n"
    str += "}"
    fo.write(str)
    fo.close() 
    os.system("mkdir "+output_path)
    os.system("/home/yzc/Projects/TIP_ECSfM/ECIM/bin/run_uno > /data/ECIM/SfM/ECIM_1DSFM/"+seq_name+"/open/log_our.txt")
    # os.system("/home/yzc/Projects/TIP_ECSfM/ECIM/bin/run_uno")
    print("End "+seq_name)

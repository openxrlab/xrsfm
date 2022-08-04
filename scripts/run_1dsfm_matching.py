import os

# seq_name_list = ['Alamo', 'Ellis_Island', 'Gendarmenmarkt', 'Madrid_Metropolis', 'Montreal_Notre_Dame', 'NYC_Library', 'Piazza_del_Popolo',
#  'Piccadilly', 'Roman_Forum', 'Tower_of_London', 'Trafalgar', 'Union_Square', 'Vienna_Cathedral', 'Yorkminster']
 
config_path = "/home/yzc/Projects/xrsfm/config_open.json"
image_dir_path = "/data/ECIM/SfM/ECIM_1DSFM/"
retrival_dir_path = "/data/ECIM/SfM/ECIM_Mat_old/"
output_dir_path = "/data/OpenData/"

for seq_name in seq_name_list:
    print("Begin "+seq_name)
    fo = open(config_path, "w")
    str = "{\n"
    str += "\t\"image_dir_path\": \""+image_dir_path+seq_name+"/images/\",\n"
    str += "\t\"retrival_path\": \"" +retrival_dir_path+seq_name+"/nv50_all.txt\",\n"
    str += "\t\"matching_type\": \"ecim\",\n"
    str += "\t\"output_path\": \""+output_dir_path+seq_name+"/\"\n"
    str += "}"
    fo.write(str)
    fo.close()
    os.system("mkdir "+output_dir_path+seq_name+"/")
    os.system("/home/yzc/Projects/xrsfm/bin/run_matching > "+output_dir_path+seq_name+"/mat.log")
    print("End "+seq_name)

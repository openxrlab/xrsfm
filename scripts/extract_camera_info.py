import os

exe_path = '~/Projects/TIP_ECSfM/colmap/build/src/exe/colmap'
dir_path = '/data/ECIM/SfM/ECIM_1DSFM/'
seq_list = ['Alamo','Ellis_Island','Gendarmenmarkt','Madrid_Metropolis','Montreal_Notre_Dame','NYC_Library','Piazza_del_Popolo','Piccadilly','Roman_Forum','Tower_of_London','Trafalgar','Piazza_del_Popolo','Union_Square','Vienna_Cathedral','Yorkminster']
seq_list = ['Madrid_Metropolis']


for seq_name in seq_list:
      print(seq_name)
      db_path = dir_path+seq_name+'/nv25.db'
      in_path = dir_path+seq_name+'/0/'
      out_path = dir_path+seq_name+'/camera_info.txt'

      os.system(exe_path+' output_camera_info\
        --database_path '+db_path+'\
        --input_path '+in_path+' >'+out_path)

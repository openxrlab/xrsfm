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
  
def test_our(txt_path):
    file = open(txt_path,'r')
    lines = file.readlines() 
    t = 0.
    str_list = ['Time Total ']
    for line in lines:
        for str in str_list:
            if line.startswith(str):
                # t = 0.74*float(line[len(str):-2])
                print(line[:-1])      
    return t
 
# for seq_name in seq_name_list: 
#     log_txt = '/data/ECIM/SfM/KITTI/log'+seq_name+'.txt'
#     test_our(log_txt)

seq_name_list = ['Alamo', 'Ellis_Island', 'Gendarmenmarkt', 'Madrid_Metropolis', 'Montreal_Notre_Dame', 'NYC_Library', 'Piazza_del_Popolo',
 'Piccadilly', 'Roman_Forum', 'Tower_of_London', 'Trafalgar', 'Piazza_del_Popolo', 'Union_Square', 'Vienna_Cathedral', 'Yorkminster']
  
for seq_name in seq_name_list: 
    print(seq_name)
    log_txt = '/data/ECIM/SfM/ECIM_1DSFM/'+seq_name+'/open/log_our2.txt'
    test_our(log_txt)
import os

seq_name_list = ['Alamo', 'Ellis_Island', 'Gendarmenmarkt', 'Madrid_Metropolis', 'Montreal_Notre_Dame', 'NYC_Library', 'Piazza_del_Popolo',
 'Piccadilly', 'Roman_Forum', 'Tower_of_London', 'Trafalgar', 'Union_Square', 'Vienna_Cathedral', 'Yorkminster']
 
# seq_name_list = ['Trafalgar', 'Union_Square', 'Vienna_Cathedral', 'Yorkminster']

def test_our(txt_path):
    file = open(txt_path,'r')
    lines = file.readlines() 
    if len(lines)>0:
        return lines[-1][:-1]
    else:
        return 'null'
for seq_name in seq_name_list: 
    log_txt = '/data/OpenData/'+seq_name+'/mat.log'
    print(seq_name, test_our(log_txt))
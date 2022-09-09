import os
from argparse import ArgumentParser

seq_name_list = ['Alamo', 'Ellis_Island', 'Gendarmenmarkt',
                 'Madrid_Metropolis', 'Montreal_Notre_Dame',
                 'NYC_Library', 'Piazza_del_Popolo', 'Piccadilly',
                 'Roman_Forum', 'Tower_of_London', 'Trafalgar',
                 'Union_Square', 'Vienna_Cathedral', 'Yorkminster']


def get_opts():
    parser = ArgumentParser()

    parser.add_argument('--data_path', type=str, required=True,
                        help='data_path')
    parser.add_argument('--bin_dir_path', type=str, required=True,
                        help='bin_dir_path')
    return parser.parse_args()


if __name__ == '__main__':
    args = get_opts()

    exe = './bin/rec_1dsfm'
    data_path = args.data_path
    bin_dir_path = args.bin_dir_path

    if not data_path.endswith('/'):
        data_path = data_path+'/'
    if not bin_dir_path.endswith('/'):
        bin_dir_path = bin_dir_path+'/'

    for seq_name in seq_name_list:
        print('Begin '+seq_name)
        bin_path = bin_dir_path+seq_name+'/'
        output_path = bin_dir_path+seq_name+'/'
        print(exe+' '+bin_path+' '+output_path)
        os.system(exe + ' ' + bin_path + ' ' + output_path
                  + ' > ' + output_path+'log_rec.txt')

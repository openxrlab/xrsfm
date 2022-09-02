import os
from argparse import ArgumentParser

seq_name_list = [
    '00', '01', '02', '03', '04', '05', '06', '07', '08', '09', '10'
]


def get_opts():
    parser = ArgumentParser()

    parser.add_argument('--data_path', type=str, required=True,
                        help='data_path')
    parser.add_argument('--bin_dir_path', type=str, required=True,
                        help='bin_dir_path')
    return parser.parse_args()


if __name__ == '__main__':
    args = get_opts()

    exe = './bin/rec_kitti'
    data_path = args.data_path
    bin_dir_path = args.bin_dir_path

    if not data_path.endswith('/'):
        data_path = data_path+'/'
    if not bin_dir_path.endswith('/'):
        bin_dir_path = bin_dir_path+'/'

    for seq_name in seq_name_list:
        init_id1 = 0
        if seq_name == '01':
            init_id1 = 500
        if seq_name == '06':
            init_id1 = 200
        init_id2 = init_id1+10

        print('Begin '+seq_name)
        bin_path = bin_dir_path+seq_name+'/'
        os.system(exe + ' ' + bin_path + ' ' + data_path+' ' +
                  seq_name+' '+bin_path+' '+str(init_id1)+' '+str(init_id2))

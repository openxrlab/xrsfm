import os
from argparse import ArgumentParser

seq_name_list = ['Alamo', 'Ellis_Island', 'Gendarmenmarkt',
                 'Madrid_Metropolis', 'Montreal_Notre_Dame', 'NYC_Library',
                 'Piazza_del_Popolo', 'Piccadilly', 'Roman_Forum',
                 'Tower_of_London', 'Trafalgar', 'Union_Square',
                 'Vienna_Cathedral', 'Yorkminster']


def get_opts():
    parser = ArgumentParser()

    parser.add_argument('--data_path', type=str, required=True,
                        help='data_path')
    parser.add_argument('--retrieval_dir_path', type=str, required=True,
                        help='retrieval_dir_path')
    parser.add_argument('--output_dir_path', type=str, required=True,
                        help='output_dir_path')
    return parser.parse_args()


if __name__ == '__main__':
    args = get_opts()

    exe = './bin/run_matching'
    data_path = args.data_path
    retrieval_dir_path = args.retrieval_dir_path
    output_dir_path = args.output_dir_path

    if not data_path.endswith('/'):
        data_path = data_path+'/'
    if not retrieval_dir_path.endswith('/'):
        retrieval_dir_path = retrieval_dir_path+'/'
    if not output_dir_path.endswith('/'):
        output_dir_path = output_dir_path+'/'

    for seq_name in seq_name_list:
        print('Begin '+seq_name)
        images_path = data_path+seq_name+'/images/'
        retrieval_path = retrieval_dir_path+seq_name+'/retrival_50.txt'
        output_path = output_dir_path+seq_name+'/'
        os.system(exe+' '+images_path+' '+retrieval_path+' covisibility '
                  + output_path+' > '+output_path+'log_mat.txt')

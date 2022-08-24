import os
from argparse import ArgumentParser

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

def get_opts():
    parser = ArgumentParser()

    parser.add_argument('--data_path', type=str, required=True,
                        help='data_path')
    parser.add_argument('--retrival_dir_path', type=str, required=True,
                        help='retrival_dir_path')
    parser.add_argument('--output_dir_path', type=str, required=True,
                        help='output_dir_path')
    return parser.parse_args()

if __name__ == "__main__":
    args = get_opts()

    exe = "./bin/run_matching" 
    data_path = args.data_path
    retrival_dir_path = args.retrival_dir_path
    output_dir_path = args.output_dir_path

    for seq_name in seq_name_list:
        print("Begin "+seq_name)
        images_path = data_path+seq_name+'/image_0/'
        retrival_path = retrival_dir_path+seq_name+'/retrival_100.txt'
        output_path = output_dir_path+seq_name+'/'
        os.system(exe+' '+images_path+' '+retrival_path+' sequential '+output_path)
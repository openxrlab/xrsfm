import os
from argparse import ArgumentParser


def get_opts():
    parser = ArgumentParser()
    parser.add_argument('--workspace_path', type=str, required=True,
                        help='workspace_path')
    parser.add_argument('--estimate_scale', action='store_true',
                        help='estimate_scale')
    # parser.add_argument('--start_from_ios_binary',action='store_true',
    #                     help='estimate_scale')
    return parser.parse_args()


def change_camera_txt_format(data_path):
    with open(data_path+'/camera.txt', 'r') as file:
        line = file.readline()

    space_index1 = line.find(' ')
    space_index2 = line.find(' ', space_index1 + 1)

    with open(data_path+'/camera_colmap.txt', 'w') as file:
        file.write(f'0 RADIAL 1440 1920 {line[space_index2+1:]}')


if __name__ == '__main__':
    args = get_opts()

    data_path = args.workspace_path

    if not data_path.endswith('/'):
        data_path = data_path+'/'

    images_path = data_path+'images/'
    camera_path = data_path+'camera_colmap.txt'
    retrieval_path = data_path+'retrieval.txt'

    output_path = data_path

    # if(args.start_from_ios_binary):
    #     ios_file = data_path+'Data.txt'
    #     os.system('./bin/unpack_collect_data '+ios_file+' '+data_path)
    change_camera_txt_format(data_path)

    os.system('./bin/run_matching ' + images_path + ' ' + retrieval_path
              + ' sequential ' + output_path)
    os.system('./bin/run_reconstruction ' + output_path + ' '
              + camera_path + ' ' + output_path)

    if (args.estimate_scale):
        output_refined_path = data_path+'refined/'
        if not os.path.exists(output_refined_path):
            os.system('mkdir '+output_refined_path)
        os.system('./bin/estimate_scale '+images_path+' '+output_path+' '
                  + output_refined_path)

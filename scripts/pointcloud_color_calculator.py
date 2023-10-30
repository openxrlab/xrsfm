from colmap.read_write_model import \
    Point3D, read_images_binary, read_points3D_binary, write_points3D_binary
import cv2
from argparse import ArgumentParser
import numpy as np


def add_color(image_dir, bin_dir):
    images = read_images_binary(bin_dir+'/images.bin')
    points = read_points3D_binary(bin_dir+'/points3D.bin')

    point_colors = {}
    for id, image in images.items():
        image_path = image_dir + image.name
        cvimage = cv2.imread(image_path)
        cvimage = cv2.cvtColor(cvimage, cv2.COLOR_BGR2RGB)
        h, w, c = cvimage.shape
        for i in range(len(image.point3D_ids)):
            p3d_id = image.point3D_ids[i]
            if p3d_id == -1:
                continue
            x, y = [int(xy_i) for xy_i in image.xys[i]]
            if x > w or x < 0 or y < 0 or y > h:
                continue
            if p3d_id not in point_colors:
                point_colors[p3d_id] = []
            point_colors[p3d_id].append(cvimage[y, x])

    points_new = {}
    for id, point in points.items():
        rgb = point.rgb
        if (id in point_colors):
            rgb_int = [0, 0, 0]
            for color in point_colors[id]:
                for i in range(3):
                    rgb_int[i] = rgb_int[i]+color[i]
            for i in range(3):
                rgb_int[i] = int(rgb_int[i]/len(point_colors[id]))
            rgb = np.array(rgb_int)
        points_new[id] = Point3D(id=id, xyz=point.xyz, rgb=rgb,
                                 error=point.error, image_ids=point.image_ids,
                                 point2D_idxs=point.point2D_idxs)

    write_points3D_binary(points_new, bin_dir+'/points3D.bin')


def get_opts():
    parser = ArgumentParser()
    parser.add_argument('--image_dir', type=str, required=True,
                        help='image_dir')
    parser.add_argument('--model_dir', type=str, required=True,
                        help='model_dir')
    return parser.parse_args()


if __name__ == '__main__':
    args = get_opts()

    image_dir = args.image_dir
    model_dir = args.model_dir

    if not image_dir.endswith('/'):
        image_dir = image_dir+'/'
    if not model_dir.endswith('/'):
        model_dir = model_dir+'/'

    add_color(image_dir, model_dir)

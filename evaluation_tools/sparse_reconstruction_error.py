'''
Author: yanyan-li yanyan.li.camp@gmail.com
Date: 2023-08-05 23:16:11
LastEditTime: 2023-09-01 21:47:35
LastEditors: yanyan-li yanyan.li.camp@gmail.com
Description:
FilePath: /tools/sparse_reconstruction_error.py
'''
'''
Author: yanyan-li yanyan.li.camp@gmail.com
Date: 2023-08-05 15:16:11
LastEditTime: 2023-09-01 16:24:46
LastEditors: yanyan-li yanyan.li.camp@gmail.com
Description:
FilePath: /tools/sparse_reconstruction_error.py
'''




import os
import numpy as np
from sklearn.metrics import mean_squared_error
import matplotlib.pyplot as plt
import math
from math import sqrt
from argparse import ArgumentParser
def prRed(skk):
    print("\033[91m {}\033[00m" .format(skk))


def prGreen(skk):
    print("\033[92m {}\033[00m" .format(skk))


def prYellow(skk):
    print("\033[93m {}\033[00m" .format(skk))


def prLightPurple(skk):
    print("\033[94m {}\033[00m" .format(skk))


def prPurple(skk):
    print("\033[95m {}\033[00m" .format(skk))


def prCyan(skk):
    print("\033[96m {}\033[00m" .format(skk))


def sparse_map_reader(map_path):
    point_landmarks = {}
    line_landmarks = {}
    with open(map_path) as f:
        lines = f.readlines()
    f.close()

    for line in lines:
        if 'Mapline:' in line.split():
            linelandmark_id = line.split()[1]
            line_landmarks[linelandmark_id] = (line.split()[2:])
            # print("mapline:",(line.split()[2:]))
        elif 'Mappoint:' in line.split():
            pointlandmark_id = line.split()[1]
            point_landmarks[pointlandmark_id] = (line.split()[2:])
            # print("mappoint:", (line.split()[2:]) )
    return point_landmarks, line_landmarks


def landmarks_association(gt_dict, esti_dict):
    ids = set(gt_dict) & set(esti_dict)
    print(sorted(ids))
    return ids


def sparse_recon_distance(common_ids, gt_dict, esti_dict):
    pt_dist = []
    for id in common_ids:
        gt_value = gt_dict[id]
        esti_value = esti_dict[id]
        # print(len(gt_value), len(esti_value))
        if len(gt_value) == 3:
            # print(float(gt_value[0])-float(esti_value[0]))
            v_dist = (float(gt_value[0])-float(esti_value[0]),
                      float(gt_value[1])-float(esti_value[1]),
                      float(gt_value[2])-float(esti_value[2]))
            n_dist = np.linalg.norm(np.array(v_dist).reshape(-1, 1))
            # print(n_dist)
            pt_dist.append(n_dist)
        if len(gt_value) == 6:
            gt_point1 = np.array(
                (float(gt_value[0]), float(gt_value[1]), float(gt_value[2])))
            esti_point1 = np.array(
                (float(esti_value[0]), float(esti_value[1]), float(esti_value[2])))

            gt_point2 = np.array(
                (float(gt_value[3]), float(gt_value[4]), float(gt_value[5])))
            esti_point2 = np.array(
                (float(esti_value[3]), float(esti_value[4]), float(esti_value[5])))

            gt_direc = gt_point1 - gt_point2
            esti_direc = esti_point1 - esti_point2

            gt_normal = np.cross(gt_point1, gt_point2)
            esti_normal = np.cross(esti_point1, esti_point2)

            # direction_angle
            # vector_angle
            # distance
            if math.isnan(np.linalg.norm(esti_direc)):
                continue

            direc_angle = abs(np.dot(
                gt_direc/np.linalg.norm(gt_direc), esti_direc/np.linalg.norm(esti_direc)))
            normal_angle = abs(np.dot(
                gt_normal/np.linalg.norm(gt_normal), esti_normal/np.linalg.norm(esti_normal)))
            
            dist_direct = np.linalg.norm(gt_direc) - np.linalg.norm(esti_direc)
            dist_normal = np.linalg.norm(
                gt_normal) - np.linalg.norm(esti_normal)
            pt_dist.append((direc_angle*180/3.14, normal_angle*180/3.14,
                           dist_direct, dist_normal))

            print("id", id, direc_angle*180/3.14, normal_angle *
                  180/3.14, dist_direct, dist_normal)
            # print("gt_point1:", gt_direc, esti_direc, gt_normal, esti_normal)
    return pt_dist


def norm_distance_plot(dist_list, MEDIAN, RMSE, type):
    # print("list", len(dist_list))
    MEDIAN = [MEDIAN for x in range(len(dist_list))]
    RMSE = [RMSE for x in range(len(dist_list))]

    x = [x for x in range(len(dist_list))]
    # plt.plot(x, dist_list, 'o', color='lightgrey', label='distance')
    s = [100*dist for dist in dist_list]
    # plt.scatter(x, dist_list, s=s,  marker='s',
    #             label='distance', color='lightblue')
    if type == 'Line':
        marker = 'o'
    else:
        marker = 's'    
    plt.scatter(x, dist_list, marker=marker,
                label='distance', color='lightblue')
    plt.plot(x, MEDIAN, label='median')
    plt.plot(x, RMSE, label='rmse')
    plt.legend()
    plt.title('%s Reconstruction Error'%type)
    plt.show()


def point_distance_metric(dist_list, plot=None):
    # print(dist_list)
    # rmse
    # rmse = sqrt(mean_squared_error(dist_list, listofzeros))
    # print(np.array(dist_list).shape)
    # print(">> RMSE is %f" %rmse)

    prRed("***** Point Reconstruction Error *****")
    MSE = np.square(np.array(dist_list)).mean()

    print("size:", np.array(dist_list).shape)
    RMSE = math.sqrt(MSE)
    MEDIAN = np.median(np.array(dist_list))
    prGreen("MSE: %f" % MSE)
    prGreen("RMSE:%f" % RMSE)
    prGreen("MEDIAN:%f" % MEDIAN)

    if plot == 'norm':
        norm_distance_plot(dist_list=dist_list,
                           MEDIAN=MEDIAN, RMSE=RMSE, type="Point")


def line_distance_metric(dist_list, plot):
    prRed("***** Line Reconstruction Error *****")

    def _dist(dist, type):
        MSE = np.square(dist).mean()
        RMSE = math.sqrt(MSE)
        MEDIAN = np.median(dist)
        prGreen("MSE of %s: %f" % (type, MSE))
        prGreen("RMSE of %s: %f" % (type, RMSE))
        prGreen("MEDIAN of %s: %f" % (type, MEDIAN))
        if plot == 'norm':
            norm_distance_plot(dist_list=list(
                dist), MEDIAN=MEDIAN, RMSE=RMSE, type='Line')
    _dist(np.array(dist_list)[:, 1], 'normal')


def sparse_recon_error(gt_map_path, esti_map_path, landmark_type, plot_type=None):
    print(">> read gt and estimation.")
    pls_gt, lls_gt = sparse_map_reader(map_path=gt_map_path)
    pls_esti, lls_esti = sparse_map_reader(map_path=esti_map_path)

    if landmark_type == 'point':
        print(">> the numbers of gt_point and esti_point are %d and %d" %
              (len(pls_gt), len(pls_esti)))
        if len(pls_gt) > 0 and len(pls_esti) > 0:
            pls_ids = landmarks_association(pls_gt, pls_esti)
            pls_dist = sparse_recon_distance(
                common_ids=pls_ids, gt_dict=pls_gt, esti_dict=pls_esti)

            point_distance_metric(pls_dist, plot_type)
            prGreen(">> the numbers of common points and lines are %d of %d" %
                    (len(pls_ids), len(pls_esti)))
        else:
            prRed(">> no points detected in the file")
    elif landmark_type == 'line':
        print(">> the numbers of gt_line and esti_line are %d and %d" %
              (len(lls_gt), len(lls_esti)))
        if len(lls_gt) > 0 and len(lls_esti) > 0:
            lls_ids = landmarks_association(lls_gt, lls_esti)
            lls_dist = sparse_recon_distance(
                common_ids=lls_ids, gt_dict=lls_gt, esti_dict=lls_esti)
            line_distance_metric(lls_dist, plot_type)
        else:
            print(">> no lines detected in the file")


if __name__ == '__main__':
    parser = ArgumentParser()
    parser.add_argument(
        "--gt_map", help="ground truth map file", dest='gt_map')
    parser.add_argument(
        "--esti_map", help="predicted map file", dest='esti_map')
    parser.add_argument("--landmark_type", help="point or line", dest='type')
    parser.add_argument("--plot", help="plot choice:", dest='plot')
    args = parser.parse_args()

    # is file existed
    if args.gt_map is None or args.esti_map is None:
        print(">>>> please feed valid file_path.")
        print(">>>> usage: sparse_reconstruction_error.py -h")
    elif os.path.isfile(args.gt_map) and os.path.isfile(args.esti_map):
        print(">>>> start recon_error evaluation.")
        # compute recon error
        sparse_recon_error(gt_map_path=args.gt_map,
                           esti_map_path=args.esti_map,
                           landmark_type=args.type,
                           plot_type=args.plot)
    else:
        print(">>>> please check your file path.")

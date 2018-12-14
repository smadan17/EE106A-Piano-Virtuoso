import cv2
import crop
import yaml
import os
import matplotlib.path as mpltPath
from scipy.ndimage.measurements import label
import numpy as np
from sklearn.decomposition import PCA
import homography 

cwd = os.getcwd() + "/"

with open(cwd + "cfg.yml", "r") as ymlfile:
    cfg = yaml.load(ymlfile)

def is_point_to_left(point, axis, center):
    scale = (point[0] - center[0])/axis[0]
    projectedy = scale * axis[1] + center[1]
    return point[1] < projectedy


def get_center(points):
    return np.average(points, axis=0)


def get_lower_center(points):
    axis, center = get_points_info(points)
    lower_points = []
    for p in points:
        if is_point_to_left(p, [-axis[1],axis[0]], center):
            lower_points.append(p)
    return get_center(lower_points)


def get_points_info(points):
    pca = PCA(n_components=2)
    pca.fit(points)
    axis = pca.components_[1]
    axis = axis/np.linalg.norm(axis)
    axis = [-1*axis[1], axis[0]]
    return axis, get_center(points)


def map_keys_to_pixels():
    # run detect black key algorithm with different shrink parameter
    # until expected number is detected
    n_features = 0
    curr_shrink = 1
    desired_num_black_keys = cfg['numOctaves'] * cfg['blackKeysPerOctave']
    while n_features != desired_num_black_keys and curr_shrink > 0.8:
        img, coords, crop_img_func = crop.get_cropped_img(shrink=curr_shrink)

        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        binary = np.zeros(gray.shape)
        path = mpltPath.Path(coords)
        white_key_pixels = []
        for i in range(gray.shape[0]):
            for j in range(gray.shape[1]):
                if path.contains_point([j, i]):
                    if gray[i][j] < 127:
                        binary[i][j] = 255
                    else:
                        white_key_pixels.append([i, j])
        cv2.imwrite(cwd + cfg['outputDir'] +
                    "gray" + cfg['imgFormat'], gray)
        cv2.imwrite(cwd + cfg['outputDir'] +
                    "binary" + cfg['imgFormat'], binary)

        labeled_img = np.zeros(binary.shape)
        struct = np.ones((3, 3))

        n_features = label(binary, structure=struct, output=labeled_img)
        curr_shrink -= 0.025
    print(n_features)
    black_key_info = []
    for n in range(1, 1 + n_features):
        points = []
        for i in range(gray.shape[0]):
            for j in range(gray.shape[1]):
                if labeled_img[i][j] == n:
                    points.append([i, j])
        axis, center = get_points_info(points)
        black_key_info.append((axis, center))

    num_black_keys_to_right_to_pixels = {}
    for p in white_key_pixels:
        num_black_keys_to_right = 0
        for black_key in black_key_info:
            axis, center = black_key
            if is_point_to_left(p, axis, center):
                num_black_keys_to_right += 1
        if num_black_keys_to_right not in num_black_keys_to_right_to_pixels:
            num_black_keys_to_right_to_pixels[num_black_keys_to_right] = []
        num_black_keys_to_right_to_pixels[num_black_keys_to_right].append(p)

    notes_to_centers = {}
    sharp_keys = []
    for i in range(cfg['numOctaves']):
        for note in ['C', 'D', 'F', 'G', 'A']:
            sharp_keys.append(note + "#" + str(i))

    black_key_info.sort(key=lambda x: x[1][1])
    for ind, val in enumerate(sharp_keys):
        notes_to_centers[val] = black_key_info[ind][1]

    num_black_keys_to_right_to_notes = {}
    for oct_num in range(cfg['numOctaves']):
        for key_num in range(cfg['blackKeysPerOctave']):
            n = oct_num * cfg['blackKeysPerOctave'] + key_num
            oct_str = str(2 - oct_num)
            if key_num == 0:
                notes = ['B', 'C']
            elif key_num == 1:
                notes = ['A']
            elif key_num == 2:
                notes = ['G']
            elif key_num == 3:
                notes = ['E', 'F']
            elif key_num == 4:
                notes = ['D']

            notes = [note + oct_str for note in notes]
            num_black_keys_to_right_to_notes[n] = notes
    num_black_keys_to_right_to_notes[15] = ['C0']

    for num_black_keys_to_right in num_black_keys_to_right_to_pixels.keys():
        pixels = num_black_keys_to_right_to_pixels[num_black_keys_to_right]
        mod_num = num_black_keys_to_right % cfg['blackKeysPerOctave']
        rev_oct_num = num_black_keys_to_right // cfg['blackKeysPerOctave']
        oct_num = cfg['numOctaves'] - rev_oct_num
        note = num_black_keys_to_right_to_notes[num_black_keys_to_right]
        if len(note) == 1:
            notes_to_centers[note[0]] = get_lower_center(pixels)
        else:
            axis, center = get_points_info(pixels)
            left_pixels = []
            right_pixels = []
            for p in pixels:
                if is_point_to_left(p, axis, center):
                    left_pixels.append(p)
                else:
                    right_pixels.append(p)
            notes_to_centers[note[0]] = get_lower_center(left_pixels)
            notes_to_centers[note[1]] = get_lower_center(right_pixels)

    img2 = np.copy(img)
    for p in num_black_keys_to_right_to_pixels[2]:
        img2[p[0]][p[1]] = [0, 0, 255]
    cv2.imwrite(cwd + cfg['outputDir'] +
                "testing" + cfg['imgFormat'], img2)

    for note in notes_to_centers:
        center = notes_to_centers[note]
        img2 = np.copy(img)
        for xs in range(-1, 2):
            for ys in range(-1, 2):
                img2[int(center[0])+xs][int(center[1])+ys] = [0, 0, 255]
        cv2.imwrite(cwd + cfg['outputDir'] + "key" +
                    note + cfg['imgFormat'], crop_img_func(img2))
        # cv2.imwrite(cwd + cfg['outputDir'] + "key" +
        #             note + cfg['imgFormat'], img2)
    #undo downsampling 
    downsample = cfg['downsample']
    for note in notes_to_centers:
        notes_to_centers[note] = notes_to_centers[note] * downsample
    return notes_to_centers
            
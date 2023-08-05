#!/usr/bin/env python3

import logging
import os
import shutil
from datetime import datetime
from pathlib import Path

import cv2


def get_data_dir():
    return Path(Path.home() / 'downloads' / 'pano')


def split_images(img_dir, input_file):
    os.makedirs(img_dir, exist_ok=True)
    cap = cv2.VideoCapture(input_file)

    frame_count = 0
    while True:
        ret, frame = cap.read()
        if not ret:
            break
        frame_count += 1
        output_path = os.path.join(img_dir, f'{frame_count}.jpg')
        cv2.imwrite(output_path, frame)
        logging.info(f'Extracting frame {frame_count} to {output_path}')

    cap.release()


def stitch_images(img_dir, out_file):
    image_dir_step = 3
    image_paths = [os.path.join(img_dir, path) for path in os.listdir(img_dir) if path.endswith('.jpg')]

    # filter
    iter_image_paths = image_paths[::image_dir_step]

    imgs = []
    for i in range(len(iter_image_paths)):
        imgs.append(cv2.imread(iter_image_paths[i]))
        imgs[i] = cv2.resize(imgs[i], (0, 0), fx=0.4, fy=0.4)
        logging.info(f'Adding image {i} to image queue and resizing')

    stitcher = cv2.Stitcher.create(cv2.Stitcher_SCANS)
    (dummy, output) = stitcher.stitch(imgs)

    if dummy != cv2.STITCHER_OK:
        logging.error('Stitching failed')
    else:
        logging.info('Stitching successful')

    cv2.imwrite(out_file, output)


def move_to_archive(archive_dir, pano):
    if not os.path.exists(archive_dir):
        os.makedirs(archive_dir)
    new_path = f'pano_{datetime.now().strftime("%Y%m%dT%H%M%S")}.jpg'
    shutil.move(pano, os.path.join(archive_dir, new_path))
    logging.info(f'Current output pano archived to {new_path}')


def clean_dirs(dirs):
    for directory in dirs:
        if os.path.isdir(directory):
            logging.info(f'Cleaning dir {directory}')
            shutil.rmtree(directory)


def main():
    logging.basicConfig(format='%(asctime)s%(message)s',
                        datefmt='%m/%d/%Y %I:%M:%S%p ',
                        level=logging.DEBUG)
    logging.getLogger().setLevel(logging.INFO)

    input_file = os.path.join(get_data_dir(), 'input.mp4')

    img_dir = os.path.join(get_data_dir(), 'img')
    if not os.path.exists(img_dir):
        split_images(img_dir, input_file)

    pano = os.path.join(get_data_dir(), 'pano.jpg')
    archive_dir = os.path.join(get_data_dir(), 'archive')
    if os.path.exists(pano):
        move_to_archive(archive_dir, pano)
    stitch_images(img_dir, pano)

    clean_dirs([img_dir])


if __name__ == '__main__':
    main()

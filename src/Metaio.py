import re

import cv2
import numpy as np

from Homography import *
from ImageUtils import *
from NNTracker import *
from Polygons import *

def to_template_warp(warp):
    return warp * np.matrix([[1,0,0],[0,1,0],[0,0,2.5]])

def to_XGA_warp(warp):
    return warp * np.matrix([[1,0,0],[0,1,0],[0,0,1.0/2.5]])

def to_template(region):
    square = np.array([[-.5,-.5],[.5,-.5],[.5,.5],[-.5,.5]]).T
    hom = to_template_warp(compute_homography(square, region))
    return apply_to_pts(hom, square)

def to_XGA(region):
    square = np.array([[-.5,-.5],[.5,-.5],[.5,.5],[-.5,.5]]).T
    hom = to_XGA_warp(compute_homography(square, region))
    return apply_to_pts(hom, square)

def parse_init_file(location): 
    re_frame = re.compile("(?:Initialization data at frame )(\d+)")
    re_number = re.compile("([-+]?(?:\d+(?:\.\d*)?|\.\d+))")

    current_frame = None
    in_pts = []
    out_pts = []
    inits = {}

    for line in open(location):
        line = line.strip()
        m = re_frame.match(line)
        if m != None:
            if current_frame != None:
                inits[current_frame] = (np.array(in_pts).T, np.array(out_pts).T)
                in_pts = []
                out_pts = []
            current_frame = int(m.group(1))
        else:
            nums = map(float, re_number.findall(line))
            if nums != []:
                in_pts.append(nums[2:4])
                out_pts.append(nums[0:2])

    return inits

def open_benchmark(directory, num):
    inits = parse_init_file(directory + "gtSeq%.2dinit.txt" % num)
    video = cv2.VideoCapture(directory + "gtSeq%.2d.avi" % num)
    return (video, inits)

def run_benchmark(directory, num, tracker, num_runs=1):
    cv2.namedWindow("metaio benchmark")
    try:
        losses = []
        corners = []
        for i in xrange(num_runs):
            loss = []
            corner = []
            tracker_initialized = False
            video, inits = open_benchmark(directory, num)
            frame = 0
            while True:
                succ, img = video.read()
                if not succ: break 
                gray_img = to_grayscale(img)
                if frame in inits.keys():
                    if tracker_initialized: 
                        tracker.set_region(to_template(inits[frame][1]))
                    else: 
                        tracker.initialize(gray_img, to_template(inits[frame][1]))
                        tracker_initialized = True
                else:
                    tracker.update(gray_img)
                loss.append(tracker.get_loss(gray_img))
                corner.append(tracker.get_corners())
                draw_region(img, tracker.get_corners(), (255,0,0), 2)
                draw_region(img, to_XGA(tracker.get_corners()), (0,255,0), 1)
                cv2.imshow("metaio benchmark", img)
                cv2.waitKey(1)
                frame += 1
            video.release()
            losses.append(loss)
            corners.append(corner)
    finally:
        cv2.destroyWindow("metaio benchmark")
    return { "losses"     : np.array(losses), 
             "corners"   : np.array(corners) }

def replay(directory, num, results):
    video, inits = open_benchmark(directory, num)
    corners = results['corners'].mean(axis=0)
    frame = 0
    cv2.namedWindow("replay")
    while True:
        succ, img = video.read()
        if not succ: break
        draw_region(img, corners[frame], (255,0,0), 2)
        cv2.imshow("replay", img)
        frame += 1
        cv2.waitKey(1)
    video.release()
    cv2.destroyWindow("replay")
    
directories = ["/Users/travisdick/Desktop/metaio/01_low_bump/",
               "/Users/travisdick/Desktop/metaio/02_low_stop/",
               "/Users/travisdick/Desktop/metaio/03_repetitive_lucent/",
               "/Users/travisdick/Desktop/metaio/04_repetetive_macmini/",
               "/Users/travisdick/Desktop/metaio/05_normal_isetta/",
               "/Users/travisdick/Desktop/metaio/06_normal_philadelphia/",
               "/Users/travisdick/Desktop/metaio/07_high_grass/",
               "/Users/travisdick/Desktop/metaio/08_high_wall/"]
nums = [1,2,3,4,5]

def run_all_benchmarks(tracker, results, num_runs = 1):
    for (d,n) in itertools.product(directories, nums):
        print (d,n)
        results[(d,n)] = run_benchmark(d, n, tracker, num_runs)
    return results
    
               

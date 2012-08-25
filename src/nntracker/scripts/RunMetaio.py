import os

import cv2
import numpy as np

from Metaio import *
from nntracker.trackers.TurnkeyTrackers import *


base_dir = "/Users/travisdick/Desktop/metaio/"
video_dirs = []
os.path.walk(base_dir, lambda l,d,u: l.append(d + "/"), video_dirs)
video_dirs = video_dirs[1:]
sequence_numbers = range(1,6)

algorithms = [ make_esm(res=(100,100), use_scv=False),
               make_nn_GN(res=(100,100), use_scv=False) ]

results = [ [] for a in algorithms ]

for d in video_dirs:
    for n in sequence_numbers:
        print "---------- Running %d of %s ----------"%(n,d)
        for i, a in enumerate(algorithms):
            results[i].append(load_and_run_benchmark(d, n, a))
            

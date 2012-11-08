#!/usr/bin/env python
import argparse
from glob import glob
import os
import os.path as path

import cv2

from nntracker.InteractiveTracking import *
from nntracker.trackers.TurnkeyTrackers import *

class ImageDirectoryTracking(InteractiveTrackingApp):

    def __init__(self, in_glob, out_dir, trackers):

        # Initialize the InteractiveTrackingApp class
        InteractiveTrackingApp.__init__(self, trackers, "Tracking", init_with_rectangle=False)

        # Transform the unix glob into a list of file names
        self.file_names = glob(path.expanduser(path.expandvars(in_glob)))

        # Make sure the output directory exists
        self.out_dir = path.expanduser(path.expandvars(out_dir))
        if not path.isdir(self.out_dir): os.mkdir(self.out_dir)

    def run(self):

        img = cv2.imread(self.file_names[0])
        self.on_frame(img)
        self.paused = True

        i = 0
        while i < len(self.file_names):
            file_name = self.file_names[i]
            if not self.paused:
                if self.annotated_img != None:
                    _, name = path.split(file_name)
                    out_path = path.join(self.out_dir, "tracked_" + name)
                    cv2.imwrite(out_path, self.annotated_img)
                img = cv2.imread(file_name)
                i += 1
            self.on_frame(img)
 
        self.cleanup()

if __name__=="__main__":        
    parser = argparse.ArgumentParser()
    parser.add_argument("image_file_glob", help="Glob pattern for input images")
    parser.add_argument("output_dir", help="output directory")
    parser.add_argument("-t", 
                        "--tracker", 
                        choices = ["esm", "bmic", "nnbmic"],
                        default = "nnbmic",
                        help="Name of tracker. If not specified, nnbmic is used.")
    args = parser.parse_args()

    if args.tracker == "esm": tracker = make_esm(use_scv=True)
    if args.tracker == "bmic": tracker = make_bmic(use_scv=True)
    if args.tracker == "nnbmic": tracker = make_nn_bmic(use_scv=True)
    
    app = ImageDirectoryTracking(args.image_file_glob, args.output_dir, [tracker])
    app.run()
    


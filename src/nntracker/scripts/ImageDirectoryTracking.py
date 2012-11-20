#!/usr/bin/env python
import argparse
from glob import glob
import os
import os.path as path

import cv2

from nntracker.InteractiveTracking import *
from nntracker.trackers.TurnkeyTrackers import *

def region_to_string(region):
    output = ""
    for i in xrange(0,4):
        for j in xrange(0,2):
            output += "\t%.2f"%region[j,i]
    return output

class ImageDirectoryTracking(InteractiveTrackingApp):

    def __init__(self, in_glob, out_dir, trackers, tracker_names, colour, thickness):

        # Initialize the InteractiveTrackingApp class
        InteractiveTrackingApp.__init__(self, trackers, "Tracking", init_with_rectangle=False, colours=colour, thickness=thickness)

        # Transform the unix glob into a list of file names
        self.file_names = glob(path.expanduser(path.expandvars(in_glob)))
        self.file_names.sort()

        # Make sure the output directory exists
        self.out_dir = path.expanduser(path.expandvars(out_dir))
        if not path.isdir(self.out_dir): os.mkdir(self.out_dir)

        # Open some log files
        self.log_files = [file(path.join(self.out_dir, name+"_log.txt"), "w") for name in tracker_names]
        for log in self.log_files: log.write("frame\tulx\tuly\turx\tury\tlrx\tlry\tllx\tlly\n")

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

                    # Log the corners of the region
                    _, frame_name = path.split(file_name)
                    for log, tracker in zip(self.log_files, self.trackers):
                        if tracker.is_initialized():
                            log.write(frame_name + region_to_string(tracker.get_region()) + "\n")

                img = cv2.imread(file_name)
                i += 1
            self.on_frame(img)
 
        for log in self.log_files: log.close()
        self.cleanup()

if __name__=="__main__":        
    parser = argparse.ArgumentParser()
    # Input and output arguments
    parser.add_argument("image_file_glob", help="Glob pattern for input images")
    parser.add_argument("output_dir", help="output directory")
    # Tracker choice arguments
    parser.add_argument("--esm", help="If present, include the esm tracker", action="store_true")
    parser.add_argument("--nnbmic", help="If present, include the nnbmic tracker", action="store_true")
    parser.add_argument("--bmic", help="If present, include the bmic tracker", action="store_true")

    # TODO: It would probably be a really good idea to allow people to also
    #       customize the parameters used for each algorithm. That way they
    #       wouldn't be stuck using the parameter values that I heuristically
    #       chose. 
    
    args = parser.parse_args()

    trackers = []
    tracker_names = []
    colour = []
    thickness = []
    if args.nnbmic: 
        trackers.append(make_nn_bmic(use_scv=True))
        tracker_names.append("nnbmic")
        colour.append((255,0,0))
        thickness.append(3)
    if args.esm: 
        trackers.append(make_esm(use_scv=True))
        tracker_names.append("esm")
        colour.append((0,255,0))
        thickness.append(2)
    if args.bmic: 
        trackers.append(make_bmic(use_scv=True))
        tracker_names.append("bmic")
        colour.append((0,0,255))
        thickness.append(1)
    
    app = ImageDirectoryTracking(args.image_file_glob, args.output_dir, trackers, tracker_names, colour, thickness)
    app.run()
    


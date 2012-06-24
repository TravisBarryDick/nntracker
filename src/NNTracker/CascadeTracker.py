from TrackerBase import *

class CascadeTracker(TrackerBase):
    def __init__(self, trackers):
        self.trackers = trackers
        self.initialized = False
        
    def set_region(self, region):
        for t in self.trackers: t.set_region(region)
        self.region = region
    
    def initialize(self, img, region):
        for t in self.trackers: t.initialize(img, region)
        self.region = region
        self.initialized = True
        
    def update(self, img):
        if not self.initialized: return
        region = None
        for t in self.trackers:
            if region != None: t.set_region(region)
            t.update(img)
            region = t.get_region()
        self.region = region
    
    def is_initialized(self):
        return self.initialized
    
    def get_region(self):
        return self.region

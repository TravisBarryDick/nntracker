"""
Author: Travis Dick (travis.barry.dick@gmail.com)
"""

from TrackerBase import *

class CascadeTracker(TrackerBase):

    def __init__(self, trackers, set_warp_directly=True):
        """ Allows multiple trackers to be combined in series.

        Given a sequence of tracking algorithms, the CascadeTracker
        will use each tracker to compute an incremental update to
        the region proposed by the previous tracker. This allows us
        to combine different algorithms in a "coarse-to-fine" tracking
        scheme, where first we roughly take care of large motions and
        then utilize some very precise trackers for the final alignment.

        Parameters:
        -----------
        trackers : [TrackerBase]
          trackers is a list of objects each implementing the TrackerBase
          interface.       

        See Also:
        ---------
        TrackerBase
        MultiProposalTracker
        ParallelTracker
        """
        self.trackers = trackers
        
        if set_warp_directly:
            def set_tracker_state(tracker, state):
                tracker.set_warp(state)
            def get_tracker_state(tracker):
                return tracker.get_warp()
        else:
            def set_tracker_state(tracker, state):
                tracker.set_region(state)
            def get_tracker_state(tracker):
                return tracker.get_region()
        self._set_state = set_tracker_state
        self._get_state = get_tracker_state
        self.initialized = False
    
    def initialize(self, img, region):
        for t in self.trackers: t.initialize(img, region)
        self.region = region
        self.initialized = True
        
    def update(self, img):
        if not self.initialized: return
        state = None
        for t in self.trackers:
            if state != None: self._set_state(t, state)
            t.update(img)
            state = self._get_state(t)
    
    def is_initialized(self):
        return self.initialized

    def set_warp(self, warp):
        for t in self.trackers: t.set_warp(warp)

    def get_warp(self):
        return self.trackers[-1].get_warp()

    def set_region(self, region):
        for t in self.trackers: t.set_region(region)
    
    def get_region(self):
        return self.trackers[-1].get_region()

    

import numpy as np

class TrackerBase:
    """ The base class for all tracking algorithms.

    This class serves two purposes. First, it demonstrates
    the minimal interface expected by a tracking algorithm.
    Second, it implements various algorithm-independent 
    converstions - mainly convenience functions for working
    with rectangles instead of arbitrary quadrilaterals.
    """

    def set_region(self, corners):
        raise NotImplementedError()

    def set_region_with_rectangle(self, ul, lr): 
        self.set_region(_rectangle_to_region(ul,lr))

    def initialize(self, img, region):
        raise NotImplementedError()

    def initialize_with_rectangle(self, img, ul, lr): 
        self.initialize(img, _rectangle_to_region(ul,lr))
    
    def update(self, img): 
        raise NotImplementedError()

    def is_initialized(self): 
        raise NotImplementedError()

    def get_region(self): 
        raise NotImplementedError()

# --- Utility Functions, Not exported --- #

def _rectangle_to_region(ul, lr):
    return np.array([ul, [lr[0],ul[1]], lr, [ul[0],lr[1]]]).T

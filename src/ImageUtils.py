import numpy as np
from scipy import weave
from scipy.weave import converters

def to_grayscale(img):
    (height, width, depth) = img.shape
    grayscale = np.empty((height,width), dtype=np.float64)
    code = \
    """
    for (int i = 0; i < height; i++) {
      for (int j = 0; j < width; j++) {
        double mean = 0;
        for (int k = 0; k < depth; k++) mean += img(i,j,k);
        grayscale(i,j) = mean / depth / 256;
      }
    }
    """
    weave.inline(code, ['height', 'width', 'depth', 'grayscale', 'img'],
                 type_converters=converters.blitz)
    return grayscale

def sample_region(img, pts):
    num_pts = pts.shape[1]
    (height, width) = img.shape
    result = np.empty(num_pts)
    support_code = \
    """
    double bilinear_interp(blitz::Array<double,2> img, int width, int height, double x, double y) {
      using std::floor;
      using std::ceil;
      const int lx = floor(x);
      const int ux = ceil(x);
      const int ly = floor(y);
      const int uy = ceil(y);
      if (lx < 0 || ux >= width || ly < 0 || uy >= height) return 0;
      const double ulv = img(ly,lx);
      const double urv = img(ly,ux);
      const double lrv = img(uy,ux);
      const double llv = img(uy,lx);
      const double dx = x - lx;
      const double dy = y - ly;
      return ulv*(1-dx)*(1-dy) + urv*dx*(1-dy) + llv*(1-dx)*dy + lrv*dx*dy;
    }
    """
    code = \
    """
    for (int i = 0; i < num_pts; i++) {
      result(i) = bilinear_interp(img, width, height, pts(0,i), pts(1,i));
    }
    """
    weave.inline(code, ["img", "result", "pts", "num_pts", "width", "height"],
                 support_code=support_code, headers=["<cmath>"],
                 type_converters=converters.blitz)
    return result - result.mean()


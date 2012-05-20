import numpy as np

def homogenize(pts):
    (h,w) = pts.shape
    results = np.empty((h+1,w))
    results[:h] = pts
    results[-1].fill(1)
    return results

def dehomogenize(pts):
    (h,w) = pts.shape
    results = np.empty((h-1,w))
    results[:h-1] = pts[:h-1]/pts[h-1]
    return results

def apply_to_pts(homography, pts):
    (h,w) = pts.shape
    result = np.empty((h+1,w))
    result[:h] = pts
    result[-1].fill(1)
    result = np.asmatrix(homography) * result
    result[:h] /= result[-1]
    return np.asarray(result[:h])

def square_to_corners_warp(corners):
    square = np.array([[-.5,-.5],[.5,-.5],[.5,.5],[-.5,.5]]).T    
    return compute_homography(square, corners)

# Note: This is not doing the preconditioning step! Only important
# if we ever want to estimate homographies from more than 4 point
# correspondences.
def compute_homography(in_pts, out_pts):
    num_pts = in_pts.shape[1]
    in_pts = homogenize(in_pts)
    out_pts = homogenize(out_pts)
    constraint_matrix = np.empty((num_pts*2, 9))
    for i in xrange(num_pts):
        p = in_pts[:,i]
        q = out_pts[:,i]
        constraint_matrix[2*i,:] = np.concatenate([[0,0,0], -p, q[1]*p], axis=1)
        constraint_matrix[2*i+1,:] = np.concatenate([p, [0,0,0], -q[0]*p], axis=1)
    U,S,V = np.linalg.svd(constraint_matrix)
    homography = V[8].reshape((3,3))
    homography /= homography[2,2]
    return np.asmatrix(homography)

def random_homography(sigma_d, sigma_t):
    square = np.array([[-.5,-.5],[.5,-.5],[.5,.5],[-.5,.5]]).T
    disturbance = np.random.normal(0,sigma_d,(2,4)) + np.random.normal(0,sigma_t,(2,1))
    return compute_homography(square, square+disturbance)


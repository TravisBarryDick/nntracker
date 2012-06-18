import numpy as np

def polygon_descriptors(corners):
    n_points = corners.shape[1]
    p, a, cx, cy = 0, 0, 0, 0
    for i in xrange(n_points):
        j = (i+1) % n_points
        dot = corners[0,i]*corners[1,j] - corners[0,j]*corners[1,i]
        a += dot
        cx += (corners[0,i] + corners[0,j]) * dot
        cy += (corners[1,i] + corners[1,j]) * dot
        p += np.linalg.norm(corners[:,i] - corners[:,j])
    a /= 2
    cx /= 6*a
    cy /= 6*a
    a = abs(a)
    return (p, a, (cx,cy))
    

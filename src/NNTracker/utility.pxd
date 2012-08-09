cimport numpy as np

cdef double bilin_interp(double [:,:] img, double x, double y)

cpdef double[:] sample_pts(double[:,:] img, int resx, int resy, double[:,:] warp) except *

cpdef double[:,:] make_hom_sl3(double[:] p) except *
cpdef double[:,:] sample_pts_grad_sl3(double[:,:] img, int resx, int resy, double[:,:] warp) except *

cpdef double [:,:] to_grayscale(np.uint8_t [:,:,:] img)


cdef normalize_hom(double[:,:] m)
cdef double[:,:] mat_mul(double[:,:] A, double[:,:] B) 


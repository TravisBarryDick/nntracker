/** 

This file contains c++ functions that I use in more than one scipy.weave call.

**/

#ifndef NNTRACKERCPPUTILS_INCLUDE
#define NNTRACKERCPPUTILS_INCLUDE

double bilinear_interp(blitz::Array<double,2> img, int width, int height, double x, double y) {
	using std::floor;
	using std::ceil;
	const int lx = floor(x);
	const int ux = ceil(x);
	const int ly = floor(y);
	const int uy = ceil(y);
	if (lx < 0 || ux >= width || ly < 0 || uy >= height) return 128;
	const double ulv = img(ly,lx);
	const double urv = img(ly,ux);
	const double lrv = img(uy,ux);
	const double llv = img(uy,lx);
	const double dx = x - lx;
	const double dy = y - ly;
	return ulv*(1-dx)*(1-dy) + urv*dx*(1-dy) + llv*(1-dx)*dy + lrv*dx*dy;
}

#endif

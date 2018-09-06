package SVO_1310.Vikit;

import matrix_types.Vector2d;
import matrix_types.Vector3d;

public class ATANCamera extends AbstractCamera{
	private double fx_;
	private double fy_;
	private double fx_inv_, fy_inv_;
	private double cx_, cy_;
	private double s_, s_inv_;
	private double tans_;                         //!< distortion model coeff
	private double tans_inv_;                     //!< distortion model coeff
	private boolean distortion_; 

	public ATANCamera(double width, double height, double fx, double fy, double cx, double cy, double s)
	{
		super((int)width, (int) height);
		fx_ = (width*fx); fy_ = (height*fy);
		fx_inv_=(1.0/fx_); fy_inv_=(1.0/fy_);
		cx_=(cx*width - 0.5); cy_=(cy*height - 0.5);
		s_=(s); s_inv_=(1.0/s_);

		if(s_ != 0.0)
		{
			tans_ = 2.0 * Math.tan(s_ / 2.0);
			tans_inv_ = 1.0 / tans_;
			s_inv_ = 1.0 / s_;
			distortion_ = true;
		}
		else
		{
			s_inv_ = 0.0;
			tans_ = 0.0;
			distortion_ = false;
		}
	}
	//! Radial distortion transformation factor: returns ration of distorted / undistorted radius.
	private double rtrans_factor(double r) 
	{
		if(r < 0.001 || s_ == 0.0)
			return 1.0;
		else
			return (s_inv_* Math.atan(r * tans_) / r);
	}


	//! Inverse radial distortion: returns un-distorted radius from distorted.
	private double invrtrans(double r) 
	{
		if(s_ == 0.0)
			return r;
		return (Math.tan(r * s_) * tans_inv_);
	}

	@Override
	public Vector3d cam2world(double x, double y) {
		Vector2d vec;
		Vector2d dist_cam = new Vector2d(); 
		dist_cam.set((x - cx_) * fx_inv_,(y - cy_) * fy_inv_);
		double dist_r = dist_cam.normF();
		double r = invrtrans(dist_r);
		double d_factor;
		if(dist_r > 0.01)
			d_factor =  r / dist_r;
		else
			d_factor = 1.0;

		Vector3d vec3d = math_utils.unprojected2d(dist_cam.times(d_factor));
		vec3d.normalize();		// ToDo check : unproject2d(d_factor * dist_cam).normalized();
		return vec3d;
	}

	@Override
	public Vector3d cam2world(Vector2d px) {
		return cam2world(px.get(0), px.get(1));
	}

	@Override
	public Vector2d world2cam(Vector3d xyz_c) {
		return world2cam(math_utils.project2d(xyz_c));
	}

	@Override
	public Vector2d world2cam(Vector2d uv) {
		double r = uv.normF();
		double factor = rtrans_factor(r);
		Vector2d dist_cam = uv.times(factor);
		Vector2d vec2d = new Vector2d();
		vec2d.set(cx_ + fx_ * dist_cam.get(0),cy_ + fy_ * dist_cam.get(1));
		return vec2d;
	}

	public Vector2d focal_length()
	{
		Vector2d vec = new Vector2d();
		vec.set(fx_,fy_);
		return vec;
	}
	@Override
	public double errorMultiplier2() {
		return fx_;
	}
	public double errorMultiplier()
	{
		return 4*fx_*fy_;
	}

}

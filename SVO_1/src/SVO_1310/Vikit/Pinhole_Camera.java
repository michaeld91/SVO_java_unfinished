package SVO_1310.Vikit;

import org.opencv.core.CvType;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;


import matrix_types.Matrix3d;
import matrix_types.Vector2d;
import matrix_types.Vector3d;

public class Pinhole_Camera extends AbstractCamera{
	private final double fx_, fy_;
	private final double cx_, cy_;
	private final boolean distortion_;
	private final double[] d_ = new double[5];
	private Mat cvK_, cvD_;
	private Mat undist_map1_, undist_map2_;
	private boolean use_optimization_;
	private Matrix3d K_;
	private Matrix3d K_inv_;


	//	public Pinhole_Camera(int width, int height) {
	//		super(width, height);
	//		// TODO Auto-generated constructor stub
	//	}
	public Pinhole_Camera(double width, double height, double fx, double fy, double cx, double cy)
	{
		super(width,height);
		d_[0] = 0; d_[1] = 0; d_[2] = 0; d_[3] = 0; d_[4] = 0;

		fx_ = fx;
		fy_ = fy;
		cx_ = cx;
		cy_ = cy;
		distortion_ = (Math.abs(d_[0])> 0.0000001);
		undist_map1_ = new Mat((int)height, (int)width, CvType.CV_16SC2);
		undist_map2_ = new Mat((int)height, (int)width, CvType.CV_16SC2);
		use_optimization_ = false;

		cvK_ = new Mat(3,3,CvType.CV_16SC1);// ToDo CvType has been guessed. Use the accurate one!
		System.out.println("channels = "+CvType.channels(CvType.CV_16SC2));
		double[] cvK_array = {fx_, 0.0, cx_, 0.0, fy_, cy_, 0.0, 0.0, 1.0};
		cvK_.put(0, 0, cvK_array);
		
		cvD_ = new Mat(1,5, CvType.CV_16SC1);//// ToDo CvType has been guessed. Use the accurate one! 
		cvD_.put(0, 0, d_[0], d_[1], d_[2], d_[3], d_[4]);

		
		// Note CvType has been guessed again
		Imgproc.initUndistortRectifyMap(cvK_, cvD_, Mat.eye(3, 3, CvType.CV_16SC1), cvK_, new Size(width, height), CvType.CV_16SC2, undist_map1_, undist_map2_);
		double[] K_vals = {fx_, 0.0, cx_, 0.0, fy_, cy_, 0.0, 0.0, 1.0};
		K_ = new Matrix3d(K_vals);
		K_inv_ = K_.inverse();
	}
	
	public Pinhole_Camera(double width, double height, double fx, double fy, double cx, double cy,
			double d0, double d1, double d2, double d3, double d4)
	{
		super(width,height);
		fx_ = fx;
		fy_ = fy;
		cx_ = cx;
		cy_ = cy;
		distortion_ = (Math.abs(d0)> 0.0000001);
		undist_map1_ = new Mat((int)height, (int)width, CvType.CV_16SC2);
		undist_map2_ = new Mat((int)height, (int)width, CvType.CV_16SC2);
		use_optimization_ = false;

		d_[0] = d0; d_[1] = d1; d_[2] = d2; d_[3] = d3; d_[4] = d4;
		cvK_ = new Mat(3,3,CvType.CV_16SC2);// ToDo CvType has been guessed. Use the accurate one!
		double[] cvK_array = {fx_, 0.0, cx_, 0.0, fy_, cy_, 0.0, 0.0, 1.0};
		for(int y =0; y<3; y++)
		{
			for(int x = 0; x<3; x++)
				cvK_.put(x, y, cvK_array[(2*y)+x]);
		}
		
		cvD_ = new Mat(1,5, CvType.CV_16SC2);//// ToDo CvType has been guessed. Use the accurate one! 
		double[] d_vals =  {d_[0], d_[1], d_[2], d_[3], d_[4]};
		for(int x = 0; x < 5; x++)
			cvD_.put(1, x, d_vals[x]);
		
		// Note CvType has been guessed again
		Imgproc.initUndistortRectifyMap(cvK_, cvD_, Mat.eye(3, 3, CvType.CV_16SC2), cvK_, new Size(width, height), CvType.CV_16SC2, undist_map1_, undist_map2_);
		double[] K_vals = {fx_, 0.0, cx_, 0.0, fy_, cy_, 0.0, 0.0, 1.0};
		K_ = new Matrix3d(K_vals);
		K_inv_ = K_.inverse();


	}
	@Override
	public Vector3d cam2world(double u, double v) {
		Vector3d xyz = new Vector3d();
		if(!distortion_)
		{
			xyz.set((u - cx_)/fx_, (v - cy_)/fy_, 1.0);
		}
		else
		{
			Point uv = new Point(u,v); 
			Point px = new Point();
			
			// ToDo Scalar used so that Mat(int, int, type, scalar) can be used. May not make logical sense
			Scalar uv_x = new Scalar(uv.x);
			Scalar px_x = new Scalar(px.x);

			Mat src_pt = new Mat(1, 1, CvType.CV_32FC2, uv_x);
			Mat dst_pt = new Mat(1, 1, CvType.CV_32FC2, px_x);
			Imgproc.undistortPoints(src_pt, dst_pt, cvK_, cvD_);
			xyz.set(px.x, px.y, 1.0);
		}
		xyz.normalize();
		return xyz;
	}

	@Override
	public Vector3d cam2world(Vector2d uv) {
		return cam2world(uv.get(0),uv.get(1));
	}

	@Override
	public Vector2d world2cam(Vector2d uv) {
		Vector2d px = new Vector2d();
		if(!distortion_)
		{
			px.set(fx_*uv.get(0) + cx_, fy_*uv.get(1) + cy_);
		}
		else
		{
			double x, y, r2, r4, r6, a1, a2, a3, cdist, xd, yd;
			x = uv.get(0);
			y = uv.get(1);
			r2 = x*x + y*y;
			r4 = r2*r2;
			r6 = r4*r2;
			a1 = 2*x*y;
			a2 = r2 + 2*x*x;
		    a3 = r2 + 2*y*y;
		    cdist = 1 + d_[0]*r2 + d_[1]*r4 + d_[4]*r6;
		    xd = x*cdist + d_[2]*a1 + d_[3]*a2;
		    yd = y*cdist + d_[2]*a3 + d_[3]*a1;
		    px.set(xd*fx_ + cx_, yd*fy_ + cy_);
		  }
	  return px;
	}

	@Override
	public Vector2d world2cam(Vector3d uv) {
		return world2cam(math_utils.project2d(uv));
	}

	@Override
	public double errorMultiplier2() {
		return Math.abs(fx_);
	}

}

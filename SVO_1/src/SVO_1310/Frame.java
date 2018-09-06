package SVO_1310;

import java.util.ArrayList;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import matrix_types.Matrix6d;
import matrix_types.Vector2d;
import matrix_types.Vector3d;
import Jama.Matrix;
import SVO_1310.Vikit.AbstractCamera;
import SVO_1310.Vikit.Vision;
import Sophus.Se3;

public class Frame {


	private ArrayList<Feature> fts_ = new ArrayList<Feature>();	//!< List of features in the image. Replaces typedef Feature.
	//private vector<cv::Mat> ImgPyr;							//Created a class called ImgPyr. May need extra work
	//private ArrayList<Mat> ImgPyr = new ArrayList<Mat>();

	private static int frame_counter_;							//!< Counts the number of created frames. Used to set the unique id.
	private int id_;											//!< Unique id of the frame.
	private double timestamp_;									//!< Timestamp of when the image was recorded.
	private AbstractCamera cam_;								//!< Camera model.
	private Se3 T_f_w_;											//!< Transform (f)rame from (w)orld. //This is set in frame_handler_mono
	private Matrix6d Cov_ = new Matrix6d();					//!< Covariance.
	private static ImgPyr img_pyr_;									//!< Image Pyramid.
	//private GVector key_pts_;							//Should be of the type vector<Feature*>	//!< Five features and associated 3D points which are used to detect if two frames have overlapping field of view.
	private ArrayList<Feature> key_pts_ = new ArrayList<Feature>(5);
	//	private Feature[] key_pts_;
	private boolean is_keyframe_;								//!< Was this frames selected as keyframe?
	//g2oFrameSE3 v_kf_;										//!< Temporary pointer to the g2o node object of the keyframe.
	private int last_published_ts_;								//!< Timestamp of last publishing.

	//Frame(vk::AbstractCamera* cam, const cv::Mat& img, double timestamp);
	//Requires the Java version of OpenCV and the class Mat.
	public Frame(AbstractCamera cam, Mat img, double timestamp)
	{
		this.id_= frame_counter_++;
		this.timestamp_= timestamp;
		cam_= cam;
		//		key_pts_ = new GVector(5);	//Maybe need to initialise to a vector that is 5 long? 
		for(int i =0;i<5;i++){key_pts_.add(null);}
		is_keyframe_ = false;
		//v_kf_(null);
		img_pyr_ = new ImgPyr();
		initFrame(img);
	}


	//	/// Initialize new frame and create image pyramid.
	//	  void initFrame(const cv::Mat& img);
	public void initFrame(Mat img)
	{
		Config conf = Config.getInstance();
		if(img.empty() || img.type()!=(CvType.CV_8UC1) || img.cols() != cam_.getWidth() || img.rows() != cam_.getHeight())
		{
			// Print out for debugging
						System.out.println("img.type() = "+img.type()+ ", Desired type = "+(CvType.CV_8UC1));
						System.out.println("img.cols() = "+img.cols()+ ", cam_.getWidth() = "+ cam_.getWidth());
						System.out.println("img.rows() = "+img.rows()+ ", cam_.getHeight() = "+ cam_.getHeight());
						System.out.println();

			throw new RuntimeException("Frame: provided image is not of the same size as the camera model or the image is not greyscale");	
		}
		//Set the keypoints to NULL
		//ToDo Dont think this is correct. Is the point to make that array location null or to make the object at that location have a null value.
		for(int i=0; i < key_pts_.size();i++)//  Feature ftr : key_pts_)
		{
			//Feature ftr = key_pts_[i];
			//ftr=null;
			//key_pts_.get(i) = null;
			key_pts_.set(i, null);
		}
		//	Commented out for Testing. Should be added back in!
		//	System.out.println("max(Config.getnPyrLevels(),Config.getKltMaxLevel()+1) = "+max(Config.getnPyrLevels(),Config.getKltMaxLevel()+1));
		Frame_utils.createImgPyramid(img, max(Config.getnPyrLevels(),Config.getKltMaxLevel()+1), img_pyr_);

	}
	//	  /// Select this frame as keyframe.
	//	  void setKeyframe();
	public void setKeyframe()
	{
		is_keyframe_ = true;
		setKeyPoints();		
	}
	//	  /// Add a feature to the image
	//	  void addFeature(Feature* ftr);
	public void addFeature(Feature ftr)
	{
		fts_.add(ftr);
	}
	//	  /// The KeyPoints are those five features which are closest to the 4 image corners
	//	  /// and to the center and which have a 3D point assigned. These points are used
	//	  /// to quickly check whether two frames have overlapping field of view.
	//	  void setKeyPoints();
	//ToDo: I believe this method may be incorrect
	public void setKeyPoints()
	{
		for(int i =0; i<5; i++)
		{
			if(key_pts_.get(i) != null && key_pts_.get(i).getPoint() == null)
			{
				key_pts_.set(i,null);
				//System.out.println("Hello 1"); //FOR TESTING

			}
		}
		for(int i = 0; i<fts_.size(); i++)
			{
			if(fts_.get(i).getPoint() != null)
			{
				checkKeyPoints(fts_.get(i));
				//System.out.println("Hello 2"); //FOR TESTING
			}
		}
	}

	//	  /// Check if we can select five better key-points.
	//	  void checkKeyPoints(Feature* ftr);
	public void checkKeyPoints(Feature ftr)
	{
		int cu = cam_.getWidth()/2;
		int cv = cam_.getHeight()/2;

		//center pixel
		if(key_pts_.get(0) == null)
		{
			key_pts_.set(0, ftr);
		}
		else if(max(Math.abs(ftr.get_px(0)-cu), Math.abs(ftr.get_px(1)-cv))	<	max(Math.abs(key_pts_.get(0).get_px(0)-cu), Math.abs(key_pts_.get(0).get_px(1)-cv)))
			key_pts_.set(0,ftr);

		if(ftr.get_px(0)>=cu && ftr.get_px(1)>=cv)
		{
			if(key_pts_.get(1) == null)
				key_pts_.set(1,ftr);
			else if((ftr.get_px(0)-cu)*(ftr.get_px(1)-cv) > (key_pts_.get(1).get_px(0)-cu)*(key_pts_.get(1).get_px(1)-cv))
				key_pts_.set(1,ftr);				
		}
		if(ftr.get_px(0) >= cu && ftr.get_px(1) < cv)
		{
			if(key_pts_.get(2) == null)
				key_pts_.set(2,ftr);
			else if((ftr.get_px(0)-cu)*(ftr.get_px(1)-cv) > (key_pts_.get(2).get_px(0)-cu)*(key_pts_.get(2).get_px(1)-cv))
				key_pts_.set(2,ftr);				
		}
		if(ftr.get_px(0) < cv && ftr.get_px(1) < cv)
		{
			if(key_pts_.get(3) == null)
				key_pts_.set(3,ftr);
			else if((ftr.get_px(0)-cu)*(ftr.get_px(1)-cv) > (key_pts_.get(3).get_px(0)-cu)*(key_pts_.get(3).get_px(1)-cv))
				key_pts_.set(3,ftr);	
		}
		if(ftr.get_px(0) < cv && ftr.get_px(1) >= cv)
		{
			if(key_pts_.get(4) == null)
				key_pts_.set(4,ftr);
			else if((ftr.get_px(0)-cu)*(ftr.get_px(1)-cv) > (key_pts_.get(4).get_px(0)-cu)*(key_pts_.get(4).get_px(1)-cv))
				key_pts_.set(4,ftr);	
		}
	}
	//	  /// If a point is deleted, we must remove the corresponding key-point.
	//	  void removeKeyPoint(Feature* ftr);
	public void removeKeyPoint(Feature ftr)
	{
		boolean found = false;
		for(int i = 0; i<key_pts_.size(); i++)
		{
			if(key_pts_.get(i) == ftr)
				key_pts_.set(i, null);
			found = true;
		}
		if(found)
			setKeyPoints();
	}
	//	  /// Return number of point observations.
	//	  inline size_t nObs() const { return fts_.size(); }
	public int nObs()
	{
		return fts_.size();
	}
	//	  /// Check if a point in (w)orld coordinate frame is visible in the image.
	//	  bool isVisible(const Vector3d& xyz_w) const;
	public boolean isVisible(Vector3d xyz_w)
	{
		//May need to transpose xyz_w for the times to work.
		Vector3d xyz_f = (Vector3d) T_f_w_.times(xyz_w ); 
		if(xyz_f.get(0,2) < 0.0)		//changed from getElement(2)
		{
			return false;
		}
		Vector2d px = f2c(xyz_f);
		if(px.get(0,0) >= 0.0 && px.get(0,1) >= 0.0 && px.get(0,0) < cam_.getWidth() && px.get(0,1) < cam_.getHeight())
			return true;
		return false;

	}
	//	  /// Full resolution image stored in the frame.
	//	  inline const cv::Mat& img() const { return img_pyr_[0]; }
	//    Requires Checking to ensure it does the same as the c++ version
	public Mat img()
	{
		return img_pyr_.getMats()[0]; //Should return a Mat, from img_pyr_ which has an array of Mat objects.
	}
	//	  /// Was this frame selected as keyframe?
	//	  inline bool isKeyframe() const { return is_keyframe_; }
	public boolean isKeyFrame()
	{
		return is_keyframe_;
	}
	//	  /// Transforms point coordinates in world-frame (w) to camera pixel coordinates (c).
	//	  inline Vector2d w2c(const Vector3d& xyz_w) const { return cam_->world2cam( T_f_w_ * xyz_w ); }
	public Vector2d w2c(Vector3d xyz_w)	
	{
		Vector3d world2cam_var = (Vector3d) T_f_w_.times(xyz_w);	//May require transpose for times to work.
		return cam_.world2cam(world2cam_var); 
	}

	//	  /// Transforms pixel coordinates (c) to frame unit sphere coordinates (f).
	//	  inline Vector3d c2f(const Vector2d& px) const { return cam_->cam2world(px[0], px[1]); }
	public Vector3d c2f(Vector2d px)
	{
		return cam_.cam2world(px.get(0, 0), px.get(0, 1));
	}
	//	  /// Transforms pixel coordinates (c) to frame unit sphere coordinates (f).
	//	  inline Vector3d c2f(const double x, const double y) const { return cam_->cam2world(x, y); }
	public Vector3d c2f(double x, double y)		//changed from a vector3d!!!
	{
		return cam_.cam2world(x, y);
	}
	//	  /// Transforms point coordinates in world-frame (w) to camera-frams (f).
	//	  inline Vector3d w2f(const Vector3d& xyz_w) const { return T_f_w_ * xyz_w; }
	public Vector3d w2f(Vector3d xyz_w) 		//changed from a Vector 3d!!!
	{
		Vector3d multipy_sol = (Vector3d) T_f_w_.times(xyz_w); //May require transpose for times to work.
		return multipy_sol;
	}
	//	  /// Transforms point from frame unit sphere (f) frame to world coordinate frame (w).
	//	  inline Vector3d f2w(const Vector3d& f) const { return T_f_w_.inverse() * f; }
	public Vector3d f2w(Vector3d f)			//changed from a vector3d.
	{
		Vector3d inv_x_f = (Vector3d) T_f_w_.inverse().times(f); //May require transpose for times to work.
		return inv_x_f	;
	}
	//	  /// Projects Point from unit sphere (f) in camera pixels (c).
	//	  inline Vector2d f2c(const Vector3d& f) const { return cam_->world2cam( f ); }
	public Vector2d f2c(Vector3d f)	//changed from vector2d
	{
		return cam_.world2cam(f);
	}
	//	  /// Return the pose of the frame in the (w)orld coordinate frame.
	//	  inline Vector3d pos() const { return T_f_w_.inverse().translation(); }
	public Vector3d pos()	//changed from gvector
	{
		return (Vector3d) T_f_w_.inverse().get_Translation();
	}
	//		   Frame jacobian for projection of 3D point in (f)rame coordinate to
	//		   unit plane coordinates uv (focal length = 1).

	//J should be of size 2x6
	//This sets the J values of the reference, not an actual field. Values are immediately lost
	public static void jacobian_xyz2uv(Vector3d xyz_in_f, Matrix J)	//changed from GVector xyz_in_f, Matrix J
	{
		// ToDo currently doesnt set anything because J is lost as soon as the method completes. Need to
		// return a Matrix and set that as J.
		double x = xyz_in_f.get(0,0);
		double y = xyz_in_f.get(0,1);
		double z_inv = 1/xyz_in_f.get(0,2);
		double z_inv_2 = z_inv*z_inv;

		J.set(0, 0, -z_inv);								// -1/z
		J.set(0, 1, 0.0);								// 0
		J.set(0, 2, x*z_inv_2);							// x/z^2
		J.set(0, 3, y*J.get(0, 2));				// x*y/z^2
		J.set(0, 4, -(1.0 +x*J.get(0, 2)));		// -(1.0 + x^2/z^2)
		J.set(0, 5, y*z_inv);							// y/z

		J.set(1, 0, 0.0);
		J.set(1, 1, -z_inv);
		J.set(1, 2, y*z_inv_2);
		J.set(1, 3, 1.0+y*J.get(0,2));
		J.set(1, 4, -(1.0+x*J.get(0,2)));
		J.set(1, 5, -x*z_inv);	
	}
	//



	//fields and methods I assumed were necessary//////////////////////
	private Vector3d pos;		//Presuming Vector3d

	public Se3 getT_f_w_(){
		return T_f_w_;
	}
	public void setT_f_w_(Se3 se3){
		this.T_f_w_ = se3;
	}
	public AbstractCamera getCam()
	{
		return cam_;
	}

	private double max(double a, double b)
	{
		if(a>=b) return a;
		if(a<b) return b;
		return 0;	//should never get here
	}
	private int max(int a, int b)
	{
		if(a>=b) return a;
		if(a<b) return b;
		return 0;	//should never get here
	}
	//Get methods used predominantly for testing
	public boolean getIsKeyFrame()
	{
		return is_keyframe_;
	}
	public ArrayList<Feature> getkey_pts_()
	{
		return key_pts_;
	}
	public ArrayList<Feature> getFts_()
	{
		return fts_;
	}
	public void setFts(ArrayList<Feature> features)
	{
		this.fts_ = features;
	}
	public int getId_()
	{
		return id_;
	}
	public ImgPyr get_img_pyr()
	{
		return img_pyr_;
	}

	public void setCov_(Matrix6d matrix_cov)
	{
		Cov_ = matrix_cov;
	}

// Is already in Frame_utils.java	
//	private void createImgPyramid(Mat img_level_0, int n_levels, ImgPyr pyr)
//	{
//		pyr.resize(n_levels);
//		pyr.getMats()[0] = img_level_0;
//		for(int i=1; i<n_levels; ++i)
//		{
//			pyr.getMats()[i] = new Mat(pyr.getMats()[i-1].rows()/2, pyr.getMats()[i-1].cols()/2, CvType.CV_8U);
//			Vision.halfSample(pyr.getMats()[i-1], pyr.getMats()[i]);
//		}
//	}
}

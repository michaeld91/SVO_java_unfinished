package SVO_1310;

import java.util.ArrayList;
import org.opencv.core.Point;

import matrix_types.Vector2d;
import matrix_types.Vector3d;
//import SVO_1310.CV.Point2f;
import SVO_1310.Vikit.math_utils;
import Sophus.Se3;


//Tracks features using Lucas-Kanade tracker and then estimates a homography.
//Note the use of both OpenCv::Point (rather than Point2f) and regular Point
// ToDo not sure that the extends Initialization is required
public class KltHomographyInit extends Initialization{
	

	private Frame frame_ref_;
	
//	private	Point[] px_ref_;//!< keypoints to be tracked in reference frame.
//	private Point[] px_cur_;//!< tracked keypoints in current frame.
//	private Vector3d[] f_ref_;//!< bearing vectors corresponding to the keypoints in the reference image.
//	private Vector3d[] f_cur_;//!< bearing vectors corresponding to the keypoints in the current image.
//	private ArrayList<Double> disparities_;//!< disparity between first and second frame.
	private int[] inliers_;//!< inliers after the geometric check (e.g., Homography).
	private Vector3d[] xyz_in_cur_;//!< 3D points computed during the geometric check.
	private Se3 T_cur_from_ref_;//!< computed transformation between the first two frames.

	public KltHomographyInit() {}
	public ArrayList<Vector3d> get_f_cur_()
	{
		return f_cur_;
	}
	public ArrayList<Vector3d> get_f_ref_()
	{
		return f_ref_;
	}
//	public ArrayList<Point> get_px_cur_()
//	{
//		return px_cur_;
//	}
	public Point[] get_px_cur_()
	{
		return px_cur_.toArray();
	}
	public Point[] get_px_ref_()
	{
		return px_ref_.toArray();
	}
	public InitResult addFirstFrame(Frame frame_ref)
	{
		reset();
		detectFeatures(frame_ref); // px_ref_ and f_ref_ are set in the super class when this method is called.
		if(get_px_ref_().length < 100)
		{
			// ToDo		    SVO_WARN_STREAM_THROTTLE(2.0, "First image has less than 100 features. Retry in more textured environment.");
			return InitResult.FAILURE;	// ToDo this should probably throw an error to be caught when addFirstFrame is called
		}
		frame_ref_ = frame_ref;
		
		Point[] px_cur_new_array = new Point[get_px_ref_().length];
		for(int i = 0; i< get_px_ref_().length; i++) //px_cur_.insert(px_cur_[0], px_ref_[0], px_ref_[px_ref_.length-1]);
		{
			px_cur_new_array[i] = get_px_ref_()[i]; //			px_cur_.set(i,get_px_ref_()[i]); 
		}
		px_cur_.fromArray(px_cur_new_array);
		
		return InitResult.SUCCESS;
	}
	public InitResult addSecondFrame(Frame frame_cur)
	{
		trackKlt(frame_ref_, frame_cur);	// ToDo Set the remaining variables in Initialization //, px_ref_, px_cur_, f_ref_, f_cur_, disparities_);
		// ToDo		  SVO_INFO_STREAM("Init: KLT tracked "<< disparities_.size() <<" features");
		
		if(disparities_.size() < Config.getInit_min_tracked())
			return InitResult.FAILURE;
		
		double disparity = math_utils.getMedian(disparities_);
		// ToDo		  SVO_INFO_STREAM("Init: KLT "<<disparity<<"px average disparity.");
		if(disparity < Config.getInit_min_disparity())
			return InitResult.NO_KEYFRAME;
		
		computeHomography(f_ref_, f_cur_, frame_ref_.getCam().errorMultiplier2(), Config.getPoseoptim_thresh(), inliers_, xyz_in_cur_, T_cur_from_ref_);
		// ToDo		  SVO_INFO_STREAM("Init: Homography RANSAC "<<inliers_.size()<<" inliers.");

		if(inliers_.length < Config.getInit_min_inliers())
		{
		//	ToDo	    SVO_WARN_STREAM("Init WARNING: "<<Config::initMinInliers()<<" inliers minimum required.");
			return InitResult.FAILURE;
		}
		
		// Rescale the map such that the mean scene depth is equal to the specified scale
		ArrayList<Double> depth_vec = new ArrayList<Double>();
		for(int i =0; i<xyz_in_cur_.length; i++)
			depth_vec.add(xyz_in_cur_[i].get(2));
		double scene_depth_median = math_utils.getMedian(depth_vec);
		double scale = Config.getMap_scale()/scene_depth_median;
		frame_cur.setT_f_w_(T_cur_from_ref_.times(frame_ref_.getT_f_w_()));
		frame_cur.getT_f_w_().set_Translation(new Vector3d((frame_cur.getT_f_w_().rotation_Matrix().times(-1)).times(frame_ref_.pos().plus(frame_cur.pos().minus(frame_ref_.pos()).times(scale))).getArray()));
		
		// For each inlier create 3D point and add feature in both frames
		Se3 T_world_cur = frame_cur.getT_f_w_().inverse();
		for(int i = 0; i< inliers_.length; i++)
		{
			Vector2d px_cur = new Vector2d(); 
			px_cur.set(get_px_cur_()[i].x, get_px_cur_()[i].y);
			Vector2d px_ref = new Vector2d(); 
			px_ref.set(get_px_ref_()[i].x, get_px_ref_()[i].y);
			if(frame_ref_.getCam().isInFrame(px_cur, 10) && frame_ref_.getCam().isInFrame(px_ref, 10) && xyz_in_cur_[i].get(2)> 0)
			{
				Vector3d pos = new Vector3d(T_world_cur.times(xyz_in_cur_[i].times(scale)).getArray());
				SVO_1310.Point new_point = new SVO_1310.Point(pos);	// ToDo may need to clear up what each type of Point is. Bad pratice to have a OpenCV::Point and a SVO::Point
				
				Feature ftr_cur = new Feature(frame_ref_, new_point, px_cur, f_cur_.get(i), 0);
				frame_cur.addFeature(ftr_cur);
				new_point.addFrameRef(ftr_cur);
				
				Feature ftr_ref = new Feature(frame_ref_, new_point, px_ref, f_ref_.get(i), 0);
				frame_ref_.addFeature(ftr_ref);
				new_point.addFrameRef(ftr_ref);
			}
		}
		return InitResult.SUCCESS;
	}

	public void reset()
	{
		px_cur_ = null;
		frame_ref_ = null;
	}
}

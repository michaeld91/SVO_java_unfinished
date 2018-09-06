package SVO_1310;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Iterator;

import org.opencv.core.MatOfByte;
import org.opencv.core.MatOfFloat;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Size;
import org.opencv.core.TermCriteria;
import org.opencv.video.Video;

//import SVO_1310.CV.Point2f;
//import SVO_1310.CV.TermCriteria;
import SVO_1310.Vikit.Homography;
import SVO_1310.Vikit.math_utils;
import Sophus.Se3;
import matrix_types.Vector2d;
import matrix_types.Vector3d;


// NOTE: This class uses openCV::Point (rather than Point2f) NOT SVO: Point
/// Bootstrapping the map from the first two views.
public class Initialization {

	//////////////////////////////////////////////////////////////////////////////////////////////////////
	// 5 Fields taken from kltHomography to attempt to solve the referenceing issue
	//protected ArrayList<Point> px_ref_ = new ArrayList<Point>();//!< keypoints to be tracked in reference frame.
	MatOfPoint2f px_ref_ = new MatOfPoint2f();
	//protected ArrayList<Point> px_cur_= new ArrayList<Point>();//!< tracked keypoints in current frame.
	MatOfPoint2f px_cur_ = new MatOfPoint2f();
	protected ArrayList<Vector3d> f_ref_ = new ArrayList<Vector3d>();//!< bearing vectors corresponding to the keypoints in the reference image.
	protected ArrayList<Vector3d> f_cur_ = new ArrayList<Vector3d>();//!< bearing vectors corresponding to the keypoints in the current image.
	protected ArrayList<Double> disparities_;//!< disparity between first and second frame.
	//////////////////////////////////////////////////////////////////////////////////////////////////////
	protected ArrayList<Point> px_vec = new ArrayList<Point>();	// Field to be set from detectFeatures(...) method to solve the c++ 'Address-of' & issue.
	protected ArrayList<Vector3d> f_vec = new ArrayList<Vector3d>(); // Field to be set from detectFeatures(...) method to solve the c++ 'Address-of' & issue.
	
	
	// Detect Fast corners in the image.
	public void detectFeatures(Frame frame)//, ArrayList<Point2f> px_vec, ArrayList<Vector3d> f_vec)
	{
		ArrayList<Feature> new_features = new ArrayList<Feature>();//Feature[] new_features;
		FastDetector detector = new FastDetector(frame.get_img_pyr().getMats()[0].cols(),frame.get_img_pyr().getMats()[0].rows(),
				Config.getGridSize(), Config.getnPyrLevels());

		// ToDo needs to set the new_features array
		detector.detect(frame, frame.get_img_pyr(), Config.getTriang_min_corner_score(), new_features);

		// now for all maximum corners. initialize a new seed
		px_vec.clear();				// ToDo this will only clear the reference, not the actual Point[]
		px_vec = new ArrayList<Point>(new_features.size());	
		f_vec.clear(); 				// ToDo this will only clear the reference, not the actual Point[]
		f_vec = new ArrayList<Vector3d>(new_features.size());	
		for(int i = 0; i < new_features.size(); i++)
		{
			px_vec.set(px_vec.size(),new Point(new_features.get(i).get_px().get(0), new_features.get(i).get_px().get(1)));	// ToDo this should be a CV:: Point2f(ftr->px[0], ftr->px[1])
			f_vec.set(f_vec.size()-1,new_features.get(i).getF());
		}
		new_features.clear();
	}
	
	
	/// Compute optical flow (Lucas Kanade) for selected keypoints.
	// double[] disparites changed to ArrayList<Double> and disparities.add has been changed also
	public void trackKlt(Frame frame_ref, Frame frame_cur)//, Point[] px_ref, Point[] px_cur, Vector3d[] f_ref, Vector3d[] f_cur, ArrayList<Double> disparities)
	{
		final double klt_win_size = 30.0;
		final int klt_max_iter = 30;
		final double klt_eps = 0.001;
		MatOfByte status = new MatOfByte();	//char[] status;
		MatOfFloat error = new MatOfFloat();
		float[] min_eig_vec;
		// ToDo TermCriteria from /home/michael/opencv/build/src/org/opencv/core
		TermCriteria termcrit = new TermCriteria(TermCriteria.COUNT + TermCriteria.EPS, klt_max_iter, klt_eps);
		// Size2i changed to OpenCV.size(....)
		// Video.calcOpticalFlowPyrLk(frame_ref.img(), frame_cur.img(), px_ref_, px_cur_, status, error, new Size(klt_win_size, klt_win_size), 4, termcrit, Video.OPTFLOW_USE_INITIAL_FLOW);
		// Using default minEigThresh 1e-4 as the last value (this is left out of the c++ version as is set automatically when left blank)
		Video.calcOpticalFlowPyrLK(frame_ref.img(), frame_cur.img(), px_ref_, px_cur_, status, error, new Size(klt_win_size, klt_win_size), 4, termcrit, Video.OPTFLOW_USE_INITIAL_FLOW, Math.pow(1, -4));

//		Iterator<Point> px_ref_iterator = px_ref_.iterator();
//		Iterator<Point> px_cur_iterator = px_cur_.iterator();
//		Iterator<Vector3d> f_ref_iterator = f_ref_.iterator();
//		f_cur_.clear();	// ToDo f_cur.clear();
//		f_cur_ = new ArrayList<Vector3d>(px_cur_.size());
//		disparities_.clear();		// ToDo disparities.clear();
//		disparities_ = new ArrayList<Double>(px_cur_.size());
//
//		Point px_ref_it = px_ref_iterator.next();
//		Point px_cur_it = px_cur_iterator.next();
//		Vector3d f_ref_it = f_ref_iterator.next();
//		
//		for(int i = 0; i < px_ref_.size(); i++)
//		while(px_ref_iterator.hasNext())
//		{
//			if(status.toArray()[i]==0)	// ToDo ensure this is equal to (!status[i])
//			{
//				px_ref_.remove(px_ref_it);	// erase() returns an iterator to the next available object in c++
//				px_cur_.remove(px_cur_it);
//				f_ref_.remove(f_ref_it);
//				continue;
//			}
//			// f_cur_.set(f_cur_.size()-1, frame_cur.c2f(px_ref_it.get(0), px_cur_it.get(1)));
//			f_cur_.add(frame_cur.c2f(px_ref_it.x, px_cur_it.y));
//
//			Vector2d px_ref_minus_cur = new Vector2d();
//			px_ref_minus_cur.set(px_ref_it.x-px_cur_it.x, px_ref_it.y - px_cur_it.y);
//			disparities_.add(px_ref_minus_cur.normF());
//			px_ref_it = px_ref_iterator.next();
//			px_cur_it = px_cur_iterator.next();
//			f_ref_it = f_ref_iterator.next();
//		}
		
		
		////////////////////////////////////////////////////////////////////////////////

		
		Point[] px_ref_array = px_ref_.toArray();
		Point[] px_cur_array = px_cur_.toArray();
		f_cur_ = new ArrayList<Vector3d>(px_cur_array.length);

		for(int i = 0; i<px_ref_array.length; i++)
		{
			if(status.toArray()[i] == 0)
			{
				px_ref_array[i] = null;	//       px_ref_it = px_ref.erase(px_ref_it);
				px_cur_array[i] = null;
				f_ref_.remove(i);
				continue;
			}
			
			f_cur_.add(frame_cur.c2f(px_ref_array[i].x, px_cur_array[i].y));
			Vector2d px_ref_minus_cur = new Vector2d();
			px_ref_minus_cur.set(px_ref_array[i].x-px_cur_array[i].x, px_ref_array[i].y - px_cur_array[i].y);
			disparities_.add(px_ref_minus_cur.normF());
		}
		
		
		
		
		
		
		
		
		
		/////////////////////////////////////////////////////////////////////////////////
	}

	// ToDo outliers is set in the c++ method computeInliers(). 
	public void computeHomography(ArrayList<Vector3d> f_ref_2, ArrayList<Vector3d> f_cur_2, double focal_length, double reprojection_threshold, int[] inliers, Vector3d[] xyz_in_cur, Se3 T_cur_from_ref)
	{	
		Vector2d[] uv_ref = new Vector2d[f_ref_2.size()];
		Vector2d[] uv_cur = new Vector2d[f_cur_2.size()];
		for(int i = 0; i < f_ref_2.size(); i++)
		{
			uv_ref[i] = math_utils.project2d(f_ref_2.get(i));
			uv_ref[i] = math_utils.project2d(f_cur_2.get(i));
		}
		Homography homography = new Homography(uv_ref, uv_cur, focal_length, reprojection_threshold); 		// ToDo	vk::Homography Homography(uv_ref, .......

		homography.computeSE3fromMatches();
		int[] outliers = null;	// ToDo depending on approach to ViKit replacing, this will need changing.
		Vector3d homog_trans = new Vector3d(homography.getT_c2_from_c1().get_Translation().getArray());	// ToDo need a check here that homography.getT_c2.... is a vector of length 3.
		
		math_utils.computeInliers(f_cur_2, f_ref_2, homography.getT_c2_from_c1().rotation_Matrix(), homog_trans, reprojection_threshold, focal_length, xyz_in_cur, inliers, outliers);
		T_cur_from_ref = homography.getT_c2_from_c1();
		
	}
	


}

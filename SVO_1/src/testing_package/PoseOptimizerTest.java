package testing_package;

import java.util.ArrayList;

import matrix_types.Matrix3d;
import matrix_types.Vector2d;
import matrix_types.Vector3d;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.features2d.FeatureDetector;
import org.opencv.imgcodecs.Imgcodecs;

import SVO_1310.Config;
import SVO_1310.FastDetector;
import SVO_1310.Feature;
import SVO_1310.Feature_alignment;
import SVO_1310.Frame;
import SVO_1310.Point;
import SVO_1310.PoseOptimizer;
import SVO_1310.Vikit.AbstractCamera;
import SVO_1310.Vikit.Blender_Utils;
import SVO_1310.Vikit.Pinhole_Camera;
import SVO_1310.Vikit.Sample;
import Sophus.Quaternion;
import Sophus.Se3;

public class PoseOptimizerTest {
	AbstractCamera cam_;
	Mat depthmap_;
	Frame frame_;
	public PoseOptimizerTest()
	{
		cam_ = new Pinhole_Camera(752, 480, 315.5, 315.5, 376.0, 240.0);
		
		// load image
		String dataset_dir = "/home/michael/Documents/SVO_datasets/sin2_tex2_h1_v8_d/";
		String img_name = "img/frame_000002_0.png";
		
		System.out.println("Loading image "+ img_name);
		Mat img = Imgcodecs.imread(dataset_dir+img_name, Imgcodecs.CV_LOAD_IMAGE_GRAYSCALE); // CV_LOAD_IMAGE_GREYSCALE is the same as 0.
		if(!img.empty())
		{
			// create frame
			frame_ = new Frame(cam_, img, 1.0);
			
			// set pose
			Vector3d t_w_ref = new Vector3d();
			t_w_ref.set(0.1131, 0.1131, 2.0000);
			double[] quat_vals = {0.0, 0.8227, 0.2149, 0.0};
			Quaternion q_w_ref = new Quaternion(quat_vals);
			Se3 new_se3 = new Se3(q_w_ref, t_w_ref);//new Se3(q_w_ref, t_w_ref);
			frame_.setT_f_w_(new_se3.inverse());
//			System.out.println("new_se3.inverse() = ");
//			new_se3.inverse().print();
			
			// load ground-truth depth
			depthmap_ = Blender_Utils.loadBlenderDepthMap(dataset_dir + "/depth/frame_000002_0.depth", cam_);
		

			
			////////////////////////////////////////Change back when using the feature detection is wanted///////////////////////////////////
			// detect features
//			FastDetector detector = new FastDetector(cam_.getWidth(), cam_.getHeight(), Config.getGridSize(), Config.getnPyrLevels());
//			detector.detect(frame_, frame_.get_img_pyr(), Config.getTriang_min_corner_score(), frame_.getFts_());
			Testing_utilities.read_features(frame_);	// ToDo:: Loads the features found by the c++ detector, so the differences of using the Java feature detection don't affect results.

			int n_fts = 0;
			for(Feature i: frame_.getFts_())
			{
// 			      Point* point(new Point(frame_->f2w(i->f*depthmap_.at<float>(i->px[1], i->px[0])), i));
//				System.out.println("frame_.f2w(..)= "+
//				frame_.f2w(i.getF().times(depthmap_.get((int)i.get_px().get(1),(int)i.get_px().get(1))[0])).get(0)+","+
//				frame_.f2w(i.getF().times(depthmap_.get((int)i.get_px().get(1),(int)i.get_px().get(1))[0])).get(1)+","+
//				frame_.f2w(i.getF().times(depthmap_.get((int)i.get_px().get(1),(int)i.get_px().get(1))[0])).get(2)				
//				);
//				System.out.println("depthmap = "+depthmap_.get((int)i.get_px().get(1),(int)i.get_px().get(0))[0]);
//				System.out.println("i.get_px x,y = "+i.get_px().get(0)+","+i.get_px().get(1));
				Point point = new Point(frame_.f2w(i.getF().times(depthmap_.get((int)i.get_px().get(1),(int)i.get_px().get(0))[0])) , i);
				i.setPoint(point);
				n_fts++;
			}
			System.out.println("Added "+n_fts+" features to the frame.");
		} 
	}
	
	public void test(Vector3d pose_disturbance, double pixel_sigma2)
	{
		System.out.println("Add "+pixel_sigma2+" px noise to each observation.");
//		System.out.println("feature coord. = "+frame_.getFts_().get(0).get_px(0)+","+frame_.getFts_().get(0).get_px(1));
		
		// Java Gaussian Feature coordinates :: ToDo this should be used rather than the c++ version 10 lines below.
//		for(Feature it: frame_.getFts_())
//		{	
//			Vector2d px_vec = new Vector2d();
//			px_vec.set(Sample.gaussian(pixel_sigma2), Sample.gaussian(pixel_sigma2));
//			it.set_px(it.get_px().plus(px_vec));
//			it.set_F(frame_.c2f(it.get_px()));
//			System.out.println("feature coord. = "+it.get_px(0)+","+it.get_px(1));
//		}
//		System.out.println("feature coord. = "+frame_.getFts_().get(0).get_px(0)+","+frame_.getFts_().get(0).get_px(1));

		// C++ loaded Gaussian Feature Coordinates.
		ArrayList<Vector2d> gaus_vecs = Testing_utilities.read_gaussian_features();
		int j = 0;
		for(Feature it: frame_.getFts_())
		{	
			it.set_px(gaus_vecs.get(j));
			it.set_F(frame_.c2f(it.get_px()));
			j++;
//			System.out.println("frame_.c2f(...) = "+frame_.c2f(it.get_px()).get(0)+","+frame_.c2f(it.get_px()).get(1)+","+frame_.c2f(it.get_px()).get(2));
//			System.out.println("feature coord. = "+it.get_px(0)+","+it.get_px(1));
		}
		
		
		
		Matrix3d mat_ident = new Matrix3d();
		mat_ident.identity();
		Se3 ident_disturbance = new Se3(mat_ident, pose_disturbance);
		frame_.setT_f_w_(frame_.getT_f_w_().times(ident_disturbance));
		double estimated_scale, error_init, error_final;
		int num_obs;
		double[] values = PoseOptimizer.optimizeGaussNewton(Config.getReproj_thresh(), 10, true, frame_); //10, true, frame_);
		estimated_scale = values[0];
		error_init = values[1];
		error_final = values[2];
		num_obs = (int)values[3];
	}
	
}

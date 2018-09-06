package testing_package;


import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.OutputStream;
import java.io.PrintStream;
import java.util.ArrayList;
import java.util.Collections;

import matrix_types.Vector2d;
import matrix_types.Vector3d;

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.imgcodecs.Imgcodecs;

import SVO_1310.Depth;
import SVO_1310.Feature;
import SVO_1310.Frame;
import SVO_1310.Matcher;
import SVO_1310.Vikit.AbstractCamera;
import SVO_1310.Vikit.Blender_Utils;
import SVO_1310.Vikit.Pinhole_Camera;
import SVO_1310.Vikit.Timer;
import Sophus.Quaternion;
import Sophus.Se3;

public class test_matcher {

	private static Mat depth_ref_;
	private static AbstractCamera cam_;
	private static Frame frame_ref_;
	private static Frame frame_cur_;
	private static Feature ref_ftr_;

	public static void main(String[] args)
	{
		System.loadLibrary(Core.NATIVE_LIBRARY_NAME);	

		setup_test_matcher();
		testEpipolarSearchFullImg();
		//  test.testWarpAffine();

	}
	public static void setup_test_matcher()
	{
		cam_ = new Pinhole_Camera(752, 480, 315.5, 315.5, 376.0, 240.0);
//		System.out.println("cam_ width,height ="+ cam_.getWidth()+","+cam_.getHeight());
		// load images
		String dataset_dir = "/home/michael/Documents/SVO_datasets/sin2_tex2_h1_v8_d/";
		String img_name = "img/frame_000002_0.png"; 
		System.out.println("Loading image "+img_name);
		Mat img_ref = Imgcodecs.imread(dataset_dir+img_name, Imgcodecs.CV_LOAD_IMAGE_GRAYSCALE); // CV_LOAD_IMAGE_GREYSCALE is the same as 0.
		String img_name_2 = "img/frame_000006_0.png";
		System.out.println("Loading image "+img_name_2);
		Mat img_cur = Imgcodecs.imread(dataset_dir+img_name_2, Imgcodecs.CV_LOAD_IMAGE_GRAYSCALE); // CV_LOAD_IMAGE_GREYSCALE is the same as 0.

		// If both images are not empty create frame, set poses and load ground truth.
		if(!img_ref.empty()||!img_cur.empty())
		{

			// Create Frame
			frame_ref_ = new Frame(cam_, img_ref, 1.0);
			frame_cur_ = new Frame(cam_, img_cur, 2.0);
			Vector2d vec2d = new Vector2d();
			vec2d.set(300, 260);
			ref_ftr_ = new Feature(frame_ref_, vec2d, 0);

			// Set Poses
			Vector3d t_w_ref = new Vector3d();
			t_w_ref.set(0.1131, 0.1131, 2.0000);
			Vector3d t_w_cur = new Vector3d();
			t_w_cur.set(0.5673, 0.5641, 2.0000);
			double[] quat_ref_vals = {0.0, 0.8227, 0.2149, 0.0};
			Quaternion q_w_ref = new Quaternion(quat_ref_vals);
			double[] quat_cur_vals = {0.0, 0.8235, 0.2130, 0.0};
			Quaternion q_w_cur = new Quaternion(quat_cur_vals);
			Se3 frame_ref_se3 = new Se3(q_w_ref, t_w_ref);
			///////////////
//			System.out.println("q_w_ref = "); q_w_ref.print();		// For testing, can be removed.
//			System.out.println("t_w_ref = "); t_w_ref.print(0, 4);
			////////////////
			frame_ref_.setT_f_w_(frame_ref_se3.inverse());
			Se3 frame_cur_se3 = new Se3(q_w_cur, t_w_cur);
			frame_cur_.setT_f_w_(frame_cur_se3.inverse());
/////////////////////////////////////////////////////////////////////////////////////////////
//			System.out.println("frame_cur_se3");
//			frame_cur_se3.print();

			
//			System.out.println("frame_cur_se3.inverse()");
//			frame_cur_se3.inverse().print();
/////////////////////////////////////////////////////////////////////////////////////////////			
			// Load ground-truth depth
			depth_ref_ = Blender_Utils.loadBlenderDepthMap(dataset_dir+"depth/frame_000002_0.depth", cam_);//, depth_ref_);
//			Mat_to_file printer = new Mat_to_file(depth_ref_, "/home/michael/Documents/SVO_datasets/depth_mat.txt");	// for testing only
		}
	}

	public static void testEpipolarSearchFullImg()
	{
		System.loadLibrary(Core.NATIVE_LIBRARY_NAME);	
		
		Matcher matcher = new Matcher();
		Mat depthMap = new Mat(frame_ref_.getCam().getHeight(), frame_ref_.getCam().getWidth(), CvType.CV_32FC1, new Scalar(0));
		Mat error_map = new Mat(depthMap.rows(), depthMap.cols(), CvType.CV_32FC1, new Scalar(0));
		Mat mask = new Mat(depthMap.rows(), depthMap.cols(), CvType.CV_8UC1, new Scalar(1));
		double depth_range = 0.8;
		int n_converged = 0;
		double sum_error = 0;
		ArrayList<Double> errors = new ArrayList<Double>(depthMap.rows()*depthMap.cols());
		for(int y = 4; y<10; y++)	//for(int y = 4; y<depthMap.rows()-4; y++)	// ToDo the y = value should be 4. Set to higher for testing only.
		{
			
			for(int x = 4; x<depthMap.cols()-4; x++) //for(int x = 4; x<depthMap.cols()-4; x++) //  ToDo 
			{
//				////////////////////////////////////////timer print out 2 y loop timer //////////////////////////////////////////////////////////
//				Timer x_loop_timer = new Timer();
//				x_loop_timer.start();

				Vector2d vec_xy = new Vector2d(); vec_xy.set(x, y);
				Feature ftr = new Feature(frame_ref_, vec_xy, 0);
//				System.out.println("depth_ref_ cols,rows "+depth_ref_.cols()+","+depth_ref_.rows());
				double depth_gt = depth_ref_.get(y, x)[0]; // .get(x,y) returns an array, the array should only have one value (as it is a single channel)
				Depth depth_estimate = new Depth(0.0); //double depth_estimate = 0;
//				System.out.println("depth_gt " +depth_gt);
				boolean res = false;
				//if(depth_gt!=0)	// ToDo this if statement has been added to skip occurrences when the depth is not found from the image.
					res = matcher.findEpipolarMatchDirect(frame_ref_, frame_cur_, ftr, depth_gt, Math.max(depth_gt-depth_range, 0), depth_gt+depth_range, depth_estimate);// true; //
//				if(res==true)
//					System.out.println("res = "+res);
////				System.out.println("x,y = "+x+","+y);
				if(res)
				{
					depthMap.put(y, x, depth_estimate.getDepth());
					error_map.put(y, x, Math.abs(depth_estimate.getDepth()-depth_gt));
//					System.out.println("error value = " + Math.abs(depth_estimate.getDepth()-depth_gt));
//					System.out.println("depth_estimate.getDepth() = " + depth_estimate.getDepth() + "\tdepth_gt = " + depth_gt);

//					System.out.println("y,x = " + y +","+x);
					n_converged++;
					sum_error += Math.abs(depth_estimate.getDepth()-depth_gt);//error_map.get(y, x)[0];
					errors.add(error_map.get(y, x)[0]);
				}
				else
					mask.put(y, x, 0);
//				////////////////////////////////////////timer print out 2 y loop timer //////////////////////////////////////////////////////////
//				System.out.println("["+x+"] position 2 x loop time = " + x_loop_timer.stop()); x_loop_timer.resume();
			}				
			System.out.println("Row #"+y+" completed");
			
		}
		
		// compute mean, median and variance of error in converged area
		System.out.println("n converged: \t" + n_converged + " mm (ref: 216114)");
		System.out.println("mean error: \t" + sum_error*100/n_converged + " mm (ref: 0.410084)");
		Collections.sort(errors);

		int fifty_perc = (int) (errors.size()*.5);
		System.out.println("50-percentile: \t"+errors.get(fifty_perc)*100+" mm (ref:0.083203");
		int eighty_perc = (int) (errors.size()*.8);
		System.out.println("80-percentile: \t"+errors.get(eighty_perc)*100+" mm (ref: 0.161824)");
		int ninetyfive_perc = (int) (errors.size()*.95);
		System.out.println("95-percentile: \t"+errors.get(ninetyfive_perc)*100+" mm (ref: 0.263539)");

		// Save results to file
		try {
			//			PrintStream output_stream = new PrintStream(new FileOutputStream("output.txt"));
			//			System.setOut(output_stream);
			//			String trace_dir = "/home/michael/Documents/SVO_datasets/tmp";
			//			String output_filename = trace_dir + "/depthmap.bin";

			OutputStream output = new FileOutputStream("/home/michael/Documents/SVO_datasets/tmp/depthmap.bin");
			PrintStream depthmap_bin = new PrintStream(output);
			//			for(int i = 0; i< depthMap.cols()*depthMap.rows(); i++)
			//			{
			//				double[] depthmap_data = depthMap.get(0, 0)
			//				depthmap_bin.println(depthMap.get(row, col, data))
			//			}
			depthmap_bin.print("Hello" +depthMap.get(0,0));
			depthmap_bin.close();



		} catch (FileNotFoundException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}


	}











}

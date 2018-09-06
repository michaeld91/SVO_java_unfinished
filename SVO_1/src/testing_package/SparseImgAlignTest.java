package testing_package;

import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.OutputStream;
import java.io.PrintStream;
import java.util.ArrayList;
import java.util.Iterator;

import matrix_types.Matrix3d;
import matrix_types.Vector3d;

import org.opencv.core.Mat;
import org.opencv.imgcodecs.Imgcodecs;

import SVO_1310.Config;
import SVO_1310.FastDetector;
import SVO_1310.Feature;
import SVO_1310.Frame;
import SVO_1310.Point;
import SVO_1310.SparseImgAlign;
import SVO_1310.Vikit.AbstractCamera;
import SVO_1310.Vikit.Blender_Utils;
import SVO_1310.Vikit.FileReader;
import SVO_1310.Vikit.ImageNameAndPose;
import SVO_1310.Vikit.Method;
import SVO_1310.Vikit.Pinhole_Camera;
import SVO_1310.Vikit.Timer;
import Sophus.Quaternion;
import Sophus.Se3;
import Sophus.So3;

public class SparseImgAlignTest 
{
	private static Pinhole_Camera cam_;
	private static Frame frame_ref_;
	private static Frame frame_cur_;

	public SparseImgAlignTest()
	{
		cam_ = new Pinhole_Camera(752, 480, 315.5, 315.5, 376.0, 240.0);
	}

	public AbstractCamera getCam()
	{
		return cam_;
	}

	public void testSequence(String dataset_dir, String experiment_name, FastDetector feature_detector) throws IOException, CloneNotSupportedException 
	{
//		double[] quat_vals = {1,0.25,0,-0.25};
//		Quaternion quat = new Quaternion(quat_vals);
//		Vector3d vec = new Vector3d(1);
//		vec.print(0, 0);
//		
//		So3 so3_new = new So3(quat);
//		
////		System.out.println("so3 = " +so3_new.toString());
//		System.out.println("so3 unit_quaternion = ");
//		so3_new.getQuaternion().print();
//
//		System.out.println("So3 times vec = ");
//		so3_new.times(vec).print(0, 4);
		
		
		
		
		////////////////////////////////////////////////////////////
		
		
		
		FileReader file_reader = new FileReader(dataset_dir+"/trajectory.txt");
		ArrayList<ImageNameAndPose> sequence = file_reader.readAllEntries();
		System.out.println("RUN EXPERIMENT: read "+sequence.size()+" dataset entries.");

//		for(ImageNameAndPose im:sequence)
//		{
//			im.getQ_().print();
//		}
		
		int seq_it = 0;	// replacement for iterator
		ArrayList<Double> translation_error = new ArrayList<Double>();
		Se3 T_prev_w = new Se3(new So3(new Matrix3d()), new Vector3d(0)); // Initialised as empty
		Se3 T_prevgt_w = new Se3(new So3(new Matrix3d()), new Vector3d(0)); // Initialised as empty
		String trace_name = "/tmp" + "/sparse_img_align_" + experiment_name + "_trans_estimate.txt";

		OutputStream ofs_ = new FileOutputStream(trace_name);		
		PrintStream myOutputFile = new PrintStream(ofs_);

		for(int i = 0; i<2 && seq_it<sequence.size(); seq_it++, i++)	//i<30
		{

			// load img
			// IMPORTANT: We need to flip the image because the Povray dataset has a
			// negative focal length in y direction which we didn't set.
			String img_name = dataset_dir + "/img/" + sequence.get(seq_it).getImage_name_() + "_0.png";
			Mat img = Imgcodecs.imread(img_name, Imgcodecs.CV_LOAD_IMAGE_GRAYSCALE); // CV_LOAD_IMAGE_GREYSCALE is the same as 0.
			if(img.empty())
			{
				break;
			}
			
			// load pose
			Se3 T_w_gt = new Se3(sequence.get(seq_it).getQ_(), sequence.get(seq_it).getT_());
			Se3 T_gt_w = T_w_gt.inverse();	// ground-truth
			
//		    System.out.println("T_w_gt = \t");/////////////////////////////////////////////////////////////////////////////////////
//		    T_w_gt.print();//////////////////////////////////////////////////////////////////////////////////////////////////////
//		    System.out.println("T_gt_w = \t");/////////////////////////////////////////////////////////////////////////////////////
//		    T_gt_w.print();//////////////////////////////////////////////////////////////////////////////////////////////////////
//		    System.out.print("sequence.get(seq_it).getT_() = \t");/////////////////////////////////////////////////////////////////////////////////////
//		    sequence.get(seq_it).getT_().print(0,6);//////////////////////////////////////////////////////////////////////////////////////////////////////
//		    System.out.print("sequence.get(seq_it).getQ_() = \t");/////////////////////////////////////////////////////////////////////////////////////
//		    sequence.get(seq_it).getQ_().print();//////////////////////////////////////////////////////////////////////////////////////////////////////

			
			if(i==0)
			{
				// set reference frame
				frame_ref_ = new Frame(cam_, img, 0.0);
				Se3 T_gt_w_copy = (Se3) T_gt_w.clone();//new Se3(T_gt_w.getSo3(), T_gt_w.get_Translation());
				frame_ref_.setT_f_w_(T_gt_w_copy);
				
//			    System.out.print("1 frame_ref_.getT_f_w_() = \t");/////////////////////////////////////////////////////////////////////////////////////
//			    frame_ref_.getT_f_w_().print();//////////////////////////////////////////////////////////////////////////////////////////////////////
//				
				// load ground-truth depth
				Mat depthmap = new Mat();
				depthmap = Blender_Utils.loadBlenderDepthMap(dataset_dir+"/depth/"+sequence.get(seq_it).getImage_name_()+"_0.depth", cam_);
				
				// extract features, generate features with 3D points.
//				feature_detector.detect(frame_ref_, frame_ref_.get_img_pyr(), Config.getTriang_min_corner_score(), frame_ref_.getFts_());
				Testing_utilities.read_features(frame_ref_);	// ToDo:: Loads the features found by the c++ detector, so the differences of using the Java feature detection don't affect results.

				for(Feature ft_i: frame_ref_.getFts_())
				{
					// ToDo cast the double values into int, so it can be used as indices
					double[] dmap_at_ = depthmap.get((int)ft_i.get_px().get(1), (int)ft_i.get_px().get(0));
					Vector3d pt_pos_cur = ft_i.getF().times(dmap_at_[0]); // ToDo presuming it is one channel, hence the 0
					Vector3d pt_pos_w = frame_ref_.getT_f_w_().inverse().times(pt_pos_cur);
					Point pt = new Point(pt_pos_w, ft_i);
					ft_i.setPoint(pt);
				}
				System.out.println("Added " + frame_ref_.nObs() + " 3d pts to the reference frame.");
				T_prev_w = new Se3(frame_ref_.getT_f_w_());//(Se3)frame_ref_.getT_f_w_().clone();
				T_prevgt_w = T_gt_w;
				System.out.println("i=0 loop finished");
				continue;
			}
			

			frame_cur_ = new Frame(cam_, img, 0.0);
		    //frame_cur_->T_f_w_ = frame_ref_->T_f_w_; // start at reference frame
		    // Below has been used as frame_ref was being altered accidentally in the run method.
			frame_cur_.setT_f_w_(T_prev_w);
		

		    // run image align
		    Timer t = new Timer();
		    SparseImgAlign img_align = new SparseImgAlign(Config.getKltMaxLevel(), Config.getKltMinLevel(), 30,	Method.GaussNewton, false, false);
//		    System.out.print("1 frame_ref_.getT_f_w_() = \t");/////////////////////////////////////////////////////////////////////////////////////
//		    frame_ref_.getT_f_w_().print();//////////////////////////////////////////////////////////////////////////////////////////////////////

		    img_align.run(frame_ref_, frame_cur_);

		    // compute error
		    Se3 T_f_gt = frame_cur_.getT_f_w_().times(T_gt_w.inverse());
		    translation_error.add(T_f_gt.get_Translation().normF());
		    
//		    System.out.println("\nframe_cur_.getT_f_w_() =");
//		    frame_cur_.getT_f_w_().print();
//		    System.out.println("\nT_gt_w.inverse() =");
//		    T_gt_w.inverse().print();
//		    System.out.println("\nT_f_gt");
//		    T_f_gt.print();
//		    System.out.print("\nT_f_gt.get_Translation() = \t");/////////////////////////////////////////////////////////////////////////////////////
//		    T_f_gt.get_Translation().print(0, 8);////////////////////////////////////////////////////////////////////////////////////////////////////

		    System.out.println("\n[ "+i+" ]\ttime = "+t.stop()+ "ms\t  |t| = "+frame_ref_.getT_f_w_().times(T_gt_w.inverse()).get_Translation().normF()+
		    		"\t translation error = "+translation_error.get(translation_error.size()-1));
//		    System.out.println("\n[ "+i+" ]\ttime = "+t.stop()+ "ms\t  |t| = "+T_prev_w.times(T_gt_w.inverse()).get_Translation().normF()+
//		    		"\t translation error = "+translation_error.get(translation_error.size()-1));
		    
		    
//		    System.out.print(i + " frame_ref_.getT_f_w_() = \t");/////////////////////////////////////////////////////////////////////////////////////
//		    frame_ref_.getT_f_w_().print();//////////////////////////////////////////////////////////////////////////////////////////////////////

		    ////////////////////////////////////////////////

//		    System.out.print("T_prev_w\n");		// A
//		    T_prev_w.print();		    
//////		    System.out.print("frame_ref_.getT_f_w_()\n");
////		    frame_ref_.getT_f_w_().print();
//		    System.out.println("T_gt_w");		// B
//		    T_gt_w.print();
//		    System.out.println("T_gt_w.inverse()");
//		    T_gt_w.inverse().print();
		    

//		    System.out.println("frame_cur_.getT_f_w_()");
//		    frame_cur_.getT_f_w_().print();
		    
		    // save old pose for the next iteration//
		    T_prev_w = new Se3(frame_cur_.getT_f_w_());//(Se3)frame_cur_.getT_f_w_().clone();
		    T_prevgt_w = T_gt_w;
		    
		    // ToDo this may need a toString() method or similar
		    myOutputFile.println(frame_cur_.getT_f_w_().inverse().get_Translation().transpose()+ " " + 
		    T_gt_w.inverse().get_Translation().transpose());
		    
		    
	    
		}
		ofs_.close();
		
		// trace error
		trace_name = "/tmp/sparse_img_align_" + experiment_name + "_trans_error.txt";
		ofs_ = new FileOutputStream(trace_name);
		PrintStream traceOutputFile = new PrintStream(ofs_);
		for(Double trans_error: translation_error)
		{
			traceOutputFile.println(trans_error);
		}
		ofs_.close();

		
		
//		for(int i = 0; i < sequence.size(); i++)
//		{
//			System.out.println("sequence("+i+") = "+ sequence.get(i).toString());
//		}
	}

//	public static void main(String[] args)
//	{
//		FastDetector fast = null;
//		try {
//			testSequence("/home/michael/Documents/SVO_datasets/sin2_tex2_h1_v8_d/", "Experiement 1", fast);
//		} catch (FileNotFoundException e) {
//			// TODO Auto-generated catch block
//			e.printStackTrace();
//		} catch (IOException e) {
//			// TODO Auto-generated catch block
//			e.printStackTrace();
//		}
//	}
}

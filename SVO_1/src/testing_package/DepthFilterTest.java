package testing_package;

import java.io.BufferedWriter;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.OutputStream;
import java.io.OutputStreamWriter;
import java.io.PrintWriter;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Iterator;

import matrix_types.Vector3d;

import org.opencv.core.Mat;
import org.opencv.features2d.FeatureDetector;
import org.opencv.imgcodecs.Imgcodecs;

import SVO_1310.Config;
import SVO_1310.DepthFilter;
import SVO_1310.FastDetector;
import SVO_1310.Frame;
import SVO_1310.Point;
import SVO_1310.Vikit.AbstractCamera;
import SVO_1310.Vikit.Blender_Utils;
import SVO_1310.Vikit.FileReader;
import SVO_1310.Vikit.ImageNameAndPose;
import SVO_1310.Vikit.Pinhole_Camera;
import SVO_1310.Vikit.Timer;
import Sophus.Se3;

public class DepthFilterTest {
	private ArrayList<ConvergedSeed> results_ = new ArrayList<ConvergedSeed>();
	private ArrayList<Double> errors_;// = new ArrayList<Double>();
	private int n_converged_seeds_;
	private AbstractCamera cam_;
	private DepthFilter depth_filter_;
	private Frame frame_ref_;
	private Frame frame_cur_;
	private Mat depth_ref_;




	public DepthFilterTest()
	{
		n_converged_seeds_ = 0;
		cam_ = new Pinhole_Camera(752, 480, 315.5, 315.5, 376.0, 240.0);
		depth_filter_ = null;
		errors_ = new ArrayList<Double>(1000);
	}	

	public void testReconstruction(String dataset_dir, String experiment_name) throws FileNotFoundException 
	{
		try{
			FileReader sequence_file_reader = new FileReader(dataset_dir + "/trajectory.txt");

			ArrayList<ImageNameAndPose> sequence;
			//		sequence_file_reader.skipComments();
			//		if(!sequence_file_reader.next())
			//			throw new RuntimeException("Failed to open sequence File");
			sequence = sequence_file_reader.readAllEntries();
			System.out.println("RUN EXPERIMENT: read " + sequence.size() + " dataset entries.");
			Iterator<ImageNameAndPose> it = sequence.iterator(); 
			Timer t = new Timer();
			ArrayList<Integer> n_converged_per_iteration = new ArrayList<Integer>();
			Config config = Config.getInstance();
			FastDetector feature_detector = new FastDetector(cam_.getWidth(), cam_.getHeight(), config.getGridSize(), config.getnPyrLevels());
			DepthFilter depth_filter_cb =  null;//		  svo::DepthFilter::callback_t depth_filter_cb = boost::bind(&DepthFilterTest::depthFilterCb, this, _1, _2);
			depth_filter_ = new DepthFilter(feature_detector);//
			depth_filter_.get_Options().set_verbose(true); //		  depth_filter_->options_.verbose = true;

			

			for(int i = 0; it.hasNext() && i < 20; i++)//it.next(), i++)
			{	
				ImageNameAndPose it_INAP = it.next();
				String img_name = dataset_dir+"img/"+ it_INAP.getImage_name_() + "_0.png";
				System.out.println("Reading image: " + img_name); 
				Mat img = Imgcodecs.imread(img_name, Imgcodecs.CV_LOAD_IMAGE_GRAYSCALE); // CV_LOAD_IMAGE_GREYSCALE is the same as 0.
				if(img.empty())
					continue;

				Se3 T_w_f = new Se3(it_INAP.getQ_(), it_INAP.getT_());
				if(i==0)
				{
					// create reference frame and load ground truth depthmap
					frame_ref_ = new Frame(cam_, img, 0.0);
					frame_ref_.setT_f_w_(T_w_f.inverse());
					depth_filter_.addKeyFrame(frame_ref_, 2, 0.5);
					depth_ref_ = Blender_Utils.loadBlenderDepthMap(dataset_dir+"/depth/" + it_INAP.getImage_name_() + "_0.depth", cam_);
					continue;
				}

				n_converged_seeds_ = 0;
				// Not sure this is neccessary::https://www.boost.org/doc/libs/1_43_0/libs/smart_ptr/make_shared.html		
				frame_cur_ = new Frame(cam_, img, 0.0);
				frame_cur_.setT_f_w_(T_w_f.inverse());
				depth_filter_.addFrame(frame_cur_);
				n_converged_per_iteration.add(n_converged_seeds_);

			}

			System.out.println("Experiment "+ experiment_name+" took "+t.stop()+" ms");


			// compute mean, median and variance of error in converged area
			System.out.println("# converged:\t"+errors_.size()+" (ref: 287)");
			double sum_error = 0;
			for(Double error:errors_)
			{
				sum_error+=error;
			}
			System.out.println("mean error:\t"+sum_error*100/errors_.size()+" cm (ref: 0.080357)");
			Collections.sort(errors_);
			System.out.println("50-percentile: \t "+errors_.get((int)(errors_.size()*0.5))*100+" cm (ref: 0.062042)\n");
			System.out.println("80-percentile: \t "+errors_.get((int)(errors_.size()*0.8))*100+" cm (ref: 0.062042)\n");
			System.out.println("95-percentile: \t "+errors_.get((int)(errors_.size()*0.95))*100+" cm (ref: 0.062042)\n");


			// trace error
			String trace_dir = "/tmp"; // C++ uses test_utils to return either the ros directory or a tmp.
			String trace_name = trace_dir + "/depth_filter_"+ experiment_name + ".txt";


			PrintWriter ofs = new PrintWriter( new BufferedWriter(new OutputStreamWriter(new FileOutputStream(trace_name))));		
			for(ConvergedSeed i: results_)
			{
				ofs.println(i.getX_() +"," + i.getY_() + "," + Math.abs(i.getError_()));
			}
			ofs.close();

			// trace convergence rate

			trace_name = trace_dir + "/depth_filter_" + experiment_name + "_convergence.txt";

			for(int int_:n_converged_per_iteration)
			{
				ofs.println(int_);
			}
			ofs.close();


			// write ply file for pointcloud visualization in Meshlab
			trace_name = trace_dir + "/depth_filter_" + experiment_name + ".ply";
			ofs.println("ply");
			ofs.println("format ascii 1.0");
			ofs.println("element vertex " + results_.size() );
			ofs.println("property float x");
			ofs.println("property float y");
			ofs.println("property float z");
			ofs.println("property uchar blue");
			ofs.println("property uchar green");
			ofs.println("property uchar red");
			ofs.println("end_header");

			for(ConvergedSeed i:results_)
			{
				double[] c_vals = frame_ref_.get_img_pyr().getMats()[0].get(i.getY_(), i.getX_());
				Vector3d c = new Vector3d();
				c.set(c_vals[0], c_vals[1], c_vals[2]);
				Vector3d p = cam_.cam2world(i.getX_(), i.getY_()).times(i.getDepth_());
				ofs.println(p.get(0) + " " + p.get(1) + " " + p.get(2) + " " + (int) c.get(0) + " " + (int) c.get(1) + " " + (int) c.get(2) );
			}
		}
		catch(FileNotFoundException E)
		{
			System.out.println("Failed to find or open sequence File");	
		}
		//
		//		  for(std::list<ConvergedSeed>::iterator i=results_.begin(); i!=results_.end(); ++i)
		//		  {
		//			cv::Vec3b c = frame_ref_->img_pyr_[0].at<cv::Vec3b>(i->y_, i->x_);
		//			Eigen::Vector3d p = cam_->cam2world(i->x_, i->y_)*i->depth_;
		//			ofs << p[0] << " " << p[1] << " " << p[2] << " "
		//			    << (int) c[0] << " " << (int) c[1] << " " << (int) c[2] << std::endl;
		//		  }		
	}

	public void depthFilterCb(Point point, double depth_sigma2)
	{
		double depth = frame_ref_.pos().minus(point.getPos()).normF();
		double error = Math.abs(depth - depth_ref_.get((int)point.getObs_().get(0).get_px().get(1), (int) point.getObs_().get(0).get_px().get(0))[0]);
		results_.add(new ConvergedSeed((int)point.getObs_().get(0).get_px().get(0), (int)point.getObs_().get(0).get_px().get(1), depth, error));
		errors_.add(error);
		point.getObs_().remove(0);
		point = null;
		n_converged_seeds_++;
	}
}

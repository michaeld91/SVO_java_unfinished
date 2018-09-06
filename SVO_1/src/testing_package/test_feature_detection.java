package testing_package;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;

import matrix_types.Vector2d;

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;

import SVO_1310.Config;
import SVO_1310.FastDetector;
import SVO_1310.Feature;
import SVO_1310.Frame;
import SVO_1310.Vikit.ATANCamera;
import SVO_1310.Vikit.AbstractCamera;
import SVO_1310.Vikit.Pinhole_Camera;
import SVO_1310.Vikit.Timer;

public class test_feature_detection {

	public static void main(String[] args) 
	{
		testCornerDetector();
	}
	
	public static void testCornerDetector()
	{
		
		System.loadLibrary(Core.NATIVE_LIBRARY_NAME);	
		
		String dataset_dir = "/home/michael/Documents/SVO_datasets/sin2_tex2_h1_v8_d/img/";
		String img_name = "frame_000002_0.png"; // Image directory and name.
//		String dataset_dir = "/home/michael/Downloads/";
//		String img_name = "corner_test_752x480.png"; //"house_image_test.png";// Image directory and name.

		System.out.println("Loading image "+img_name);
		Mat img = Imgcodecs.imread(dataset_dir+img_name, Imgcodecs.CV_LOAD_IMAGE_GRAYSCALE); //, Imgcodecs.CV_LOAD_IMAGE_GRAYSCALE); this makes the image of type CV_8UC1
		
		
		if(img.type() != CvType.CV_8UC1)// && img.empty() == false)
		{
			System.out.println("img.type() does not equal CV_8U1");
			System.out.println("img.type() = "+img.type());
		}
		if(img.empty() == true)
			System.out.println("img is empty");
		
		//vk::ATANCamera(752, 480, 0.511496, 0.802603, 0.530199, 0.496011, 0.934092);
		AbstractCamera cam = new ATANCamera(752, 480, 0.511496, 0.802603, 0.530199, 0.496011, 0.934092);
		
//		System.out.println("cam width,height = "+ cam.getWidth()+","+ cam.getHeight());
		// height and width should be 480 and 752 as set above. Line below used for debugging.
		//System.out.println("cam height and width = "+cam.getHeight()+", "+cam.getWidth());
		Frame frame = new Frame(cam, img, 0.0);
		// Corner Detection
		Timer t = new Timer();
		t.start();


		// need a list of features called fts
		ArrayList<Feature> fts = new ArrayList<Feature>();

		FastDetector fast_detector = new FastDetector(img.cols(), img.rows(), Config.getGridSize(), 1);//Config.getnPyrLevels());//
//		System.out.println("n_pyr_levels_ = "+fast_detector.get_n_pyr_levels_());

		// ToDo why iterate over one image 100 times?? = Just for testing the amount of time taken.
		for(int i=0; i<1; i++)
		{
			fast_detector.detect(frame, frame.get_img_pyr(), Config.getTriang_min_corner_score(), fts);
//			fast_detector.detect(frame, frame.get_img_pyr(), 17, fts);	// Tester with a low detection threshold.
			System.out.print("-");
		}

		// Removed the *10 from t.stop()*10 as i believe vikit java already returns time in ms
		System.out.println("\nFast corner detection took "+t.stop()+" ms, "+fts.size()+" corners detected (ref i7-W520: 7.166360ms, 40000)\n");
		System.out.println("Note, in this case, feature detection also contains the cam2world projection of the feature.\n");
		Mat img_rgb = new Mat(img.size(), CvType.CV_8UC3);
		Imgproc.cvtColor(img, img_rgb, Imgproc.COLOR_GRAY2RGB);	// CV_GRAY2RGB changed to COLOR_GRAY2RGB
		
//		int i = 0;
		for(Feature ft : fts)
		{
			Point point = new Point(ft.get_px().get(0), ft.get_px().get(1));	// ToDo NOTE: Field swapped from "ft.get_px().get(0), ft.get_px().get(1)" to help it fit the image (issue with x&y swapped)
//			System.out.println("point = " + point.x+","+point.y);
			Scalar scalar = new Scalar(0,255,0);
			Imgproc.circle(img_rgb, point, 4*(ft.getLevel()+1), scalar, 1);
//			System.out.println("features number = "+i);
//			i++;
		}
		System.out.println("img_rgb size= "+img_rgb);

		Imshow imshow = new Imshow("ref_img");
		imshow.showImage(img_rgb);
	
	}

}

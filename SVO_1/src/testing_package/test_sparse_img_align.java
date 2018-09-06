package testing_package;

import java.io.FileNotFoundException;
import java.io.IOException;

import org.opencv.core.Core;

import SVO_1310.Config;
import SVO_1310.FastDetector;

public class test_sparse_img_align {

	/**
	 * @param args
	 */
	public static void main(String[] args) {
		System.loadLibrary(Core.NATIVE_LIBRARY_NAME);	
		String experiment_name = "flying_room_1_rig_1_fast_minlev0";
		String dataset_dir = "/home/michael/Documents/SVO_datasets/sin2_tex2_h1_v8_d/";
		Config config = Config.getInstance();
		config.setTriang_min_corner_score(20);
		config.setkltMinLevel(0);
		SparseImgAlignTest test = new SparseImgAlignTest();
		FastDetector detector = new FastDetector(test.getCam().getWidth(), test.getCam().getHeight(), config.getGridSize(), config.getnPyrLevels());
		try {
			test.testSequence(dataset_dir, experiment_name, detector);
		} catch (FileNotFoundException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		} catch (CloneNotSupportedException e) {
			// TODO Auto-generated catch block
						e.printStackTrace();
		}
		
		
		System.out.println("End of test.");
	}

}

//
//std::string experiment_name("flying_room_1_rig_1_fast_minlev0");
//std::string dataset_dir(svo::test_utils::getDatasetDir() + "/sin2_tex2_h1_v8_d");
//svo::Config::triangMinCornerScore() = 20;
//svo::Config::kltMinLevel() = 0;
//SparseImgAlignTest test;
//svo::feature_detection::FastDetector detector(
//    test.cam_->width(), test.cam_->height(), svo::Config::gridSize(), svo::Config::nPyrLevels());
//test.testSequence(dataset_dir, experiment_name, &detector);
//return 0;
package testing_package;

import java.io.FileNotFoundException;

import org.opencv.core.Core;

public class test_depth_filter {

	/**
	 * @param args
	 */
	public static void main(String[] args) {
		System.loadLibrary(Core.NATIVE_LIBRARY_NAME);	

		DepthFilterTest test = new DepthFilterTest();
		String dataset_dir = "/home/michael/Documents/SVO_datasets/sin2_tex2_h1_v8_d/";
		String experiment_name = "sin2_tex2_h1_v8_d";
		try {
			test.testReconstruction(dataset_dir, experiment_name);
		} catch (FileNotFoundException e) {
			// TODO Auto-generated catch block
			System.out.println("File Not Found Exception thrown.");
			e.printStackTrace();
		}

	}

}

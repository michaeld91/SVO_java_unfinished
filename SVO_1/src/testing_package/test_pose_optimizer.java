package testing_package;

import org.opencv.core.Core;

import matrix_types.Vector3d;

public class test_pose_optimizer {

	/**
	 * @param args
	 */
	public static void main(String[] args) 
	{
		System.loadLibrary(Core.NATIVE_LIBRARY_NAME);	

		PoseOptimizerTest test = new PoseOptimizerTest();
		Vector3d vec = new Vector3d();
		vec.set(0.2,0.2,0.2);
		test.test(vec, 1.0);
	}

	
}

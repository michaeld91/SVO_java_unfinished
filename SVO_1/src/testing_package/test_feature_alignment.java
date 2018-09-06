package testing_package;

import matrix_types.Vector2d;

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.imgcodecs.Imgcodecs;

import SVO_1310.Feature_alignment;
import SVO_1310.Vikit.Timer;

public class test_feature_alignment {

	/**
	 * @param args
	 */
	public static void main(String[] args) 
	{
		System.loadLibrary(Core.NATIVE_LIBRARY_NAME);	

		String dataset_dir = "/home/michael/Documents/SVO_datasets/sin2_tex2_h1_v8_d/img/";
		String img_name = "frame_000002_0.png"; 
		System.out.println("Loading image "+img_name);
		Mat img = Imgcodecs.imread(dataset_dir+img_name, Imgcodecs.CV_LOAD_IMAGE_GRAYSCALE); // CV_LOAD_IMAGE_GREYSCALE is the same as 0.
		if(img.type()!=CvType.CV_8UC1)
		{
			System.out.println("Mat not of type CV_8UC1");
			return;
		}

		// ToDo test on corner/gradient features!
		Vector2d px_true = new Vector2d();
		px_true.set(130.2,120.3);

		// create reference patch with border
		byte[] img_data = new byte[img.rows()*img.cols()];
		img.get(0, 0, img_data);
		int cols = img.cols();
//		System.out.println("img_data.length = " + img_data.length);
		Mat ref_patch_with_border = generateRefPatchNoWarpInterpolate(img_data, cols, px_true);
		// create reference patch, aligned
		byte[] ref_patch = new byte[64];//aligned_mem.aligned_alloc(64,16);
		int ref_patch_ptr = 0;
		
		byte[] ref_patch_with_border_data = new byte[ref_patch_with_border.rows()*ref_patch_with_border.cols()];
		ref_patch_with_border.get(0, 0, ref_patch_with_border_data);
		
		for(int y = 1; y<9; y++)
		{
			int ref_patch_border_ptr = y*10 + 1;
			for(int x=0; x<8; x++, ref_patch_border_ptr++, ref_patch_ptr++)
				ref_patch[ref_patch_ptr] = ref_patch_with_border_data[ref_patch_border_ptr];
		}

		Vector2d px_est = new Vector2d();
		Vector2d px_error = new Vector2d();
		px_error.set(-1.1, -0.8);
		double h_inv = 0;
		Timer t = new Timer();

		Feature_alignment feature_alignment = new Feature_alignment();
		for(int i=0; i<1000; i++)
		{
			px_est = px_true.minus(px_error);
			Vector2d dir = px_error;
			dir.normalize();
			feature_alignment.align1D(img, dir, ref_patch_with_border_data, ref_patch, 3, px_est, h_inv);
//			System.out.println("px_est =" + px_est);

		}
//		System.out.println("px_est = " + px_est);
//		System.out.println("px_true = " + px_true);
		Vector2d e = px_est.minus(px_true);
//		System.out.println("e =" + e.get(0)+","+e.get(1));

		System.out.println("1000Xalign 1D took "+t.stop()+"ms, error = "+e.normF()+"px \t (ref i7-W520: 1.982000ms, 0.000033px)");

		t.start();
		for(int i = 0; i<1000; i++)
		{
			px_est = px_true.minus(px_error);
			feature_alignment.align2D(img, ref_patch_with_border_data, ref_patch, 3, px_est);
		}

		e = px_est.minus(px_true);
		System.out.println("1000Xalign 2D took "+t.stop()+"ms, error = "+e.normF()+"px \t (ref i7-W520: 2.306000ms, 0.015102px)");

		// #ifdef __SSE2__
		
		// #ifdef __ARM_NEON__
		System.out.println("End of test.");
	}

	// stride is essentially the number of columns
	// Should return a 10x10 Mat of 
	private static Mat generateRefPatchNoWarpInterpolate(byte[] img_data, int stride, Vector2d px)//, Mat ref_patch_with_border)
	{
		// compute interpolation weights
		final int u_r = (int)px.get(0);
		final int v_r = (int)px.get(1);
		final double subpix_u = px.get(0)-u_r;
		final double subpix_v = px.get(1)-v_r;
		final double wTL = (1.0-subpix_u)*(1.0-subpix_v);
		final double wTR = subpix_u * (1.0-subpix_v);
		final double wBL = (1.0-subpix_u)*subpix_v;
		final double wBR = subpix_u * subpix_v;

		// loop through search_patch, interpolate
		Mat ref_patch_patch_with_border = new Mat(10,10, CvType.CV_8UC1);
		int patch_ptr = 0;
		double[] ref_patch_with_border_data = new double[100];
		for(int y = 0; y < 10; y++)
		{
			int img_ptr = (v_r+y-4-1)*stride + u_r-4-1;	// index position for data.
			for(int x = 0; x < 10; x++, img_ptr++, patch_ptr++)
			{
//				System.out.println("x,y = " + x +","+ y + "\t img_ptr = " + img_ptr + "\t patch_ptr = " + patch_ptr);
				ref_patch_with_border_data[patch_ptr] = wTL*img_data[img_ptr] + wTR*img_data[img_ptr+1] + wBL*img_data[img_ptr+stride] + wBR*img_data[img_ptr+stride+1];
			}
		}
		ref_patch_patch_with_border.put(0, 0, ref_patch_with_border_data);
		return ref_patch_patch_with_border;
	}
}

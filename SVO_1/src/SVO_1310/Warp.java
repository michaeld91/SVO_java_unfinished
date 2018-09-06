package SVO_1310;

import org.opencv.core.CvType;
import org.opencv.core.Mat;

//import matrix_types.Matrix23d;
import matrix_types.Matrix2d;
import matrix_types.Vector;
import matrix_types.Vector2d;
import matrix_types.Vector3d;
import SVO_1310.Vikit.AbstractCamera;
import SVO_1310.Vikit.Vision;
import SVO_1310.Vikit.math_utils;
import Sophus.Se3;

public class Warp {

	public Warp(){}
	//Currently returns A_cur_ref instead of setting it from a passed variable and returning void. may affect other classes
	//Check that other classes can accept a Matrix2d from this method
	//SE3 has been swapped to Se3. May need further work
	public Matrix2d getWarpMatrixAffine(AbstractCamera cam_ref, AbstractCamera cam_cur, Vector2d px_ref, Vector3d f_ref,
			double depth_ref, Se3 T_cur_ref, int level_ref) //, Matrix2d A_cur_ref)//Not sure whether SE3 should be se3. Seems not??
	{
		// Compute affine warp matrix A_ref_cur
		final int halfpatch_size = 5;
		Vector3d xyz_ref = new Vector3d(f_ref.times(depth_ref).getArray());
		Vector3d xyz_du_ref = new Vector3d(); //cam_ref.cam2world(px_ref, Vector2d(halfpatch_size,0)*(1<<level_ref)));
		// ToDo Possible BIDMAS error. check that the addition and multiplication occurs in the correct order wrt C++.
		Vector2d halfPatch_Zero = new Vector2d();
		halfPatch_Zero.set(halfpatch_size,0); 
		xyz_du_ref.set(cam_ref.cam2world(px_ref.plus(halfPatch_Zero.times(1<<level_ref))));

		Vector3d xyz_dv_ref = new Vector3d(); //cam_ref.cam2world(px_ref, Vector2d(halfpatch_size,0)*(1<<level_ref)));
		Vector2d zero_halfPatch = new Vector2d();
		zero_halfPatch.set(0,halfpatch_size); 
		xyz_dv_ref.set(cam_ref.cam2world((Vector2d)px_ref.plus(zero_halfPatch.times(1<<level_ref))));

		xyz_du_ref.times(xyz_ref.get(2)/xyz_du_ref.get(2));
		xyz_dv_ref.times(xyz_ref.get(2)/xyz_dv_ref.get(2));

		Vector3d T_cur_ref_times_xyz_ref = new Vector3d(T_cur_ref.times(xyz_ref).getArray());
		Vector2d px_cur = new Vector2d(cam_cur.world2cam(T_cur_ref_times_xyz_ref).getArray());

		Vector3d T_cur_ref_times_xyz_du_ref = new Vector3d(T_cur_ref.times(xyz_du_ref).getArray());
		Vector2d px_du = new Vector2d(cam_cur.world2cam(T_cur_ref_times_xyz_du_ref).getArray());

		Vector3d T_cur_ref_times_xyz_dv_ref = new Vector3d(T_cur_ref.times(xyz_dv_ref).getArray());
		Vector2d px_dv = new Vector2d(cam_cur.world2cam(T_cur_ref_times_xyz_dv_ref).getArray());

		Matrix2d A_cur_ref = new Matrix2d();
		//Check this equates to the same as:
		//		  A_cur_ref.col(0) = (px_du - px_cur)/halfpatch_size;
		//		  A_cur_ref.col(1) = (px_dv - px_cur)/halfpatch_size;
		A_cur_ref.set(0, 0,(px_du.get(0) - px_cur.get(0))/halfpatch_size);
		A_cur_ref.set(1, 0,(px_du.get(1) - px_cur.get(1))/halfpatch_size);
		A_cur_ref.set(0, 1,(px_dv.get(0) - px_cur.get(0))/halfpatch_size);
		A_cur_ref.set(1, 1,(px_dv.get(1) - px_cur.get(1))/halfpatch_size);

		//		if(depth_ref==0)
		//		{
		//			System.out.println("depth_ref = " + depth_ref);
		//		}

		if(Double.isInfinite(A_cur_ref.get(0,0)))
		{
			//			System.out.println("depth_ref = " + depth_ref);

			//			f_ref.print(0,5);
			//			System.out.println("");//depth_ref = " + depth_ref);
			//			math_utils.project2d(T_cur_ref_times_xyz_ref).print(0, 5);
			//			px_cur.print(0, 5);
			//			px_du.print(0, 5);
			//			px_dv.print(0, 5);
			//System.out.println(A_cur_ref.get(0,0)+","+A_cur_ref.get(0,1)+","+A_cur_ref.get(1,0)+","+A_cur_ref.get(1,1));
		}
		return A_cur_ref;
	}

	public int getBestSearchLevel(Matrix2d A_cur_ref, int max_level)
	{
		// Compute patch level in other image
		int search_level = 0;
		double D =  A_cur_ref.det();
		while(D > 3.0 && search_level < max_level)
		{
			search_level += 1;
			D *= 0.25;
		}
		return search_level;
	}

	// ToDo May need try catch for null/index out of bounds when there is no inverse
	public void warpAffine(Matrix2d A_cur_ref, Mat img_ref, Vector2d px_ref, int level_ref, int search_level, int halfpatch_size, byte[] patch) throws Exception
	{
		int patch_size = halfpatch_size*2;
		Matrix2d A_ref_cur = new Matrix2d();
		A_ref_cur = new Matrix2d(A_cur_ref.inverse().getArray());//(Matrix2d) A_cur_ref.inverse();//

		//		A_ref_cur.print(0, 4);

		if(Double.isNaN(A_ref_cur.get(0, 0)))
		{
			System.out.println("Affine warp is NaN, the camera probably has no translation\n");
			//			System.out.println("The Matrix that could not be inverted is: ");
			//			A_cur_ref.print(0, 4);
			return;
		}

		// Perform a warp on a larger patch
		int patch_ptr = 0;//patch;	//perhaps use double or long?
		Vector2d px_ref_pyr = (Vector2d) px_ref.times((1/(1<<level_ref)));	//Cast may not work
		
		byte[] data = new byte[img_ref.cols()*img_ref.rows()];
		img_ref.get(0, 0, data);
		for(int y = 0; y<patch_size-1; y++) 
		{
			for(int x = 0; x< patch_size; x++, patch_ptr++)
			{
				Vector2d px_patch = new Vector2d();							//Originally a Vector2f. May cause inaccuracies
				px_patch.set(x-halfpatch_size, y-halfpatch_size);
				px_patch.times(1<<search_level);
				Vector2d px = new Vector2d();								//Originally this was Vector2f. Possible inaccuracies?
				Vector2d a_ref_cur_times_px_patch = A_ref_cur.times(px_patch);
				Vector2d a_ref_cur_times_px_patch_plus_px_ref_pyr = a_ref_cur_times_px_patch.plus(px_ref_pyr);
				px = a_ref_cur_times_px_patch_plus_px_ref_pyr;
//				System.out.println("X, Y, Patch pointer = " + x +"," + y + ", " + patch_ptr);

				if(px.get(0)<0|| px.get(1)<0 || px.get(0)>=img_ref.cols()-1 || px.get(1)>=img_ref.rows()-1)
				{
					patch[patch_ptr] = 0; 
				}
				else
				{
					if(img_ref.type()==CvType.CV_8U)
					{
						patch[patch_ptr] = (byte) Vision.interpolateMat_8u(data,img_ref.cols(), img_ref.rows(), (float) px.get(0),(float) px.get(1));
//						System.out.println("Mat.size() = "+img_ref.get(0,0).length);
					}
					else
						throw new IllegalArgumentException("The Matrix is not of type CV_8U");
//					catch(ArrayIndexOutOfBoundsException E)
//					{
//						System.out.println("ArrayIndexOutOfBoundsException");
//					}
				}
			}
		}
	}
}

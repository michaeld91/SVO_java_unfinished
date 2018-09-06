package SVO_1310.Vikit;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Vector;

import matrix_types.JamaUtils;
import matrix_types.Matrix2d;
import matrix_types.Matrix3d;
import matrix_types.Vector2d;
import matrix_types.Vector3d;
import Jama.Matrix;





public class math_utils {
	
	// ToDo changed field from Matrix to Vector3d
	public static Vector2d project2d(Vector3d v)	
	{	
		Vector2d v_new = new Vector2d();		
		v_new.set(0,0, v.get(0,0)/v.get(0,2));
		v_new.set(0,1, v.get(0,1)/v.get(0,2));
		return v_new;
	}



	public static double norm_max(matrix_types.Vector v)
	{
		double max = -1;
		for (int i=0; i<v.getRowDimension(); i++)
		{
			double abs = Math.abs(v.get(i));
			if(abs>max)
				max=abs;
		}
		return max;
	}



	public static double getMedian(ArrayList<Double> chi2_vec_init) {
		try
		{
			int middle_index = (int) Math.floor(chi2_vec_init.size()/2);
			ArrayList<Double> chi2_for_sorting = (ArrayList<Double>) chi2_vec_init.clone();
			
			Collections.sort(chi2_for_sorting);	// This sorts the copy of chi2_vec, but I dont want it permenantly sorted.
			return chi2_for_sorting.get(middle_index);
		}
		catch( NullPointerException E)
		{
			System.out.println("There is no data to return a Median for.");
		}
		return -1;	// This should only be reached if chi2_vec_init is empty.
	}



	public static double computeInliers(ArrayList<Vector3d> features1,	// in frame c1
									ArrayList<Vector3d> features2,		// in frame c2
									Matrix3d rotation_Matrix, 			// R
									Vector3d translation,					// c1_t
									double reprojection_threshold, 		// reproj_thresh
									double focal_length,				//error_multiplier2
									Vector3d[] xyz_in_cur, 				// xyz_vec in frame c1
									int[] inliers, 	
									int[] outliers) {
		
		ArrayList<Integer> inliers_AL = new ArrayList<Integer>();//inliers = new int[features1.size()];
		ArrayList<Integer> outliers_AL = new ArrayList<Integer>();//outliers = new int[features1.size()];
		xyz_in_cur = new Vector3d[features1.size()];
		double tot_error = 0;
		// triangulate all features and compute reprojection errors and inliers
		ArrayList<Vector3d> xyz_in_cur_arraylist = new ArrayList<Vector3d>();
		
		for(int j=0; j<features1.size(); j++)
		{	
			xyz_in_cur_arraylist.add(triangulateFeatureNonLin(rotation_Matrix, translation, features1.get(j), features2.get(j)));
			double e1 = reprojError(features1.get(j), xyz_in_cur_arraylist.get(xyz_in_cur_arraylist.size()-1), focal_length);
			double e2 = reprojError(features2.get(j), (Vector3d) rotation_Matrix.transpose().times((xyz_in_cur_arraylist.get(xyz_in_cur_arraylist.size()-1).minus(translation))), focal_length);
			
			if(e1 > reprojection_threshold || e2 > reprojection_threshold)
			{
				outliers_AL.add(j);
			}
			else
			{
				inliers_AL.add(j);
				tot_error += e1 + e2;
			}
		
		}
		xyz_in_cur = (Vector3d[]) xyz_in_cur_arraylist.toArray();
		inliers = new int[features1.size()];
		int inliers_index = 0;
		for(int inlier_from_AL: inliers_AL)
			inliers[inliers_index] = inlier_from_AL;
		
		outliers = new int[features1.size()];
		int outliers_index = 0;
		for(int outlier_from_AL: outliers_AL)
			outliers[outliers_index] = outlier_from_AL;
		
		
		return tot_error;
	}



	private static double reprojError(Vector3d f1, Vector3d f2, double focal_length)//focal_length also known as error_multiplier2
	{
		Vector2d e = project2d(f1).minus(project2d(f2));
		return focal_length * e.normF();
	}



	private static Vector3d triangulateFeatureNonLin(
							Matrix3d rotation_Matrix,
							Vector3d translation, 
							Vector3d feature1, 
							Vector3d feature2) 
	{
		Vector3d f2 = new Vector3d(rotation_Matrix.times(feature1).getArray());
		Vector2d b = new Vector2d();
		b.set(JamaUtils.dotproduct(translation, feature1), JamaUtils.dotproduct(translation, f2));
		double[] A_vals = {JamaUtils.dotproduct(feature1, feature1),
						   JamaUtils.dotproduct(feature1, f2),
						   -JamaUtils.dotproduct(feature1, f2),
						   -JamaUtils.dotproduct(f2, f2)};
		
		Matrix2d A = new Matrix2d(A_vals);
		Vector2d lambda = new Vector2d(A.inverse().times(b).getArray());
		Vector3d xm = feature1.times(lambda.get(0));
		Vector3d xn = translation.plus(f2.times(lambda.get(1)));
		return (xm.plus(xn)).times(.5);
	
	}



	public static Vector3d unprojected2d(Vector2d uv_best) {
		Vector3d vec3d = new Vector3d();
		vec3d.set(uv_best.get(0), uv_best.get(1), 1.0);
		return vec3d;
	}
	
	// Causes error.
	public static double sampsonusError(Vector2d v2Dash, Matrix3d Essential, Vector2d v2)
	{
		Vector3d v3Dash = unprojected2d(v2Dash);
		Vector3d v3 = unprojected2d(v2);
		
		double dError = v3Dash.times(Essential).times(v3);
		
		Vector3d fv3 = new Vector3d(Essential.times(v3.transpose()).getArray());
		// There is an error attempting to multiply the matrix by v3Dash
		Vector3d fTv3Dash = new Vector3d(Essential.transpose().times(v3Dash.transpose()).getArray());
		
		Vector2d fv3Slice = new Vector2d();
		double val1 = fv3.get(0,0);
		double val2 = fv3.get(1,0);	// changed from usual vector get(int). This is a Matrix of (3,1) rather than Matrix(1,3);
		fv3Slice.set(val1, val2);
		Vector2d fTv3DashSlice = new Vector2d(); 
		fTv3DashSlice.set(fTv3Dash.get(0,0), fTv3Dash.get(1,0));
		
		return (dError * dError / (JamaUtils.dotproduct(fv3Slice, fv3Slice) + JamaUtils.dotproduct(fTv3DashSlice, fTv3DashSlice)));		
	}

	public static Matrix3d sqew(Vector3d v)
	{
		double[] values = {0, -v.get(2), v.get(1), v.get(2), 0, -v.get(0), -v.get(1), v.get(0), 0};
		Matrix3d v_sqew = new Matrix3d(values);
		return v_sqew;	
	}

}


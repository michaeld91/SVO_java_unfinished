package SVO_1310;


import java.util.Arrays;

import org.opencv.core.Mat;
import Jama.Matrix;

//This could extend GVector? for access to the GVector field. currently just using a getVector()
public class ImgPyr {
	private Mat[] mats;
	//	GVector vec;
	Matrix vec; //This perhaps should be a specific vector type
	public ImgPyr()
	{
		mats = new Mat[10];	// ToDo : setting base level to 10. Stops the need for resizing often
	}

	public Mat[] getMats()
	{
		return mats;
	}

	public void resize(int n_levels) {
		int current_size = mats.length;
		Mat[] new_mat = new Mat[n_levels];
		System.arraycopy(mats, 0, new_mat, 0, current_size);
		//		System.out.println("new_mat size = "+ new_mat.length);
	}
	public void add(Mat new_mat)
	{
		int current_length = mats.length;
		boolean is_null = false;
		int index_for_new_mat = 0;
		while(!is_null)
		{
			if(mats[index_for_new_mat] != null)
			{
				index_for_new_mat++;
				continue;
			}
			is_null=true;
		}
		mats[index_for_new_mat] = new_mat;
	}
}

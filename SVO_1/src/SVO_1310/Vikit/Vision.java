package SVO_1310.Vikit;

import org.opencv.core.Mat;
import org.opencv.core.CvType;

public class Vision {

	public Vision()
	{}
	//! Return value between 0 and 255
	//! WARNING This function does not check whether the x/y is within the border	
	public static float interpolateMat_8u(byte[] data,int cols, int rows, float u, float v) throws Exception	//Mat mat, float u, float v) throws Exception
	{
//		if(mat.type()!=CvType.CV_8U)
//		{
//			throw new IllegalArgumentException("The Matrix is not of type CV_8U");
//		}
//		else
//		{
			int x = (int) Math.floor(u);
			int y = (int) Math.floor(v);
			float subpix_x = u-x;
			float subpix_y = v-y;

			float w00 = (float) ((1.0-subpix_x)*(1.0-subpix_y));
			float w01 = (float) (1.0-subpix_x)*subpix_y;
			float w10 = (float) (subpix_x*(1.0-subpix_y));
			float w11 = (float) (1.0 - w00 -w01 -w10);

			final int stride = cols;//mat.cols();	

			int ptr = y*stride +x;
//			int size = cols*rows;//			int size = mat.cols()*mat.rows();
//			byte[] data = new byte[size];

//			mat.get(0, 0, data);	// ToDo this is very time consuming to do every iteration. Perhaps passing this data would be quicker

			return w00*data[ptr] + w01*data[ptr+stride] + w10*data[ptr+1] + w11*data[ptr+stride+1];
//		}

	}
	public static float shiTomasiScore(Mat img, int u, int v)
	{
		if(img.type()!=CvType.CV_8U)
		{
			throw new IllegalArgumentException("The Matrix is not of type CV_8U");
		}
		else
		{
			float dXX = 0.0f;
			float dYY = 0.0f;
			float dXY = 0.0f;
			final int halfbox_size = 4;
			final int box_size = 2*halfbox_size;
			final int box_area = box_size*box_size;
			final int x_min = u-halfbox_size;
			final int x_max = u+halfbox_size;
			final int y_min = v-halfbox_size;
			final int y_max = v+halfbox_size;

			if(x_min < 1 || x_max >= img.cols()-1 || y_min < 1 || y_max >= img.rows()-1)
				return 0.0f;		// patch is too close to the boundary

			final int stride = img.cols(); //final int stride = img.step1().p[0];
			byte[] data = new byte[img.cols()*img.rows()];
			img.get(0,0,data);

			for(int y = y_min; y<y_max; y++)
			{
				int ptr_left   = stride*y + x_min - 1;		//index position to the left
				int ptr_right  = stride*y + x_min + 1;		//index position to the right
				int ptr_top    = stride*(y-1) + x_min;		//index position above
				int ptr_bottom = stride*(y+1) + x_min;		//index position below

				//				int size = img.cols()*img.rows();
				//				int[] data = new int[size];
				//				img.get(0, 0, data);

				for(int x = 0; x < box_size; x++, ptr_left++, ptr_right++, ptr_top++, ptr_bottom++)
				{
					float dx = data[ptr_right] - data[ptr_left];
					float dy = data[ptr_bottom] - data[ptr_top];
					dXX += dx*dx;
					dYY += dy*dy;
					dXY += dx*dy;
				}
			}				  
			dXX = dXX / (2.0f * box_area);
			dYY = dYY / (2.0f * box_area);
			dXY = dXY / (2.0f * box_area);
			return (float) (0.5 * (dXX + dYY - Math.sqrt((dXX + dYY) * (dXX + dYY) - 4 * (dXX * dYY - dXY * dXY) )));
		}
	}
	// Takes the intensities of the Mat 'in' at the points i, i+1, i+in.cols, i+in.cols+1 and averages them.
	// That value then becomes the intensity in the half size matrix 'out' and the value of i is increased until all pixels have been addressed.
	public static void halfSample(Mat in, Mat out)
	{
		if(in.rows()/2!=out.rows() || in.cols()/2!=out.cols())
			throw new IllegalArgumentException("The Matricies are not of the correct size. The second Mat should have half the dimensions of the first");
		if(in.type()!=CvType.CV_8U || out.type()!=CvType.CV_8U)
			throw new IllegalArgumentException("The Matricies are not of equal type");

		// ifdef __SSE2__...
		// ifdef __ARM_NEON__

		int stride = in.cols(); 		// in.step.p[0];
		int top = 0;					// (uint8_t*) in.data;
		int bottom = top+stride;
		int end = top + stride*in.rows();
		int out_width = out.cols();
		int p = 0;	// set as initial position in data // (uint8_t*) out.data;

		byte[] in_data = new byte[in.rows()*in.cols()];// ToDo not sure if this size is correct. 
		in.get(0,0, in_data);

		byte[] out_data = new byte[out.rows()*out.cols()];

		while(bottom < end)
		{
			for(int j = 0; j < out_width; j++)
			{
				// This basically takes an average of a square of 4 pixel intensity values and sets that as the intensity in the half sample matrix (out)
				out_data[p] = (byte) ((in_data[top] + in_data[top+1] + in_data[bottom] + in_data[bottom+1])/4);
				p++;			// Moves the out matrix index along one
				top +=2;		// Moves the in matrix indices along 2.
				bottom += 2;
			}
			top += stride;
			bottom += stride;
		}

	}

}

package SVO_1310;

import matrix_types.Matrix2d;
import matrix_types.Matrix3d;
import matrix_types.Vector2d;
import matrix_types.Vector3d;
import org.opencv.core.Mat;


/// Subpixel refinement of a reference feature patch with the current image.
/// Implements the inverse-compositional approach (see "Lucas-Kanade 20 Years on"
/// paper by Baker.
public class Feature_alignment {

	public Feature_alignment(){}
	public boolean align1D(
			Mat cur_img,
			Vector2d dir,			// Direction in which the patch is allowed to move
			byte[] ref_patch_with_border,	// ToDo should this be an int or an int[]
			byte[] ref_patch, 	// ToDo should this be an int or an int[]
			int n_iter, 
			Vector2d cur_px_estimate, 
			double h_inv){

		int halfpatch_size_ = 4;
		int patch_size = 8;
		int patch_area = 64;
		boolean converged = false;

		// compute derivative of template and prepare inverse compositional
		float ref_patch_dv = patch_area;//float __attribute__((__aligned__(16))) ref_patch_dv[patch_area];	ToDo
		Matrix2d H = new Matrix2d();

		// compute gradient and hessian
		int ref_step = patch_size+2;
		float it_dv = ref_patch_dv;
		for(int y=0; y<patch_size; y++)
		{
//			int it = ref_patch_with_border + (y+1)*ref_step + 1;	//ToDo this should be an integer memory location 
//			int it = ref_patch_with_border[(y+1)*ref_step + 1];	//ToDo this should be an array of intensity values I believe 
//
//			for(int x=0; x<patch_size; x++, it++, it_dv++)
//			{
//				Vector2d J = new Vector2d();
//				J.set(0.5*(dir.get(0)*(it[1] - it[-1]) + dir.get(1)*(it[ref_step] - it[-ref_step])), 1);
//				it_dv =(float) J.get(0);
//				H = new Matrix2d(H.plus(J.times(J.transpose())).getArray());
//			}	
			
			int it = (y+1)*ref_step + 1;	//ToDo I believe this should be an array of intensity values 

			for(int x=0; x<patch_size; x++, it++, it_dv++)
			{
				Vector2d J = new Vector2d();
				J.set(0.5*(dir.get(0)*(ref_patch_with_border[it+1] - ref_patch_with_border[it-1]) + dir.get(1)*(ref_patch_with_border[it+ref_step] - ref_patch_with_border[it-ref_step])), 1);
				it_dv =(float) J.get(0);
				Matrix2d J_Jt = J.times(J.transpose());
				H = new Matrix2d(H.plus(J_Jt).getArray());
			}	
		}
		h_inv = 1.0/H.get(0,0)*patch_size*patch_size;	//ToDo must set this rather than attempting to set the reference.
		Matrix2d Hinv = new Matrix2d(H.inverse().getArray());
		float mean_diff = 0;

		// Compute pixel location in new image;
		float u = (float) cur_px_estimate.get(0);
		float v = (float) cur_px_estimate.get(1);

		// Termination condition
		final float min_update_squared = 0.0009f; //0.03f*0.03f;
		// final int cur_step = cur_img.getStep().p[0];	ToDo initial solution for cur_img.getStep().p[0]
		final int cur_step = (int) cur_img.step1(0);

		float chi2 = 0;
		Vector2d update = new Vector2d();
		for(int iter = 0; iter < n_iter; iter++)
		{
			int u_r = (int) Math.floor(u);
			int v_r = (int) Math.floor(v);
			if(u_r < halfpatch_size_ || v_r < halfpatch_size_ || u_r >= cur_img.cols()-halfpatch_size_ 
					|| v_r >= cur_img.rows()-halfpatch_size_)
				break;

			if(Float.isNaN(u) || Float.isNaN(v))	// ToDo very rarely this can happen, maybe H is singular? should not be at corner.. check
				return false;

			// compute interpolation weights
			float subpix_x = u-u_r;
			float subpix_y = v-v_r;
			float wTL = (1.0f-subpix_x)*(1.0f-subpix_y);
			float wTR = subpix_x * (1.0f-subpix_y);
			float wBL = (1.0f-subpix_x)*subpix_y;
			float wBR = subpix_x * subpix_y;

			// loop through search_patch, interpolate
			int it_ref = 0;//ref_patch;
			int it_ref_dv = 0;// changed from float to int
			float new_chi2 = 0.0f;
			Vector2d Jres = new Vector2d();
			for(int y=0; y<patch_size; y++)
			{	
				// ToDo I believe the following line should set the 'it' value to point at the cur_img.data array at position (v_r+y-halfpatch_size_)*cur_step + u_r - halfpatch_size_
				//int it = cur_img.getData() + (v_r+y-halfpatch_size_)*cur_step + u_r - halfpatch_size_;	
				int it = (v_r+y-halfpatch_size_)*cur_step + u_r - halfpatch_size_;	

				// ToDo does cur_img.data

				for(int x = 0; x < patch_size; x++, it++, it_ref++, it_ref_dv++)
				{
					// Below is the added methodology to retrieve the data as in 'cur_img.data'
					//  1. create a new array ready for the data in cur_img
					//	2. fill the array with the data values. ToDo: are these values the pixel intensities? I think they should be.
					// 	3. compute search_pixel
					int framesize = cur_img.rows()*cur_img.cols();
					byte[] data = new byte[framesize*cur_img.channels()];	// ToDo may not need '*cur_img.channels()'
					
					cur_img.get(y, x, data);
					
					float search_pixel = wTL*data[it] + wTR*data[it+1] + wBL*data[it+cur_step] + wBR*data[it+cur_step+1];

//					float search_pixel = wTL*it[0] + wTR*it[1] + wBL*it[cur_step] + wBR*it[cur_step+1];
					float res = search_pixel - ref_patch[it_ref] + mean_diff;
					Jres.set(Jres.get(0)-res*(ref_patch[it_ref_dv]), Jres.get(1)-res);
					new_chi2 += res*res;
				}
			}

			if(iter > 0 && new_chi2 > chi2)
			{
				//ToDo unsure about this.	
				//#if SUBPIX_VERBOSE
				//    cout << "error increased." << endl;
				//#endif	
				u -= update.get(0);
				v -= update.get(1);
				break;			
			}

			chi2 = new_chi2;
			update = new Vector2d(Hinv.times(Jres).getArray());
//			update.print(0, 5);
			u += update.get(0)*dir.get(0);
			v += update.get(0)*dir.get(1);
			mean_diff += update.get(1);

			// ToDo unsure how to implement this.
			//			#if SUBPIX_VERBOSE
			//		    cout << "Iter " << iter << ":"
			//		         << "\t u=" << u << ", v=" << v
			//		         << "\t update = " << update[0] << ", " << update[1]
			//		         << "\t new chi2 = " << new_chi2 << endl;
			//		#endif
			if(update.get(0)*update.get(0)+update.get(1)*update.get(1) < min_update_squared)
			{
				//				#if SUBPIX_VERBOSE
				//				cout << "converged." << endl;
				//				#endif
				converged=true;
				break;
			}
		}

		cur_px_estimate.set(u, v);	// ToDo check same as   cur_px_estimate << u, v;
		return converged;
	}
	public boolean align2D(
			Mat cur_img,
			byte[] ref_patch_with_border,
			byte[] ref_patch, 
			int n_iter, 
			Vector2d cur_px_estimate)
		{



		//		#ifdef __ARM_NEON__
		//		  if(!no_simd)
		//		    return align2D_NEON(cur_img, ref_patch_with_border, ref_patch, n_iter, cur_px_estimate);
		//		#endif

		final int halfpatch_size_ = 4;
		final int patch_size_ = 8;
		final int patch_area_ = 64;
		boolean converged = false;

		// compute derivative of template and prepare inverse compositional
		float ref_patch_dx = patch_area_;
		float ref_patch_dy = patch_area_;
		Matrix3d H = new Matrix3d();

		// compute gradient and hessian
		final int ref_step = patch_size_+2;
		float it_dx = ref_patch_dx;
		float it_dy = ref_patch_dy;
		for(int y=0; y<patch_size_; y++)
		{
			// ToDo 'it' should point to the position after (y+1)*ref_step +1, not be equal to the value at ref_patch_with_border[(y+1)*ref_step + 1];
//			int it = ref_patch_with_border[(y+1)*ref_step + 1];
			int it = (y+1)*ref_step + 1;

			for(int x=0; x<patch_size_; x++, it++, it_dx++, it_dy++)
			{
				Vector3d J = new Vector3d();
				J.set(0.5*(ref_patch_with_border[it+1] - ref_patch_with_border[it-1]), 0.5 * (ref_patch_with_border[it+ref_step] - ref_patch_with_border[it-ref_step]), 1);
				it_dx = (float) J.get(0);
				it_dy = (float) J.get(1);
				H = H.plus(J.times_vector_direct(J));//new Matrix3d(H.plus(J.times(J.transpose()));
			}
		}
		Matrix3d Hinv = H.inverse();
		float mean_diff = 0;

		// Compute pixel location in new image:	
		double u =  cur_px_estimate.get(0);				// ToDo These are just 0, suggesting that cur_px_estimate is not set correctly
		double v =  cur_px_estimate.get(1);

		// termination condition
		final float min_update_squared = 0.0009f;
		//final int cur_step = cur_img.getStep().p[0];		ToDo initial solution for cur_img.getStep().p[0]
		final int cur_step = (int)cur_img.step1(0);
		// float chi2 = 0;
		Vector3d update = new Vector3d();
		for(int iter = 0; iter<n_iter; iter++)
		{
			int u_r = (int) Math.floor(u);
			int v_r = (int) Math.floor(v);
			if(u_r < halfpatch_size_ || v_r < halfpatch_size_ || u_r >= cur_img.cols()-halfpatch_size_
					|| v_r >= cur_img.rows()-halfpatch_size_)
				break;
			if(Double.isNaN(u) || Double.isNaN(v))	// ToDo very rarely this can happen, maybe H is singular? should not be at corner.. check
				return false;

			//	compute interpolation weights
			float subpix_x = (float) u-u_r;	// ToDo cast to float may not be needed if u and v are floats
			float subpix_y = (float) v-v_r;
			float wTL = (1.0f-subpix_x) * (1.0f-subpix_y);
			float wTR = subpix_x * (1.0f-subpix_y);
			float wBL = (1.0f-subpix_x) * subpix_y;
			float wBR = subpix_x * subpix_y;

			//	loop through search_patch, interpolate
			int it_ref = 0;//ref_patch;
			float it_ref_dx = ref_patch_dx;
			float it_ref_dy = ref_patch_dy;

			//	float new_chi2 = 0.0
			Vector3d Jres = new Vector3d();
			for(int y=0; y<patch_size_; y++)
			{
//				int it = cur_img.getData() + (v_r+y-halfpatch_size_)*cur_step + u_r - halfpatch_size_;
				int it = (v_r+y-halfpatch_size_)*cur_step + u_r - halfpatch_size_;

				
				for(int x = 0; x < patch_size_; x++, it++, it_ref++, it_ref_dx++, it_ref_dy++)
				{
					//////////////////////////////////////
					// Below is the added methodology to retrieve the data as in 'cur_img.data'
					//  1. create a new array ready for the data in cur_img
					//	2. fill the array with the data values. ToDo: are these values the pixel intensities? I think they should be.
					// 	3. compute search_pixel
					int framesize = cur_img.rows()*cur_img.cols();
					byte[] data = new byte[framesize*cur_img.channels()];	// ToDo may not need '*cur_img.channels()'
					
					cur_img.get(y, x, data);
					
					float search_pixel = wTL*data[it] + wTR*data[it+1] + wBL*data[it+cur_step] + wBR*data[it+cur_step+1];
					float res = search_pixel - ref_patch[it_ref] + mean_diff;
					Jres.set(Jres.get(0)-res*(it_ref_dx), Jres.get(1)-res*(it_ref_dy), Jres.get(2)-res);
					//	new_chi2 += res*res;
				}
			}
			/*
    if(iter > 0 && new_chi2 > chi2)
    {
#if SUBPIX_VERBOSE
      cout << "error increased." << endl;
#endif
      u -= update[0];
      v -= update[1];
      break;
    }
    chi2 = new_chi2;
			 */
			update = Jres.times(Hinv);
			u += update.get(0);
			v += update.get(1);
			mean_diff += update.get(2);

			/*			#if SUBPIX_VERBOSE
	System.out.println("Iter "+iter+" : "+"\t update = "+update.get(0)+", "+update.get(1)+
					"\t new chi2 = "+new_chi2);
					#endif*/

			if(update.get(0)*update.get(0)+update.get(1)*update.get(1)< min_update_squared)
			{
				//if SUBPIX_VERBOSE
//				System.out.println("converged.");
				//endif
				converged=true;
				break;
			}
		}

//		cur_px_estimate.set(u, v);
		return converged;
	}
	//	ToDo align2D_SSE2 probably not going to be used with Java
	public boolean align2D_SSE2 (
			Mat cur_img, 
			int[] ref_patch_with_border, 
			int ref_patch, 
			int n_iter, 
			Vector2d cur_px_estimate)
	{
		// ToDo: this function should not be used as the alignment is not robust to illumination changes!
		final int halfpatch_size = 4;
		final int patch_size = 8;
		final int patch_area = 64;
		boolean converged = false;
		final int W_BITS = 14;

		// Compute derivatives of template and prepare inverse compositional
		int ref_patch_dx = patch_area;
		int ref_patch_dy = patch_area;

		// Compute gradient and hessian
		final int ref_step = patch_size+2;
		int it_dx = ref_patch_dx;
		int it_dy = ref_patch_dy;
		float A11=0;	float A12=0;	float A22 = 0;
		for(int y=0; y<patch_size; y++)
		{
			// ToDo 'it' should point to the position after (y+1)*ref_step +1, not be equal to the value at ref_patch_with_border[(y+1)*ref_step + 1];
//			int it = ref_patch_with_border[(y+1)*ref_step + 1];
			int it = (y+1)*ref_step + 1;

			for(int x=0; x<patch_size; x++, it++, it_dx++, it_dy++)
			{
				int dx = ref_patch_with_border[it+1] - ref_patch_with_border[it-1];
				int dy = ref_patch_with_border[it+ref_step] - ref_patch_with_border[it-ref_step];
				it_dx = dx;
				it_dy = dy;		// we are missing a factor 1/2
				A11	+= dx*dx;	//	we are missing a factor 1/4
				A12 += dx*dy;
				A22 += dy*dy;
			}
		}

		// Compute pixel location in new image:
		float u = (float) cur_px_estimate.get(0);
		float v = (float) cur_px_estimate.get(1);

		// termination condition
		final float min_update_squared = 0.0009f; // 0.03*0.03
		// final int cur_step = cur_img.getStep().p[0];
		final int cur_step = (int) cur_img.step1(0);
		final float Dinv = 1.0f/(A11*A22 - A12*A12);	// we are missing an extra factor 16
		float chi2 = 0;
		float update_u = 0; float update_v = 0;

		for(int iter =0; iter<n_iter; iter++)
		{
			int u_r = (int) Math.floor(u);
			int v_r = (int) Math.floor(v);
			if(u_r < halfpatch_size || v_r < halfpatch_size || u_r >= cur_img.cols()-halfpatch_size
					|| v_r >= cur_img.rows()-halfpatch_size)
				break;

			if(Float.isNaN(u) || Float.isNaN(v))	// ToDo very rarely this can happen, maybe H is singular? should not be at corner.. check
				return false;

			float subpix_x = u-u_r;
			float subpix_y = v-v_r;
			float b1 = 0; float b2 = 0;
			float new_chi2 = 0.0f;

			//			//#ifdef	__SSE2__
			//			// compute bilinear interpolation weights
			//			int wTL = (int)((1.0f-subpix_x)*(1.0f-subpix_y)*(1<< W_BITS));
			//			int wTR = (int)(subpix_x * (1.0f-subpix_y)*(1<< W_BITS));
			//			int wBL = (int)((1.0f-subpix_x)*subpix_y*(1<< W_BITS));
			//			int wBR = (1 << W_BITS) - wTL - wTR - wBL;
			//			
			//			__m128i qw0 = _mm_set1_epi32(wTL + (wTR << 16)); // Sets the 4 signed 32-bit integer values to [wTL, wTR].
			//		    __m128i qw1 = _mm_set1_epi32(wBL + (wBR << 16));
			//		    __m128i z = _mm_setzero_si128();
			//		    __m128 qb0 = _mm_setzero_ps(); // 4 floats
			//		    __m128 qb1 = _mm_setzero_ps(); // 4 floats
			//		    __m128i qdelta = _mm_set1_epi32(1 << (W_BITS-1));
			//		    
			//		    
			//		    UNFINISHED...A11.
			//			#endif

			// compute -A^-1*b
			update_u = ((A12*b2 - A22*b1) * Dinv) * 2; // * 2 to compensate because above, we did not compute the derivate correctly
			update_v = ((A12*b1 - A11*b2) * Dinv) * 2;
			u += update_u;
			v += update_v;

			//			#if SUBPIX_VERBOSE
			//		    cout << "Iter " << iter << ":"
			//		         << "\t u=" << u << ", v=" << v
			//		         << "\t update = " << update_u << ", " << update_v
			//		         << "\t new chi2 = " << new_chi2 << endl;
			//			#endif

			if(update_u*update_u+update_v*update_v < min_update_squared)
			{
				//			#if SUBPIX_VERBOSE
				//			      cout << "converged." << endl;
				//			#endif
				converged = true;
				break;
			}
			chi2 = new_chi2;
		}

		cur_px_estimate.set(u,v);
		return converged;
	}
	public boolean align2D_NEON(
			Mat cur_img, 
			int[] ref_patch_with_border, 
			int ref_patch, 
			int n_iter,
			Vector2d cur_px_estimate)
	{
		final int halfpatch_size = 4;
		final int patch_size = 8;
		final int patch_area = 64;
		boolean converged = false;
		final int W_BITS = 14;

		// compute derivatives of template and prepare inverse compositional
		int ref_patch_dx = patch_area;
		int ref_patch_dy = patch_area;

		// compute gradient and hessian
		int ref_step = patch_size +2;
		int it_dx = ref_patch_dx;
		int it_dy = ref_patch_dy;
		Matrix3d H = new Matrix3d();
		for(int y = 0; y < patch_size; y++)
		{
			// ToDo 'it' should point to the position after (y+1)*ref_step +1, not be equal to the value at ref_patch_with_border[(y+1)*ref_step + 1];
//			int it = ref_patch_with_border[(y+1)*ref_step +1];
			int it = (y+1)*ref_step +1;

			for(int x=0; x<patch_size; x++, it++, it_dx++, it_dy++)
			{
				it_dx = (int) (ref_patch_with_border[it+1] - ref_patch_with_border[it-1]);
				it_dy = (int) (ref_patch_with_border[it+ref_step] - ref_patch_with_border[it-ref_step]);	// divide by 2 missing
				Vector3d J = new Vector3d();//(Double)it_dx, (Double)it_dy, 1.0);
				J.set(it_dx, it_dy, 1.0);
				H = new Matrix3d(H.plus(J.times(J.transpose())).getArray());
			}
		}
		Matrix3d Hinv = H.inverse();
		float mean_diff = 0.0f;

		// Compute pixel location in new image
		double u = cur_px_estimate.get(0);
		double v = cur_px_estimate.get(1);

		// Termination condition
		final float min_update_squared = 0.0009f;	// 0.03*0.03;
		// final int cur_step = cur_img.getStep().p[0];	ToDo initial solution for cur_img.getStep().p[0]
		final long cur_step = cur_img.step1(0);
		Vector3d update;
		Vector3d Jres = new Vector3d();
		for(int iter = 0; iter < n_iter; iter++)
		{
			int u_r = (int) Math.floor(u);
			int v_r = (int) Math.floor(v);
			if(u_r < halfpatch_size || v_r < halfpatch_size || u_r >= cur_img.cols()-halfpatch_size || v_r >= cur_img.rows()-halfpatch_size)
				break;
			if(Double.isNaN(u) || Double.isNaN(v))	// TODO very rarely this can happen, maybe H is singular? should not be at corner.. check
				return false;

			float subpix_x = (float) u-u_r;
			float subpix_y = (float) v-v_r;
			float b1=0;	float b2=0;

			// #ifdef __ARM_NEON__
			//ARM NEON implementation left out.
			// Jres 
			// 
			update = Jres.times(Hinv).times(2); // * 2 to compensate because above, we did not compute the derivative correctly
			u += update.get(0);
			v += update.get(1);
			mean_diff += update.get(2);

//		#if SUBPIX_VERBOSE
			System.out.println("Iter "+iter+" :\t u="+u+", v="+v+"\t update = "+update.get(0)+", "+update.get(1));
//		#endif
			if(update.get(0)*update.get(0)+update.get(1)*update.get(1) < min_update_squared)
			{
//		#if SUBPIX_VERBOSE
				System.out.println("converged.");
//		#endif
				converged = true;
				break;				
			}
		}

		cur_px_estimate.set(u, v);
		return converged;
	}
}

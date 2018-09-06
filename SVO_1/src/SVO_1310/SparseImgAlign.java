package SVO_1310;

import java.io.FileNotFoundException;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.io.UnsupportedEncodingException;
import java.util.ArrayList;
import java.util.Iterator;

import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;

import matrix_types.Matrix6d;
import matrix_types.Vector;
import matrix_types.Vector2d;
import matrix_types.Vector3d;
import matrix_types.Vector6d;

import Jama.Matrix;
import SVO_1310.Vikit.Method;
import SVO_1310.Vikit.ModelType;
import SVO_1310.Vikit.NLLSSolver;
import Sophus.Quaternion;
import Sophus.Se3;

public class SparseImgAlign extends NLLSSolver	// extends/implements vik::NLLSSolver<6, Se3>
{
	private static int patch_halfsize_ = 2;
	private static int patch_size_ = 2*patch_halfsize_;
	private static int patch_area_ = patch_size_*patch_size_;

	private Mat resimg_;

	private Frame ref_frame_;			//!< reference frame, has depth for gradient pixels.
	private Frame cur_frame_;			//!< only the image is known!
	private int level_;					//!< current pyramid level on which the optimization runs.
	private boolean display_;			//!< display residual image.
	private int max_level_;				//!< coarsest pyramid level for the alignment.
	private int min_level_;				//!< finest pyramid level for the alignment.

	// cache:
	private Matrix jacobian_cache_; // = new Matrix(6,6);	//   Matrix<double, 6, Dynamic, ColMajor> jacobian_cache_;
	private boolean have_ref_patch_cache_;
	private Mat ref_patch_cache_;
	private ArrayList<Boolean> visible_fts_ = new ArrayList<Boolean>();		//vector<bool> visible_fts_;


	public SparseImgAlign(int max_level, int min_level, int n_iter, Method method, boolean display, boolean verbose)
	{
		super(n_iter, n_iter, method, verbose, 0.000001);
		//		n_iter_ = n_iter;			// These are set in the super class *nlls_solver
		//		n_iter_init_ = n_iter_;
		//		method_ = method;
		//		verbose_ = verbose;
		//		eps_ = 0.000001;
		display_ = display;
		max_level_ = max_level;
		min_level_ = min_level;
	}

	public void precomputeReferencePatches()
	{
		int border = patch_halfsize_+1;
		Mat ref_img = ref_frame_.get_img_pyr().getMats()[level_];
		int stride = ref_img.cols();
		float scale = 1.0f / (1<<level_);
		Vector3d ref_pos = ref_frame_.pos();
		double focal_length = ref_frame_.getCam().errorMultiplier2();
		int feature_counter = 0;

		//		std::vector<bool>::iterator visiblity_it = visible_fts_.begin();
		//		Iterator<Boolean> visib_it = visible_fts_.iterator();
		int ref_patch_cache_size = ref_patch_cache_.cols()*ref_patch_cache_.rows();
		float[] ref_patch_cache_data = new float[ref_patch_cache_size];
		int visible_fts_index = 0;
		for(Feature it : ref_frame_.getFts_())
		{
			//	check if reference with patch size is within image
			float u_ref = (float) (it.get_px().get(0)*scale);
			float v_ref = (float) (it.get_px().get(1)*scale);
//			System.out.println("u_ref = "+u_ref+", "+"v_ref = "+v_ref);
//			System.out.println("it.get_px().get(0) = "+it.get_px().get(0)+", "+"it.get_px().get(1) = "+it.get_px().get(1));

			int u_ref_i = (int) Math.floor((double)u_ref);	//	ToDo	does this floor act the same as floorf (floor of float)?
			int v_ref_i = (int) Math.floor((double)v_ref);	// 
			if((it).getPoint() == null || u_ref_i - border < 0 || v_ref_i - border < 0 || u_ref_i + border >= ref_img.cols() || v_ref_i + border >= ref_img.rows())
			{
				continue;
			}

			visible_fts_.set(visible_fts_index, true);//	ToDo Include this!	// replacement for visib_it iterator.

			// cannot just take the 3rd points coordinate because of the reprojection errors in the reference image!!
			double depth = (it.getPoint().getPos().minus(ref_pos)).normF();
			Vector3d xyz_ref = it.getF().times(depth);

			// evaluate projection jacobian
			Matrix frame_jac = new Matrix(2,6);
			Frame.jacobian_xyz2uv(xyz_ref, frame_jac);

			// compute bilateral interpolation weights for reference image
			float subpix_u_ref = u_ref-u_ref_i;
			float subpix_v_ref = v_ref-v_ref_i;
			float w_ref_tl = (float)((1.0 - subpix_u_ref) * (1.0 - subpix_v_ref));
			float w_ref_tr = (float)(subpix_u_ref * (1.0 - subpix_v_ref));
			float w_ref_bl = (float)((1.0 - subpix_u_ref) * subpix_v_ref);
			float w_ref_br = (float)(subpix_u_ref * subpix_v_ref);
			int pixel_counter = 0;

//			System.out.println("subpix_u_ref = "+subpix_u_ref+", "+"subpix_v_ref = "+subpix_v_ref);
//			System.out.println("w_ref_tl ="+w_ref_tl+"\tw_ref_tr ="+w_ref_tr+"\tw_ref_bl ="+w_ref_bl+"\tw_ref_br ="+w_ref_br);
			//			float cache_ptr = ref_patch_cache_.getData() + (patch_area_*feature_counter);	// ToDo ref_patch_catch_.dataAddr() instead of ..._cache_.getData()
			int cache_ptr = (patch_area_*feature_counter);	// ToDo ref_patch_catch_.dataAddr() instead of ..._cache_.getData()

//			int ref_patch_cache_size = ref_patch_cache_.cols()*ref_patch_cache_.rows();
//			float[] ref_patch_cache_data = new float[ref_patch_cache_size];
			
			//////////////////////////////////////////////////
			int framesize = ref_img.cols()*ref_img.rows();
			byte[]  ref_img_data = new byte[framesize*ref_img.channels()];	// ToDo may not need '*cur_img.channels()'


			ref_img.get(0,0, ref_img_data);
			//////////////////////////////////////////////////
//			System.out.println();
			
			try {
				FileWriter fileWriter = new FileWriter("/home/michael/Documents/SVO_datasets/ref_img_data.txt");
				PrintWriter writer = new PrintWriter(fileWriter);
				int ref_img_length = ref_img_data.length;
				for(int i=0; i<ref_img_length; i++)
				{

					if(i%47==0)
					{
						writer.println();
					}					
					writer.print(ref_img_data[i]+"\t");

				}
				writer.println( "STOP");
				writer.close();
			} catch (FileNotFoundException e) {
				// TODO Auto-generated catch block
				System.out.println("Could not find the file.");
				e.printStackTrace();
			} catch (UnsupportedEncodingException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			} catch (IOException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}

			
//			PrintWriter writer = new PrintWriter("the-file-name.txt", "UTF-8");
//			writer.println("The first line");
//			writer.println("The second line");
//			writer.close();
//			
			
			for(int y=0; y<patch_size_; y++)//for(int y=0; y<patch_size_; y++)
			{
				// int ref_img_ptr = ref_img.getData() + (v_ref_i + y - patch_halfsize_) * stride + (u_ref_i - patch_halfsize_); ToDo changed getData() to dataAddr() and int to long
				int ref_img_ptr = (v_ref_i + y - patch_halfsize_) * stride + (u_ref_i - patch_halfsize_);
//				System.out.println("ref_img_ptr = " + ref_img_ptr);
				for(int x = 0; x < patch_size_; x++, ref_img_ptr++, cache_ptr++, pixel_counter++)//for(int x = 0; x < patch_size_; x++, ref_img_ptr++, cache_ptr++, pixel_counter++)
				{
					// precompute interpolated reference patch colour
					ref_patch_cache_data[cache_ptr] =  (w_ref_tl * ref_img_data[ref_img_ptr] + w_ref_tr * ref_img_data[ref_img_ptr+1] +  w_ref_bl * ref_img_data[ref_img_ptr+stride] + w_ref_br * ref_img_data[ref_img_ptr+stride+1]);

					
//					System.out.println("*cache_ptr\t"+(w_ref_tl * ref_img_data[ref_img_ptr] + w_ref_tr * ref_img_data[ref_img_ptr+1] +  w_ref_bl * ref_img_data[ref_img_ptr+stride] + w_ref_br * ref_img_data[ref_img_ptr+stride+1]));
//					System.out.print("\t"+w_ref_tl * ref_img_data[ref_img_ptr]+"\t"+ w_ref_tr * ref_img_data[ref_img_ptr+1] +"\t"+   w_ref_bl * ref_img_data[ref_img_ptr+stride] +"\t"+  w_ref_br * ref_img_data[ref_img_ptr+stride+1]);
//					System.out.print("\nw_ref_tl , ref_img_data[ref_img_ptr] = "+w_ref_tl+", "+ ref_img_data[ref_img_ptr]);
//					System.out.print("\tw_ref_tr , ref_img_data[ref_img_ptr+1] = "+w_ref_tr+", "+ ref_img_data[ref_img_ptr+1]);
//					System.out.print("\tw_ref_bl , ref_img_data[ref_img_ptr+stride] = "+w_ref_bl+", "+ ref_img_data[ref_img_ptr+stride]);
//					System.out.print("\tw_ref_br , ref_img_data[ref_img_ptr+1+stride] = "+w_ref_br+", "+ ref_img_data[ref_img_ptr+1+stride]);
//					System.out.println("cache_ptr = "+w_ref_tl * ref_img_data[ref_img_ptr]+"\t"+w_ref_tr * ref_img_data[ref_img_ptr+1]+"\t"+w_ref_bl * ref_img_data[ref_img_ptr+stride]+"\t"+w_ref_br * ref_img_data[ref_img_ptr+stride+1]);
//					System.out.println("ref_img_data[ref_img_ptr] = " + ref_img_data[ref_img_ptr]+ "\tref_img_data[ref_img_ptr+1] = " + ref_img_data[ref_img_ptr+1]+
//										"\tref_img_data[ref_img_ptr+stride] = " + ref_img_data[ref_img_ptr+stride]+ "\tref_img_data[ref_img_ptr+stride+1] = " + ref_img_data[ref_img_ptr+stride+1]);
			
					// we use the inverse compositional: thereby we can take the gradient always at the same position
					// get gradient of warped image (~gradient at warped position)
					double dx = 0.5 * ((w_ref_tl*ref_img_data[ref_img_ptr+1] + w_ref_tr*ref_img_data[ref_img_ptr+2] + w_ref_bl*ref_img_data[ref_img_ptr+stride+1]
							+ w_ref_br*ref_img_data[ref_img_ptr+stride+2])-(w_ref_tl*ref_img_data[ref_img_ptr-1] + w_ref_tr*ref_img_data[ref_img_ptr] 
									+ w_ref_bl*ref_img_data[ref_img_ptr+stride-1] + w_ref_br*ref_img_data[ref_img_ptr+stride]));

					double dy = 0.5 * ((w_ref_tl*ref_img_data[ref_img_ptr+stride] + w_ref_tr*ref_img_data[ref_img_ptr+1+stride] 
							+ w_ref_bl*ref_img_data[ref_img_ptr+stride*2] + w_ref_br*ref_img_data[ref_img_ptr+stride*2+1])-(w_ref_tl*ref_img_data[ref_img_ptr-stride] 
									+ w_ref_tr*ref_img_data[ref_img_ptr+1-stride] + w_ref_bl*ref_img_data[ref_img_ptr] + w_ref_br*ref_img_data[ref_img_ptr+1]));

					// cache the jacobian
					//jacobian_cache_.col(feature_counter*patch_area_ + pixel_counter) = (dx*frame_jac.row(0) + dy*frame_jac.row(1))*(focal_length / (1<<level_));
					Matrix new_row_0 = frame_jac.getMatrix(0, 0, 0, frame_jac.getColumnDimension() - 1).times(dx);
					Matrix new_row_1 = frame_jac.getMatrix(1, 1, 0, frame_jac.getColumnDimension() - 1).times(dy);
					Matrix combined_rows = new_row_0.plus(new_row_1).times(focal_length / (1<<level_));
					int[] rows = {0,1,2,3,4,5};
					int[] cols = {(feature_counter*patch_area_ + pixel_counter)};
					jacobian_cache_.setMatrix(rows,cols, combined_rows.transpose());  
				}
			}	
			feature_counter++; visible_fts_index++;
		}
		ref_patch_cache_.put(0, 0, ref_patch_cache_data);
		have_ref_patch_cache_ = true;
	}

	public double computeResiduals(Se3 T_cur_from_ref, boolean linearize_system, boolean compute_weight_scale) // SparseImgAlign.h = false)
	{
		compute_weight_scale = false;	// from sparseImgAlign.h method field

		// Warp the (cur)rent image such that it aligns with the (ref)erence image
		Mat cur_img = cur_frame_.get_img_pyr().getMats()[(level_)];

		if(linearize_system && display_)
		{
			resimg_ = new Mat(cur_img.size(),CvType.CV_32F,  new Scalar(0));	// ToDo will inherit Mat from Java CV
		}

		if(have_ref_patch_cache_ == false)
		{
			precomputeReferencePatches();
		}

		// compute the weights on the first iteration
		ArrayList<Float> errors = new ArrayList<Float>();
		if(compute_weight_scale)
		{
			errors.ensureCapacity(visible_fts_.size());
		}

		int stride = cur_img.cols();
		int border = patch_halfsize_+1;
		float scale = (float) (1.0/(1<<level_));
		Vector3d ref_pos = new Vector3d(ref_frame_.pos().getArray());
		float chi2 = 0;
		int feature_counter = 0;	// is used to compute the index of the cached jacobian
		//		Iterator<Boolean> vis_it = visible_fts_.iterator();
		int vis_index = 0; 
		for(Feature it: ref_frame_.getFts_())
		{
			Boolean visibility_index_bool = visible_fts_.get(vis_index);

			// check if feature is within image
			if(!visibility_index_bool)
				continue;

			// compute pixel location in cur img
			double depth = it.getPoint().getPos().minus(ref_pos).normF();
			Vector3d xyz_ref = it.getF().times(depth);
			Vector3d xyz_cur = (Vector3d) T_cur_from_ref.times(xyz_ref);			//ToDo this should return a Vector3d
			Vector2d uv_cur_pyr = cur_frame_.getCam().world2cam(xyz_cur).times(scale);
			float u_cur = (float) uv_cur_pyr.get(0);
			float v_cur = (float) uv_cur_pyr.get(1);
			int u_cur_i = (int) Math.floor(u_cur);
			int v_cur_i = (int) Math.floor(v_cur);

			// check if projection is within the image
			if(u_cur_i < 0 || v_cur_i < 0 || u_cur_i - border < 0 || v_cur_i - border < 0 || u_cur_i + border >= cur_img.cols() || v_cur_i + border >= cur_img.rows())
				continue;

			// compute bilateral interpolation weights for the current image
			float subpix_u_cur = u_cur-u_cur_i;
			float subpix_v_cur = v_cur-v_cur_i;
			float w_cur_tl = (float) ((1.0-subpix_u_cur) * (1.0-subpix_v_cur));	// ToDo would swapping type for double instead of casting to float be better?
			float w_cur_tr = (float) (subpix_u_cur * (1.0-subpix_v_cur));
			float w_cur_bl = (float) ((1.0-subpix_u_cur) * subpix_v_cur);
			float w_cur_br = subpix_u_cur * subpix_v_cur;

			//			float ref_patch_cache_ptr = (ref_patch_cache_.getData()) + patch_area_*feature_counter;		// ToDo should ref_patch_cache_ptr be a float of an int
			int ref_patch_cache_ptr = patch_area_*feature_counter;		// ToDo rename ref_patch_cache_ptr since no pointer as in c++?
			float[] ref_patch_cache_data = new float[ref_patch_cache_.rows()*ref_patch_cache_.cols()];
			ref_patch_cache_.get(0, 0, ref_patch_cache_data);	

			int pixel_counter = 0; 		// is used to compute the index of the cached jacobian
			for(int y = 0; y < patch_size_; y++)
			{
				//				int cur_img_ptr = cur_img.getData() + (v_cur_i + y - patch_halfsize_) * stride + (u_cur_i - patch_halfsize_);
				int cur_img_ptr = (v_cur_i + y - patch_halfsize_) * stride + (u_cur_i - patch_halfsize_);

				for(int x = 0; x < patch_size_; x++, pixel_counter++, cur_img_ptr++, ref_patch_cache_ptr++)  
				{

					/////////////////////////////////////////////////
					int framesize = cur_img.cols()*cur_img.rows();
					byte[] data = new byte[framesize*cur_img.channels()];	// ToDo may not need '*cur_img.channels()'

					cur_img.get(y, x, data);

					/////////////////////////////////////////////////
					// compute residual
					float intensity_cur = w_cur_tl * data[cur_img_ptr] + w_cur_tr*data[cur_img_ptr+1] + w_cur_bl*data[cur_img_ptr+stride] + w_cur_br*data[cur_img_ptr+stride+1];
					//			        const float intensity_cur = w_cur_tl*cur_img_ptr[0] + w_cur_tr*cur_img_ptr[1] + w_cur_bl*cur_img_ptr[stride] + w_cur_br*cur_img_ptr[stride+1];
					float res = intensity_cur - ref_patch_cache_data[(ref_patch_cache_ptr)];
//					System.out.println("data[cur_img_ptr] = "+data[cur_img_ptr]+"\tdata[cur_img_ptr+1] = "+data[cur_img_ptr+1]+"\tdata[cur_img_ptr+stride] = "+data[cur_img_ptr+stride]+"\tdata[cur_img_ptr+stride+1] = "+data[cur_img_ptr+stride+1]);
//					System.out.println("w_cur_tl = "+w_cur_tl+"\tw_cur_tr = "+w_cur_tr+"w_cur_bl = "+w_cur_bl+"\tw_cur_br = "+w_cur_br);
//					System.out.println("intensity_cur = " + intensity_cur +"\t, ref_patch_cache_data[(ref_patch_cache_ptr)] = "+ref_patch_cache_data[(ref_patch_cache_ptr)]);
//					System.out.println("res = " + res );

					// used to compute scale for robust cost
					if(compute_weight_scale)
						errors.add(Math.abs(res));

					// robustification
					float weight = 1;
					if(use_weights_)			// use_weights_ Inherited from nlls_solver
					{
						weight = weight_function_.getValue(res/scale);		// weight_function_ inherited from nlls_solver
					}
//					System.out.println("res = " + res);
					chi2 += res*res*weight;
					n_meas_++;

					if(linearize_system)
					{
						// compute Jacobian, weighted Hessian and weighted "steepest descend images" (times error)
						//						Vector6d J = new Vector6d(jacobian_cache_.getMatrix(0, jacobian_cache_.getRowDimension(), 
						//								feature_counter*patch_area_ + pixel_counter,
						//								feature_counter*patch_area_ + pixel_counter).getRowPackedCopy());	// ToDo check the multiplication order against C++. Should be identical


						int col_val = feature_counter*patch_area_ + pixel_counter;
						double[] J_vals = {
								jacobian_cache_.get(0, col_val),
								jacobian_cache_.get(1, col_val),
								jacobian_cache_.get(2, col_val),
								jacobian_cache_.get(3, col_val),
								jacobian_cache_.get(4, col_val),
								jacobian_cache_.get(5, col_val)
						};
						Vector6d J = new Vector6d(J_vals);
						Vector6d jT_times_weight = J.times(weight);	// No transpose as it is taken into account in the times method.
						Matrix6d j_times_jT = J.times(jT_times_weight);
						H_.plusEquals(j_times_jT);//J.times(J.transpose().times(weight)));	// ToDo noalias() has been removed
						Jres_ = Jres_.minus(J.times(res*weight));			// ToDo noalias() has been removed
						if(display_){
							//resimg_.at((int) v_cur+y-patch_halfsize_, (int) u_cur+x-patch_halfsize_) = res/255;	// ToDo is this setting a value of Mat at a coordinate as res/255?
							resimg_.put((int) v_cur+y-patch_halfsize_, (int) u_cur+x-patch_halfsize_, res/255);		// ToDo this should replace the above, when OpenCV Java is installed
						}
					}
				}

			}
			feature_counter++;	// ToDo ensure this is at the end of for each loop





		}
		// compute the weights on the first iteration
		if(compute_weight_scale && iter_ ==0)
			scale_ = scale_estimator_.Compute(errors);		// ToDo scale_estimator and scale_ from nlls_solver

		return chi2/n_meas_;

	}
	public int solve()
	{
		x_ = new Vector6d(H_.chol().solve(Jres_).getArray());
		if(Double.isNaN(x_.get(0, 0))) //[0]))
			return 0;
		return 1;
	}

	// ToDo This is currently never called. changed T_curold_from_ref to Se2 instead of ModelType
	//	private void update(Se3 T_curold_from_ref, Se3 T_curnew_from_ref)
	//	{
	//		T_curnew_from_ref = T_curold_from_ref.times(Se3.exp(x_.times(-1))); //SE3::exp(-x_);	// ToDo Model Type: type of the model, e.g. SE2, SE3
	//	}

	// ToDo complete method from Img_Align
	public void startIteration()
	{

	}
	public void finishIteration()
	{
		if(display_)
		{
			//		    cv::namedWindow("residuals", CV_WINDOW_AUTOSIZE);			ToDo
			//		    cv::imshow("residuals", resimg_*10);
			//		    cv::waitKey(0);
		}
	}


	// Return fisher information matrix, i.e. the Hessian of the log-likelihood at the converged state.
	public Matrix getFisherInformation()
	{
		double sigma_i_sq = 5e-4 * 255 * 255;	// image noise
		Matrix I = new Matrix(6,6);
		I = H_.times(1/sigma_i_sq);				// H (Hessian approximation) from nlls_solver
		return I;
	}




	public int run(Frame ref_frame, Frame cur_frame) //Frame ref_frame, Frame cur_frame
	{
		reset();

		if(ref_frame.getFts_().isEmpty())
		{
			//		    SVO_WARN_STREAM("SparseImgAlign: no features to track!");
			return 0;
		}

		ref_frame_ = ref_frame;
		cur_frame_ = cur_frame;
		ref_patch_cache_ = new Mat(ref_frame_.getFts_().size(), patch_area_, CvType.CV_32F); //CvType.CV_32SC1);//CvType.CV_8UC1);//
		jacobian_cache_ = new Matrix(6, ref_patch_cache_.rows()*patch_area_);
		//		visible_fts_.ensureCapacity(ref_patch_cache_.rows());	//visible_fts_.resize(ref_pat_cache_.rows, false) // ToDo: should it be reset at each level
		for(int i=0; i < ref_patch_cache_.rows(); i++)
		{
			visible_fts_.add(false);

		}
		//	    System.out.print("1 SPARSE.. ref_frame.getT_f_w_() = \t");/////////////////////////////////////////////////////////////////////////////////////
		//	    ref_frame.getT_f_w_().print();/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

		Se3 T_cur_from_ref = cur_frame_.getT_f_w_().times(ref_frame_.getT_f_w_().inverse());

		//		System.out.print("\nT_cur_from_ref = \t");/////////////////////////////////////////////////////////////////////////////////////
		//		T_cur_from_ref.print();/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		//		System.out.print("\ncur_frame.getT_f_w_() = \t");/////////////////////////////////////////////////////////////////////////////////////
		//	    cur_frame.getT_f_w_().print();/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		//		System.out.print("\nref_frame.getT_f_w_().inverse() = \t");/////////////////////////////////////////////////////////////////////////////////////
		//	    ref_frame.getT_f_w_().inverse().print();/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

		for(level_ = max_level_; level_>=min_level_; level_--)
		{
			mu_ = 0.1;			// mu_ from nlls_solver
			//jacobian_cache_.setZero(); Not necessary since Jama Matrix sets values to 0 from the start.
			have_ref_patch_cache_ = false;
			if(verbose_)
				System.out.println("Pyramid Level = "+ level_);
			//			      printf("\nPYRAMID LEVEL %i\n---------------\n", level_);
			optimize(T_cur_from_ref);		// ToDo optimize() inherited from nlls_solver
		}

		System.out.print("\nT_cur_from_ref = \t");/////////////////////////////////////////////////////////////////////////////////////
		T_cur_from_ref.print();/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

		// ToDo Se3.times(se3)so3
		//		System.out.println("T_cur_from_ref.so3 =");
		//		T_cur_from_ref.getSo3().print();
		//		System.out.println("ref_frame T_f_w_");
		//		ref_frame_.getT_f_w_().print();

		Se3 T_cur_from_ref_times_ref_frame_T_f_w_ = T_cur_from_ref.times(ref_frame_.getT_f_w_());
		cur_frame_.setT_f_w_(T_cur_from_ref_times_ref_frame_T_f_w_);

		return n_meas_/patch_area_;	//n_meas_ inheritted from nlls_solver
	}
}

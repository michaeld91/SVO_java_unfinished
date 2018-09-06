package SVO_1310;

import java.util.ArrayList;

import org.opencv.core.Mat;

import matrix_types.JamaUtils;
import matrix_types.Matrix2d;
import matrix_types.Matrix3d;
import matrix_types.Vector2d;
import matrix_types.Vector3d;
import Jama.Matrix;
import SVO_1310.Vikit.Timer;
import SVO_1310.Vikit.ZMSSD;
import SVO_1310.Vikit.math_utils;
import Sophus.Se3;

// Patch-matcher for reprojection-matching and epipolar search in triangulation.
public class Matcher {

	private static int halfpatch_size_ = 4;
	private static int patch_size_ = 8;
	private ZMSSD PatchScore = new ZMSSD(halfpatch_size_);	// ToDo what should this be initialized to?
	private Matcher_Options options_ = new Matcher_Options();

	private byte[] patch_ = new byte[patch_size_*patch_size_]; // For c++ compiler __attribute__ ((aligned (16)));
	private byte[] patch_with_border_ = new byte[(patch_size_+2)*(patch_size_+2)];// For c++ compiler __attribute__ ((aligned (16)));

	private Matrix2d A_cur_ref_;			// affine warp matrix
	private Vector2d epi_dir_ = new Vector2d();		// set as 0,0.				
	private double epi_length_;				// length of epipolar line segment in pixels (only used for epipolar search)
	private double h_inv_;					// hessian of 1d image alignment along epipolar line
	private int search_level_;
	private boolean reject_;
	private Feature ref_ftr_;
	private Vector2d px_cur_ = new Vector2d();
	private Vector2d px_scaled = new Vector2d();				// Removed from find match direct so its accessible for other methods

	private Feature_alignment feature_alignment = new Feature_alignment(); //Java addition. may need for this to be a singleton object?
	Warp warp = new Warp();

	public Matcher()
	{

	}
	public Vector2d get_px_cur_()
	{
		return px_cur_;
	}
	//SE3 changed to Se3
	public double depthFromTriangulation(Se3 T_search_ref, Vector3d f_ref, Vector3d f_cur)//, double depth)
	{
	
		double depth = -1; // equivalent to false boolean
		Vector3d rotation_mat_times_f_ref = T_search_ref.rotation_Matrix().times(f_ref);
		Matrix3d rotationMat = T_search_ref.rotation_Matrix();
		// A has 3 rows and 2 cols.
		Matrix A = new Matrix(3,2);
		
		A.set(0, 0, rotation_mat_times_f_ref.get(0));
		A.set(1, 0, rotation_mat_times_f_ref.get(1));
		A.set(2, 0, rotation_mat_times_f_ref.get(2));
		A.set(0, 1, f_cur.get(0));
		A.set(1, 1, f_cur.get(1));
		A.set(2, 1, f_cur.get(2));


		Matrix2d AtA = new Matrix2d( A.transpose().times(A).getArray());
		if(AtA.det() < 0.000001)
			return depth;	// return depth;	
		Matrix AtA_inv_times_A_transp = AtA.inverse().times(A.transpose());
		Matrix depth2_prestep = AtA_inv_times_A_transp.times(T_search_ref.get_Translation().transpose());
		Vector2d depth2 = new Vector2d();
		depth2.set(-depth2_prestep.get(0, 0),-depth2_prestep.get(1, 0));
//		Vector2d depth2 = (Vector2d) AtA.inverse().times(A.transpose()).times(T_search_ref.get_Translation()).times(-1);
		
		
		depth = Math.abs(depth2.get(0));
//		System.out.println("Depth = "+ depth);
		
//		System.out.println("\nT_search_ref.quaternion = "+T_search_ref.getSo3().getQuaternion().toString());
//		System.out.println("\nT_search_ref.translation = "+T_search_ref.get_Translation());
//		
//		System.out.println("f_ref() = "+f_ref);
//		System.out.println("f_cur = "+f_cur);
//		System.out.println("depth = " + depth);
	
		return depth;	//return depth;	This should be 0 or positive, implying a true boolean	 
	}
	// Find a match by directly applying subpix refinement.
	// IMPORTANT! This function assumes that px_cur is already set to an estimate that is within ~2-3 pixel of the final result!
	public boolean findMatchDirect(Point pt, Frame cur_frame, Vector2d px_cur)
	{
		if(!pt.getCloseViewObs(cur_frame.pos(), this))
		{
			return false;
		}
		if(!ref_ftr_.getFrame().getCam().isInFrame((Vector2d)ref_ftr_.get_px().times(1/(1<<ref_ftr_.getLevel())), halfpatch_size_+2, ref_ftr_.getLevel()))
		{
			return false;
		}
		//		System.out.println("Is this line read?");
		//
		//		System.out.println("ref_ftr_.getFrame().pos()"+ref_ftr_.getFrame().pos());
		//		System.out.println("pt.getPos()"+pt.getPos());
		Warp warp = new Warp();
		Matrix2d A_cur_ref_ = warp.getWarpMatrixAffine(ref_ftr_.getFrame().getCam(),
				cur_frame.getCam(), ref_ftr_.get_px(), ref_ftr_.getF(),
				ref_ftr_.getFrame().pos().minus(pt.getPos()).normF(),
				(Se3)cur_frame.getT_f_w_().times(ref_ftr_.getFrame().getT_f_w_().inverse()), ref_ftr_.getLevel());	//normF takes Frobenius norm, same as Eigen::norm() in C++

		///////////////////////////////


		//////////////////////////////

		search_level_ = warp.getBestSearchLevel(A_cur_ref_ , Config.getnPyrLevels()-1);
		try {
			warp.warpAffine(A_cur_ref_, ref_ftr_.getFrame().get_img_pyr().getMats()[ref_ftr_.getLevel()], ref_ftr_.get_px(),
					ref_ftr_.getLevel(), search_level_, halfpatch_size_+1, patch_with_border_);
		} catch (Exception e) {
			// TODO Auto-generated catch block //Not sure why try/catch exception is needed
			e.printStackTrace();
		}
		createPatchFromPatchWithBorder();

		// px_cur should be set
		px_scaled = (Vector2d) px_cur.times(1/(1<<search_level_));

		boolean success = false;
		if(ref_ftr_.getType()==FeatureType.EDGELET)
		{
			Vector2d dir_cur = (Vector2d)A_cur_ref_.times(ref_ftr_.getGrad());
			dir_cur.normF();
			success = feature_alignment.align1D(cur_frame.get_img_pyr().getMats()[search_level_],
					dir_cur, patch_with_border_, patch_, options_.getAlign_max_iter(), px_scaled, h_inv_);
		}
		else
		{
			success = feature_alignment.align2D(cur_frame.get_img_pyr().getMats()[search_level_],
					patch_with_border_, patch_, options_.getAlign_max_iter(), px_scaled);
		}
		px_cur = px_scaled.times(1<<search_level_);
		return success;

	}

	// Find a match by searching along the epipolar line without using any features.
	public boolean findEpipolarMatchDirect(Frame ref_frame, Frame cur_frame, Feature ref_ftr, double d_estimate, double d_min, double d_max, Depth depth)//double depth)
	{
		//		System.out.println("ref_frame.getT_f_w_() = ");
		//		ref_frame.getT_f_w_().print();
		//		System.out.println("============================");

//		////////////////////////////////////////timer print out 2 y loop timer //////////////////////////////////////////////////////////
//		Timer timer = new Timer();
//		timer.start();

		// double null_depth = -1;
		Se3 T_cur_ref = (Se3) cur_frame.getT_f_w_().times(ref_frame.getT_f_w_().inverse());
		int zmssd_best = PatchScore.getThreshold();
		Vector2d uv_best = new Vector2d();

		//////////////////////////////////////////////////////////////////////////////////////
		//// Compute start and end of epipolar line in old_kf for match search, on unit plane!
		//System.out.println("(ref_ftr.getF().times(d_min)).transpose() = ");
		//(ref_ftr.getF().times(d_min)).transpose().print(0, 4);
		//
		//System.out.println("T_cur_ref = ");
		//T_cur_ref.print(0, 4);depthFromTriangulation
		//
		//System.out.println("T_cur_ref.times((ref_ftr.getF().times(d_min)).transpose())) = ");
		//T_cur_ref.times((ref_ftr.getF().times(d_min)).transpose()).print(0, 4);
		//////////////////////////////////////////////////////////////////////////////////////


		Vector2d A = math_utils.project2d((T_cur_ref.times((ref_ftr.getF().times(d_min)))));	//Still in C++ format. Should project2d be in math_util in vikit??
		Vector2d B = math_utils.project2d((T_cur_ref.times((ref_ftr.getF().times(d_max)))));

		//		System.out.println("d_min, d_max = "+d_min+", "+d_max);
		//		A.print(0,6);
		//		B.print(0,6);

		Vector2d A_minus_B = A.minus(B);
		//		A_minus_B.print(0, 9);
		epi_dir_.set(A_minus_B.get(0), A_minus_B.get(1));

		// Compute affine warp matrix
		//Warp warp = new warp(); Moved into fields
		if(d_estimate==0)
		{
			System.out.println("d_estimate = " + d_estimate);
		}
		A_cur_ref_ = warp.getWarpMatrixAffine(ref_frame.getCam(), cur_frame.getCam(), ref_ftr.get_px(), ref_ftr.getF(), d_estimate, T_cur_ref, ref_ftr.getLevel());


		// Feature pre-selection
		reject_ = false;

		if(ref_ftr.getType() == FeatureType.EDGELET && options_.getEpi_search_edgelet_filtering())
		{
			Vector2d grad_cur = new Vector2d();
			grad_cur.set(JamaUtils.normalize(A_cur_ref_.times(ref_ftr.getGrad())));
			double cosangle = Math.abs(JamaUtils.dotproduct(grad_cur, JamaUtils.normalize(epi_dir_)));
			if(cosangle < options_.getEpi_search_edgelet_max_angle())
			{
				reject_ = true;
				return false; //null_depth; //return false;
			}
		}
		search_level_ = warp.getBestSearchLevel(A_cur_ref_, Config.getnPyrLevels()-1);


		// Find length of search range on epipolar line
		Vector2d px_A = cur_frame.getCam().world2cam(A);
		Vector2d px_B = cur_frame.getCam().world2cam(B);
		epi_length_ = (px_A.minus(px_B)).normF()/(1<<search_level_);

		// Warp reference patch at ref_level
		try {
			warp.warpAffine(A_cur_ref_, ref_frame.get_img_pyr().getMats()[ref_ftr.getLevel()], ref_ftr.get_px(), ref_ftr.getLevel(), search_level_, halfpatch_size_+1, patch_with_border_);
		} catch (Exception e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		createPatchFromPatchWithBorder();


		if(epi_length_< 2.0)
		{

			Vector2d px_A_plus_B = px_A.plus(px_B);
			Vector2d px_A_plus_B_halved = px_A_plus_B.times(.5);
			px_cur_.set(px_A_plus_B_halved);
			boolean res;
			if(options_.getAlign_1d())
			{
				Vector2d pxA_B_norm = new Vector2d();
				pxA_B_norm.set(JamaUtils.normalize((px_A.minus(px_B))));
				res = feature_alignment.align1D(cur_frame.get_img_pyr().getMats()[search_level_],
						pxA_B_norm, patch_with_border_, patch_, options_.getAlign_max_iter(), px_scaled, h_inv_);
			}
			else
			{
				res = feature_alignment.align2D(cur_frame.get_img_pyr().getMats()[search_level_], patch_with_border_, patch_, options_.getAlign_max_iter(), px_scaled);
			}
			if(res)
			{
//				System.out.println("res = true");
				//////////////////////////////////////
				px_cur_ = px_scaled.times(1<<search_level_);
				depth.setDepth(depthFromTriangulation(T_cur_ref, ref_ftr.getF(), cur_frame.getCam().cam2world(px_cur_)));//, depth.getDepth()));
				if(depth.getDepth()>=0)	//
				{
//					System.out.println("Return true: 0");
					return true;//depthFromTriangulation(T_cur_ref, ref_ftr.getF(), cur_frame.getCam().cam2world(px_cur_));	// return true;
				}
				return false;//null_depth;	// return false;
			}
		}

		double n_steps =  (epi_length_/0.7); // one step per pixel // ToDo should probably be an integer
		Vector2d step = epi_dir_.times(1/n_steps);

		if(n_steps > options_.getMax_epi_search_steps())
		{
			System.out.println("WARNING: skip epipolar search: "+ n_steps+ " evaluations, length= "+epi_length_+ ", d_min= "+d_min+", d_max = "+d_max);
			return false; //null_depth;		// return false;
		}

		// for matching, precompute sum and sum2 of warped reference patch
		int pixel_sum = 0;
		int pixed_sum_square = 0;
		ZMSSD patch_score = PatchScore; //new ZMSSD(patch_);	// ToDo Confusion between ZMSSD and PatchScore and Patch_Score // PatchScore patch_score(patch_);
		patch_score.set_ZMSSD(patch_);
		// Now we sample along the epipolar line
		Vector2d uv = new Vector2d(); 
		uv.set(B.minus(step));
		Vector2d last_checked_pxi = new Vector2d(); last_checked_pxi.set(0,0);
		n_steps++;
//		////////////////////////////////////////timer print out 2 y loop timer //////////////////////////////////////////////////////////
//		System.out.println("position 6 time = " + timer.stop()); timer.resume();

		Mat cur_frame_Mat = cur_frame.get_img_pyr().getMats()[search_level_];
		int  cur_frame_cols = cur_frame_Mat.cols();
		int  cur_frame_rows = cur_frame_Mat.rows();
		for(int i=0; i<n_steps; i++, uv.plusEquals(step))
		{
			Vector2d px = cur_frame.getCam().world2cam(uv);
			Vector2d pxi = new Vector2d();
			pxi.set((double)Math.round(px.get(0)/(1<<search_level_)+0.5), (double)Math.round(px.get(1)/(1<<search_level_)+0.5));	// +0.5 to round to closest int

			if(pxi == last_checked_pxi)
			{
				continue;
			}
			last_checked_pxi = pxi;

			// Check if the patch is full within the new frame
			if(!cur_frame.getCam().isInFrame(pxi, patch_size_, search_level_))
			{
				continue;
			}

			// ToDo interpolation would probably be a good idea (comment from C++ SVO)
			//			int cur_patch_ptr = (int)(cur_frame.get_img_pyr().getMats()[search_level_].getData()+
			int cur_patch_ptr = //cur_frame.get_img_pyr().getMats()[search_level_].data()+
					(int) ((pxi.get(1)-halfpatch_size_)*cur_frame_cols+(pxi.get(0)-halfpatch_size_));


			byte[] data = new byte[cur_frame_cols*cur_frame_rows];
			//			System.out.println("Mat type = " + cur_frame.get_img_pyr().getMats()[search_level_].type());
			cur_frame.get_img_pyr().getMats()[search_level_].get(0, 0, data);

			int zmssd = patch_score.computeScore(data, cur_frame_cols, cur_patch_ptr);

			if(zmssd < zmssd_best)
			{
				zmssd_best = zmssd;
				uv_best = uv;
			}
		}
//		////////////////////////////////////////timer print out 2 y loop timer //////////////////////////////////////////////////////////
//		System.out.println("position 7 time = " + timer.stop()); timer.resume();

		if(zmssd_best < PatchScore.getThreshold())
		{
			if(options_.getSubpix_refinement())
			{
//				///////////////////////////////////////// to be deleted ///////////////////////////////
//				uv_best.set(-1.15835, -0.673435);
				px_cur_ = cur_frame.getCam().world2cam(uv_best);
				Vector2d px_scaled = new Vector2d();
				px_scaled.set(px_cur_.timesEquals(1/(1<<search_level_)));
				boolean res;
				if(options_.getAlign_1d())
				{
					Vector2d pxA_B_norm = new Vector2d();
					pxA_B_norm.set(JamaUtils.normalize(px_A.minus(px_B)));
					res = feature_alignment.align1D(cur_frame.get_img_pyr().getMats()[search_level_], pxA_B_norm,
							patch_with_border_, patch_, options_.getAlign_max_iter(), px_scaled, h_inv_);
				}
				// ToDo finish this method with below
				else
					res = feature_alignment.align2D(cur_frame.get_img_pyr().getMats()[search_level_], patch_with_border_, patch_, options_.getAlign_max_iter(), px_scaled);
				// cur_frame.img_pyr_[search_level_], patch_with_border_, patch_, options_.align_max_iter, px_scaled
				if(res)
				{
					px_cur_ = px_scaled.times(1<< search_level_);
					depth.setDepth(depthFromTriangulation(T_cur_ref, ref_ftr.getF(), cur_frame.getCam().cam2world(px_cur_)));//, depth);
//					System.out.println("\nT_cur_ref = "+T_cur_ref.getSo3().getQuaternion().toString());
//					System.out.println("ref_ftr.getF() = "+ref_ftr.getF());
//					System.out.println("cur_frame.getCam().cam2world(px_cur_) = "+cur_frame.getCam().cam2world(px_cur_));
//					System.out.println("depth = " + depth.getDepth());
					if(depth.getDepth()>=0)
					{
//						System.out.println("Return true: 1");
						return true;//depthFromTriangulation(T_cur_ref, ref_ftr.getF(), cur_frame.getCam().cam2world(px_cur_));
					}
				}
				return false; //null_depth;
			}
			px_cur_ = cur_frame.getCam().world2cam(uv_best);
			Vector3d unproj_uv_best_norm = new Vector3d();
			unproj_uv_best_norm.set(JamaUtils.normalize(math_utils.unprojected2d(uv_best)));
			depth.setDepth(depthFromTriangulation(T_cur_ref, ref_ftr.getF(), unproj_uv_best_norm));//, depth);
			if(depth.getDepth()>=0)
				{
//				System.out.println("Return true: 2");
					return true;//depthFromTriangulation(T_cur_ref, ref_ftr.getF(), unproj_uv_best_norm);
				}
		}

		return false; //null_depth;

	}



	//ToDo. Not making any progress here so stopping for now.
	public void createPatchFromPatchWithBorder()
	{
		//		int ref_patch_ptr = patch_[0];
		//		for(int y=1; y<patch_size_+1; y++, ref_patch_ptr += patch_size_)
		//		{
		//			int ref_patch_border_ptr = patch_with_border_ + y*(patch_size_+2) + 1;
		//			for(int x = 0; x < patch_size_; x++)
		//				ref_patch_ptr[x] = ref_patch_border_ptr[x];
		//		}
		ArrayList<Byte> al = new ArrayList<Byte>();
		// Note that the 2 in patch_size_ +2 (etc) refers to the border of 1 on all sides
		for(int y=(patch_size_+2); y< (patch_size_+2)*(patch_size_+2-1); y++)
		{
			if(y%(patch_size_+2)==0||y%(patch_size_+2)==(patch_size_+2-1))
				continue;
			else
				al.add(patch_with_border_[y]);
		}
		for(int i =0; i < al.size(); i++)
		{
			patch_[i] = al.get(i);
		}
	}

	public void set_ref_ftr_(Feature ft)
	{
		ref_ftr_=ft;
	}
	public Feature get_ref_ftr_()
	{
		return ref_ftr_;
	}
	public int get_search_level_()
	{
		return search_level_;
	}
	public Matrix2d get_A_cur_ref_()
	{
		return A_cur_ref_;
	}
}

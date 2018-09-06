package SVO_1310;

import java.util.ArrayList;

import matrix_types.Matrix6d;
import matrix_types.Vector2d;
import matrix_types.Vector3d;
import matrix_types.Vector6d;
import Jama.Matrix;
import SVO_1310.Vikit.MADScaleEstimator;
import SVO_1310.Vikit.TukeyWeightFunction;
import SVO_1310.Vikit.math_utils;
import Sophus.Se3;

public class PoseOptimizer {

	// ToDo Note that there has been a temporary solution for the TukeyWeightFunction, where the initial value can be set as -1.
	// This is purely as I cannot find the value set anywhere in the original code.
	public static double[] optimizeGaussNewton(
			double reproj_thresh,
			int n_iter, 
			boolean verbose, 
			Frame frame)//,double estimated_scale, double error_init,double error_final, int num_obs) 
	{			

		double[] return_values = new double[4];	//double array for double estimated_scale, double error_init,double error_final & int num_obs
		//	init
		double chi2 = 0.0;
		ArrayList<Double> chi2_vec_init;// = new ArrayList<Double>(); 
		ArrayList<Double> chi2_vec_final;// = new ArrayList<Double>();
		
		TukeyWeightFunction weight_function = new TukeyWeightFunction(-1);
		Se3 T_old = frame.getT_f_w_();
				
		Vector6d b = new Vector6d();
		Matrix6d A = new Matrix6d();
		// compute the scale of the error for robust estimation
		ArrayList<Double> errors = new ArrayList<Double>(frame.getFts_().size());
		for(Feature ftr: frame.getFts_())
		{

			if(ftr.getPoint() == null)
				continue;
			Vector2d e = math_utils.project2d(ftr.getF()).minus(math_utils.project2d(frame.getT_f_w_().times(ftr.getPoint().getPos())));
			e = e.times(1.0 / (1<<ftr.getLevel()));
			errors.	add(e.normF());
		}
		if(errors.isEmpty())
			return null;
		MADScaleEstimator scale_estimator = new MADScaleEstimator();
		double estimated_scale = scale_estimator.compute(errors);	// ToDo if this is to be used elsewhere, it should be either returned or set more permanently
		
		int num_obs = errors.size(); 	// ToDo same field setting issue as above
		chi2_vec_init = new ArrayList<Double>(num_obs);//ensureCapacity(num_obs);
		chi2_vec_final = new ArrayList<Double>(num_obs);//.ensureCapacity(num_obs);
		double scale = estimated_scale;
		
		for(int iter = 0; iter<n_iter; iter++)//ToDo change to reduce the number of test results:: iter<n_iter; iter++)
		{
			// overwrite scale
			if(iter == 5)
				scale = 0.85/frame.getCam().errorMultiplier2();
			

			double new_chi2 = 0.0;
			
			// compute residual		
			/////////////////////////////////////////////////////////////////////////////
			////////////////////////MUST CHANGE BACK NEXT LINE///////////////////////////
			//Feature it = frame.getFts_().get(0);//
			for(Feature it: frame.getFts_())
			{
				if(it.getPoint() == null)
					continue;
				Matrix J= new Matrix(2, 6);
//				Vector3d xyz_f = it.getPoint().getPos().times(frame.getT_f_w_());
//				frame.getT_f_w_().print();
//				System.out.println("Feature coordinates = "+ it.get_px(0)+","+it.get_px(1));
				Vector3d xyz_f = frame.getT_f_w_().times(it.getPoint().getPos());
				Frame.jacobian_xyz2uv(xyz_f, J);
				Vector2d e = math_utils.project2d(it.getF()).minus(math_utils.project2d(xyz_f));
				
//				System.out.println("it.getF() = "+it.getF());
//				System.out.println("xyz_f = "+xyz_f);
//				System.out.println("frame.getT_f_w_() = ");
//				frame.getT_f_w_().print();
//				System.out.println("it.getPoint().getPos()" + it.getPoint().getPos());
//				System.out.println("e = " + e.get(0) +"," + e.get(1));
//				System.out.println("math_utils.project2d(it.getF())="+math_utils.project2d(it.getF()));
//				System.out.println("math_utils.project2d(xyz_f)="+math_utils.project2d(xyz_f));

				double sqrt_inv_cov = 1.0 / (1<<(it.getLevel()));
				e = e.times(sqrt_inv_cov);
				if(iter == 0)
				{
					chi2_vec_init.add(chi2_vec_init.size(), e.normF()*e.normF());	// Just for debug	// ToDo ensure same as e.squaredNorm()
				}
				J = J.times(sqrt_inv_cov);
				double weight = weight_function.value(e.normF()/scale);
				Matrix6d mat6_6 = new Matrix6d(J.transpose().times(J).getArray());
				Matrix6d mat6_6_times_weight = mat6_6.times(weight);
				A = A.plus(mat6_6_times_weight);
				Matrix J_trans = J.transpose();
				Vector6d J_trans_times_e = new Vector6d(J_trans.times(e.transpose()).getArray());
				b = b.minus(J_trans_times_e.times(weight));
				new_chi2 = e.squaredNorm()*weight;	
				
//				System.out.println("e = " + e.get(0) +"," + e.get(1));
//				System.out.println("e.norm*e.norm = " + e.normF()*e.normF());
//				System.out.println("weight = " + weight+"\n");
			}
			
			// solve linear system
			Vector6d dT = new Vector6d(A.chol().solve(b.transpose()).transpose().getArray());
			// check if error increased
			if((iter > 0 && new_chi2 > chi2) || Double.isNaN(dT.get(0)))
			{
				if(verbose)	// ToDo check that System.out equates to the std::cout in c++
					System.out.println("it "+ iter + "\t FAILURE \t new_chi2 = "+ new_chi2);
				frame.setT_f_w_(T_old);
				break;
			}
			
			// update the model
			Se3 T_new = Se3.exp(dT);//.times(frame.getT_f_w_());
			T_old = frame.getT_f_w_();
			frame.setT_f_w_(T_new);	
			chi2 = new_chi2;
			if(verbose)
				System.out.println("it "+ iter +"\t Success \t new_chi2 = "+new_chi2+"\t norm(dT) = "+math_utils.norm_max(dT));
			
			// Stop when converged
			if(math_utils.norm_max(dT)<= Global.EPS)
				break;		
		}
		
		// Set covariance as inverse information matrix. Optimistic estimator!
		double pixel_variance = 1.0;
		// frame is not set here, just t
		frame.setCov_((A.times(Math.pow(frame.getCam().errorMultiplier2(),2))).times(pixel_variance).inverse());
		
		// Remove measurements with too large reprojection error
		double reproj_thresh_scaled = reproj_thresh / frame.getCam().errorMultiplier2();
		int n_deleted_refs = 0;
		
		for(Feature it: frame.getFts_())
		{
			if(it.getPoint() == null)
				continue;
			Vector2d e = math_utils.project2d(it.getF()).minus(math_utils.project2d(frame.getT_f_w_().times(it.getPoint().getPos())));
			double sqrt_inv_cov = 1.0 / (1<<(it.getLevel()));
			e = e.times(sqrt_inv_cov);
			chi2_vec_final.add(chi2_vec_final.size(), e.normF()*e.normF());
			if(e.normF() > reproj_thresh_scaled)
			{
				// we don't need to delete a reference in the point since it has not been created yet
				it.setPoint(null);
				n_deleted_refs++;
			}
		}
		
		double error_init = 0.0;
		double error_final = 0.0;
		if(!chi2_vec_init.isEmpty())
			error_init = Math.sqrt(math_utils.getMedian(chi2_vec_init))*frame.getCam().errorMultiplier2();
		if(!chi2_vec_final.isEmpty())
			error_final = Math.sqrt(math_utils.getMedian(chi2_vec_final))*frame.getCam().errorMultiplier2();
		
		estimated_scale *= frame.getCam().errorMultiplier2();
		if(verbose)
			System.out.println("n deleted obs = "+n_deleted_refs+"\t scale = "+estimated_scale+
					"\t error init = "+ estimated_scale+ "\t error end = "+ error_final);
		num_obs -= n_deleted_refs;
		return_values[0] = estimated_scale;
		return_values[1] = error_init;
		return_values[2] = error_final;
		return_values[3] = num_obs;
		return return_values;			
	}
}




























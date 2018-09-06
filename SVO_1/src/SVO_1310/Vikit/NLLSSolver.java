package SVO_1310.Vikit;

import matrix_types.Matrix3d;
import matrix_types.Vector;
import matrix_types.Vector3d;
import matrix_types.Vector6d;
import Jama.Matrix;
import SVO_1310.SparseImgAlign;
import Sophus.Se3;
import Sophus.So3;

public class NLLSSolver {
	
	// ToDo D and T should be set somewhere!
	protected int D = 6; // Dimension of the residual	// ToDo:: Note that D has been set to 6, but in the C++ is a variable.
	protected ModelType T; // Type of the model, e.g Se2, Se3

	protected int n_iter_; // Number of Iterations
	protected int n_iter_init_;
	protected Method method_;
	protected boolean verbose_;
	protected double eps_;				//!< Stop if update norm is smaller than eps

	protected boolean use_weights_;
	protected WeightFunctionPtr weight_function_;  // ToDo robust_cost::WeightFunctionPtr weight_function_;

	protected int n_meas_; //!< Number of measurements
	protected int iter_;

	protected float scale_;
	protected ScaleEstimator scale_estimator_;	//ToDo Changed from ScaleEstimatorPtr to ScaleEstimator


	protected Matrix H_; 					// Matrix<double, D, D> H_; //!< Hessian approximation
	protected Matrix Jres_; 				// Matrix<double, D, 1> Jres_; //!< Jacobian x Residual
	protected Vector6d x_ = new Vector6d(D);					//  Matrix<double, D, 1>  x_;       //!< update step
	protected boolean have_prior_;

	protected double chi2_;
	protected double rho_;


	private double mu_init_;
	protected double mu_;
	private double nu_init_;
	private double nu_;          			//!< Increase factor of mu after fail
	private int n_trials_;              	//!< Number of trials
	private int n_trials_max_;        		//!< Max number of trials
	private boolean stop_;                  //!< Stop flag

	public NLLSSolver(int n_iter, int n_iter_init, Method method, boolean verbose, double eps)
	{
		n_iter_ = n_iter;
		n_iter_init_ = n_iter_init;
		method_ = method;
		verbose_ = verbose;
		eps_ = eps;
	}
	public void reset()
	{
		have_prior_ = false;
		chi2_ = Math.pow(1, 10);
		mu_ = mu_init_;
		nu_ = nu_init_;
		n_meas_ = 0;
		n_iter_ = n_iter_init_;
		iter_ = 0;
		stop_ = false;
	}
	// ToDo I have changed ModelType to Se3, as it appears to be the only one used. Check this!!!
	public void optimize(Se3 model)
	{
		if(method_ == Method.GaussNewton)
			optimizeGaussNewton(model);
		if(method_ == Method.LevenbergMarquardt)
			optimizeLevenbergMarquardt(model);
	}
	
	// ToDo I have changed ModelType to Se3, as it appears to be the only one used. Check this!!!
	private void optimizeLevenbergMarquardt(Se3 model) {
		// Compute weight scale
		// ToDo Perhaps can leave this out and just use GaussNewton for now
	}
	
	// ToDo: Is ModelType a generic for Se3 etc?
	// If Se3 is the only type used, can I just set model to Se3
	private void optimizeGaussNewton(Se3 model) {

		// Compute weight scale
		if(use_weights_)	
			computeResiduals(model, false, true);	// Call the child class SparseImgAlign for the fully implemented computeResiduals method
		// Save the old model to rollback in case of unsuccessful update
		Se3 old_model = new Se3(model);

//		System.out.println("3. Model = ");
//		model.print();
		
		// Perform iterative estimation
		for (iter_ = 0; iter_<n_iter_; iter_++)
		{
			rho_ = 0;
			startIteration();	// ToDo should change to use SparseImgAlign
			
			H_ = new Matrix(D,D);	// All values set to zero.
			Jres_ = new Vector6d();// ToDo:: changed from Matrix(D,1);
			
			// compute initial error
			n_meas_ = 0;
			double new_chi2 = computeResiduals(model, true, false);
			
			// add prior
			if(have_prior_)
				applyPrior(model);
			
			// solve the linear system
			if(!solve())
			{
				// matrix was singular and could not be computed
//				System.out.println("Matrix is close to singular! Stop optimizing. ");
//				System.out.println("H =  ");
//				H_.print(0, 0);
//				
//				System.out.println("Jres =  ");
//				Jres_.print(0, 0);
				stop_ = true;
			}
			
			// check if the error increased since the last optimization
			if((iter_ > 0 && new_chi2 > chi2_) || stop_)
			{
				if(verbose_)
				{
					System.out.println("It. "+ iter_ +"\t failure. \t new_chi2 = "+ new_chi2 + "\t Error increased. Stop optimizing.");
				}
				model = old_model; // rollback
				break;
			}
			
			// ToDo this has been changed from ModelType to Se3. Ensure it does the same
			// Update the model
			Se3 new_model = new Se3(new So3(new Matrix3d()), new Vector3d(0));
			update(model, new_model);	// ToDo should call SparseImgAlign
			old_model = model;
			model = new_model;
			
			chi2_ = new_chi2;
			
			if(verbose_)
			{
				System.out.println("It. "+iter_+"t Success.\t new_chi2 = "+ new_chi2+"\t n_meas = "+ n_meas_+"\t x_norm = "+ math_utils.norm_max(x_));
			}
			
			finishIteration();
			
			// Stop when converged. i.e. the update step is too small.
			if(math_utils.norm_max(x_)<= eps_)
				break;
		}	

//		System.out.println("4. Model = ");
//		model.print();
		
	}
	// ToDo these methods require further work. Shouldn't call the child class but require the child class methodology
	private void finishIteration() {
		// TODO Auto-generated method stub
		
	}
	// ToDo these methods require further work. Shouldn't call the child class but require the child class methodology
	private boolean solve() {
		// TODO Auto-generated method stub
		return false;
	}
	// ToDo these methods require further work. Shouldn't call the child class but require the child class methodology
	private void startIteration() {
		// TODO Auto-generated method stub
		
	}
	// ToDo this method doesnt appear to have any implementation for SVO c++.
	//	virtual void applyPrior(const ModelType& current_model) { }
	private void applyPrior(Se3 model) {
		// TODO Auto-generated method stub
		
	}
	// ToDo Placeholder method. Implemented in child class SparseImgAlign
	public double computeResiduals(Se3 T_cur_from_ref, boolean linearize_system, boolean compute_weight_scale)
	{
		return 0;
	}
	
//	// ToDo Placeholder method. Implemented in child class SparseImgAlign
//	private void update(Se3 model, Se3 new_model)
//	{ 
//	}
	private void update(Se3 T_curold_from_ref, Se3 T_curnew_from_ref)
	{
		T_curnew_from_ref = T_curold_from_ref.times(Se3.exp(x_.times(-1))); //SE3::exp(-x_);	// ToDo Model Type: type of the model, e.g. SE2, SE3
	}
}

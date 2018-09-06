package SVO_1310;

public class DepthFilter_Options {
	private boolean check_ftr_angle;
	private boolean epi_search_1d;
	private boolean verbose;
	private boolean use_photometric_disparity_error;
	private int max_n_kfs;
	private double sigma_i_sq;
	private double seed_convergence_sigma2_thresh;
	
	public DepthFilter_Options()
	{
		check_ftr_angle = false;								//!< gradient features are only updated if the epipolar line is orthogonal to the gradient.
		epi_search_1d = false;									//!< restrict Gauss Newton in the epipolar search to the epipolar line.
		verbose = false;										//!< display output.
		use_photometric_disparity_error = false;				//!< use photometric disparity error instead of 1px error in tau computation.
		max_n_kfs = 3;											//!< maximum number of keyframes for which we maintain seeds.
		sigma_i_sq = (0.0005); 			//sigma_i_sq(5e-4),		//!< image noise.
		seed_convergence_sigma2_thresh = 200.0;					//!< threshold on depth uncertainty for convergence.    
	}
	public boolean get_verbose()
	{
		return verbose;
	}
	public void set_verbose(boolean bool)
	{
		verbose = bool;
	}
	public int get_max_n_kfs()
	{
		return max_n_kfs;
	}
	public double get_seed_convergence_sigma2_thresh()
	{
		return seed_convergence_sigma2_thresh;
	}
}

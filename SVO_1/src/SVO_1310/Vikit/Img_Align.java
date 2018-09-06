package SVO_1310.Vikit;

import matrix_types.Vector;
import Sophus.Se3;

public class Img_Align extends NLLSSolver{

	public Img_Align(int n_iter, int n_iter_init, Method method, boolean verbose, double eps)
	{
		super(n_iter, n_iter_init, method, verbose, eps);
	}
	public double computeResiduals(Se3 model, boolean linearize_system, boolean compute_weight_scale)
	{
		return 1.0;
	}


	public static void startIteration() {
		// TODO Auto-generated method stub
		
	}
	public static void finishIteration() {
		// TODO Auto-generated method stub
		
	}
	// ToDO this method is already in SparseImgAlign, so may not be required.
	public static boolean solve() {
		
		  x_ = new Vector(H_.chol().solve(Jres_.times(-1)).getArray());
		  if(Double.isNaN(x_.get(0)))
			  return false;
		  return true;
	}
}

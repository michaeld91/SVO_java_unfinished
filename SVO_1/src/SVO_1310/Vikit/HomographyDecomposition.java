package SVO_1310.Vikit;

import Jama.Matrix;
import Sophus.Se3;
import matrix_types.Matrix3d;
import matrix_types.Vector3d;

public class HomographyDecomposition implements Comparable{
	
	private Vector3d t;
	private Matrix3d R;
	private double   d;
	private Vector3d n;

	// Resolved  Composition
	private Sophus.Se3 T; //!< second from first
	private int score;

	public Vector3d get_vector_t() {
		return t;
	}
	public void set_vector_t(Vector3d vec) {
		t = vec;
	}
	public Matrix3d getR() {
		return R;
	}
	public void setR(Matrix mat)
	{
		R	=	new Matrix3d(mat.getArray());
	}
	public double getD() {
		return d;
	}
	public Vector3d getN() {
		return n;
	}
	public void set_n(Vector3d vec)
	{
		n = vec;
	}
	public Sophus.Se3 getT() {
		return T;
	}
	public void setT(Se3 T)
	{
		this.T = T;
	}
	public int getScore() {
		return score;
	}
	public void setScore(int score)
	{
		this.score = score;
	}
	public void setD(double e) {
		d = e;
	}
	// ToDo unsure how they should be compared. This could be wrong
	public int compareTo(Object arg0) {
		int result = 0;
		if(arg0 instanceof HomographyDecomposition)
		{
		result = score - ((HomographyDecomposition) arg0).getScore();
		//if(result>0)	// This is greater than the compared to HD
		}
		
		return result;
	}
}

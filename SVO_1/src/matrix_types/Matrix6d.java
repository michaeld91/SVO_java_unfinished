package matrix_types;

import Jama.Matrix;

public class Matrix6d extends Matrix {

	private static int num_cols = 6;
	private static int num_rows = 6;

	public Matrix6d(double[][] arg0) {
		super(arg0, num_rows, num_cols);
	}
	public Matrix6d() {
		super(num_rows, num_cols);
	}
	public Matrix6d(double[] arg1) {
		super(arg1, num_rows);
	}
	//Creates a 6x6 matrix where all values are s
	public Matrix6d(double s){
		super(num_rows, num_cols, s);
	}
	public void identity()
	{
		super.set(0, 0, 1);
		super.set(0, 1, 0);
		super.set(0, 2, 0);
		super.set(1, 0, 0);
		super.set(1, 1, 1);
		super.set(1, 2, 0);
		super.set(2, 0, 0);
		super.set(2, 1, 0);
		super.set(2, 2, 1);
		
		super.set(3, 0, 1);
		super.set(3, 1, 0);
		super.set(3, 2, 0);
		super.set(4, 0, 0);
		super.set(4, 1, 1);
		super.set(4, 2, 0);
		super.set(5, 0, 0);
		super.set(5, 1, 0);
		super.set(5, 2, 1);
	}

	public Matrix6d inverse()
	{
		Matrix6d inv = new Matrix6d(super.inverse().getArray());
		return inv;
	}
	public Matrix6d times(Matrix6d mat)
	{
		Matrix6d multiplied = new Matrix6d(super.times(mat).getArray());
		return multiplied;
	}
	public Matrix6d times(double dbl)
	{
		Matrix6d multiplied = new Matrix6d(super.times(dbl).getArray());
		return multiplied;
	}
	public Matrix6d plus(Matrix6d mat)
	{
		Matrix6d new_mat = new Matrix6d(super.plus(mat).getArray());
		return new_mat;
	}
	//	currently fails but the reverse order works
//	public Vector3d times(Vector3d vec_3d)
//	{
//		Vector3d multiplied = new Vector3d(super.times(vec_3d.transpose()).getArray());
//		return multiplied;
//	}

}

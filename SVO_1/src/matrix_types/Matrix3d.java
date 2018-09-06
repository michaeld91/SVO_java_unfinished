package matrix_types;

import Jama.Matrix;

public class Matrix3d extends Matrix {

	private static int num_cols = 3;
	private static int num_rows = 3;

	public Matrix3d(double[][] arg0) {
		super(arg0, num_rows, num_cols);
	}
	public Matrix3d() {
		super(num_rows, num_cols);
	}
	public Matrix3d(double[] arg1) {
		super(arg1, num_rows);
	}
	//Creates a 3x3 matrix where all values are s
	public Matrix3d(double s){
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
	}
	public Matrix3d transpose()
	{
		return new Matrix3d(super.transpose().getArray());
	}
	public Matrix3d inverse()
	{
		Matrix3d inv = new Matrix3d(super.inverse().getArray());
		return inv;
	}
	public Matrix3d times(Matrix3d mat)
	{
		Matrix3d multiplied = new Matrix3d(super.times(mat).getArray());
		return multiplied;
	}
	public Matrix3d times(double dbl)
	{
		Matrix3d multiplied = new Matrix3d(super.times(dbl).getArray());
		return multiplied;
	}
	public Matrix3d plus(Matrix3d mat)
	{
		Matrix3d plus = new Matrix3d(super.plus(mat).getArray());
		return plus;
	}
	public Matrix3d minus(Matrix3d mat)
	{
		Matrix3d minus = new Matrix3d(super.minus(mat).getArray());
		return minus;
	}
	//	currently fails but the reverse order works
	public Vector3d times(Vector3d vec_3d)
	{		
		double[][] mat33_vals = this.getArray();
		Matrix mat33 = new Matrix(mat33_vals,3,3);
		
		double[][] mat31_vals = vec_3d.getArray();
		double[] first_line = {1,0,0};
//		if(mat33_vals[0]==first_line)
//			System.out.println(true);
//		else
//			System.out.println(mat33_vals[0][0]+","+mat33_vals[0][1]+","+mat33_vals[0][2]);
//		double[][] mat13_vals = {{mat31_vals[0][0], mat31_vals[0][1], mat31_vals[0][2]}};
		
		Matrix mat31 = new Matrix(3,1);
		mat31.set(0, 0, vec_3d.get(0,0));
		mat31.set(1, 0, vec_3d.get(0,1));
		mat31.set(2, 0, vec_3d.get(0,2));

		Matrix result_array = mat33.times(mat31);
		
		double[] vec_vals = {result_array.get(0, 0), result_array.get(1, 0), result_array.get(2, 0)};
		Vector3d result = new Vector3d();
		result.set(vec_vals);

		
		
//		double[][] vec_vals = result_array.getArray();
//		Vector3d result = new Vector3d(vec_vals);
//		System.out.println(result.get(0,0));
//		System.out.println(result.get(1,0));
//		System.out.println(result.get(2,0));
		return result;
//		Vector3d multiplied = new Vector3d(super.times(vec_3d.transpose()).getArray());
//		System.out.println("Is this method used?");
//		return multiplied;
	}
	
	public static Vector3d times(Matrix3d mat33, Vector3d vec31)
	{
		Matrix mat = new Matrix(mat33.getArray());		
		double[][] values = {vec31.getColumnPackedCopy()};
		Matrix vec = new Matrix(values);
		Vector3d result = new Vector3d(vec.getArray());

		return result;
	}

}

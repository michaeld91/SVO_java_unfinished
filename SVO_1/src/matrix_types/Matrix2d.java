package matrix_types;

import Jama.Matrix;

public class Matrix2d extends Matrix {
	private static int num_cols = 2;
	private static int num_rows = 2;
	
	public Matrix2d(double[][] arg0) {
		super(arg0, num_rows, num_cols);
	}
	public Matrix2d() {
		super(num_rows, num_cols);
	}
	public Matrix2d(double[] arg1) {
		super(arg1, num_rows);
	}
	//Creates a 3x3 matrix where all values are s
	public Matrix2d(double s){
		super(num_rows, num_cols, s);
	}
	public Vector2d times(Vector2d vec)
	{
		Vector2d result = new Vector2d();
		double first = this.get(0, 0)*vec.get(0) + this.get(0, 1)*vec.get(1);
		double second = this.get(1, 0)*vec.get(0) + this.get(1, 1)*vec.get(1);
		result.set(first, second);
		return result;
	}
	public Matrix2d times(double d)
	{
		Matrix2d result;
		
		double[][] values = {{this.get(0, 0)*d, this.get(0,1)*d},{this.get(1, 0)*d, this.get(1,1)*d}};
		
		result = new Matrix2d(values);
		return result;
	}
	public Matrix2d inverse()
	{
		Matrix2d mat;
		double a = this.get(0,0);
		double b = this.get(0,1);
		double c = this.get(1,0);
		double d = this.get(1,1);
		
		double one_over_det = 1/((a*d)-(b*c));
		// inverse should = 1/det(d,-b,-c,a) where d,-c is the top row and -c,a is the bottom row
		
		double[][] values = {{d,-b},{-c,a}};

		mat = new Matrix2d(values);
		
		mat = mat.times(one_over_det);
		
		return mat;
	}
	
}

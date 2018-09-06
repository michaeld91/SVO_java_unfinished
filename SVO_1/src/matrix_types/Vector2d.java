package matrix_types;

import Jama.Matrix;

public class Vector2d extends Vector {

	private static int num_rows = 1;
	private static int num_cols = 2;
	
	//Construct a Vector from a 2-D array
	public Vector2d(double[][] arg1){
		super(arg1, num_cols);
	}
	//Issue here were one could pass a longer array than 2 and the result would be a Vector2d of more than 2 values.
	//currently just have to use array of arrays
//	public Vector2d(double[] arg1){
//		super(arg1);
//	}
	public Vector2d(){
		super(num_cols);
	}
	public Vector2d(double arg1){
		super(num_cols, arg1);
	}

	public void set(double first, double second)
	{
		super.set(0, 0, first);
		super.set(0, 1, second);
	}
	public void set(Matrix mat)
	{
		super.set(0,0,mat.get(0,0));
		super.set(0,1,mat.get(0,1));
	}
	public void set(Vector2d vec)
	{
		this.set(vec.get(0), vec.get(1));
	}
	// ToDo This needs to contain checks that you want vector v*v.transpose()
	public double times(Vector2d vec)
	{
		Matrix mat = this.times(vec.transpose());
		
		double solution = mat.get(0,0);;
		return solution;
	}
	public Matrix2d times(Matrix vec_transp)
	{
		if(vec_transp.getRowDimension()==2 && vec_transp.getColumnDimension()== 1)
		{
			double[] values = {this.get(0)*vec_transp.get(0, 0), this.get(1)*vec_transp.get(1,0), this.get(0)*vec_transp.get(1,0), this.get(1)*vec_transp.get(0, 0)};
			Matrix2d solution = new Matrix2d(values);
			return solution;
		}
		else
			throw new IllegalArgumentException("The Matrix is not of size 2 rows x 1 column");
	}
	public Vector2d times(double dbl)
	{
//		Matrix mat = this.timesEquals(dbl);
//		
//		Vector2d vec2d = new Vector2d(mat.getArray());
//		return vec2d;
//		
		Vector2d result = new Vector2d();
		result.set(this.get(0)*dbl,this.get(1)*dbl);
		return result;
	}
	public Vector2d minus(Vector2d vec)
	{
		Vector2d new_vec = new Vector2d();
		new_vec.set(0,0,super.get(0,0)-vec.get(0,0));
		new_vec.set(0,1,super.get(0,1)-vec.get(0,1));
		return new_vec;
	}
	public Vector2d plus(Vector2d vec)
	{
		Vector2d new_vec = new Vector2d();
		new_vec.set(0,0,super.get(0,0)+vec.get(0,0));
		new_vec.set(0,1,super.get(0,1)+vec.get(0,1));
		return new_vec;
	}
	public Vector2d plusEquals(Vector2d vec)
	{
		Vector2d result = new Vector2d();
		result.set(this.get(0)+vec.get(0), this.get(1)+vec.get(1));
		return result;
	}
}

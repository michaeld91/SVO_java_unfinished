package matrix_types;

import Jama.Matrix;

public class Vector3d extends Vector {

	private static int num_cols = 3;
	
	//Construct a Vector from a 2-D array
	public Vector3d(double[][] arg1){
		super(arg1, num_cols);
	}
	//Issue here were one could pass a longer array than 2 and the result would be a Vector2d of more than 2 values.
	//currently just have to use array of arrays
//	public Vector3d(double[] arg1){
//		super(arg1);
//	}
	public Vector3d(){
		super(num_cols);
	}
	public Vector3d(double arg1){
		super(num_cols, arg1);
	}
	
//	public Vector3d(Matrix mat)
//	{
//		
//	}
	// This method seems to not actually work
	public void set(double first, double second, double third)
	{
		super.set(0, 0, first);
		super.set(0, 1, second);
		super.set(0, 2, third);
	}
	public void set(double[] values)
	{
		set(values[0], values[1], values[2]);
	}
	
	public void set(Matrix mat)
	{
		super.set(0,0,mat.get(0,0));
		super.set(0,1,mat.get(0,1));
		super.set(0,2,mat.get(0,2));

	}
	public Vector3d times(double d)
	{
		Vector3d new_vec = new Vector3d();
		double[] values = {super.get(0)*d,super.get(1)*d,super.get(2)*d};
		new_vec.set(values);
//		new_vec.set(0,0,super.get(0,0)*d);
//		new_vec.set(0,1,super.get(0,1)*d);
//		new_vec.set(0,2,super.get(0,2)*d);
		return new_vec;
	}
	public Vector3d times(Matrix3d mat3d)
	{
		Vector3d multiplied = new Vector3d(super.times(mat3d).getArray());
		return multiplied;
	}
	// multiplied this vector by another. As in v * u^T. No need to transpose the second vector first.
	public double times(Vector3d vec3d)
	{
		double multi = this.get(0)*vec3d.get(0) + this.get(1)*vec3d.get(1) + this.get(2)*vec3d.get(2);
		return multi;
	}
	// Vector Direct Product
	public Matrix3d times_vector_direct(Vector3d vec)
	{
		double[] values = {this.get(0)*vec.get(0),this.get(1)*vec.get(0),this.get(2)*vec.get(0),
						   this.get(0)*vec.get(1),this.get(1)*vec.get(1),this.get(2)*vec.get(1),
						   this.get(0)*vec.get(2),this.get(1)*vec.get(2),this.get(2)*vec.get(2)};
		Matrix3d mat = new Matrix3d(values);
		return mat;

	}
	public Vector3d minus(Vector3d vec)
	{
		Vector3d new_vec = new Vector3d();
		new_vec.set(0,0,super.get(0,0)-vec.get(0,0));
		new_vec.set(0,1,super.get(0,1)-vec.get(0,1));
		new_vec.set(0,2,super.get(0,2)-vec.get(0,2));
		return new_vec;
	}
	public Vector3d plus(Vector3d vec)
	{
		Vector3d new_vec = new Vector3d();
//		System.out.println("this row,col size = "+ this.getRowDimension()+","+this.getColumnDimension());
//		System.out.println("vec row_col size = "+ vec.getRowDimension()+","+vec.getColumnDimension());
//		System.out.print("this = " );this.print(0, 3);
//		System.out.print("vec = " );vec.print(0, 3);

		new_vec.set(super.get(0)+vec.get(0),super.get(1)+vec.get(1),super.get(2)+vec.get(2));
//		new_vec.set(0,0,super.get(0,0)+vec.get(0,0));
//		new_vec.set(0,1,super.get(0,1)+vec.get(0,1));
//		new_vec.set(0,2,super.get(0,2)+vec.get(0,2));
		return new_vec;
	}
}

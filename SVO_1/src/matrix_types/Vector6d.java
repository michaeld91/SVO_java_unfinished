package matrix_types;

import Jama.Matrix;

public class Vector6d extends Vector {

	private static int num_cols = 6;
	
	//Construct a Vector from a 2-D array
	public Vector6d(double[][] arg1){
		super(arg1, num_cols);
	}
	//Issue here were one could pass a longer array than 2 and the result would be a Vector2d of more than 2 values.
	//currently just have to use array of arrays
//	public Vector3d(double[] arg1){
//		super(arg1);
//	}
	public Vector6d(){
		super(num_cols);
	}
	public Vector6d(double arg1){
		super(num_cols, arg1);
	}
	public Vector6d(double[] arg1)
	{
		super(arg1);
	}
	public void set(double first, double second, double third, double forth, double fifth, double sixth)
	{
		super.set(0, 0, first);
		super.set(0, 1, second);
		super.set(0, 2, third);
		super.set(0, 3, forth);
		super.set(0, 4, fifth);
		super.set(0, 5, sixth);
	}
	
	public void set(Matrix mat)
	{
		super.set(0,0,mat.get(0,0));
		super.set(0,1,mat.get(0,1));
		super.set(0,2,mat.get(0,2));
		super.set(0,3,mat.get(0,3));
		super.set(0,4,mat.get(0,4));
		super.set(0,5,mat.get(0,5));

	}
	public Vector6d times(double d)
	{
		Vector6d new_vec = new Vector6d();
//		System.out.println("col,row= "+this.getColumnDimension()+","+this.getRowDimension());
		new_vec.set(0,0,super.get(0,0)*d);
		new_vec.set(0,1,super.get(0,1)*d);
		new_vec.set(0,2,super.get(0,2)*d);
		new_vec.set(0,3,super.get(0,3)*d);
		new_vec.set(0,4,super.get(0,4)*d);
		new_vec.set(0,5,super.get(0,5)*d);

		return new_vec;
	}
	public Vector6d minus(Vector6d vec)
	{
		Vector6d new_vec = new Vector6d();
		new_vec.set(0,0,super.get(0,0)-vec.get(0,0));
		new_vec.set(0,1,super.get(0,1)-vec.get(0,1));
		new_vec.set(0,2,super.get(0,2)-vec.get(0,2));
		new_vec.set(0,3,super.get(0,3)-vec.get(0,3));
		new_vec.set(0,4,super.get(0,4)-vec.get(0,4));
		new_vec.set(0,5,super.get(0,5)-vec.get(0,5));
		
		return new_vec;
	}
	public Matrix6d times(Vector6d vec)
	{	
		double[][] mat_vals = {
				{this.get(0)*vec.get(0), this.get(0)*vec.get(1), this.get(0)*vec.get(2), this.get(0)*vec.get(3), this.get(0)*vec.get(4), this.get(0)*vec.get(5)},
				{this.get(1)*vec.get(0), this.get(1)*vec.get(1), this.get(1)*vec.get(2), this.get(1)*vec.get(3), this.get(1)*vec.get(4), this.get(1)*vec.get(5)},
				{this.get(2)*vec.get(0), this.get(2)*vec.get(1), this.get(2)*vec.get(2), this.get(2)*vec.get(3), this.get(2)*vec.get(4), this.get(2)*vec.get(5)},
				{this.get(3)*vec.get(0), this.get(3)*vec.get(1), this.get(3)*vec.get(2), this.get(3)*vec.get(3), this.get(3)*vec.get(4), this.get(3)*vec.get(5)},
				{this.get(4)*vec.get(0), this.get(4)*vec.get(1), this.get(4)*vec.get(2), this.get(4)*vec.get(3), this.get(4)*vec.get(4), this.get(4)*vec.get(5)},
				{this.get(5)*vec.get(0), this.get(5)*vec.get(1), this.get(5)*vec.get(2), this.get(5)*vec.get(3), this.get(5)*vec.get(4), this.get(5)*vec.get(5)}
		};
		Matrix6d mat = new Matrix6d(mat_vals);
		
		return mat;
	}
}

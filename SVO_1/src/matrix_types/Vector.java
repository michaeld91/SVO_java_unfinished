package matrix_types;

import Jama.Matrix;

/*
 * A Matrix of one row. 
 */
public class Vector extends Matrix{
	/**
	 * 
	 */
	private static final long serialVersionUID = 3647763869193018254L;
	private static int num_rows = 1;

	//Construct a Vector from a 2-D array
	public Vector(double[][] arg1){
		super(arg1);
	}
	//Construct a vector from an array, specifying the first arg1 elements
	public Vector(double[][] arg0, int arg1) {
		super(arg0, num_rows, arg1);
	}
	//Construct a vector from an array
	public Vector(double[] arg0) {
		super(arg0, num_rows);
	}
	//Constructs a vector of 0's with arg1 columns
	public Vector(int arg1) {
		super(num_rows, arg1);
	}
	//Construct a vector of size arg1 and constant value arg2
	public Vector(int arg1, double arg2) {
		super(num_rows, arg1, arg2);
	}


	public double get(int i)
	{
		//		int cols = this.getColumnDimension();
		//		int rows = this.getRowDimension();
		////		
		//		if(cols>rows)	// horizontal vector
		//			return this.get(i, 0);
		// otherwise vertical vector
		double value = 0;

		if(this.getRowDimension()==1)
		{
			try
			{
				value = this.get(0,i);
			}
			catch(IndexOutOfBoundsException E)
			{
				//System.out.println("This Vector does not have a value at position "+i);
			}
			return value;
		}
		if(this.getColumnDimension()==1)
		{
			try
			{
				value = this.get(i,0);
			}
			catch(IndexOutOfBoundsException E)
			{
				//System.out.println("This Vector does not have a value at position "+i);
			}
			return value;
		}
		return value;
	}




	public void set(Vector vec)
	{
		int max = 0;
		if(vec.getColumnDimension()>vec.getRowDimension()){max=vec.getColumnDimension();}
		if(vec.getRowDimension()>vec.getColumnDimension()){max=vec.getRowDimension();}
		for(int i=0;i<max;i++)
		{
			this.set(0, i, vec.get(i));
		}
	}
	public Vector times(double d)
	{
		Vector multiplied = new Vector(super.times(d).getArray());
		return multiplied;
	}

	@Override
	public String toString()
	{
		String string = "";
		for(int i = 0; i<super.getColumnDimension()-1; i++)
		{
			string = string + super.get(0, i)+", ";
		}
		string = string + super.get(0, super.getColumnDimension()-1);
		return string;
	}
	public void normalize()
	{
		double norm = 0;
		double sqrt_val = 0;
		for(int i = 0; i<this.getColumnDimension(); i++)
		{
			sqrt_val += this.get(i)*this.get(i);
		}
		this.set(this.times(1/Math.sqrt(sqrt_val)));
	}
	public double squaredNorm()
	{
		double squared_total = 0;
		for(int i = 0; i<super.getColumnDimension()-1; i++)
		{
			squared_total+=(super.get(0, i)*super.get(0, i));
		}
		return squared_total;
	}

}

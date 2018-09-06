package matrix_types;

import Jama.Matrix;

public class Matrix23d extends Matrix {

	private static int num_cols = 3;
	private static int num_rows = 2;
	
	public Matrix23d(double[][] arg0) {
		super(arg0, num_rows, num_cols);
	}
	public Matrix23d() {
		super(num_rows, num_cols);
	}
	public Matrix23d(double[] arg1) {
		super(arg1, num_rows);
	}
	//Creates a 3x3 matrix where all values are s
	public Matrix23d(double s){
		super(num_rows, num_cols, s);
	}
	public void set(Matrix mat)
	{
		super.set(0,0,mat.get(0,0));
		super.set(0,1,mat.get(0,1));
		super.set(0,2,mat.get(0,2));
		super.set(1,0,mat.get(1,0));
		super.set(1,1,mat.get(1,1));
		super.set(1,2,mat.get(1,2));
	}
	public boolean equals(Matrix23d mat)
	{
		if(this.getColumnDimension()==mat.getColumnDimension()&&this.getRowDimension()==mat.getRowDimension()){
			if(		mat.get(0, 0)==this.get(0, 0) &&
					mat.get(0, 1)==this.get(0, 1) &&
					mat.get(0, 2)==this.get(0, 2) &&
					mat.get(1, 0)==this.get(1, 0) &&
					mat.get(1, 1)==this.get(1, 1) &&
					mat.get(1, 2)==this.get(1, 2) )
			{
				return true;
			}
		}
		return false;
	}
}

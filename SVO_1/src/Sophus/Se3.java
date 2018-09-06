package Sophus;

import matrix_types.Matrix3d;
import matrix_types.Vector3d;
import matrix_types.Vector6d;
import Jama.Matrix;

//If this extended GMatrix/GVector i could then use multiply in Frame::isVisible()

public class Se3 implements Cloneable{//extends Matrix3d	{
	private Vector3d translation_ = new Vector3d(); //translation_ should be a constructor field and be passed as a variable to Se3.
	private So3 so3;
	public static double SMALL_EPS = Math.pow(1,-10);
	//Creates a 3x3 GMatrix, with values double[]
	//If extending GMatrix is unsuitable, remove this
//	public Se3(double[] arg0) {
//		super(arg0, 3); 
//		// TODO Auto-generated constructor stub
//	}
	public Se3(So3 so3, Vector3d translation)
	{
		this.so3 = so3;
		this.translation_ = translation;
	}
//	public Se3(){}
	public Se3(Matrix3d rotation_matrix, Vector3d translation)
	{	
		so3 = new So3(rotation_matrix);
		translation_ = translation;
		//so3_(rotation_matrix), translation_(translation){}
	}
	public Se3(Quaternion quaternion, Vector3d translation)
	{
		so3 = new So3(quaternion);
		translation_ = translation;
	}
	public Se3(Se3 other)
	{
		so3 = other.getSo3();//new So3(other.getSo3());
		translation_ = other.translation_;

	}
//	public Se3(double[] values_array)
//	{
//		super(values_array);
//		
//	}
//	public Se3(double[][] values_array)
//	{
//		super(values_array);
//	}
	public Matrix3d rotation_Matrix()
	{
		return so3.matrix();
	}

	//Has been checked for accurate results.
	public Se3 inverse()
	{	
		So3 so3_inv = so3.inverse();
		Vector3d trans_inv = so3_inv.times(translation_.times(-1));
		Se3 ret = new Se3(so3_inv, trans_inv);
//		ret.setSo3(so3.inverse());
//		ret.set_Translation(ret.getSo3().times(translation_.times(-1)));
		return ret;
	}
	//ToDo
	public Vector3d get_Translation(){
		return translation_;
	}
	public void set_Translation(Vector3d vec3d)
	{
		translation_ = vec3d;
	}
	
	
	public Se3 times(Se3 other)
	{
//		Se3 result = this;	
//		if(so3.getQuaternion()==null)
//			System.out.println("so3.getQuaternion() = null" );
//		//so3.print();
//		Vector3d so3_times_other_translation = so3.times(other.get_Translation());
//		result.set_Translation(translation_.plus(so3_times_other_translation));
//		result.setSo3(result.getSo3().times_equals(other.getSo3()));
//		return result;
		Se3 result = new Se3(this);
		result.set_Translation(result.get_Translation().plus(so3.times(other.get_Translation())));
		result.setSo3(result.getSo3().times(other.getSo3()));
		
		return result;
	}
	public Vector3d times(Vector3d vec)
	{
//		System.out.println("translation = ");
//		translation_.print(0, 3);
		return so3.times(vec).plus(translation_);
		
	}

	
	
	
	public static Se3 exp(Vector6d update)
	{
		//update.print(0,6);
		Vector3d upsilon = new Vector3d();
		upsilon.set(update.get(0), update.get(1), update.get(2));
		Vector3d omega = new Vector3d();
		omega.set(update.get(3), update.get(4), update.get(5));
		
		double theta = 0;
		So3 so3 = So3.expAndTheta(omega, theta);
		
		Matrix3d Omega = So3.hat(omega);
		Matrix3d Omega_sq = Omega.times(Omega);
		Matrix3d V = new Matrix3d();
		
		if(theta<SMALL_EPS)
		{
			V = so3.matrix();
			// Note: That is an accurate expansion!
		}
		else
		{
			double theta_sq = theta*theta;
			Matrix3d mat_ident = new Matrix3d();
			mat_ident.identity();
			V = mat_ident.plus(Omega.times((1-Math.cos(theta))/(theta_sq))).plus(
					Omega_sq.times((theta-Math.sin(theta))/(theta_sq*theta)));
		}
		Se3 se3_ret = new Se3(so3,V.times(upsilon));
		return se3_ret;
	}


	public Object clone() throws CloneNotSupportedException
	{
		return super.clone();
	}
	
	
	public So3 getSo3()
	{
		return so3;
	}
	protected void setSo3(So3 so3)
	{
		this.so3=so3;
	}
	public void print()
	{
		System.out.println("So3.log() = "+so3.toString());

		System.out.println("Translation = "+translation_.get(0)+", "+translation_.get(1)+", "+translation_.get(2));
		
	}
}

package Sophus;

import matrix_types.Matrix3d;
import matrix_types.Vector3d;


//import javax.vecmath.GMatrix;
 
public class So3 {
	private Quaternion unit_Quaternion;
	private final static double SMALL_EPS = Math.pow(1, -10);
//	public So3(){}
	public So3(So3 so3)
	{
		unit_Quaternion = so3.getQuaternion();
	}
	public So3(Matrix3d R)
	{
		unit_Quaternion = new Quaternion(R);
	}
	public So3(Quaternion quat)
	{
		unit_Quaternion = quat;
		if(unit_Quaternion.squaredNorm() > SMALL_EPS)
			unit_Quaternion.normalize();
	}
	public Matrix3d matrix()
	{
		return unit_Quaternion.toRotationMatrix();
	}
	public So3 inverse()
	{
		if(unit_Quaternion == null)
			System.out.println("The Unit Quaternion has not been established");
//		System.out.println("Unit quaternion = ");
//		unit_Quaternion.print();
		So3 inverse = new So3(unit_Quaternion.conjugate());
		return inverse;
	}
	public Vector3d times(Vector3d other)
	{
//		System.out.println("other = ");
//		other.print(0, 3);
//		System.out.println("this quaternion = ");
//		this.unit_Quaternion.print();
		Vector3d result = this.unit_Quaternion._transformVector(other);
//		System.out.print("So3 * Vector3d = ");result.print(0, 6);
		return result;
	}
	public void times_equals(So3 other)
	{
		unit_Quaternion = Quaternion.times_equals(unit_Quaternion, other.getQuaternion());
		unit_Quaternion.normalize();
	}
	

	public Quaternion getQuaternion()
	{
		return unit_Quaternion;
	}
	public void setQuaternion(Quaternion quat)
	{
		unit_Quaternion = quat;
	}
	public static So3 expAndTheta(Vector3d omega, double theta)
	{
		theta = omega.normF();
		double half_theta = 0.5*theta;
		
		double imag_factor;
		double real_factor = Math.cos(half_theta);
		if((theta)<SMALL_EPS)
		{
			double theta_sq = theta*theta;
			double theta_po4 = theta_sq*theta_sq;
			imag_factor = 0.5-0.0208333*theta_sq+0.000260417*theta_po4;
		}
		else
		{
			double sin_half_theta = Math.sin(half_theta);
		    imag_factor = sin_half_theta/(theta);
		}
		double[] vals = {real_factor, imag_factor*omega.get(0),imag_factor*omega.get(1),imag_factor*omega.get(2)};
		Quaternion quat = new Quaternion(vals);
		So3 new_so3 = new So3(quat);
		return new_so3;
	}
	public static Matrix3d hat(Vector3d v)
	{
		double[] vals = {0,-v.get(2),v.get(1),v.get(2),0,-v.get(0),-v.get(1),v.get(0),0};
		Matrix3d Omega = new Matrix3d(vals);
		return Omega;
	}
	public String toString()
	{
//		System.out.println("Unit_Quaternion = "+unit_Quaternion.get_quaternion().get(0)+", "+unit_Quaternion.get_quaternion().get(1)+", "+
//												unit_Quaternion.get_quaternion().get(2)+", "+unit_Quaternion.get_quaternion().get(3));
		Vector3d so3_log = log();
		String s = so3_log.get(0)+","+so3_log.get(1)+","+so3_log.get(2);
		return s;
	}

	public Vector3d log() 
	{
		
		So3 other = this;
		double n = other.getQuaternion().getVec().normF();
		double w = other.getQuaternion().get_w();
		double squared_w = w*w;
		
		double two_atan_nbyw_by_n;
		// Atan-based log thanks to
	    //
	    // C. Hertzberg et al.:
	    // "Integrating Generic Sensor Fusion Algorithms with Sound State
	    // Representation through Encapsulation of Manifolds"
	    // Information Fusion, 2011

	    if (n < SMALL_EPS)
	    {
	      // If quaternion is normalized and n=1, then w should be 1;
	      // w=0 should never happen here!
	      assert(Math.abs(w)>SMALL_EPS);

	      two_atan_nbyw_by_n = 2./w - 2.*(n*n)/(w*squared_w);
	    }
	    else
	    {
	      if (Math.abs(w)<SMALL_EPS)
	      {
	        if (w>0)
	        {
	          two_atan_nbyw_by_n = Math.PI/n;
	        }
	        else
	        {
	          two_atan_nbyw_by_n = -Math.PI/n;
	        }
	      }
	      two_atan_nbyw_by_n = 2*Math.atan(n/w)/n;
	    }

	    return other.getQuaternion().getVec().times(two_atan_nbyw_by_n);
		
	}
	public So3 times(So3 other) {
		So3 result = new So3(this);
		result.setQuaternion(Quaternion.times_equals(result.getQuaternion(), other.getQuaternion()));
		result.getQuaternion().normalize();
		
		return result;
	}

}

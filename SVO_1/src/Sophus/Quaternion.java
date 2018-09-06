package Sophus;
//import javax.vecmath.GMatrix;
//import javax.vecmath.GVector;

import matrix_types.JamaUtils;
import matrix_types.Vector;
import matrix_types.Matrix3d;
import matrix_types.Vector3d;
import Jama.Matrix;
import java.lang.Math;

import Jama.util.*;


/*
 * Presuming we have a rotation Matrix 3x3 with positions as follows
 * 1 2 3
 * 4 5 6
 * 7 8 9
 * 
 */
public class Quaternion {
	//Why does this already have values?
	private double[] quat_vals = new double[4];
	private Vector quaternion = new Vector(quat_vals);
	private double a; //w value
	private double b; // x
	private double c; // y
	private double d; // z
	
	Math math;
	
	static Matrix3d Rotation = new Matrix3d();
	
	private JamaUtils JU = new JamaUtils();
	
	/*
	public static void main(String[] args) {

		quaternion.normalize();
		
		a = quaternion.getElement(0);
		b = quaternion.getElement(1);
		c = quaternion.getElement(2);
		d = quaternion.getElement(3);
		
		setRotation();

	}
	*/
	
	public Quaternion(Matrix quaternion)
	{
		//this.quaternion = quaternion;
		//quaternion.normalize();
		this.quaternion = new Vector(JamaUtils.normalize(quaternion).getArray());
		
		this.a = quaternion.get(0,0);
		this.b = quaternion.get(0,1);
		this.c = quaternion.get(0,2);
		this.d = quaternion.get(0,3);
	
		quat_vals = new double[]{a,b,c,d};
		setRotation();
	}
	public Quaternion(Matrix3d mat)
	{
		Quaternion quat = RotationMatToQuat(mat);

		Matrix new_mat = new Matrix(quat.get_quat_values(),1);
		this.quaternion = new Vector(JamaUtils.normalize(quaternion).getArray());
		
		this.a = quaternion.get(0,0);
		this.b = quaternion.get(0,1);
		this.c = quaternion.get(0,2);
		this.d = quaternion.get(0,3);
	
		quat_vals = new double[]{a,b,c,d};
		setRotation();
	}
	private void Quaternion(double[] quat) {
		if(quat.length>4){
			double[] new_quat = {quat[0], quat[1],quat[2], quat[3]};
		}
		else if(quat.length<4)
		{
		//need to throw illegal argument here.
		}
		else {
		Vector quat_vec = new Vector(quat);
		quat_vec.normalize();
		this.quaternion = quat_vec;//(Vector) JamaUtils.normalize(quat_vec);//quat_vec;
		//quaternion.normalize();
		
		a = quaternion.get(0,0);
		b = quaternion.get(0,1);
		c = quaternion.get(0,2);
		d = quaternion.get(0,3);
		
		setRotation();
		}		
	}
	public Quaternion(Quaternion quat)
	{
		Matrix new_mat = new Matrix(quat.get_quat_values(),1);
		this.quaternion = (Vector) JU.normalize(quaternion);
		
		this.a = quaternion.get(0,0);
		this.b = quaternion.get(0,1);
		this.c = quaternion.get(0,2);
		this.d = quaternion.get(0,3);
	
		quat_vals = new double[]{a,b,c,d};
		setRotation();
	}
	public double squaredNorm()
	{
		double norm = quaternion.normF();
		double norm_squared = norm*norm;
		return norm_squared;		
	}
	
	public void normalize()
	{
		this.quaternion = quaternion.times(1/Math.sqrt((a*a)+(b*b)+(c*c)+(d*d)));
	}

	public Quaternion(double[] quat) 
	{
		if(quat.length>4){
			double[] new_quat = {quat[0], quat[1],quat[2], quat[3]};
		}
		else if(quat.length<4)
		{
		//need to throw illegal argument here.
		}
		else {
		Vector quat_vec = new Vector(quat);
		quat_vec.normalize();
		this.quaternion = quat_vec;//(Vector) JamaUtils.normalize(quat_vec);//quat_vec;
		//quaternion.normalize();
		
		a = quaternion.get(0,0);
		b = quaternion.get(0,1);
		c = quaternion.get(0,2);
		d = quaternion.get(0,3);
		
		setRotation();
		}
		
	}
	public void setRotation(){
		double[][] matrix_vals = {{set_pos1(), set_pos2(), set_pos3()},
								{set_pos4(), set_pos5(), set_pos6()},	
								{set_pos7(), set_pos8(), set_pos9()}};
		//Rotation.set(matrix_vals);
		Rotation = new Matrix3d(matrix_vals);
		

	}
	public Matrix3d toRotationMatrix(){
		setRotation();
		return Rotation;
	}
	private double set_pos1(){
		return Math.pow(a, 2)+Math.pow(b, 2)-Math.pow(c, 2)-Math.pow(d, 2);		
	}
	private double set_pos2(){
		return  2*b*c -2*a*d;
	}
	private double set_pos3(){
		return  2*b*d +2*a*c;
	}
	private double set_pos4(){
		return  2*b*c +2*a*d;
	}
	private double set_pos5(){
		return Math.pow(a, 2)-Math.pow(b, 2)+Math.pow(c, 2)-Math.pow(d, 2);		
	}
	private double set_pos6(){
		return  2*c*d -2*a*b;
	}
	private double set_pos7(){
		return  2*b*d -2*a*c;
	}
	private double set_pos8(){
		return  2*c*d +2*a*b;
	}
	private double set_pos9(){
		return Math.pow(a, 2)-Math.pow(b, 2)-Math.pow(c, 2)+Math.pow(d, 2);		
	}
	public Quaternion conjugate()
	{
		double[] conj_val = {a,-b,-c,-d};
		Quaternion conj = new Quaternion(conj_val);
		return conj;
	}
	public Vector3d _transformVector(Vector3d v)
	{
//		Vector3d transform = new Vector3d();
//		v.transpose().print(0, 0);
//		toRotationMatrix().print(0, 0);
//
//		System.out.println("v cols, rows = "+v.getColumnDimension()+","+v.getRowDimension());
//		Vector3d result = toRotationMatrix().times(v);
//		//Vector3d vec = new Vector3d(toRotationMatrix().times(v).getArray());
		Matrix3d rotation = toRotationMatrix();
//		rotation.print(0, 0);
		Vector3d result = rotation.times(v);
		
//		Vector3d result = Matrix3d.times(rotation, v);
		return result;//vec;
	}
	public void print()
	{
		System.out.println(quaternion.get(0)+", "+quaternion.get(1)+", "+quaternion.get(2)+", "+quaternion.get(3));
	}
	public String toString()
	{
		return quaternion.get(0)+", "+quaternion.get(1)+", "+quaternion.get(2)+", "+quaternion.get(3);
	}
	public Vector get_quaternion()
	{
		return quaternion;
	}
	
	public static Quaternion times_equals(Quaternion q, Quaternion r)
	{
		Quaternion result;
		double q0 = q.get_quaternion().get(0);
		double q1 = q.get_quaternion().get(1);
		double q2 = q.get_quaternion().get(2);
		double q3 = q.get_quaternion().get(3);
		
		double r0 = r.get_quaternion().get(0);
		double r1 = r.get_quaternion().get(1);
		double r2 = r.get_quaternion().get(2);
		double r3 = r.get_quaternion().get(3);
		
		double n0 = r0*q0 - r1*q1 - r2*q2 - r3*q3;
		double n1 = r0*q1 + r1*q0 - r2*q3 + r3*q2;
		double n2 = r0*q2 + r1*q3 + r2*q0 - r3*q1;
		double n3 = r0*q3 - r1*q2 + r2*q1 + r3*q0;
		
		double[] quat_vals = {n0, n1, n2, n3};
		result = new Quaternion(quat_vals);
//		Matrix quat_mat = new Matrix(quat_vals, 1);
//		result = new Quaternion(quat_mat);		
		return result;
	}
	public double[] get_quat_values()
	{
		return quat_vals;
	}
	
	public static Quaternion RotationMatToQuat(Matrix3d mat)
	{
		double m00 = mat.get(0, 0);
		double m01 = mat.get(0, 1);
		double m02 = mat.get(0, 2);
		double m10 = mat.get(1, 0);
		double m11 = mat.get(1, 1);
		double m12 = mat.get(1, 2);
		double m20 = mat.get(2, 0);
		double m21 = mat.get(2, 1);
		double m22 = mat.get(2, 2);
		
		double tr = m00 + m11 + m22;
		
		double qw, qx, qy, qz;
		
		if (tr > 0) { 
			  double S = Math.sqrt(tr+1.0) * 2; // S=4*qw 
			  qw = 0.25 * S;
			  qx = (m21 - m12) / S;
			  qy = (m02 - m20) / S; 
			  qz = (m10 - m01) / S; 
			} else if ((m00 > m11)&(m00 > m22)) { 
			  double S = Math.sqrt(1.0 + m00 - m11 - m22) * 2; // S=4*qx 
			  qw = (m21 - m12) / S;
			  qx = 0.25 * S;
			  qy = (m01 + m10) / S; 
			  qz = (m02 + m20) / S; 
			} else if (m11 > m22) { 
			  Double S = Math.sqrt(1.0 + m11 - m00 - m22) * 2; // S=4*qy
			  qw = (m02 - m20) / S;
			  qx = (m01 + m10) / S; 
			  qy = 0.25 * S;
			  qz = (m12 + m21) / S; 
			} else { 
			  double S = Math.sqrt(1.0 + m22 - m00 - m11) * 2; // S=4*qz
			  qw = (m10 - m01) / S;
			  qx = (m02 + m20) / S;
			  qy = (m12 + m21) / S;
			  qz = 0.25 * S;
			}
		double[] vals = {qw,qx,qy,qz};
		Quaternion new_quat = new Quaternion(vals);
		return new_quat;
	}
	
	public Vector3d getVec()
	{
		Vector3d quat_vec = new Vector3d();
		quat_vec.set(b,c,d);
		return quat_vec;
	}
	public double get_w()
	{
		return a;
	}
	
}

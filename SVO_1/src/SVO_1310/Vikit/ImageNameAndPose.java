package SVO_1310.Vikit;

import Sophus.Quaternion;
import matrix_types.Vector3d;

public class ImageNameAndPose {
	private double timestamp_;
	private String image_name_;
	private Vector3d t_;
	private Quaternion q_;
	
	String dataline;

	public ImageNameAndPose()
	{}
	public ImageNameAndPose(String string)
	{
		this.dataline = string;
		String[] ss = string.split(" ");
		if(ss.length==9)
		{
			this.timestamp_ = Double.parseDouble(ss[0]);
			this.image_name_ = ss[1];
			this.t_ = new Vector3d();
			this.t_.set(Double.parseDouble(ss[2]),Double.parseDouble(ss[3]),Double.parseDouble(ss[4]));
			
			// Position x = 5, y = 6, z = 7, w = 8 
			double[] quat_vals = {Double.parseDouble(ss[8]),Double.parseDouble(ss[5]),Double.parseDouble(ss[6]),Double.parseDouble(ss[7]),};
			this.q_ = new Quaternion(quat_vals);
			this.q_.normalize();
//			this.q_.print();
		}
		else
			System.out.println("The input line does not contain the 9 pieces of data per line.");
	}

	public double getTimestamp_() {
		return timestamp_;
	}
	public void setTimestamp_(double timestamp_) {
		this.timestamp_ = timestamp_;
	}
	public String getImage_name_() {
		return image_name_;
	}
	public void setImage_name_(String image_name_) {
		this.image_name_ = image_name_;
	}
	public Vector3d getT_() {
		return t_;
	}
	public void setT_(Vector3d t_) {
		this.t_ = t_;
	}
	public Quaternion getQ_() {
		return q_;
	}
	public void setQ_(Quaternion q_) {
		this.q_ = q_;
	}
	public String toString()
	{
		return dataline;
	}
	public void print()
	{
		System.out.println(image_name_+", t_ = "+ t_ +", Q = "+q_);
	}
}

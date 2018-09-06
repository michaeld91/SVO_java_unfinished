package SVO_1310;

import matrix_types.Matrix2d;

public class Seed {
	
	private static int batch_counter = 0;
	private static int seed_counter = 0;
	private int batch_id;						//!< Batch id is the id of the keyframe for which the seed was created.
	private int id;								//!< Seed ID, only used for visualization.
	private Feature ftr;						//!< Feature in the keyframe for which the depth should be computed.
	private float a;							//!< a of Beta distribution: When high, probability of inlier is large.
	private float b;							//!< b of Beta distribution: When high, probability of outlier is large.
	private float mu;							//!< Mean of normal distribution.
	private float z_range;						//!< Max range of the possible depth.
	private float sigma2;						//!< Variance of normal distribution.
	Matrix2d patch_cov;							//!< Patch covariance in reference image.

	
	
	public Seed(Feature ftr, float depth_mean, float depth_min)
	{
		batch_id = batch_counter;
		id = seed_counter++;
		this.ftr = ftr;
		a = 10;
		b = 10;
		mu = 1/(depth_mean);
		z_range = 1/(depth_min);
		sigma2 = (z_range*z_range/36);
	}
	public Seed(Feature ftr, double depth_mean, double depth_min)
	{
		batch_id = batch_counter;
		id = seed_counter++;
		this.ftr = ftr;
		a = 10;
		b = 10;
		mu = 1/((float)depth_mean);
		z_range = 1/((float)depth_min);
		sigma2 = (z_range*z_range/36);
	}
	public float get_a()
	{
		return a;
	}
	public void set_a(float a)
	{
		this.a = a;
	}
	public float get_b()
	{
		return b;
	}
	public void set_b(float b)
	{
		this.b = b;
	}
	public static int get_batch_counter()
	{
		return batch_counter;
	}
	public static void increment_batch_counter()
	{
		batch_counter++;
	}
	public int get_seed_counter()
	{
		return seed_counter;
	}
	public int get_batch_id()
	{
		return batch_id;
	}
	public float get_mu()
	{
		return mu;
	}
	public void set_mu(float mu)
	{
		this.mu = mu;
	}
	public Feature getFeature()
	{
		return ftr;
	}
	public float get_sigma2()
	{
		return sigma2;
	}
	public void set_sigma2(float sigma2)
	{
		this.sigma2 = sigma2; 
	}
	public void increment_b()
	{
		b++;
	}
	public float get_z_range()
	{
		return z_range;
	}
}

package SVO_1310.Vikit;

import java.util.Random;

public class Sample {

//	public static void main(String[] args) 
//	{
//		double gaus = get_gaussian(1,1);
//		System.out.println("gaus = " + gaus);
//	}
	public static double gaussian(double stddev)
	{
		Random rand = new Random();
		double gaussian_distribution = rand.nextGaussian()*stddev + 0.0;//get_gaussian(rand.nextDouble(),stddev);//
		return gaussian_distribution;
	}
	
	private static double get_gaussian(double x, double stddev)
	{
		double gaus = (1/(stddev*Math.sqrt(Math.PI*2))) * (Math.pow(Math.E, -(x*x)/(2*stddev*stddev)));
		
		return gaus;
	}
}

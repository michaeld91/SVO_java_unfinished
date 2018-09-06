package SVO_1310;

import java.util.Arrays;
import java.util.Iterator;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import SVO_1310.Vikit.Vision;


//Some helper functions for the frame object.
public class Frame_utils {

	public static void createImgPyramid(Mat img_level_0, int n_levels, ImgPyr pyr)
	{
//		pyr.resize(n_levels);
//		System.out.println(pyr.getMats().length+","+ n_levels);
		pyr.getMats()[0] = img_level_0;
		for(int i = 1; i < n_levels; i++)
		{
//			pyr.getMats()[i] = new Mat(pyr.getMats()[i-1].rows()/2, pyr.getMats()[i-1].cols()/2, CvType.CV_8U);
			pyr.add(new Mat(pyr.getMats()[i-1].rows()/2, pyr.getMats()[i-1].cols()/2, CvType.CV_8U));
			Vision.halfSample(pyr.getMats()[i-1], pyr.getMats()[i]);
		}
	}
	public static double[] getSceneDepth(Frame frame)//, double depth_mean, double depth_min)
	{
		// Different to C++ in that an array of 2 doubles is returned, the first is the min the second is the mean depth.
		// Instead of returning false, if both values are still null, issue warning.
		
		double[] depth_vec = new double[frame.getFts_().size()];
		double depth_mean;
		double depth_min;
		
		double[] min_and_mean = new double[2]; //array to be returned. min first then mean 2nd
		
		depth_min = Double.MAX_VALUE;
		Iterator<Feature> it = frame.getFts_().iterator();
		int arrayIndex =0;
		while(it.hasNext())
		{
			double z = frame.w2f(it.next().getPoint().getPos()).get(2);
			depth_vec[arrayIndex] = z;
			arrayIndex++;
			depth_min = Math.min(z, depth_min);
		}
		min_and_mean[0] = depth_min;
		Boolean no_point_obs = false;
		for(double d: depth_vec)
		{
			if(d==0){ no_point_obs = true; }
		}
		// This needs to be altered so that depth_vec is checked for being empty (not equal to zero)
		if(no_point_obs = true)
		{
			// SVO_WARN_STREAM("Cannot set scene depth. Frame has no point-observations!");
			return min_and_mean;
		}
		depth_mean = median(depth_vec);
		min_and_mean[1] = depth_mean;
		
		return min_and_mean;
	}
	
	// median for the depth_vec array
	private static double median(double[] m)
	{
		Arrays.sort(m);
		int mid = m.length/2;
	    if (m.length%2 == 1) 
	    {
	        return m[mid];
	    } 
	    else 
	    {
	        return (m[mid-1] + m[mid]) / 2.0;
	    }
	}
}

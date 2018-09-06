package SVO_1310.Vikit;

import java.util.ArrayList;

public class MADScaleEstimator {
	final static float NORMALIZER = 1.48f;	//=(1/.6745)
	
	public double compute(ArrayList<Double> errors) {
		// error must be in absolute values!
		
		return NORMALIZER * math_utils.getMedian(errors);
	}

}

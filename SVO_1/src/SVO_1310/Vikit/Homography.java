package SVO_1310.Vikit;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint2f;
import org.opencv.calib3d.Calib3d;
import org.opencv.core.Point;

import Jama.Matrix;
import Jama.SingularValueDecomposition;
import Sophus.Se3;
import matrix_types.JamaUtils;
import matrix_types.Matrix3d;
import matrix_types.Vector2d;
import matrix_types.Vector3d;

import org.opencv.core.Point;

import Sophus.Se3;
import matrix_types.Vector2d;

// ToDo implement from homography.cpp & homography.h
// /home/michael/Documents/rpg_vikit/vikit_common/include/vikit
// /home/michael/Documents/rpg_vikit/vikit_common/src
public class Homography {

	private Se3 T_c2_from_c1;
	private double thresh;
	private double error_multiplier2;
	private Vector2d[] fts_c1;
	private Vector2d[] fts_c2;
	private ArrayList<HomographyDecomposition> decompositions = new ArrayList<HomographyDecomposition>();
	private Matrix3d H_c2_from_c1 = new Matrix3d(); 
	
	private ArrayList<Boolean> inliers = new ArrayList<Boolean>();
	
	public Homography(Vector2d[] _fts1, Vector2d[] _fts2,
			double _error_multiplier2, double reprojection_threshold) 
	{
		thresh = reprojection_threshold;
		this.error_multiplier2 = error_multiplier2;
		fts_c1 = _fts1;
		fts_c2 = _fts2;
	
	}

	public boolean computeSE3fromMatches() 
	{
		calcFromMatches();
		boolean res = decompose();
		if(!res)
			return false;
		computeMatchesInliers();
		findBestDecomposition();
		T_c2_from_c1 = decompositions.get(0).getT();
		return true;
	}
	
	private void findBestDecomposition() 
	{
		if(decompositions.size() ==8)
		{
			for(int i = 0; i < decompositions.size(); i++)
			{
				HomographyDecomposition decom = decompositions.get(i);
				int nPositive = 0;
				for(int m = 0; m < fts_c1.length; m++)
				{
					if(!inliers.get(m))
						continue;
					final Vector2d v2 = fts_c1[m];
					double dVisibilityTest = (H_c2_from_c1.get(2,0) * v2.get(0) + H_c2_from_c1.get(2,1) * v2.get(1) + H_c2_from_c1.get(2,2)) / decom.getD();
					if(dVisibilityTest > 0.0)
						nPositive++;
				}
				decom.setScore(-nPositive);
			}
			
			Collections.sort(decompositions);
			decompositions.ensureCapacity(4);
			
			for(int i = 0; i < decompositions.size(); i++)
			{
				HomographyDecomposition decom = decompositions.get(i);
				int nPositive = 0;
				for(int m = 0; m < fts_c1.length; m++)
				{
					if(!inliers.get(m))
						continue;
					Vector3d v3 = math_utils.unprojected2d(fts_c1[m]);
					double dVisibilityTest = JamaUtils.dotproduct(v3, fts_c1[m])/decom.getD();
					if(dVisibilityTest > 0.0)
						nPositive++;
				}
				decom.setScore(-nPositive);
			}
			
			Collections.sort(decompositions);
			decompositions.ensureCapacity(2);
			
			// According to Faugeras and Lustman, ambiguity exists if the two scores are equal
			// but in practive, better to look at the ratio!
			double dRatio = decompositions.get(1).getScore()/decompositions.get(0).getScore();
			
			if(dRatio < 0.9)	// no ambiguity!
				decompositions.remove(1);	// ToDo is this the same: decompositions.erase(decompositions.begin() + 1);
			else // two-way ambiguity. Resolve by sampsonus score of all points.
			{
				double dErrorSquaredLimit = thresh * thresh * 4;
				double[] adSampsonusScores = new double[2];
				for(int i = 0; i < 2; i++)
				{
					Se3 T = decompositions.get(i).getT();
					Matrix3d Essential = T.rotation_Matrix().times(math_utils.sqew(T.get_Translation()));
					double dSumError = 0;
					for(int m=0; m < fts_c1.length; m++)
					{
						double d = math_utils.sampsonusError(fts_c1[m], Essential, fts_c2[m]);
						if(d > dErrorSquaredLimit)
							d = dErrorSquaredLimit;
						dSumError += d;
					}
					adSampsonusScores[i] = dSumError;
				}
				
				if(adSampsonusScores[0] <= adSampsonusScores[1])
					decompositions.remove(1);
				else
					decompositions.remove(0);
			}
		}
	}

	private int computeMatchesInliers() 
	{
		inliers.clear(); inliers.ensureCapacity(fts_c1.length);
		int n_inliers = 0;
		for(int i = 0; i < fts_c1.length; i++)
		{
			Vector2d projected = math_utils.project2d(H_c2_from_c1.times(math_utils.unprojected2d(fts_c1[i])));
			Vector2d e = fts_c2[i].minus(projected);
			double e_px = error_multiplier2 * e.normF();
			inliers.set(i, (e_px < thresh));
			
			if(inliers.get(i)==true)
				n_inliers++;
		}
		return n_inliers;
	}

	public void calcFromMatches() 
	{
		MatOfPoint2f src_pts;
		MatOfPoint2f dst_pts;
		
		Point[] src_pts_list = new Point[fts_c1.length];
		Point[] dst_pts_list = new Point[fts_c1.length];
		for(int i=0; i<fts_c1.length; ++i)
	  {
	    src_pts_list[i]	=	new Point(fts_c1[i].get(0), fts_c1[i].get(1));
	    dst_pts_list[i]	=	new Point(fts_c2[i].get(0), fts_c2[i].get(1));
	  }
		src_pts = new MatOfPoint2f(src_pts_list);
		dst_pts = new MatOfPoint2f(dst_pts_list);
		
	  // TODO: replace this function to remove dependency from opencv!
	  Mat cvH = Calib3d.findHomography(src_pts, dst_pts, Calib3d.RANSAC, 2./error_multiplier2);
//	  System.out.println(cvH.type());

	  //System.out.print(cvH.type()+"\n");
	  // ToDo not sure what method to use to retrieve the value. using get(row,col) returns double[].
	  // should i just use that and then get the first value, as below?
	  byte[] data = new byte[9];
//	  short[] data = new short[9];
//	  int[] data2 = new int[9];
//	  float[] data3 = new float[9];
//	  double[] data4 = new double[9];
//	  int num_failures = 0;

	  //data
	  try{
		  cvH.get(0, 0, data);
	  }
	  catch(UnsupportedOperationException E)
	  {
		  System.out.println("Unsupported Operation Exception. Placing the data in to a byte[] failed");
	  }

	  
	  
//	  for(int i=0; i<data.length; i++)
//	  {
//		  System.out.println(data[i]);
//	  }
	  
	  
	  H_c2_from_c1.set(0,0, data[0]);	  
	  H_c2_from_c1.set(0,1, data[1]);
	  H_c2_from_c1.set(0,2, data[2]);
	  H_c2_from_c1.set(1,0, data[3]);
	  H_c2_from_c1.set(1,1, data[4]);
	  H_c2_from_c1.set(1,2, data[5]);
	  H_c2_from_c1.set(2,0, data[6]);
	  H_c2_from_c1.set(2,1, data[7]);
	  H_c2_from_c1.set(2,2, data[8]);
	  
//	  H_c2_from_c1.set(0,1, cvH.at(0,1));
//	  H_c2_from_c1.set(0,2, cvH.at(0,2));
//	  H_c2_from_c1.set(1,0, cvH.at(1,0));
//	  H_c2_from_c1.set(1,1, cvH.at(1,1));
//	  H_c2_from_c1.set(1,2, cvH.at(1,2));
//	  H_c2_from_c1.set(2,0, cvH.at(2,0));
//	  H_c2_from_c1.set(2,1, cvH.at(2,1));
//	  H_c2_from_c1.set(2,2, cvH.at(2,2));
	}

	public Se3 getT_c2_from_c1()
	{
		return T_c2_from_c1;
	}
	
	public boolean decompose()
	{
		decompositions.clear();
		
		// ToDo JacobiSVD<MatrixXd> svd(H_c2_from_c1, ComputeThinU | ComputeThinV);
		SingularValueDecomposition svd = new SingularValueDecomposition(H_c2_from_c1);	// ToDo this should be checked		

		double[] singular_values_array = svd.getSingularValues();
//		Vector3d singular_values = new Vector3d();
//		singular_values.set(singular_values_array);
		
		double d1 = Math.abs(singular_values_array[0]);	// The paper suggests the square of these (e.g. the evalues of AAT)
		double d2 = Math.abs(singular_values_array[1]);		// should be used, but this is wrong. c.f. Faugeras' book.
		double d3 = Math.abs(singular_values_array[2]);
		
		Matrix3d U = new Matrix3d(svd.getU().getArray());
		Matrix3d V = new Matrix3d(svd.getV().getArray());
		
		double s = U.det() * V.det();
		double dPrime_PM = d2;
		
		int nCase;
		if(d1 != d2 && d2 != d3)
			nCase = 1;
		else if( d1 == d2 && d2 == d3 )
			nCase = 3;
		else
			nCase = 2;
		
		if(nCase != 1)
		{
			System.out.println("FATAL Homography Initialization: This motion case is not implemented or is degenerate. Try again. ");
			return false;
		}
		
		double x1_PM;
		double x2;
		double x3_PM;
		
		// All below deals with the case = 1.
		// Case 1 implies (d1 != d3). Not sure why this is in parenthesis.
		{ // Eq. 12
			x1_PM = Math.sqrt((d1*d1 - d2*d2) / (d1*d1 - d3*d3));
			x2 = 0;
		    x3_PM = Math.sqrt((d2*d2 - d3*d3) / (d1*d1 - d3*d3));
		}
		
		double[] e1 = {1.0, -1.0, 1.0, -1.0};
		double[] e3 = {1.0, 1.0,-1.0,-1.0};
		
		Vector3d np = new Vector3d();
		HomographyDecomposition decomp = null;
		
		// Case 1, d' > 0;
		decomp.setD(s * dPrime_PM);
		for(int signs = 0; signs < 4; signs++)
		{
			// Eq 13
			double dSinTheta = (d1 - d3) * x1_PM * x3_PM * e1[signs] * e3[signs] / d2;
			double dCosTheta = (d1 * x3_PM * x3_PM + d3 * x1_PM * x1_PM) / d2;

		    double[] R_values = {dCosTheta, 0, -dSinTheta, 0, 1, 0, dSinTheta, 0, dCosTheta};
		    decomp.setR(new Matrix3d(R_values));
		    
		    // Eq 14
		    Vector3d t_values = new Vector3d();
		    t_values.set((d1 - d3) * x1_PM * e1[signs],0.0,(d1 - d3) * -x3_PM * e3[signs]);
		    decomp.set_vector_t(t_values);
		    
		    np.set(x1_PM * e1[signs], x2, x3_PM * e3[signs]);
		    decomp.set_n(np.times(V));	// ToDo this was V * np, other way around returns a different result.
		    
		    decompositions.add(decomp);
		}
		
		// Case 1, d' < 0;
		decomp.setD(s*-dPrime_PM);
		for(int signs = 0; signs<4; signs++)
		{
			// Eq 15
			
			double dSinPhi = (d1 + d3) * x1_PM * x3_PM * e1[signs] * e3[signs] / d2;
		    double dCosPhi = (d3 * x1_PM * x1_PM - d1 * x3_PM * x3_PM) / d2;
		    
		    double[] R_values = {dCosPhi, 0, dSinPhi, 0, -1, 0, dSinPhi, 0, -dCosPhi};
		    decomp.setR(new Matrix3d(R_values));
		    
		    // Eq 16
		    Vector3d t_values = new Vector3d();
		    t_values.set((d1 + d3) * x1_PM * e1[signs],0.0,(d1 + d3) * -x3_PM * e3[signs]);
		    decomp.set_vector_t(t_values);		    
		    
		    np.set(x1_PM * e1[signs], x2, x3_PM * e3[signs]);
		    decomp.set_n(np.times(V));	// ToDo this was V * np, other way around returns a different result.
		    
		    decompositions.add(decomp);		    
		}
		
		// Save rotation and translation of the decomposition
		for(int i = 0; i<decompositions.size(); i++)
		{
			Matrix3d R = U.times(s).times(decompositions.get(i).getR()).times(V.transpose());
			Vector3d t = new Vector3d(U.times(decompositions.get(i).get_vector_t()).getArray());
			decompositions.get(i).setT(new Se3(R, t));
		}
		return true;
	}
}

package SVO_1310;

import java.util.ArrayList;
import java.util.List;

import org.opencv.features2d.FastFeatureDetector;
import org.opencv.core.KeyPoint;
import org.opencv.core.MatOfKeyPoint;


import SVO_1310.Vikit.Vision;
//import SVO_1310.Vikit.math_utils;
import svo.Fast_Detector.Fast12;
import svo.Fast_Detector.FeaturePoint;
import matrix_types.Vector2d;

// ToDo find the references for this!!
/// FAST detector by Edward Rosten.
public class FastDetector extends AbstractDetector {

	public FastDetector(int img_width, int img_height, int cell_size, int n_pyr_levels)
	{
		super(img_width, img_height, cell_size, n_pyr_levels);
	}

	@Override
	public void detect(Frame frame, ImgPyr img_pyr, int detection_threshold, ArrayList<Feature> fts) 
	{
		// Corners corners(grid_n_cols_*grid_n_rows_, Corner(0,0,detection_threshold,0,0.0f));
		Corner[] corners = new Corner[grid_n_cols_*grid_n_rows_];// ToDo Corners corners(grid_n_cols_*grid_n_rows_, Corner(0,0,detection_threshold,0,0.0f));

		for(int i = 0; i < corners.length-1; i++)	// ToDo Is this only required for the line ~68 if (score > corners[k].getScore()).....
		{
			corners[i] = new Corner(0,0,detection_threshold, 0, 0.0f);
		}

//		List<FeaturePoint> fast_corners = new ArrayList<FeaturePoint>(); // For testing if fast_corners are found correctly
		
		for(int L=0; L < n_pyr_levels_; L++)//		for(int L=0; L < n_pyr_levels_; L++)
		{
//			System.out.println("Level = "+L+" corner size = "+corners.length+" cell size = "+cell_size_+" grid_n_cols, grid_n_rows = "+ grid_n_cols_+","+grid_n_rows_);
			final int scale = (int) (1<<L); // ToDo could also use Math.pow(2,L); (used to halve the size each level)
//			//List<FeaturePoint> fast_corners;//Fast_xy[] fast_corners;
//
//			// #if __SSE2__
//			//
//			// #elif HAVE_FAST_NEON
//			//
//			// #else ToDo may need to include the else lines
//
//			// ToDo issue here is that detect requires the img as [][]
//			// fast_corners = Fast12.detect(img_pyr.getMats()[L].getData(), img_pyr.getMats()[L].getCols(), img_pyr.getMats()[L].getRows(), img_pyr.getMats()[L].getCols(), 20); ToDo altered when OpenCv library imported
//			// ToDo the img_pyr.getMats()[L].data 
//			//System.out.println("img_pyr.getMats() = "+img_pyr.getMats());
//			byte[] data = new byte[img_pyr.getMats()[L].cols()*img_pyr.getMats()[L].rows()];
//			img_pyr.getMats()[L].get(0, 0, data);
//
//			// ToDo width and height may need to be swapped to work
//			int[][] data_int_arrays = Global.convert_from_array_to_double_array(Global.toIntArray(data), img_pyr.getMats()[L].cols(), img_pyr.getMats()[L].rows());
//
//			// Swapping the rows and cols fields in to the oposite position appears to work.
//			// ToDo: rename this part to avoid future confusion!
//			//			List<FeaturePoint> fast_corners = Fast12.detect(data_int_arrays, img_pyr.getMats()[L].cols(), img_pyr.getMats()[L].rows(), img_pyr.getMats()[L].cols(), 20);
//			List<FeaturePoint> fast_corners = Fast12.detect(data_int_arrays,  img_pyr.getMats()[L].rows(), img_pyr.getMats()[L].cols(), detection_threshold,Config.getdesired_number_of_corners_per_img());// ToDo: changed last two values from this : img_pyr.getMats()[L].cols(), 20);

			// OpenCV fast corner version
			
			FastFeatureDetector fast = FastFeatureDetector.create();
			fast.setThreshold(detection_threshold);
			fast.setNonmaxSuppression(false);	// set to true to remove the need for non_max corners
			MatOfKeyPoint fast_keypoints = new MatOfKeyPoint();

			fast.detect(img_pyr.getMats()[L], fast_keypoints);
//			System.out.println("fast_keypoints size = " + fast_keypoints.toArray().length);
//			for(KeyPoint kp:fast_keypoints.toArray())
//			{
//				System.out.println(kp.pt.x+","+kp.pt.y);
//			}
			int corners_counter = 0;

			for(KeyPoint corner: fast_keypoints.toArray())
			{
				KeyPoint xy = corner;
				final int k = (int)(((xy.pt.y*scale)/cell_size_)*grid_n_cols_ + ((xy.pt.x*scale)/cell_size_));
//				System.out.println("xy.pt.x = "+xy.pt.x+"   xy.pt.y = "+xy.pt.y);
				// ToDo Grid occupancy should go here
				final float score = Vision.shiTomasiScore(img_pyr.getMats()[L], (int)xy.pt.y, (int)xy.pt.x);	// ToDo Note: The .y() and .x() fields have been swapped. Solves the corners fitting the image. x&y swapping issue! 
//				if (score > corners[k].getScore()) ToDo include this!!
				if(k < corners.length)	// ToDo this if is just for testing. should not be included!
				{
					corners_counter++;
					corners[k] = new Corner((int)xy.pt.x*scale, (int)xy.pt.y*scale, score, L, 0.0f);
				}

			}
//			System.out.println("Number of keypoints found = "+fast_keypoints.toArray().length);
//			System.out.println("Corners_counter = " + corners_counter);
			
			int counter = 0;
			for(int i =0; i<corners.length; i++)
			{
				if(corners[i]!=null)
					if(corners[i].getScore()!=0)
						counter++;
			}
//			System.out.println("counter = "+counter);
			
			
			
			
			
			
			
			
			

//			//System.out.println("fast_corners list is of size = "+ fast_corners.size());
//			//int[] scores;	// ToDo possibly not required due to scores being saved within a FeaturePoint
//			//int[] nm_corners;
//			// This line sets the scores, which are already in fast_corners 	Fast12.cornerScore(null, scale, scale);
//
//
//			// ToDo: swapped the width and height field positions again!
//			//			List<FeaturePoint> nm_corners = Fast12.detectWithNonMax(data_int_arrays, get_img_width(), get_img_height(), detection_threshold, -1); //nm_corners = Fast12.fast_nonmax_3x3(fast_corners, scores);	// ToDo this should call nonMaxSupression I believe?
//			List<FeaturePoint> nm_corners = Fast12.detectWithNonMax(data_int_arrays, get_img_height(), get_img_width(), detection_threshold, Config.getdesired_number_of_corners_per_img()); //nm_corners = Fast12.fast_nonmax_3x3(fast_corners, scores);	// ToDo this should call nonMaxSupression I believe?
//
//			//System.out.println("nm_corners list is of size = "+ nm_corners.size());
//
//			// ToDo nm_corners needs to be set to a specific size.
//			for(FeaturePoint nm: nm_corners)//(int it = 0; it <nm_corners.size() ; it++)
//			{
//				FeaturePoint xy = fast_corners.get(nm_corners.indexOf(nm));
////				FeaturePoint xy = fast_corners.get(it);	//Fast_xy xy = fast_corners.at(it);
//				final int k = ((xy.x()*scale)/cell_size_)*grid_n_cols_ + ((xy.y()*scale)/cell_size_);
////				System.out.println("((xy.y()*scale)/cell_size_)*grid_n_cols_ = "+((xy.y()*scale)/cell_size_)*grid_n_cols_);
////				System.out.println("k = "+k+" xy.x, xy.y = "+xy.y()+","+xy.x()+" Scale = "+scale+" cell_size_ = "+cell_size_ + " grid_n_cols_ = "+grid_n_cols_);
/////////////////////////////////////////////// Testing if grid_occupancy is always null
//				//				for(int i = 0; i<grid_occupancy_.length; i++)
//				//					System.out.println("grid_occupance_["+i+"] = " + grid_occupancy_[i]);
//				//				System.out.println("grid_occupancy length = "+ grid_occupancy_.length);
//				//				System.out.println("grid_n_cols, grid_n_rows_ = "+ grid_n_cols_+", "+ grid_n_rows_);
//
//
//				///////////////////////////////////////////
////				System.out.println("it = "+it);
//
//				// ToDo grid occupancy needs to be added back in!
//				//				if(grid_occupancy_[k])	// I dont think this will occur as it is only used in depth_filter
//				//					continue;
//				final float score = Vision.shiTomasiScore(img_pyr.getMats()[L], xy.y(), xy.x());	// ToDo Note: The .y() and .x() fields have been swapped. Solves the corners fitting the image. x&y swapping issue! 
//
//				if (score > corners[k].getScore())
//				{
////					corners[k] = new Corner(xy.x()*scale, xy.y()*scale, score, L, 0.0f);
/////////////////////////////////////////////					change back							///////////////////////////////////////////
//					// ToDo Change back::changed k to it 
//					corners[k] = new Corner(xy.x()*scale, xy.y()*scale, score, L, 0.0f);
//					System.out.println(corners[k].getX()+","+corners[k].getY());
////					System.out.println("Corner "+k+" = "+corners[k].getX()+","+corners[k].getY());
//
//				}
//			}
//			// print out each of the corner lists values for evaluation
////			for(int i = 0; i< 100; i++)
////			{
////				System.out.println("fast_corners = "+fast_corners.get(i).x()+","+fast_corners.get(i).y()+", nm_corners = "+nm_corners.get(i).x()+","+nm_corners.get(i).y()+", corners = "+corners[i].getX()+","+corners[i].getY());
////			}

		}

		// Create feature for every corner that has a high enough corner score
		for(int j=0; j<Config.getdesired_number_of_corners_per_img(); j++)// ToDo changed from this: (Corner c: corners) so it is only going through the amount specified by the detect(....., 20) field
		{
			//			System.out.println("score = "+c.getScore());
			//			System.out.println("detection_threshold = "+detection_threshold);
			Corner c = corners[j];
			if(c.getScore() > -1)//detection_threshold)
			{
				Vector2d vec2d = new Vector2d();	
				vec2d.set(c.getX(), c.getY());	// ToDo The issue with the x and y being the wrong way around was causing corners to be shown transposed!
				fts.add(new Feature(frame, vec2d, c.getLevel()));
//				System.out.println(vec2d.get(0)+","+vec2d.get(1));
			}
		}
//		for(int j=0; j<Config.getdesired_number_of_corners_per_img(); j++)// ToDo changed from this: (Corner c: corners) so it is only going through the amount specified by the detect(....., 20) field
//		{
//			//			System.out.println("score = "+c.getScore());
//			//			System.out.println("detection_threshold = "+detection_threshold);
//			FeaturePoint fp = fast_corners.get(j);
////			if(j.getScore() > detection_threshold)
//			{
//				Vector2d vec2d = new Vector2d();	
//				vec2d.set(fp.x(), fp.y());
//				fts.add(new Feature(frame, vec2d, 0));
//				System.out.println(vec2d.get(0)+","+vec2d.get(1));
//			}
//		}
		
		resetGrid();
		//return fts;
		
		
	}

	// Method for finding the max corner x and y values for testing.
	private void findMax(ArrayList<Feature> fts)
	{
		int x_val_max = 0;
		int y_val_max = 0;
		for(Feature ft: fts)
		{
			if(ft.get_px(0)>x_val_max)
				x_val_max=(int) ft.get_px(0);
			if(ft.get_px(1)>y_val_max)
				y_val_max=(int) ft.get_px(1);
		}
		System.out.println("Max X = "+ x_val_max);
		System.out.println("Max Y = "+ y_val_max);
	}
	
}

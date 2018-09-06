package SVO_1310;

import java.util.ArrayList;

import matrix_types.Vector2d;


/*
 * All Detectors should be derived from this class.
 * 
 */
public abstract class AbstractDetector {


	protected static int border_ = 8; //No feature should be within 8px of the border.
	protected int cell_size_;
	protected int n_pyr_levels_;
	protected int grid_n_cols_;
	protected int grid_n_rows_;
	protected boolean[] grid_occupancy_;//protected vector<bool> grid_occupancy_;	// ToDo What is the best way to code this?
	
	private int img_width;
	private int img_height;
	

	public AbstractDetector(int img_width, int img_height, int cell_size, int n_pyr_levels)
	{
		this.cell_size_ = cell_size;
		this.n_pyr_levels_ = n_pyr_levels;
		this.grid_n_cols_ = (int) Math.ceil((img_width)/cell_size_);
		this.grid_n_rows_ = (int) Math.ceil((img_height)/cell_size_);
		this.grid_occupancy_ = new boolean[grid_n_cols_*grid_n_rows_]; //grid_occupancy_(grid_n_cols_*grid_n_rows_, false)
		for(boolean bool : grid_occupancy_) bool=false;
//		System.out.println("grid_occupancy_ is of size ="+ grid_occupancy_.length);
		this.img_width = img_width;
		this.img_height = img_height;

	}

	abstract public void detect(Frame frame, ImgPyr img_pyr, int detection_threshold, ArrayList<Feature> fts);
	
	//Flag the grid cell as occupied
	public void setGridOccupancy(Vector2d px)
	{

//		grid_occupancy_.at(
//				static_cast<int>(px[1]/cell_size_)*grid_n_cols_
//				+ static_cast<int>(px[0]/cell_size_)) = true;
		grid_occupancy_[(int)(px.get(0,1)/cell_size_)*grid_n_cols_+ (int)(px.get(0,0)/cell_size_)] = true;
		

	}
	
	//Set grid cells of existing features as occupied
	public void setExistingFeatures(ArrayList<Feature> fts)
	{
		for(Feature feat : fts)
		{
			grid_occupancy_[(int)(feat.get_px(1)/(cell_size_)*grid_n_cols_)+(int)(feat.get_px(0)/(cell_size_))]= true;
		}
	}

	public void resetGrid()
	{
		//fill(grid_occupancy_.begin(), grid_occupancy_.end(), false); fill() assigns the given value to everything in the Range. 
		for(Boolean bool : grid_occupancy_){bool = false;}
	}

	public int getCellIndex(int x, int y, int level)
	{
		int scale = (int) (1*Math.pow(2, level));	//This appears to be what 1<<level means?
		return (scale*y)/cell_size_*grid_n_cols_ + (scale*x)/cell_size_;
	}

	public int get_img_width()
	{
		return img_width;
	}
	public int get_img_height()
	{
		return img_height;
	}

//	public ArrayList<Feature> detect(Frame frame, ImgPyr img_pyr,
//			int detection_threshold, ArrayList<Feature> fts) {
//		// TODO Auto-generated method stub
//		return null;
//	}
	
	public void set_n_pyr_levels_(int pyr_levels)
	{
		n_pyr_levels_ = pyr_levels;
	}
	public int get_n_pyr_levels_()
	{
		return n_pyr_levels_;
	}
}

package SVO_1310;

import java.util.ArrayList;

public class Grid {
//	 struct Grid
//	  {
//	    CandidateGrid cells;
//	    vector<int> cell_order;
//	    int cell_size;
//	    int grid_n_cols;
//	    int grid_n_rows;
//	  };
	
	private CandidateGrid cells;
	//private int[] cell_order;	//This is a vector in the c++
	private ArrayList<Integer> cell_order = new ArrayList<Integer>();
	private int cell_size;
	private int grid_n_cols;
	private int grid_n_rows;
	
	public Grid()
	{
		
	}
	
	public CandidateGrid getCells() {
		return cells;
	}
	public void setCells(CandidateGrid cells) {
		this.cells = cells;
	}
	public ArrayList<Integer> getCell_order() {
		return cell_order;
	}
	public void setCell_order(ArrayList<Integer> cell_order) {
		this.cell_order = cell_order;
	}
	public int getCell_size() {
		return cell_size;
	}
	public void setCell_size(int cell_size) {
		this.cell_size = cell_size;
	}
	public int getGrid_n_cols() {
		return grid_n_cols;
	}
	public void setGrid_n_cols(int grid_n_cols) {
		this.grid_n_cols = grid_n_cols;
	}
	public int getGrid_n_rows() {
		return grid_n_rows;
	}
	public void setGrid_n_rows(int grid_n_rows) {
		this.grid_n_rows = grid_n_rows;
	}
	
	
}

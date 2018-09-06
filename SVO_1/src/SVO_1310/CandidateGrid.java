package SVO_1310;

import java.util.ArrayList;

// A list of Cells
public class CandidateGrid {
	//This is a vector in the C++, Used an ArrayList to immitate this.
	private ArrayList<Cell> allCell = new ArrayList<Cell>();
	
	public CandidateGrid()
	{
		
	}
	public ArrayList<Cell> getAllCells()
	{
		return allCell;
	}
	public void addCell(Cell cell1)
	{
		allCell.add(cell1);
	}
	public boolean removeCell(Cell cell1)
	{
		return allCell.remove(cell1);
	}
	public Cell at(int i)
	{
		return allCell.get(i);
	}
	
	public int size()
	{
		return allCell.size();
	}
}

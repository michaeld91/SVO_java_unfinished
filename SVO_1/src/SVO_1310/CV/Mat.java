package SVO_1310.CV;

/*
 * This package and class is a place holder for OpenCV. 
 *  OpenCV version: home/michael/opencv/build/src/org/opencv/core/Mat.java
 */
public class Mat {

	private String type = "CV_8UC1";
	private int cols;
	private int rows;
	private int data; // Required in Matcher
	
	private static final double step=0;	
	
	public Mat(int rows, int cols, String type)
	{
		this.rows = rows;
		this.cols = cols;
		this.type = type;
	}
	public double getStep()
	{
		return step;
	}
	
	public boolean empty()
	{
		return false;
	}
	
	public String type()
	{
		return type;
	}
	public int getCols()
	{
		return cols;
	}
	public int getRows()
	{
		return rows;
	}
	public int getData()
	{
		return data;
	}
	public int size()
	{
		// ToDo add method to retrieve size
		return 0;
	}
}

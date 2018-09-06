package SVO_1310;

public class Corner {

	private int x;			// x coordinate 
	private int y;			// y coordinate
	private int level;		// Pyramid level
	private double score;	// Intensity score
	private float angle;
	
	// Temporary container used for corner detection. Features are initialized from these.
	public Corner(int x, int y, double score, int level, float angle)
	{
		this.x = x;
		this.y = y;
		this.score = score;
		this.level = level;
		this.angle = angle;
	}
	
	public double getScore()
	{
		return score;
	}
	public int getLevel()
	{
		return level;
	}
	public int getX()
	{
		return x;
	}
	public int getY()
	{
		return y;
	}
}

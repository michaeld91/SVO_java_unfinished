package SVO_1310;

public class FrameDoublePair implements Comparable<FrameDoublePair>{
	private Frame frame;
	private double dbl;
	
	public FrameDoublePair(Frame frame, Double dbl)
	{
		this.frame = frame;
		this.dbl = dbl;
	}
	public Frame getFrame()
	{
		return frame;
	}
	public double getDouble()
	{
		return dbl;
	}
	public void increaseDouble()
	{
		dbl++;
	}
	public int compareTo(FrameDoublePair n) {
		 if(this.dbl==n.getDouble())  
		      return 0;  
		   else if(this.dbl>n.getDouble())  
		      return 1;  
		   else  
		      return -1; 
	}
}

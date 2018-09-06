package SVO_1310.Vikit;

// ToDo not the same as C++ ViKit. may not be required!
public class Timer {
	private long start_time_;
	private double time_;
	private double accumulated_;
	
	public Timer()
	{
		time_ = 0.0;
		accumulated_ = 0.0;
		start();
	}
	
	
	public double getTime() 
	{
		return time_;
	}

	public void start() {
		accumulated_ = 0.0;
		start_time_ = System.currentTimeMillis();
	}

	// returns accumulated time in milliseconds. 
	public double stop() 
	{
		long end_time = System.currentTimeMillis();
		long milliseconds = end_time - start_time_;
		time_ = milliseconds + accumulated_;
		accumulated_ = time_;
		return time_;
	}

	public void resume()
	{
		start_time_ = System.currentTimeMillis();
	}
	
	public void reset()
	{
		time_ = 0.0;
		accumulated_ = 0.0;
	}

}

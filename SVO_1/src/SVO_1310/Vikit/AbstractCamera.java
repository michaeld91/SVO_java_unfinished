package SVO_1310.Vikit;

//import javax.vecmath.GVector;

import matrix_types.Vector2d;
import matrix_types.Vector3d;
import Jama.Matrix;


public abstract class AbstractCamera {
	
	private int width;
	private int height;
	
	public AbstractCamera(int width, int height)
	{
		this.width = width;
		this.height = height;
	}
	public AbstractCamera(double width, double height)
	{
		this.width = (int) Math.floor(width);
		this.height = (int) Math.floor(height);
	}
	
	public abstract Vector3d cam2world(double element, double element2);
	
	public abstract Vector3d cam2world(Vector2d vec2d);
	
	public abstract Vector2d world2cam(Vector3d xyz_c);
	
	public abstract Vector2d world2cam(Vector2d uv);
	
	// ToDo may need to cast 'obs' values to integers rather than doubles.
	public boolean isInFrame(Vector2d obs) 
	{
		return isInFrame(obs, 0);
	}
	
	public boolean isInFrame(Vector2d obs, int boundary) 
	{
		if(obs.get(0)>= boundary && obs.get(0)<getWidth()-boundary && obs.get(1)>= boundary && obs.get(1)<getHeight()-boundary)
			return true;
		return false;
	}
	
	public boolean isInFrame(Vector2d obs, int boundary,  int level) {
		if(obs.get(0)>= boundary && obs.get(0)<getWidth()/(1<<level)-boundary && obs.get(1)>= boundary && obs.get(1)<getHeight()/(1<<level)-boundary)
			return true;
		return false;
	}
	
	public abstract double errorMultiplier2();
	
	
	public int getWidth()
	{
		return width; 
	}	
	
	public int getHeight()
	{
		return height;
	}
}

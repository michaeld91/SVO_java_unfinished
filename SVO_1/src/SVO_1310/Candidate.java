package SVO_1310;

import matrix_types.Vector2d;

	/// A candidate is a point that projects into the image plane and for which we
	/// will search a matching feature in the image.
public class Candidate {
	


	private Point pt;
	private Vector2d px;
	
	public Candidate(Point pt, Vector2d px)
	{
		this.pt = pt;
		this.px = px;
	}
	
	public Point get_pt()
	{
		return pt;
	}
	public Vector2d get_px()
	{
		return px;
	}
}

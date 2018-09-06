package SVO_1310;

public class PointCandidate {

	private Point point;
	private Feature feature;
	
	public PointCandidate(Point point, Feature feature)
	{
		this.point = point;
		this.feature = feature;
	}
	
	public Point getPoint(){
		return point;	
	}
	public Feature getFeature(){
		return feature;
	}
	public void setNullFeature(){
		feature = null;
	}
}

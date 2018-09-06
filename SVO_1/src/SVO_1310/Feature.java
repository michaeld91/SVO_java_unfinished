package SVO_1310;


import matrix_types.Vector2d;
import matrix_types.Vector3d;
//import Jama.Matrix;

public class Feature {
	private FeatureType type; //{CORNER, EDGELET;}
	private Frame frame;
	private Vector3d f;
	private Vector2d px = new Vector2d();//private Matrix px = new Matrix(1,2);		//changed from Vector2d
	private int level;
	private Point point;
	private Vector2d grad = new Vector2d();		//changed from Vector2d
	//Very simple constructor, just for testing.
	//ToDo : Delete this when no longer needed.
	//	public Feature(Frame frame){
	//		this.frame=frame;
	//		double[][] d = {{1,2,1}};	//for testing. should be dynamic!
	//		f = new Vector3d(d); // Changed from .set(d);
	//	}

	public Feature(Frame _frame, Vector2d _px, int _level){	//_px changed from Vector2d to Matrix
		type=FeatureType.CORNER;
		this.frame = _frame;
		this.px = _px;
		this.f = frame.getCam().cam2world(px);
		this.level =  _level;
		this.point = null;
		double[][] vec = {{1.0,0}};
		this.grad = new Vector2d(vec);	// Changed from this.grad.set(1.0, 0); 


	}

	//C++ version has _f as a Vector3d. Check!!
	public Feature(Frame _frame, Vector2d _px, Vector3d _f, int _level){ //_px changed from Vector2d to Matrix, _f changed from Vector3d to Matrix
		type=FeatureType.CORNER;
		this.frame = _frame;
		this.px = _px;
		this.f = _f;

		this.level =  _level;
		this.point = null;
		double[][] vec = {{1.0,0}};
		this.grad = new Vector2d(vec);//this.grad.set(1.0, 0);		
	}
	//C++ version has _f as a Vector3d. Check!!
	public Feature(Frame _frame, Point point, Vector2d _px, Vector3d _f, int _level){ //_px changed from Vector2d to Matrix, _f changed from GVector to Matrix
		type=FeatureType.CORNER;
		this.frame = _frame;
		this.px = _px;
		this.f = _f;
		this.level =  _level;
		this.point = point;
		double[][] vec = {{1.0,0}};
		this.grad = new Vector2d(vec);//this.grad.set(1.0, 0);		

	}

	public Frame getFrame() { return frame;}
	public Vector3d getF()	{ return f;}
	public Point getPoint() { return point;}
	public FeatureType getType() { return type;}
	public Vector2d getGrad() { return grad; }
	public void setGrad(Vector2d grad)	{ this.grad = grad; }
	public void setType(FeatureType type)
	{
		this.type = type;
	}
	//Required in the Reprojector reprojectCell method line 271~
	public void setPoint(Point point)
	{
		this.point = point;
	}
	public double get_px(int i) {
		// TODO Auto-generated method stub
		if(i==0)return px.get(0);		//return px.getX();
		else if(i==1) return px.get(1);	//return px.getY();
		return i;//should never get to this point!
	}
	public Vector2d get_px(){
		return px;
	}
	public void set_px(Vector2d px)
	{
		this.px = px; 
	}
	public int getLevel()
	{
		return level;
	}
	public void set_F(Vector3d new_f)
	{
		f = new_f;
	}
}

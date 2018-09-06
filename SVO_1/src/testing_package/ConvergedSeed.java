package testing_package;

public class ConvergedSeed {
	private int x_, y_;
	private double depth_, error_;
	
	public ConvergedSeed(int x, int y, double depth, double error)
	{
		this.setX_(x);
		this.setY_(y);
		this.setDepth_(depth);
		this.setError_(error);
	}

	public int getX_() {
		return x_;
	}

	public void setX_(int x_) {
		this.x_ = x_;
	}

	public int getY_() {
		return y_;
	}

	public void setY_(int y_) {
		this.y_ = y_;
	}

	public double getError_() {
		return error_;
	}

	public void setError_(double error_) {
		this.error_ = error_;
	}

	public double getDepth_() {
		return depth_;
	}

	public void setDepth_(double depth_) {
		this.depth_ = depth_;
	}
}

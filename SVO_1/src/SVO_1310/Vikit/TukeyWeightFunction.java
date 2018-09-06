package SVO_1310.Vikit;

public class TukeyWeightFunction {
	private float b_square;
	private final float DEFAULT_B;


	// ToDo :: Set float b as -1 if you wish to use the DEFAULT_B
	public TukeyWeightFunction(float b)
	{
		DEFAULT_B = 4.6851f;
		if(b==-1)
		{
			b=DEFAULT_B;
		}
		configure(b);
	}
	public void configure(float param)
	{
		b_square = param * param;
	}
	public double value(double x) 	// C++ Vikit uses float
	{
		float x_square = (float) (x * x);
		if(x_square < b_square)
		{
			float tmp = 1.0f - x_square / b_square;
			return tmp * tmp;
		}
		else
			return 0.0f;
	}

}

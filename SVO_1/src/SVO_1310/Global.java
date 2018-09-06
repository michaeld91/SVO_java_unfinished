package SVO_1310;

// As this class is fairly sparse without the inclusion of the c++ ' #define ' lines,
// it may be sensible to absorb this into Config?
public class Global {
	
	public static double EPS = 0.0000000001;
	
	// converts a byte[] to an int[].
	public static int[] toIntArray(byte buf[]) 
	{
		int[] int_array = new int[buf.length];
		for(int i=0; i<buf.length; i++)
		{
			int_array[i] = buf[i];
		}
		return int_array;
	}
	// converts an int[] to an int[][].
	public static int[][] convert_from_array_to_double_array(int[] single_array,int data_width_1, int data_height_1)
	{

//		System.out.println("data_width_1= "+data_width_1);
//		System.out.println("data_height_1= "+data_height_1);

		int[][] double_array = new int[data_width_1][data_height_1];
		int index = 0;
		for(int i=0; i< data_width_1; i++)
			for(int j=0; j< data_height_1; j++)
			{
				double_array[i][j] = single_array[index];
				index++;
			}
		
		return double_array;
	}
	// prints out a int[][]. For testing.
	public static void print_double_array(int[][] double_array, int data_width, int data_height)
	{
		for(int i=0; i<data_height; i++)
		{
			for(int j=0; j< data_width; j++)
			{
				System.out.print(double_array[i][j]+" ");
			}
			System.out.print("\n");
		}
	}

	// prints out a int[]. For testing.
	public static void print_single_array(int[] single)
	{
		for(int val: single)
			System.out.print(val+" ");

		System.out.print("\n\n");
	}
}

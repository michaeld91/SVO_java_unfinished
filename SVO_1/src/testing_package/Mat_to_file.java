package testing_package;

import java.io.FileNotFoundException;
import java.io.PrintWriter;
import java.io.UnsupportedEncodingException;
import org.opencv.core.Mat;

public class Mat_to_file {

	public Mat_to_file(Mat mat, String file)
	{
		PrintWriter writer;
		try {
			writer = new PrintWriter(file, "UTF-8");
			for(int y = 0; y<mat.rows(); y++)
			{
				for(int x = 0; x<mat.rows(); y++)
				{			
					writer.print(mat.get(y, x)[0]+"\t");
				}
				writer.print("\n");
			}
			writer.close();
		} catch (FileNotFoundException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		} catch (UnsupportedEncodingException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}

	}
}

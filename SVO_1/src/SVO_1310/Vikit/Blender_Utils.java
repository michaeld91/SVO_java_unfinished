package SVO_1310.Vikit;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.io.InputStream;
import java.util.Scanner;

import matrix_types.Vector2d;

import org.opencv.core.CvType;
import org.opencv.core.Mat;

public class Blender_Utils {
	// Note that file_name must also contain the location, ie ~/Documents/svo/etc...
	public static Mat loadBlenderDepthMap(String file_name, AbstractCamera cam)//, Mat img)
	{
		Mat img = new Mat(cam.getHeight(), cam.getWidth(), CvType.CV_32FC1); //CvType.CV_8UC1); //
// 		To Be Deleted
// 		try
//		{
//			InputStream file_stream = new FileInputStream(file_name);
//
//			int img_ptr = 0;	//   float * img_ptr = img.ptr<float>();
//			float depth;
//			for(int y = 0; y<cam.getHeight(); y++)
//			{
//				for(int x = 0; x < cam.getWidth(); x++, img_ptr++)
//				{
//					depth = file_stream.read();
//					// blender:
////					if(x==12&&y==18)
////						System.out.println("depth = "+depth);
//					Vector2d uv = math_utils.project2d(cam.cam2world(x, y));
//					img.put(y, x, depth * Math.sqrt(uv.get(0)*uv.get(0) + uv.get(1)*uv.get(1) + 1.0));
//					// povray
//					// img_ptr = depth/100.0; // depth is in cm and we want m.
//// ToDo The following causes a logic error
////					if(depth == -1 && x != cam.getWidth()-1 && y != cam.getHeight()-1){	// ToDo depth == '\n' has been changed to depth == 0. The ascii value of \n being 10 causes issues when the depth = 10.
////						System.out.println(depth=='\n');
////						System.out.println("depth = "+depth+" x = "+x+" cam width - 1 = "+(cam.getWidth()-1) + "y = "+y+" cam.getHeight()-1 " + (cam.getHeight()-1));
////						System.out.println("WARNING: did not read the full depthmap. \n");
////					}
//				}
//			}
//		}

		try
		{
			Scanner scan = new Scanner(new File(file_name));
			int img_ptr = 0;
			float depth;
			for(int y = 0; y<cam.getHeight(); y++)
			{
				for(int x = 0; x < cam.getWidth(); x++, img_ptr++)
				{
					depth = scan.nextFloat();
//					System.out.println("depth = "+depth);
					Vector2d uv = math_utils.project2d(cam.cam2world(x, y));
					img.put(y, x, depth * Math.sqrt(uv.get(0)*uv.get(0) + uv.get(1)*uv.get(1) + 1.0));
				}
			}
			scan.close();
		}
		catch (FileNotFoundException E)
		{
			System.out.println("WARNING: unable to find the file.\n");

		}		
		catch (IOException E)
		{
			System.out.println("WARNING: unable to read the file.\n");

		}
		return img;
	}
}

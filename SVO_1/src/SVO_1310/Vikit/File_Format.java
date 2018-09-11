package SVO_1310.Vikit;

import java.io.IOException;
import java.io.OutputStream;

public class File_Format {

	public OutputStream output_line(OutputStream out, ImageNameAndPose gt)
	{
		String output_line = gt.getTimestamp_() + " " + gt.getImage_name_() + " " + 
				gt.getT_().get(0) + " " + gt.getT_().get(1) + " " + gt.getT_().get(2) + " " + 
				gt.getQ_().get_quaternion().get(0) + " " + gt.getQ_().get_quaternion().get(3) + " " + gt.getQ_().get_quaternion().get(4) + " " + gt.getQ_().get_quaternion().get(1)+ " ";

		try {
			out.write(output_line.getBytes());
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		return out;
	}
}

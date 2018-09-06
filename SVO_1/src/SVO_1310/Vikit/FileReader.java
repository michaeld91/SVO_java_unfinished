package SVO_1310.Vikit;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.InputStream;
import java.util.ArrayList;
import java.util.Scanner;

public class FileReader {

	private boolean hasEntry_;
	private String file_;
	private InputStream file_stream_;
	public FileReader(String file) throws FileNotFoundException // Ensure file contains full location also!
	{
		hasEntry_ = false;
		this.file_ = file;
		this.file_stream_ = new FileInputStream(file);
	}

	//Automatically skips comments so i have removed the 'skipComments()' method.
	public ArrayList<ImageNameAndPose> readAllEntries() throws FileNotFoundException
	{
		Scanner scan = new Scanner(new File(file_));
		ArrayList<ImageNameAndPose> sequence = new ArrayList<ImageNameAndPose>();
		
		while(scan.hasNextLine())
		{
			String new_line = scan.nextLine();
			if(new_line.startsWith("#"))
			{
				continue;
			}
//			System.out.println(new_line);
			ImageNameAndPose newINAP = new ImageNameAndPose(new_line);
//			newINAP.getQ_().print();
			sequence.add(newINAP);
//			sequence.get(sequence.size()-1).getQ_().print();
		}
//		for(int i = 0; i<sequence.size(); i++)
//		{
//			sequence.get(i).getQ_().print();
//		}
		scan.close();
		return sequence;
	}



}

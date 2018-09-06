package testing_package;

import java.io.BufferedReader;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;

import matrix_types.Vector2d;
import SVO_1310.Feature;
import SVO_1310.Frame;

public class Testing_utilities {


	
	   public static void read_features(Frame frame_)
	   {
		   ArrayList<Feature> feats = new ArrayList<Feature>();
	        // The name of the file to open.
	        String fileName = "/home/michael/Documents/SVO_datasets/sin2_tex2_h1_v8_d/svo_feature_points.txt";

	        // This will reference one line at a time
	        String line = null;

	        try {
	            // FileReader reads text files in the default encoding.
	            FileReader fileReader = 
	                new FileReader(fileName);

	            // Always wrap FileReader in BufferedReader.
	            BufferedReader bufferedReader = 
	                new BufferedReader(fileReader);

	            while((line = bufferedReader.readLine()) != null) {
//	                System.out.println(line);
	                String[] fields = line.split(",");
//	                System.out.println(fields[0]);
//	                System.out.println(fields[1]);
	                Vector2d vec = new Vector2d();
	                vec.set(Double.parseDouble(fields[0]),Double.parseDouble(fields[1]));
	                Feature feat = new Feature(frame_, vec, 0);
//	                System.out.println("Feature = " + feat.get_px(0)+","+feat.get_px(1));
	                feats.add(feat);
	            }   

	            // Always close files.
	            bufferedReader.close();         
	        }
	        catch(FileNotFoundException ex) {
	            System.out.println(
	                "Unable to open file '" + 
	                fileName + "'");                
	        }
	        catch(IOException ex) {
	            System.out.println(
	                "Error reading file '" 
	                + fileName + "'");                  
	            // Or we could just do this: 
	            // ex.printStackTrace();
	        }
	        frame_.setFts(feats);
	    }
	   
	   public static ArrayList<Vector2d> read_gaussian_features()
	   {
		   ArrayList<Vector2d> gaus_vec = new ArrayList<Vector2d>();
	        // The name of the file to open.
	        String fileName = "/home/michael/Documents/SVO_datasets/sin2_tex2_h1_v8_d/svo_feature_points_after_gaussian.txt";

	        // This will reference one line at a time
	        String line = null;

	        try {
	            // FileReader reads text files in the default encoding.
	            FileReader fileReader = 
	                new FileReader(fileName);

	            // Always wrap FileReader in BufferedReader.
	            BufferedReader bufferedReader = 
	                new BufferedReader(fileReader);

	            while((line = bufferedReader.readLine()) != null) {
//	                System.out.println(line);
	                String[] fields = line.split(",");
//	                System.out.println(fields[0]);
//	                System.out.println(fields[1]);
	                Vector2d vec = new Vector2d();
	                vec.set(Double.parseDouble(fields[0]),Double.parseDouble(fields[1]));
//	                System.out.println("Feature = " + feat.get_px(0)+","+feat.get_px(1));
	                gaus_vec.add(vec);
	            }   

	            // Always close files.
	            bufferedReader.close();         
	        }
	        catch(FileNotFoundException ex) {
	            System.out.println(
	                "Unable to open file '" + 
	                fileName + "'");                
	        }
	        catch(IOException ex) {
	            System.out.println(
	                "Error reading file '" 
	                + fileName + "'");                  
	            // Or we could just do this: 
	            // ex.printStackTrace();
	        }
	        return gaus_vec;
	    }
}

package SVO_1310.Vikit;

import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.OutputStream;
import java.io.PrintStream;
import java.util.HashMap;
import java.util.Map.Entry;

import org.apache.commons.math3.util.MultidimensionalCounter.Iterator;

import SVO_1310.Map;

public class PerformanceMonitor {

	private HashMap<String, Timer> timers_ = new HashMap<String, Timer>();
	private HashMap<String, LogItem> logs_ = new HashMap<String, LogItem>(); 
	private String trace_name_;			//	name of the thread that started the performance monitor
	private String trace_dir_;			//	directory where the logfiles are saved
	private OutputStream ofs_;					// 	for writing to an output file.
	
	public void init(String trace_name, String trace_dir)
	{
		trace_name_ = trace_name;
		trace_dir_ = trace_dir;
		String filename = trace_dir +".csv";			// ToDo may need to change from csv to txt or alternative.
		try{
		ofs_ = new FileOutputStream(filename);													
		}
		catch(FileNotFoundException E)
		{
			PrintStream myOutputFile = new PrintStream(ofs_);
			myOutputFile.println("Tracefile = "+filename);
			System.out.println("Cannot open "+ filename + "for writing");
		}
		traceHeader();
	}
	public void addTimer(String name)
	{
		timers_.put(name, new Timer());
	}
	public void addLog(String name)
	{
		logs_.put(name, new LogItem());
	}
	public void writeToFile()
	{
		trace();
		
		for(Entry<String, Timer> entry: timers_.entrySet())
		{
			entry.getValue().reset();
		}
		for(Entry<String, LogItem> entry: logs_.entrySet())
		{
			entry.getValue().setSet(false);
			entry.getValue().setData(-1);
		}
	}
	
	private void traceHeader() {
		try
		{
			boolean first_value = true;
			PrintStream myOutputFile = new PrintStream(ofs_);
			for(Entry<String, Timer> it: timers_.entrySet())
			{
				if(first_value)
				{
					myOutputFile.print(it.getKey());
					first_value = false;
				}
				else
				{
					myOutputFile.print(","+it.getKey());
				}
			}
			
			for(Entry<String, LogItem> it: logs_.entrySet())
			{
				if(first_value)
				{
					myOutputFile.print(it.getKey());
					first_value = false;
				}
				else
				{
					myOutputFile.print(","+it.getKey());
				}
			}
			
			myOutputFile.print("\n");
		}
		catch( Exception E)
		{
			throw new RuntimeException("Performance monitor not correctly initialized");
		}

		
	}
	
	
	
	public void trace()
	{
		char[] buffer = new char[128];
		boolean first_value = true;
		try
		{
			//ofs_.precision(15);
			//ofs_.setf(fixed, floatfield);
			
			PrintStream myOutputFile = new PrintStream(ofs_);
			for(Entry<String, Timer> it: timers_.entrySet())
			{
				if(first_value)
				{
					myOutputFile.print(it.getValue().getTime());
					first_value = false;
				}
				else
				{
					myOutputFile.print(","+it.getValue().getTime());
				}
			}
			
			for(Entry<String, LogItem> it: logs_.entrySet())
			{
				if(first_value)
				{
					myOutputFile.print(it.getValue().getData());
					first_value = false;
				}
				else
				{
					myOutputFile.print(","+it.getValue().getData());
				}
			}
			
			myOutputFile.print("\n");
		}
		catch( Exception E)
		{
			throw new RuntimeException("Performance monitor not correctly initialized");
		}
	}
	
}




































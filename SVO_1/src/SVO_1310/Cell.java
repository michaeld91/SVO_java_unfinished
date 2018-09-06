package SVO_1310;

import java.util.ArrayList;

public class Cell {
	private ArrayList<Candidate> candidates = new ArrayList<Candidate>();
	
	public Cell()
	{
		
	}
	public void add(Candidate cand)
	{
		candidates.add(cand);
	}
	public int candidatesSize()
	{
		return candidates.size();
	}
	public Candidate get(int i)
	{
		return candidates.get(i);
	}
	public void delete(int i)
	{
		candidates.remove(i);
	}
}

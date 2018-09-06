package SVO_1310.Vikit;

import java.util.ArrayList;

// ToDo: Removing the generic T for ease.
public class RingBuffer {

	private ArrayList<Double> values;
	
	private int begin_;
	private int end_;
	private int num_elem_ ;
	private int arr_size_;
	
	public RingBuffer(int size)
	{
		values = new ArrayList<Double>(size);
		begin_ = 0;
		end_ = -1;		// the index of the last object in the arrayList
		num_elem_ = 0;
		arr_size_ = size;
	}
	
	public boolean empty()
	{
		return values.isEmpty();
	}
	
	// ToDo should it just be 10 every time?
	public int size()
	{
		return values.size();
	}
	
	public void push_back(Double elem)
	{
		if(num_elem_ < arr_size_)
		{
			end_++;
			values.add(end_,elem);
			num_elem_++;			
		}
		// This implements the ring part.
		// As the number of elements is equal to the size of the arrayList, place the 
		// new elem in the position of the oldest elem, replacing it. This ensures the n newest elements are used.
		else {
			end_ = (end_+1)%arr_size_;
			begin_ = (begin_+1)%arr_size_;
			values.add(end_, elem);
		}
	}
	public Double get(int i)
	{
		return values.get(i);
	}
	public Double getSum()
	{	
		
		double sum = 0;
		for(int i = 0; i< num_elem_; i++)
			sum += (double)values.get(i);
		return sum;
	}
	public double getMean()
	{
		if(num_elem_ == 0)
			return 0;
		return getSum()/num_elem_;
	}
	
}

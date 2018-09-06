package SVO_1310.Vikit;

public class LogItem {
	private double data;
	private boolean set;
	
	public LogItem(){}
	public LogItem(double d, boolean s)
	{
		data = d;
		set = s;
	}
	public double getData() {
		return data;
	}
	public void setData(double data) {
		this.data = data;
	}
	public boolean isSet() {
		return set;
	}
	public void setSet(boolean set) {
		this.set = set;
	}
}

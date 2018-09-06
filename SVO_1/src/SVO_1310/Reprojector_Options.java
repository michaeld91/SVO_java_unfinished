package SVO_1310;

//The options for the Reprojector Class
public class Reprojector_Options {

	  /// Reprojector config parameters
//	  struct Options {
//	    size_t max_n_kfs;   //!< max number of keyframes to reproject from
//	    bool find_match_direct;
//	    Options()
//	    : max_n_kfs(10),
//	      find_match_direct(true)
//	    {}
//	  }
	private int max_n_kfs;	// max number of keyframes to reproject from
	private boolean find_match_direct;
	
	public Reprojector_Options()
	{
		max_n_kfs = 10;
		find_match_direct = true;
	}

	public int getMax_n_kfs() {
		return max_n_kfs;
	}

	public void setMax_n_kfs(int max_n_kfs) {
		this.max_n_kfs = max_n_kfs;
	}

	public boolean getFind_match_direct() {
		return find_match_direct;
	}

	public void setFind_match_direct(boolean find_match_direct) {
		this.find_match_direct = find_match_direct;
	}
}

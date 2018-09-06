package SVO_1310.Vikit;

// Zero Mean Sum of Squared Differences Cost
public class ZMSSD {

	private static int half_patch_size;
	private static int patch_size;
	private static int patch_area_;
	private static int threshold_; 
	byte[] ref_patch_; 
	int sumA_, sumAA_;


	public ZMSSD(int half_patch_size)
	{
		this.half_patch_size = half_patch_size;
		patch_size = 2*half_patch_size;
		patch_area_ = patch_size*patch_size;
		threshold_ = 2000*patch_area_;
	}

	public void set_ZMSSD(byte[] ref_patch)
	{
		this.ref_patch_ = ref_patch;
		//		this.half_patch_size = HALF_PATCH_SIZE;
		int sumA_uint = 0;
		int sumAA_uint = 0;
		for(int r = 0; r < patch_area_; r++)
		{
			int n = ref_patch_[r];	// suggests that halfPatchSize should be an array
			sumA_uint += n;
			sumAA_uint += n*n;
		}
		sumA_ = sumA_uint;
		sumAA_ = sumAA_uint;		
	}


	public int getThreshold()
	{
		return threshold_;
	}

	public int computeScore(int[] cur_patch)
	{
		int sumB_uint = 0;
		int sumBB_uint = 0;
		int sumAB_uint = 0;

		for(int r = 0; r < patch_area_; r++)
		{
			final int cur_pixel = cur_patch[r];
			sumB_uint  += cur_pixel;
			sumBB_uint += cur_pixel*cur_pixel;
			sumAB_uint += cur_pixel * ref_patch_[r];
		}
		int sumB = sumB_uint;
		int sumBB = sumBB_uint;
		int sumAB = sumAB_uint;
		return sumAA_ - 2*sumAB + sumBB - (sumA_*sumA_ - 2*sumA_*sumB + sumB*sumB)/patch_area_;

	}


	public int computeScore(byte[] cur_patch, int stride, int starting_index) 	//changed int[] to byte[] ToDo ensure this doesnt cause further issues
	{
//		System.out.println("cur_patch_size = "+cur_patch.length);
		int sumB, sumBB, sumAB;

		// #if __SSE2__ implementation has been skipped
		int sumB_uint = 0;
		int sumBB_uint = 0;
		int sumAB_uint = 0;
		for(int y=0, r=0; y < patch_size; ++y)
		{
			int cur_patch_index = y*stride;
			for(int x=0; x < patch_size; ++x, ++r)
			{
				int cur_px = cur_patch[cur_patch_index+x+starting_index];// ToDo starting_index field is an addition to counter for the border. Check it is required!
				sumB_uint  += cur_px;
				sumBB_uint += cur_px * cur_px;
				sumAB_uint += cur_px * ref_patch_[r];
			}
		}
		sumB = sumB_uint;
		sumBB = sumBB_uint;
		sumAB = sumAB_uint;

		return sumAA_ - 2*sumAB + sumBB - (sumA_*sumA_ - 2*sumA_*sumB + sumB*sumB)/patch_area_;

	}
	// For testing
	public int get_patch_size()
	{
		return patch_size;
	}
}

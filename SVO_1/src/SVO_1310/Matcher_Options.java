package SVO_1310;

public class Matcher_Options {

	private boolean align_1d;							// in epipolar search: align patch 1D along epipolar line
	private int align_max_iter;							// number of iterations for aligning the feature patches in gauss newton
	private double max_epi_length_optim;				// max length of epipolar line to skip epipolar search and directly go to img align
	private int max_epi_search_steps;					// max number of evaluations along epipolar line
	private boolean subpix_refinement;					// do gauss newton feature patch alignment after epipolar search
	private boolean epi_search_edgelet_filtering;
	private double epi_search_edgelet_max_angle;

	public Matcher_Options()
	{
		align_1d = false;
		align_max_iter = 10;
		max_epi_length_optim = 2.0;
		max_epi_search_steps = 1000;
		subpix_refinement = true;
		epi_search_edgelet_filtering = true;
		epi_search_edgelet_max_angle = 0.7;
	}

	public boolean getAlign_1d() {
		return align_1d;
	}

	public int getAlign_max_iter() {
		return align_max_iter;
	}

	public double getMax_epi_length_optim() {
		return max_epi_length_optim;
	}

	public int getMax_epi_search_steps() {
		return max_epi_search_steps;
	}

	public boolean getSubpix_refinement() {
		return subpix_refinement;
	}

	public boolean getEpi_search_edgelet_filtering() {
		return epi_search_edgelet_filtering;
	}

	public double getEpi_search_edgelet_max_angle() {
		return epi_search_edgelet_max_angle;
	}
 	
}

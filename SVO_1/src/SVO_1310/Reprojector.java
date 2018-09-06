package SVO_1310;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Iterator;
import java.util.TreeMap;

import matrix_types.Vector2d;

import SVO_1310.Vikit.AbstractCamera;

public class Reprojector {
	private Reprojector_Options options_;
	private int n_matches_;
	private int n_trials_;

	private Grid grid_;
	private Matcher matcher_;
	private Map map_;

	public Reprojector(AbstractCamera cam, Map map){
		this.map_ = map;
		initializeGrid(cam);
	}


	/// Project points from the map into the image. First finds keyframes with
	/// overlapping field of view and projects only those map-points.
	/// FramePtr was in the C++, ensure this does the exact same thing!!
	
	// ToDo should return the ArrayList<FrameDoublePair> overlap_kfs rather than void
	public void reprojectMap(Frame frame, ArrayList<FrameDoublePair> overlap_kfs)//TreeMap<Frame, Integer> overlap_kfs)
	{
		resetGrid();

		// Identify those keyframes which share a common field of view

		//SVO_START_TIMER("reproject_kfs");
		ArrayList<FrameDoublePair> close_kfs = new ArrayList<FrameDoublePair>();//a list of FramePtr double pairs,
		map_.getCloseKeyframes(frame, close_kfs);

		// ToDo Sort KFs with overlap according to their closeness
		//close_kfs.sort(boost::bind(&std::pair<FramePtr, double>::second, _1) < boost::bind(&std::pair<FramePtr, double>::second, _2));
		Collections.sort(close_kfs);	// ToDo ensure close_kfs is sorted in the correct manner


		// Reproject all mappoints of the closest N kfs with overlap. We only store
		// in which grid cell the points fall.
		int n = 0;
		overlap_kfs.ensureCapacity(options_.getMax_n_kfs());
		//for(auto it_frame=close_kfs.begin(), ite_frame=close_kfs.end(); it_frame!=ite_frame && n<options_.max_n_kfs; ++it_frame, ++n)
		//Check does the for loop below do as required?
		for(int i=0; i<close_kfs.size(); i++)
		{
			Frame ref_frame = close_kfs.get(i).getFrame();
			overlap_kfs.add(new FrameDoublePair(ref_frame,0.0));

			// Try to reproject each mappoint that the other KF observes
			for(int j=0;j<ref_frame.getFts_().size(); j++)
			{
				// Check if the feature has a mapppoint assigned
				if((ref_frame.getFts_().get(i).getPoint() == null))
				{
					continue;
				}

				// Make sure we project a point only once
				if((ref_frame.getFts_().get(i).getPoint().get_last_projected_kf_id()== frame.getId_()))
				{
					continue;
				}
				ref_frame.getFts_().get(i).getPoint().set_last_projected_kf_id(frame.getId_());
				if(reprojectPoint(frame, ref_frame.getFts_().get(i).getPoint()))
				{
					overlap_kfs.get(overlap_kfs.size()).increaseDouble();
				}
			}
			//SVO_STOP_TIMER("reproject_kfs");
		}

		// Now project all point candidates
		//SVO_START_TIMER("reproject_candidates");
		{
			// boost::unique_lock<boost::mutex> lock(map_.point_candidates_.mut_);
			Iterator<PointCandidate> it =map_.getPoint_Candidates().getPointCandidateList().iterator();

			while(it.hasNext())
			{	
				PointCandidate pc = it.next();
				if(!reprojectPoint(frame, pc.getPoint()))
				{
					pc.getPoint().set_n_failed_reproj_(pc.getPoint().get_n_failed_reproj_()+3);
					if(pc.getPoint().get_n_failed_reproj_() > 30)
					{
						map_.getPoint_Candidates().deleteCandidate(pc);
						//Not very clear with this line:: it = map_.point_candidates_.candidates_.erase(it);
						map_.getPoint_Candidates().getPointCandidateList().remove(pc);//ToDo
						continue;
					}
				}
			}
		} //unlock the mutex when out of scope
		//SVO_STOP_TIMER("reproject_candidates");
		
		// Now we go through each grid cell and select one point to match.
		// At the end, we should have at maximum one reprojected point per cell.
		//SVO_START_TIMER("feature_align");
		for(int i=0; i<grid_.getCells().size(); i++)
		{
			// we prefer good quality points over those of unknown quality (more likely to match)
			// and unknown quality over candidates (position not optimized)
			if(reprojectCell(grid_.getCells().getAllCells().get(grid_.getCell_order().get(i)), frame))
			{
				n_matches_++;
			}
			if(n_matches_ > Config.getMaxFts())
			{
				break;
			}
		}
		//SVO_STOP_TIMER("feature_align");
	}

	// ToDo ensure these implementations equate to the same thing. They appear to under first inspection.
	private static boolean pointQualityComparator(Candidate lhs, Candidate rhs)
	{
//		if(lhs.get_pt().getType() > rhs.get_pt().getType())
//		{
//			return true;
//		}
//		return false;

		if(lhs.get_pt().getType()==PointType.TYPE_GOOD && rhs.get_pt().getType()!=PointType.TYPE_GOOD)
			return true;
		else if(lhs.get_pt().getType()==PointType.TYPE_UNKNOWN && rhs.get_pt().getType()!=PointType.TYPE_UNKNOWN && rhs.get_pt().getType()!=PointType.TYPE_GOOD)
			return true;
		else if(lhs.get_pt().getType()==PointType.TYPE_CANDIDATE && rhs.get_pt().getType()==PointType.TYPE_DELETED)
			return true;
		else
			return false;
		
		
		
	}
	private void initializeGrid(AbstractCamera cam)
	{
		grid_.setCell_size(Config.getGridSize());
		grid_.setGrid_n_cols((int)(cam.getWidth()/grid_.getCell_size())+1);	//  Cast to int rounds down, plus one returns same as ceil
		grid_.setGrid_n_rows((int)(cam.getHeight()/grid_.getCell_size())+1);	//  Cast to int rounds down, plus one returns same as ceil
		//resize cells to grid_n_cols*grid_n_rows
		grid_.getCells().getAllCells().ensureCapacity(grid_.getGrid_n_cols()*grid_.getGrid_n_rows());
		grid_.getCell_order().ensureCapacity(grid_.getGrid_n_cols()*grid_.getGrid_n_rows());

		for(int i = 0; i<(grid_.getGrid_n_cols()*grid_.getGrid_n_rows()); i++)
		{
			Cell cell = new Cell();
			grid_.getCells().addCell(cell);	
			grid_.getCell_order().set(i,i);
		}
		Collections.shuffle(grid_.getCell_order());

	}
	private void resetGrid()
	{
		n_matches_ = 0;
		n_trials_ = 0;
		// Below was a for loop in the c++, clearing each item individually
		grid_.getCells().getAllCells().clear();

	}

	//FramePtr was in the C++, ensure this does the exact same thing!!
	private boolean reprojectCell(Cell cell, Frame frame)
	{
		//ToDo cell.sort(.cell.........)
		for(int i=0;i<cell.candidatesSize();i++)
		{
			//Point point_cell_i = cell.get(i).get_pt(); //may be more efficient but requires testing that it still does as intended
			n_trials_++;
			if(cell.get(i).get_pt().getType()==PointType.TYPE_DELETED)
			{
				cell.delete(i);// removes the Point at index i
			}

			boolean found_match = true;
			if(options_.getFind_match_direct())
			{
				found_match = matcher_.findMatchDirect(cell.get(i).get_pt(), frame, cell.get(i).get_px());
			}
			if(!found_match)
			{
				cell.get(i).get_pt().set_n_failed_reproj_(cell.get(i).get_pt().get_n_failed_reproj_()+1);
				if(cell.get(i).get_pt().getType() == PointType.TYPE_UNKNOWN && cell.get(i).get_pt().get_n_failed_reproj_()>15)
				{
					map_.safeDeletePoint(cell.get(i).get_pt());
				}
				if(cell.get(i).get_pt().getType() == PointType.TYPE_CANDIDATE && cell.get(i).get_pt().get_n_failed_reproj_()>30)
				{
					map_.getPoint_Candidates().deleteCandidatePoint(cell.get(i).get_pt());
				}
				cell.delete(i); //is this the same as it = cell.erase(it);
			}
			cell.get(i).get_pt().set_n_succeeded_reproj_(cell.get(i).get_pt().get_n_succeeded_reproj_()+1);
			if(cell.get(i).get_pt().getType() == PointType.TYPE_UNKNOWN && cell.get(i).get_pt().get_n_succeeded_reproj_()>10)
			{
				cell.get(i).get_pt().setType(PointType.TYPE_GOOD);				
			}

			Feature new_feature = new Feature(frame, cell.get(i).get_px(), matcher_.get_search_level_());
			frame.addFeature(new_feature);

			// Here we add a reference in the feature to the 3D point, the other way 
			// round is only done if this frame is selected as keyframe.
			new_feature.setPoint(cell.get(i).get_pt());

			if(matcher_.get_ref_ftr_().getType() == FeatureType.EDGELET)
			{
				//check that the assumed get methods match other uses of them
				new_feature.setType(FeatureType.EDGELET);
				Vector2d ref_ftr_times_A_cur_ref = new Vector2d(matcher_.get_ref_ftr_().getGrad().times(matcher_.get_A_cur_ref_()).getArray());
				ref_ftr_times_A_cur_ref.times(1/ref_ftr_times_A_cur_ref.normF());
				new_feature.setGrad(ref_ftr_times_A_cur_ref);
			}

			// If the keyframe is selected and we reproject the rest, we don't have to check this point anymore.
			cell.delete(i);

			// Maximum one point per cell.
			return true;
		}
		return false;
	}


	//FramePtr was in the C++, ensure this does the exact same thing!!
	private boolean reprojectPoint(Frame frame, Point point)
	{
		Vector2d px = new Vector2d(frame.w2c(point.getPos()).getArray());
		// 8 px is the patch size in the matcher
		if(frame.getCam().isInFrame(px, 8))
		{
			int k = (int)(px.get(1)/grid_.getCell_size())*grid_.getGrid_n_cols()
					+ (int)(px.get(0)/grid_.getCell_size());

			Candidate cand = new Candidate(point, px);
			//Adds a new candidate to the end of the ArrayList held within
			//the Cell at position k in the CandidateGrid Cells.
			grid_.getCells().at(k).add(cand);
			return true;
		}
		return false;
	}
	public int get_n_matches_()
	{
		return n_matches_;
	}
	public int get_n_trials_()
	{
		return n_trials_;
	}
}

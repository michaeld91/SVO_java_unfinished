package SVO_1310;

import java.util.ArrayList;
import java.util.Iterator;

/// Container for converged 3D points that are not already assigned to two keyframes.

public class MapPointCandidates {

	private ArrayList<PointCandidate> PointCandidateList = new ArrayList<PointCandidate>();	//known as candidates_ in C++
	//C++::The depth-filter is running in a parallel thread and fills the candidate list.
	//C++::This mutex controls concurrent access to point_candidates.
	//boost::mutex mut_; 

	//C++:: Candidate points are created from converged seeds.
	//C++:: Until the next keyframe, these points can be used for reprojection and pose optimization.

	//PointCandidateList candidates_; unnecessary if using the HashMap.

	private ArrayList<Point> trash_points_ = new ArrayList<Point>();

	public MapPointCandidates()
	{}

	//ToDo Destructor in C++ calls the reset() method. Not included

	/// Add a candidate point.
	public void newCandidatePoint(Point point, double depth_sigma2)
	{
		//	This line gets the type of a point and sets it equal to the enum TYPE_CANDIDATE;
		point.setType(PointType.TYPE_CANDIDATE);
		//	boost::unique_lock<boost::mutex> lock(mut_);				ToDo requires multi-thread work
		synchronized(this)
		{
			PointCandidate new_pc = new PointCandidate(point, point.getObs_().get(0));

			PointCandidateList.add(new_pc);	
		}
	}

	/// Adds the feature to the frame and deletes candidate from list.
	public synchronized void addCandidatePointToFrame(Frame frame){
		//Set a mutex lock. ToDo, requires multi-thread work.
		for(PointCandidate pc: PointCandidateList)
		{
			if(pc.getPoint().getObs_().get(0).getFrame() == frame)
			{
				pc.getPoint().setType(PointType.TYPE_UNKNOWN);
				pc.getPoint().set_n_failed_reproj_(0);
				pc.getFeature().getFrame().addFeature(pc.getFeature());	//ToDo strange line, add the feature object to a field owned by the feature?
				// ToDo this line it = candidates_.erase(it);
			}
		}
	}

	/// Remove a candidate point from the list of candidates.
	public synchronized boolean deleteCandidatePoint(Point point){
		//Set a mutex lock. ToDo  requires multi-thread work.
		for(PointCandidate pc : PointCandidateList)
		{
			if(pc.getPoint() == point){
				deleteCandidate(pc);
				PointCandidateList.remove(pc); 	// ToDo check this is equivalent to candidates_.erase(it);
				return true;				
			}
		}
		return false;
	}

	/// Remove all candidates that belong to a frame.
	public synchronized void removeFrameCandidates(Frame frame){
		//Set a mutex lock. ToDo, requires multi-thread work.

		//Used a for loop rather than an iterator as its easier to delete objects from a list
		for(PointCandidate pc: PointCandidateList)
		{
			if(pc.getFeature().getFrame() == frame)
			{
				deleteCandidate(pc);
				PointCandidateList.remove(pc);
			}
		}

	}

	// Reset the candidate list, remove and delete all points.
	public synchronized void reset(){
		//Set a mutex lock. ToDo, requires multi-thread work. 	boost::unique_lock<boost::mutex> lock(mut_);
		//Is there a need to delete each point, feature then clear the list? Is it plausible to just reset the list?
		PointCandidateList.clear();
	}

	// C++ version uses the field deleteCandidate(PointCandidate& c).
	// sets the Feature to null, sets the type to deleted and adds to trash_points_
	// camera-rig: another frame might still be pointing to the candidate point
	// therefore, we can't delete it right now.
	public void deleteCandidate(PointCandidate c){

		c.setNullFeature(); //perhaps a setFeature method is neccessary? or a setFeatureNull()??
		c.getPoint().setType(PointType.TYPE_DELETED);
		trash_points_.add(c.getPoint());

	}

	public void emptyTrash()
	{
		trash_points_.clear();
	}
	public ArrayList<PointCandidate> getPointCandidateList()
	{
		return PointCandidateList;
	}

}

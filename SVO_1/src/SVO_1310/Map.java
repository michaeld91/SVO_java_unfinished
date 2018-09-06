package SVO_1310;

import java.util.ArrayList;
import java.util.Iterator;

import Sophus.Se3;

import matrix_types.Matrix3d;
import matrix_types.Vector3d;

/// Map object which saves all keyframes which are in a map.

public class Map {

	private ArrayList<Frame> keyframes_ = new ArrayList<Frame>();			//!< List of keyframes in the map.
	// ToDo not sure if trash_points_ is necessary due to automatic garbage collector, or if its needed for visualizer
	private ArrayList<Point> trash_points_;	//!< A deleted point is moved to the trash bin. Now and then this is cleaned. One reason is that the visualizer must remove the points also.
	private MapPointCandidates point_candidates_;
	//	PointCandidate[] candidates_;	//should be a PointCandidateList
	private static Map instance = null;
	
	protected Map()
	{

	}
	public static Map getInstance()
	{
		if(instance == null)
			instance = new Map();
		return instance;
	}
	// Reset the map. Delete all keyframes and reset the frame and point counters.
	public void reset()
	{
		// ToDo reset is called in the C++ destructor, which doesnt exist here.
		keyframes_.clear();
		point_candidates_.reset();
		emptyTrash();
		
	}
	// Delete a point in the map and remove all references in keyframes to it.
	public void safeDeletePoint(Point pt) 
	{
		// Delete references to mappoints in all keyframes
		for(Feature ftr:pt.getObs_())
		{
		// ToDo    ftr->point=NULL;	Dont think this is necessary.
		ftr.getFrame().removeKeyPoint(ftr);	
		}		
		pt.getObs_().clear();
		
		// Delete map point
		deletePoint(pt);
		
	}
	// Moves the point to the trash queue which is cleaned now and then.
	public void deletePoint(Point pt)
	{
		pt.setType(PointType.TYPE_DELETED);
		trash_points_.add(pt);	// ToDo check adds to back
	}
	// Moves the frame to the trash queue which is cleaned now and then.
	public boolean safeDeleteFrame(Frame frame)
	{
		boolean found = false;
		for(Frame kf : keyframes_)
		{
			for(Feature ftr : kf.getFts_())
			{
				removePtFrameRef(kf, ftr);
			}
			keyframes_.remove(kf);
			found = true;
			break;
		}

		point_candidates_.removeFrameCandidates(frame);

		if(found)
			return true;

		//		  SVO_ERROR_STREAM("Tried to delete Keyframe in map which was not there.");
		return false;
	}
	// Remove the references between a point and a frame.
	private void removePtFrameRef(Frame frame, Feature ftr)
	{
		if(ftr.getPoint() == null)
		{
			return;	// mappoint may have been deleted in a previous reference removal
		}
		Point pt = ftr.getPoint();
		if(pt.getObs_().size()<=2)
		{
			// If the reference list of mappoint has only size =2, delete mappoint
			safeDeletePoint(pt);
			return;
		}
		pt.deleteFrameRef(frame);	// Remove reference from map_point
		frame.removeKeyPoint(ftr);	// Check if mp was keyMp in keyframe
	}

	// Add a new keyframe to the map.
	public void addKeyframe(Frame new_keyframe)
	{
		keyframes_.add(new_keyframe);
	}

	// Given a frame, return all keyframes which have an overlapping field of view.
	public void getCloseKeyframes(Frame frame, ArrayList<FrameDoublePair> close_kfs)
	{
		for(Frame kf : keyframes_)
		{
			// check if kf has overlapping field of view with frame, use therefore KeyPoints
			for(Feature  keypoint : kf.getkey_pts_())
			{
				if(keypoint == null)	// ToDo if(keypoint == nullptr)
				{
					continue;
				}
				if(frame.isVisible(keypoint.getPoint().getPos()))	// ToDo keypoint->point->pos_
				{
					close_kfs.add(new FrameDoublePair(kf, frame.getT_f_w_().get_Translation().minus(kf.getT_f_w_().get_Translation()).normF()));
					break;	// this keyframe has an overlapping field of view -> add to close_kfs
				}
			}
		}
	}

	// Return the keyframe which is spatially closest and has overlapping field of view.
	public Frame getClosestKeyframe(Frame frame)
	{
		ArrayList<FrameDoublePair> close_kfs = new ArrayList<FrameDoublePair>();
		getCloseKeyframes(frame, close_kfs);
		if(close_kfs.isEmpty())
		{
			return null;	//ToDo return nullptr;
		}
		
		//	Sort KFs with overlap according to their closeness
		close_kfs.sort(null); // ToDo close_kfs.sort(boost::bind(&std::pair<FramePtr, double>::second, _1) < boost::bind(&std::pair<FramePtr, double>::second, _2));
		
		if(close_kfs.get(0).getFrame()!= frame)
		{
			return close_kfs.get(0).getFrame();
		}
		close_kfs.remove(0);	// ToDo		  close_kfs.pop_front();

		return close_kfs.get(0).getFrame();
	}

	// Return the keyframe which is furthest apart from pos.
	public Frame getFurthestKeyframe(Vector3d pos)
	{
		Frame furthest_kf = null;
		double maxdist = 0.0;
		for(Frame kf : keyframes_)
		{
			double dist = (kf.pos().minus(pos)).normF();
			if(dist > maxdist)
			{
				maxdist = dist;
				furthest_kf = kf;
			}			
		}
		return furthest_kf;
	}

	// This is only called in FrameHandlerMono, so can be easily changed to return Frame instead of a boolean
	public Frame getKeyFrameById(int id)	// ToDo bool getKeyframeById(const int id, FramePtr& frame) const;
	{
		Frame frame = null;
		for(int i=0; i < keyframes_.size(); i++)
		{
			if(keyframes_.get(i).getId_() == id)
			{
				frame = keyframes_.get(i);
				break;
			}
		}
		//return found;
		return frame;
	}

	// Transform the whole map with rotation R, translation t and scale s.
	public void transform(Matrix3d R, Vector3d t, double s)
	{
		Iterator<Frame> frame_it = keyframes_.iterator();
		while(frame_it.hasNext())
		{
			Frame it = frame_it.next();
			
			
			Vector3d pos = new Vector3d();
			pos.set(R.times(s).times(it.pos()).plus(t));
			
			Matrix3d rot = R.times(it.getT_f_w_().rotation_Matrix().inverse());
			it.setT_f_w_(new Se3(rot,pos).inverse());
			for(Feature ftr: it.getFts_())
			{
				if(ftr.getPoint() == null)
					continue;
				if(ftr.getPoint().get_last_published_ts_()== -1000)
					continue;
				ftr.getPoint().set_last_published_ts_(-1000);
				ftr.getPoint().setPos((ftr.getPoint().getPos()).times(R.times(s)).plus(t));		// ToDo ensure that this ordering returns the same result as :
//		      (*ftr)->point->pos_ = s*R*(*ftr)->point->pos_ + t;
			}	
		}
	}

	/// Empty trash bin of deleted keyframes and map points. We don't delete the
	/// points immediately to ensure proper cleanup and to provide the visualizer
	/// a list of objects which must be removed.
	/// ToDo ensure the parts regarding garbage collection are in Java format.
	public void emptyTrash()
	{
		trash_points_.clear();
		point_candidates_.emptyTrash();

	}
	// Return the keyframe which was last inserted in the map.
	public Frame lastKeyframe()
	{
		return keyframes_.get(keyframes_.size());
	}

	// Return the number of keyframes in the map
	public int size()
	{
		return keyframes_.size();
	}

	public MapPointCandidates getPoint_Candidates() {
		return point_candidates_;
	}
	public ArrayList<Frame> get_keyframes()
	{
		return keyframes_;
	}
}
package SVO_1310;

import java.util.ArrayList;

public class Map_debug {
	// ToDo empty class and methods

	public void mapValidation(Map map, int id)
	{
		for(Frame frame: map.get_keyframes())
		{
			frameValidation(frame, id);
		}
	}
	
	//ToDo check System.out behave similarly to printf
	public void frameValidation(Frame frame, int id)
	{
		for(Feature it : frame.getFts_())
		{
			if(it.getPoint()==null)
			continue;
			
			if(it.getPoint().getType()==PointType.TYPE_DELETED)
				System.out.println("ERROR DataValidation "+id+": Referenced point was deleted.\n");
			
			if(it.getPoint().findFrameRef(frame)==null)
				System.out.println("ERROR DataValidation "+id+": Frame has reference but point does not have a reference back.\n");
		
			pointValidation(it.getPoint(), id);		
		}
		
		for(Feature it:frame.getkey_pts_())
		{
			if(it != null)
				if(it.getPoint()==null)
					System.out.println("ERROR DataValidation "+id+": KeyPoints not correct!\n");
		}
	}
	
	//ToDo check System.out behave similarly to printf
	public void pointValidation(Point point, int id)
	{
		for(Feature it: point.getObs_())
		{
			boolean found = false;
			for(Feature it_ftr: it.getFrame().getFts_())
			{
				if(it_ftr.getPoint() == point)
					found = true; break;
			}
			if(!found)
				System.out.println("ERROR DataValidation "+id+": Point "+point.getId()+" has inconsistent reference in frame "+it.getFrame().getId_()+", is candidate = "+point.getType()+"\n");
		}
	}
	public void mapStatistics(Map map)
	{
		// compute the average number of features which each frame observes
		int n_pt_obs = 0;
		for(Frame frame: map.get_keyframes())
			n_pt_obs+= frame.nObs();
		System.out.println("\n\nMap Statistics: Frame avg. point obs = " +n_pt_obs/map.size());
		
		// compute the average number of observations that each point has
		int n_frame_obs = 0;
		int n_pts = 0;
		ArrayList<Point> points = new ArrayList<Point>();
		for(Frame frame: map.get_keyframes())
		{
			for(Feature ftr: frame.getFts_())
			{
				if(ftr.getPoint()==null)
					continue;
				if(points.add(ftr.getPoint()))
				{
					n_pts++;
					n_frame_obs += ftr.getPoint().nRefs();
				}
			}
		}
		
		System.out.println("Map Statistics: Point avg. frame obs = "+ n_frame_obs/n_pts+"\n\n");
		
	}
	
}

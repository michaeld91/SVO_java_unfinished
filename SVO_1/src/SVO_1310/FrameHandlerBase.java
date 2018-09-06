package SVO_1310;

import java.util.ArrayList;
import java.util.Collection;
import java.util.Collections;
import java.util.Comparator;
import java.util.Deque;
import java.util.Iterator;
import java.util.LinkedList;

import SVO_1310.Vikit.PerformanceMonitor;
import SVO_1310.Vikit.RingBuffer;
import SVO_1310.Vikit.Timer;

public class FrameHandlerBase {

	protected Stage stage_;								//!< Current stage of the algorithm.
	private boolean set_reset_;							//!< Flag that the user can set. Will reset the system before the next iteration.
	private boolean set_start_;							//!< Flag the user can set to start the system when the next image is received.
	protected Map map_; 									//!< Map of keyframes created by the slam system.
	private Timer timer_;// VK: Timer timer_;								//!< Stopwatch to measure time to process frame.
	private RingBuffer acc_frame_timings_; //RingBuffer<double> acc_frame_timings_;		//!< Total processing time of the last 10 frames, used to give some user feedback on the performance.
	private RingBuffer acc_num_obs_;//RingBuffer<size_t> acc_num_obs_;			//!< Number of observed features of the last 10 frames, used to give some user feedback on the tracking performance.
	private int num_obs_last_;							//!< Number of observations in the previous frame.
	protected TrackingQuality tracking_quality_;			//!< An estimate of the tracking quality based on the number of tracked features.

	private PerformanceMonitor g_permon = null;			// ViKit performance monitor

	private static final int acc_array_size_limit = 10;	// This is the size limit for the array's of acc_frame_timings_ and acc_num_obs_
	
	
	private static final Comparator<Point> ptLastOptimComparator = new Comparator<Point>(){
		public int compare(Point lhs, Point rhs){
			return Integer.compare(lhs.get_last_structure_optim_(), rhs.get_last_structure_optim_());
		}
	};


	public FrameHandlerBase()
	{
		this.stage_= Stage.STAGE_PAUSED;
		this.set_reset_ = false;
		this.set_start_ = false;
		this.num_obs_last_ = 0;
		this.tracking_quality_ = TrackingQuality.TRACKING_INSUFFICIENT;
		
		
		this.acc_frame_timings_ = new RingBuffer(acc_array_size_limit);//new ArrayList<Double>(acc_array_size_limit);
		this.acc_num_obs_ = new RingBuffer(acc_array_size_limit);
		//		#ifdef SVO_TRACE
		//		  // Initialize Performance Monitor
		//		  g_permon = new vk::PerformanceMonitor();
		//		  g_permon->addTimer("pyramid_creation");
		//		  g_permon->addTimer("sparse_img_align");
		//		  g_permon->addTimer("reproject");
		//		  g_permon->addTimer("reproject_kfs");
		//		  g_permon->addTimer("reproject_candidates");
		//		  g_permon->addTimer("feature_align");
		//		  g_permon->addTimer("pose_optimizer");
		//		  g_permon->addTimer("point_optimizer");
		//		  g_permon->addTimer("local_ba");
		//		  g_permon->addTimer("tot_time");
		//		  g_permon->addLog("timestamp");
		//		  g_permon->addLog("img_align_n_tracked");
		//		  g_permon->addLog("repr_n_mps");
		//		  g_permon->addLog("repr_n_new_references");
		//		  g_permon->addLog("sfba_thresh");
		//		  g_permon->addLog("sfba_error_init");
		//		  g_permon->addLog("sfba_error_final");
		//		  g_permon->addLog("sfba_n_edges_final");
		//		  g_permon->addLog("loba_n_erredges_init");
		//		  g_permon->addLog("loba_n_erredges_fin");
		//		  g_permon->addLog("loba_err_init");
		//		  g_permon->addLog("loba_err_fin");
		//		  g_permon->addLog("n_candidates");
		//		  g_permon->addLog("dropout");
		//		  g_permon->init(Config::traceName(), Config::traceDir());
		//		#endif

		//SVO_INFO_STREAM("SVO initialized");						// Needs to be set up from global.h
	}

	// Get the current map
	public Map map()
	{
		return map_;
	}

	// Will reset the map as soon as the current frame is finished processing
	public void reset()
	{
		set_start_ = true;
	}

	// Start processing
	public void start()
	{
		set_start_ = true;
	}

	// Get the current stage of the algorithm.
	public Stage stage()
	{
		return stage_;
	}

	// Get tracking quality
	public TrackingQuality trackingQuality()
	{
		return tracking_quality_;
	}

	// Get the processing time of the previous iterations.
	public double lastprocessingTime()
	{
		return timer_.getTime();
	}

	// Get the number of feature observations of the last frame.
	public int lastNumObservations()
	{
		return num_obs_last_;
	}

	// Before a frame is processed this function is called.
	protected boolean startFrameProcessingCommon(double timestamp)
	{
		if(set_start_)
		{
			resetAll();
			stage_ = Stage.STAGE_FIRST_FRAME;
		}

		if(stage_ == Stage.STAGE_PAUSED)
		{
			return false;
		}

		//		  SVO_LOG(timestamp);
		//		  SVO_DEBUG_STREAM("New Frame");
		//		  SVO_START_TIMER("tot_time");

		timer_.start();
		// some cleanup from last iteration, can't do before because of visualization
		map_.emptyTrash();	// Garbage collection does some of this, but what about removing old frames that are no longer required?
		return true;


	}

	// When a frame has finished processing, this function is called.
	protected int finishFrameProcessingCommon(int update_id, UpdateResult dropout, int num_observations)
	{
		//		  SVO_DEBUG_STREAM("Frame: "<<update_id<<"\t fps-avg = "<< 1.0/acc_frame_timings_.getMean()<<"\t nObs = "<<acc_num_obs_.getMean());
		//		  SVO_LOG(dropout);

		// save processing time to calculate fps. if statements ensure a max of the 10 newest processing times are kept
		acc_frame_timings_.push_back(timer_.stop());
		
		
		if(stage_ == Stage.STAGE_DEFAULT_FRAME)
		{
			acc_num_obs_.push_back((double)num_observations);		
		}
		num_obs_last_ = num_observations;
		//		SVO_STOP_TIMER("tot_time");

		//		#ifdef SVO_TRACE
		//		  g_permon->writeToFile();
		//		  {
		//		    boost::unique_lock<boost::mutex> lock(map_.point_candidates_.mut_);
		//		    size_t n_candidates = map_.point_candidates_.candidates_.size();
		//		    SVO_LOG(n_candidates);
		//		  }
		//		#endif
		if(dropout == UpdateResult.RESULT_FAILURE && (stage_ == stage_.STAGE_DEFAULT_FRAME || stage_ == Stage.STAGE_RELOCALIZING))
		{
			stage_ = Stage.STAGE_RELOCALIZING;
			tracking_quality_ = TrackingQuality.TRACKING_INSUFFICIENT;
		}
		else if (dropout == UpdateResult.RESULT_FAILURE)
		{
			resetAll();
		}
		return 0;

	}

	// Reset the map and frame handler to start from scratch.
	protected void resetCommon()
	{
		map_.reset();
		stage_ = Stage.STAGE_PAUSED;
		set_reset_ = false;
		set_start_ = false;
		tracking_quality_ = TrackingQuality.TRACKING_INSUFFICIENT;
		num_obs_last_ = 0;
		//		SVO_INFO_STREAM("RESET");

	}

	// Reset the frame handler. Implement in derived class.
	private void resetAll()	
	{
		resetCommon();
	}

	// Set the tracking quality based on the number of tracked features.
	protected void setTrackingQuality(int num_observations)
	{
		tracking_quality_ = TrackingQuality.TRACKING_GOOD;
		if(num_observations < Config.getQualityMinFts())
		{
			//		    SVO_WARN_STREAM_THROTTLE(0.5, "Tracking less than "<< Config::qualityMinFts() <<" features!");
			tracking_quality_ = TrackingQuality.TRACKING_INSUFFICIENT;
		}
		int feature_drop = Math.min(num_obs_last_, Config.getMaxFts())- num_observations;
		if(feature_drop > Config.getQualityMaxDropFts())
		{
			//		    SVO_WARN_STREAM("Lost "<< feature_drop <<" features!");
			tracking_quality_ = TrackingQuality.TRACKING_INSUFFICIENT;
		}
	}

	// ToDo Unfinished method
	// Optimize some of the observed 3d points.
	protected void optimizeStructure(Frame frame, int max_n_pts, int max_iter)
	{
		ArrayList<Point> pts = new ArrayList<Point>();
		Iterator<Feature> it = frame.getFts_().iterator();
		while(it.hasNext())
		{
			Feature ft = it.next();
			if(ft.getPoint()!= null)
			{
				pts.add(ft.getPoint());
			}
		}
		max_n_pts = Math.min(max_n_pts, pts.size());
		//nth_element() // ToDo currently used .sort to order all of pts rather than nth_element as in c++.
		
		Collections.sort(pts, ptLastOptimComparator);
		
		for(Point pt: pts)
		{	
			pt.optimize(max_iter);
			pt.set_last_structure_optim_(frame.getId_());
		}
	}

	
}

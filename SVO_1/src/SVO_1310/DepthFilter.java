package SVO_1310;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.Queue;
import java.util.concurrent.locks.Condition;

import org.apache.commons.math3.distribution.NormalDistribution;

import matrix_types.JamaUtils;
import matrix_types.Vector3d;

import SVO_1310.Vikit.PerformanceMonitor;
import Sophus.Se3;

/*
 * ToDo	The locks requires locks to be added.
 */
public class DepthFilter {
	private AbstractDetector feature_detector_;		//changed from a DetectorPtr to an AbstractDetector
//	private callback_t seed_converged_cb_;
	private ArrayList<Seed> seeds_ = new ArrayList<Seed>();
	//private Mutex seeds_mut_;						// ToDo this may not be necessary at all
	private boolean seeds_updating_halt_;			//!< Set this value to true when seeds updating should be interrupted.
	private Thread thread_;							// ToDo possibly remove thread so SVO_JAVA is sequential.
	private Queue<Frame> frame_queue_;				// C++ uses FramePtr instead of Frame
	//	private Mutex frame_queue_mut_;					// boost::mutex frame_queue_mut_;
	private Condition frame_queue_cond_;	// boost::condition_variable frame_queue_cond_;
	private Frame new_keyframe_; 							// FramePtr new_keyframe_;	//!< Next keyframe to extract new seeds.
	private boolean new_keyframe_set_;				//!< Do we have a new keyframe to process?.
	private double new_keyframe_min_depth_;					//!< Minimum depth in the new keyframe. Used for range in new seeds.
	private double new_keyframe_mean_depth_;				//!< Maximum depth in the new keyframe. Used for range in new seeds.
	private PerformanceMonitor permon_;		// vk::PerformanceMonitor permon_;		//!< Seperate performance monitor since the DepthFilter runs in a parallel thread.
	private Matcher matcher_;

	private DepthFilter_Options options_ = new DepthFilter_Options();


	public DepthFilter(AbstractDetector feature_detector)//, callback_t seed_converged_cb)	// Changed from DetectorPtr to AbstractDetector
	{
		this.feature_detector_ = feature_detector;
//		this.seed_converged_cb_ = seed_converged_cb;	Commented out as i've removed the need for this ToDo : ensure this is correct!
		this.seeds_updating_halt_ = false;
		this.thread_ = null;
		this.new_keyframe_set_ = false;
		this.new_keyframe_min_depth_ = 0.0;
		this.new_keyframe_mean_depth_ = 0.0;
		matcher_ = new Matcher();
	}

	// Start this thread hen seed updating should be in a parallel thread.
	public DepthFilter_Options get_Options()
	{
		return options_;
	}
	public void startThread()
	{
		thread_ = new Thread();
		//  ToDo Thread in a different form to C++. Will cause issues.
		// thread_ = new boost::thread(&DepthFilter::updateSeedsLoop, this);
	}
	// Stop the parallel thread that is running.
	public void stopThread()
	{

		//		SVO_INFO_STREAM("DepthFilter stop thread invoked.");
		if(thread_!= null)
		{
			//		    SVO_INFO_STREAM("DepthFilter interrupt and join thread... ");
			seeds_updating_halt_ = true;
			thread_.interrupt();			// ToDo the java thread doesnt act like the c++ thread, so is .interrupt() and .join() correct?
			try {
				thread_.join();
			} catch (InterruptedException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
			thread_ = null;
		}
	}
	// Add frame to the queue to be processed.
	// ToDo Requires some attention to the locks
	public void addFrame(Frame frame)				// Changed from FramePtr
	{
		if(thread_ != null)
		{
			// ToDo 	lock_t lock(frame_queue_mut_);		
			if(frame_queue_.size() >2)
			{
				frame_queue_.poll();
			}
			frame_queue_.offer(frame);
			seeds_updating_halt_ = false;
			frame_queue_cond_.signal();	//ToDo changed from notify_one() to signal(). Ensure it's the same.
		}
		else
			updateSeeds(frame);
	}
	// Add new keyframe to the queue.
	public void addKeyFrame(Frame frame, double depth_mean, double depth_min)
	{
		new_keyframe_min_depth_ = depth_min;
		new_keyframe_mean_depth_ = depth_mean;
		if(thread_ != null)
		{
			// ToDo include locks!!
			new_keyframe_ = frame;
			new_keyframe_set_ = true;
			seeds_updating_halt_ = true;
			frame_queue_cond_.signal();	//ToDo changed from notify_one() to signal(). Ensure it's the same.
		}
		else
			initializeSeeds(frame);
	}
	// FROM C++:
	// Remove all seeds which are initialzed from the specified keyframe. This
	// function is used to make sure that no seeds points to a non-existant frame
	// when a frame is removed from the map.
	public void removeKeyframe(Frame frame)			// Changed from FramePtr
	{
		seeds_updating_halt_ = true;
		//lock_t lock(seeds_mut_);					ToDo
		synchronized(this){
			int n_removed = 0;
			for(Seed seed:seeds_)
			{
				if(seed.getFeature().getFrame() == frame)
				{
					seeds_.remove(seed);
					n_removed++;
				}
			}
			seeds_updating_halt_ = false;
		}
	}

	// If the map is reset, call this function such that we don't have pointers to old frames.
	public void reset()
	{
		seeds_updating_halt_ = true;
		synchronized(this)		// ToDo work required on synchronised blocks
		{
			//lock_t lock(seeds_mut_);	ToDo
			seeds_.clear();
		}
		//lock_t lock();	// ToDo Perhaps this is not required at all. 
		synchronized(this){
			while(!frame_queue_.isEmpty())
			{
				frame_queue_.poll();
			}

			seeds_updating_halt_ = false;

			if(options_.get_verbose())
			{
				//		    SVO_INFO_STREAM("DepthFilter: RESET.");
			}
		}
	}

	// FROM C++:
	// Returns a copy of the seeds belonging to frame. Thread-safe.
	// Can be used to compute the Next-Best-View in parallel.
	// IMPORTANT! Make sure you hold a valid reference counting pointer to frame
	// so it is not being deleted while you use it.
	public ArrayList<Seed> getSeedsCopy(Frame frame)	// ToDo need to remove the array seeds field and return arraylist seeds
	{
		ArrayList<Seed> seeds = new ArrayList<Seed>();

		//		  lock_t lock(seeds_mut_);
		synchronized(this)
		{
			for(Seed seed: seeds_)
			{
				if(seed.getFeature().getFrame() == frame)
					seeds.add(seed);
			}
			return seeds;
		}
	}
	// FROM C++:
	// Return a reference to the seeds. This is NOT THREAD SAFE
	public ArrayList<Seed> getSeeds()
	{
		return seeds_;
	}
	// Bayes update of the seed, x is the measurement, tau2 the measurement uncertainty
	public void updateSeed(float x, float tau2, Seed seed)
	{
		float norm_scale = (float) Math.sqrt(seed.get_sigma2()+ tau2);
		if(Float.isNaN(norm_scale))
			return;
		// ToDo may need to create my own normal distribution method rather than use apache_commons
		NormalDistribution nd = new NormalDistribution(seed.get_mu(), norm_scale);
		float s2 = (float) (1./(1./seed.get_sigma2() + 1./tau2));
		float m = s2*(seed.get_mu()/seed.get_sigma2() + x/tau2);
		float C1 = (float) (seed.get_a()/(seed.get_a() + seed.get_b()) * nd.density(x));
		float C2 = (float) (seed.get_b()/(seed.get_a()+ seed.get_b()) * 1./seed.get_z_range());
		float normalization_constant = C1 + C2;
		C1 /= normalization_constant;
		C2 /= normalization_constant;
		float f = (float) (C1*(seed.get_a() + 1.)/(seed.get_a() + seed.get_b()+1.) + C2*seed.get_a()/(seed.get_a() + seed.get_b()+1.));
		float e = (float) (C1*(seed.get_a()+1.)*(seed.get_a()+2.)/((seed.get_a()+seed.get_b()+1.)*(seed.get_a()+seed.get_b()+2.)) + 
				C2*seed.get_a()*(seed.get_a()+1.0f)/((seed.get_a()+seed.get_b()+1.0f)*(seed.get_a()+seed.get_b()+2.0f)));

		// update parameters
		float mu_new = C1*m + C2*seed.get_mu();
		seed.set_sigma2(C1*(s2 + m*m) + C2*(seed.get_sigma2() + seed.get_mu()*seed.get_mu()) - mu_new*mu_new);
		seed.set_mu(mu_new);
		seed.set_a((e-f)/(f-e/f));
		seed.set_b(seed.get_a()*(1.0f-f)/f);

	}
	// Compute the uncertainty of the measurement.
	public static double computeTau(Se3 T_ref_cur, Vector3d f, double z, double px_error_angle)
	{
		Vector3d t = new Vector3d(T_ref_cur.get_Translation().getArray());
		Vector3d a = f.times(z).minus(t);
		double t_norm = t.normF();
		double a_norm = a.normF();
		double alpha = Math.acos(JamaUtils.dotproduct(f, t)/t_norm);	// dot product
		double beta = Math.acos(JamaUtils.dotproduct(a, t.times(-1))/(t_norm * a_norm)); 	// dot product
		double beta_plus = beta + px_error_angle;
		double gamma_plus = Math.PI-alpha-beta_plus;		// Triangle angles sum to Pi
		double z_plus = t_norm * Math.sin(beta_plus)/Math.sin(gamma_plus);		// law of sines
		return (z_plus - z);		// tau
	}
	// Initialize new seeds from a frame.
	public void initializeSeeds(Frame frame)		// FramePtr frame
	{
		ArrayList<Feature> new_features = new ArrayList<Feature>();	// Changed from Features new_features
		feature_detector_.setExistingFeatures(frame.getFts_());
		feature_detector_.detect(frame, frame.get_img_pyr(), Config.getTriang_half_patch_size(), new_features);

		// initialize a seed for every new feature
		seeds_updating_halt_ = true;
		synchronized(this)		// instead of the lock_t lock(seeds_mut_)	// To Do: by locking, the updateSeeds function should stop
		{
			Seed.increment_batch_counter();		// ToDo should seed be a singleton class?
			for(Feature ftr: new_features)
			{
				seeds_.add(new Seed(ftr, new_keyframe_mean_depth_, new_keyframe_min_depth_)); 	//ToDo may need to change from double to float
			}
			if(options_.get_verbose())
				//			    SVO_INFO_STREAM("DepthFilter: Initialized "<<new_features.size()<<" new seeds");
				seeds_updating_halt_ = false;
		}
	}
	// Update all seeds with a new measurement frame.
	public void updateSeeds(Frame frame)			// virtual void updateSeeds(Frameptr frame);
	{
		// update only a limited number of seeds, because we don't have time to do it for all the seeds in every frame.
		int n_updates = 0;
		int n_failed_matches = 0;
		int n_seeds = seeds_.size();
		//lock_t lock(seeds_mut_); 			ToDo
		synchronized(this)
		{		//ToDo must ensure only one DepthFilter is used, to ensure synchronicity
			Iterator<Seed> it = seeds_.iterator();


			final double focal_length = frame.getCam().errorMultiplier2();
			double px_noise = 1.0;
			double px_error_angle = Math.atan(px_noise/(2.0*focal_length))*2.0;		// law of chord (sehnensatz)

			while (it.hasNext())
			{
				Seed seed_it = it.next();

				// Set this value true when seeds updating should be interrupted
				if(seeds_updating_halt_)
					return;

				// check if seed is not already too old
				if((Seed.get_batch_counter() - seed_it.get_batch_id()) > options_.get_max_n_kfs())
				{
					seeds_.remove(seed_it);
					continue;
				}

				// Check if point is visible in the current image
				Se3 T_ref_cur = seed_it.getFeature().getFrame().getT_f_w_().times(frame.getT_f_w_().inverse());
				final Vector3d xyz_f = new Vector3d(T_ref_cur.inverse().times(seed_it.getFeature().getF().times(1.0/seed_it.get_mu())).getArray());
				if(xyz_f.get(2) < 0.0) {
					continue;		// behind the camera, so go to next seed
				}

				if(!frame.getCam().isInFrame(frame.f2c(xyz_f)))	//	ToDo there is no isInFrame(Vector3d) method within abstract_camera
				{
					continue;
				}

				// we are using inverse depth coordinates
				float z_inv_min = (float) (seed_it.get_mu() + Math.sqrt(seed_it.get_sigma2()));
				float z_inv_max = Math.max((float)(seed_it.get_mu() - Math.sqrt(seed_it.get_sigma2())), 0.00000001f);
				Depth z_val = new Depth(0);
				
				if(!matcher_.findEpipolarMatchDirect(seed_it.getFeature().getFrame(), frame, seed_it.getFeature(),1.0/(seed_it.get_mu()), 1.0/(z_inv_min), 1.0/(z_inv_max), z_val))	// ToDo z should be set to a different value to 0 at this method call
				{
					seed_it.increment_b();
					n_failed_matches++;
					continue;
				}
				double z = z_val.getDepth();

				// Compute tau
				double tau = computeTau(T_ref_cur, seed_it.getFeature().getF(), z, px_error_angle);
				double tau_inverse = 0.5 * (1.0/Math.max(0.0000001, z-tau) - 1.0/(z+tau));

				// update the estimate 
				updateSeed((float)(1.0/z), (float)(tau_inverse*tau_inverse), seed_it);
				n_updates++;

				if(frame.isKeyFrame())
				{
					// The feature detector should not initialise new seeds close to this location
					feature_detector_.setGridOccupancy(matcher_.get_px_cur_());
				}

				// If the seed has converged, we initialize a new candidate point and remove the seed
				if(Math.sqrt(seed_it.get_sigma2()) < seed_it.get_z_range()/options_.get_seed_convergence_sigma2_thresh())
				{
					assert seed_it.getFeature().getPoint() == null;	// ToDo is this necessary // ToDo this should not happen anymore
					Vector3d xyz_world = new Vector3d(seed_it.getFeature().getFrame().getT_f_w_().inverse().times(seed_it.getFeature().getF().times((1.0/seed_it.get_mu()))).getArray());
					Point point = new Point(xyz_world, seed_it.getFeature());
					seed_it.getFeature().setPoint(point);

					// ToDo This is the replacement for the FrameHandlerMono issue including callback_t and boost::bind
					Map map_ = Map.getInstance();
					map_.getPoint_Candidates().newCandidatePoint(point, seed_it.get_sigma2());
					//seed_converged_cb_ = new callback_t(point, seed_it.get_sigma2());	//ToDo not sure how to translate this	seed_converged_cb_(point, seed_it.get_sigma2());

					seeds_.remove(seed_it);			// ToDo this may not be accurate					
				}

				else if(Float.isNaN(z_inv_min))
				{
					//				      SVO_WARN_STREAM("z_min is NaN");	
					seeds_.remove(seed_it);
				}
				else
					continue;
			}
		}
	}
	// When a new keyframe arrives, the frame queue should be cleared.
	public void clearFrameQueue()
	{
		frame_queue_.clear();
	}
	// A thread that is continuously updating the seeds.
	public void updateSeedsLoop()
	{

		while(!Thread.interrupted())
		{
			Frame frame;
			synchronized(this)
			{
				while(frame_queue_.isEmpty() && new_keyframe_set_ == false)
				{
					try {
						frame_queue_cond_.await();
					} catch (InterruptedException e) {
						// TODO Auto-generated catch block
						e.printStackTrace();
					} // ToDo changed from wait(lock) to await(). Ensure it is the same.
				}
				if(new_keyframe_set_)
				{
					new_keyframe_set_ = false;
					seeds_updating_halt_ = false;
					clearFrameQueue();
					frame = new_keyframe_;
				}
				else
				{
					frame = frame_queue_.poll();
				}
			}
			updateSeeds(frame);
			if(frame.getIsKeyFrame())
				initializeSeeds(frame);
		}

	}


}

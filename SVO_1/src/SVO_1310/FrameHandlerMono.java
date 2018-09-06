package SVO_1310;

import java.util.ArrayList;
import java.util.Collections;
import org.opencv.core.Mat;
import org.opencv.core.Point;

import matrix_types.Matrix3d;
import matrix_types.Vector2d;
import matrix_types.Vector3d;
//import SVO_1310.CV.Point2f;
import SVO_1310.Vikit.AbstractCamera;
import SVO_1310.Vikit.Method;
import Sophus.Se3;

// ToDo due to this being a friend class in c++ to initialization, it should be able to access the private fields there.
// This may need some going over!

// Monocular Visual Odometry Pipeline as described in the SVO paper.
public class FrameHandlerMono extends FrameHandlerBase{
	private AbstractCamera cam_;									//!< Camera model, can be ATAN, Pinhole or Ocam (see vikit).
	private Reprojector reprojector_;                     			//!< Projects points from other keyframes into the current frame
	private Frame new_frame_;				                        //!< Current frame.
	private Frame last_frame_;                  				    //!< Last frame, not necessarily a keyframe.
	private ArrayList<Frame> core_kfs_= new ArrayList<Frame>();		//!< Keyframes in the closer neighbourhood.
	// Overlap requires methods similar to Arraylist
	private ArrayList<FrameDoublePair> overlap_kfs_ = new ArrayList<FrameDoublePair>(); //Should be FrameInt pair //!< All keyframes with overlapping field of view. the paired number specifies how many common mappoints are observed TODO: why vector!?
	// private NOTSUREWHATCLASS<Frame,int> overlap_kfs_;				
	private KltHomographyInit klt_homography_init_ = new KltHomographyInit(); //initialization//!< Used to estimate pose of the first two keyframes by estimating a homography.
	private DepthFilter depth_filter_;								//!< Depth estimation algorithm runs in a parallel thread and is used to initialize new 3D points.


	public FrameHandlerMono(AbstractCamera cam)
	{
		super();
		this.cam_ = cam;
		this.reprojector_ = new Reprojector(cam_,map_);
		this.depth_filter_ = null;
		initialize();
	}

	// Provide an image
	public void addImage(Mat img, double timestamp)
	{
		if(!startFrameProcessingCommon(timestamp))
			return;

		// some cleanup from last iteration, can't do before because of visualisation
		core_kfs_.clear();
		overlap_kfs_.clear();

		// create new frame
		//		  SVO_START_TIMER("pyramid_creation");
		Mat img_clone = img;
		new_frame_= new Frame(cam_, img_clone, timestamp);
		//		  SVO_STOP_TIMER("pyramid_creation");

		// process frame
		UpdateResult res = UpdateResult.RESULT_FAILURE;
		if(super.stage_ == Stage.STAGE_DEFAULT_FRAME)
			res = processFrame();
		else if(super.stage_ == Stage.STAGE_SECOND_FRAME)
			res = processSecondFrame();
		else if(super.stage_ == Stage.STAGE_FIRST_FRAME)
			res = processFirstFrame();
		else if(super.stage_ == Stage.STAGE_RELOCALIZING)
		{
			Matrix3d mat_identity = new Matrix3d();
			mat_identity.identity();
			Vector3d vec_zero = new Vector3d(0);
			res = relocalizeFrame(Se3(mat_identity, vec_zero), map_.getClosestKeyframe(last_frame_));
		}

		// set last frame
		last_frame_ = new_frame_;
		//new_frame_.reset(); Unnecessary?
		// finish processing
		finishFrameProcessingCommon(last_frame_.getId_(), res, last_frame_.nObs());
	}

	private Se3 Se3(Matrix3d mat_identity, Vector3d vec_zero) {
		// TODO Auto-generated method stub
		return null;
	}

	// Set the first frame (used for synthetic datasets in benchmark mode)
	public void setFirstFrame(Frame first_frame)
	{
		resetAll();
		last_frame_ = first_frame;
		last_frame_.setKeyframe();
		map_.addKeyframe(last_frame_);
		stage_ = Stage.STAGE_DEFAULT_FRAME;
	}

	// Get the last frame that has been processed
	public Frame lastFrame()
	{
		return last_frame_;
	}

	// Get the set of spatially closest keyframes of the last frame.
	public ArrayList<Frame> coreKeyFrames()
	{ 
		return core_kfs_;
	}

	// Return the feature track to visualise the KLT tracking during initialisation.
	//	ToDo changed Point2f to Point public ArrayList<Point2f> initFeatureTrackRefPx()
	public Point[] initFeatureTrackRefPx()
	{
		return klt_homography_init_.get_px_ref_();
	}
	public Point[] initFeatureTrackCurPx()
	{
		return klt_homography_init_.get_px_cur_();
	}

	// Access the depth filter.
	public DepthFilter depthFiler()
	{
		return depth_filter_;
	}

	// An external place recognition module may know where to relocalize.
	public boolean relocalizeFrameAtPose(int keyframe_id, Se3 T_f_kf, Mat img, double timestamp)
	{
		Frame ref_keyframe = null;
		if(map_.getKeyFrameById(keyframe_id)==null)
			return false;
		//		new_frame_.reset(new Frame(cam_, img.clone(), timestamp));
		UpdateResult res = relocalizeFrame(T_f_kf, ref_keyframe);
		if(res != UpdateResult.RESULT_FAILURE)
		{
			last_frame_ = new_frame_;
			return true;
		}
		return false;

	}

	// Initialize the visual odometry algorithm.
	private void initialize()
	{
		//FastDetector fast_detector = new FastDetector(cam_.getWidth(), cam_.getHeight(), Config.getGridSize(), Config.getnPyrLevels());
		//AbstractDetector feature_detector = new AbstractDetector(fast_detector);
		AbstractDetector feature_detector = new FastDetector(cam_.getWidth(), cam_.getHeight(), Config.getGridSize(), Config.getnPyrLevels());
		// ToDo		  DepthFilter::callback_t depth_filter_cb = boost::bind(MapPointCandidates::newCandidatePoint, &map_.point_candidates_, _1, _2);
//		callback_t depth_filter_cb = new callback_t(newCandidatePoint, map_.getPoint_Candidates());
		depth_filter_ = new DepthFilter(feature_detector);//, depth_filter_cb);
		depth_filter_.startThread();
	}

	// Processes the first frame and sets it as a keyframe.
	private UpdateResult processFirstFrame()
	{
		Matrix3d mat_identity = new Matrix3d();
		mat_identity.identity();
		Vector3d vec_zero = new Vector3d(0);
		Se3 se3 = new Se3(mat_identity, vec_zero);
		new_frame_.setT_f_w_(se3);
		if(klt_homography_init_.addFirstFrame(new_frame_) == InitResult.FAILURE)
		{
			return UpdateResult.RESULT_NO_KEYFRAME;
		}
		new_frame_.setKeyframe();
		map_.addKeyframe(new_frame_);
		stage_ = Stage.STAGE_SECOND_FRAME;
		//		SVO_INFO_STREAM("Init: Selected first frame.");
		return UpdateResult.RESULT_IS_KEYFRAME;
	}

	// Processes all frames adter the first frame until a keyframe is selected.
	private UpdateResult processSecondFrame()
	{

		InitResult res = klt_homography_init_.addSecondFrame(new_frame_);
		if(res == InitResult.FAILURE)
			return UpdateResult.RESULT_FAILURE;
		else if(res == InitResult.FAILURE)
			return UpdateResult.RESULT_NO_KEYFRAME;

		// two-frame bundle adjustment. Not currently using Bundle Adjustment
		//		#ifdef USE_BUNDLE_ADJUSTMENT
		//		  ba::twoViewBA(new_frame_.get(), map_.lastKeyframe().get(), Config::lobaThresh(), &map_);
		//		#endif

		new_frame_.setKeyframe();
		double depth_mean, depth_min;
		double[] min_and_mean = Frame_utils.getSceneDepth(new_frame_);//, depth_mean, depth_min);
		depth_mean = min_and_mean[1];
		depth_min = min_and_mean[0];
		depth_filter_.addKeyFrame(new_frame_, depth_mean, 0.5*depth_min);

		// add frame to map
		map_.addKeyframe(new_frame_);
		stage_ = Stage.STAGE_DEFAULT_FRAME;
		klt_homography_init_.reset();
		//  SVO_INFO_STREAM("Init: Selected second frame, triangulated initial map.");
		return UpdateResult.RESULT_IS_KEYFRAME;
	}

	// Processes all frames after the first two frames.
	private UpdateResult processFrame()
	{
		// Set initial pose TODO use prior
		new_frame_.setT_f_w_(last_frame_.getT_f_w_());

		// sparse image align
		//		  SVO_START_TIMER("sparse_img_align");
		SparseImgAlign img_align = new SparseImgAlign(Config.getKltMaxLevel(), Config.getKltMinLevel(), 30, Method.GaussNewton, false, false);
		int img_align_n_tracked = img_align.run(last_frame_, new_frame_);
		//		  SVO_STOP_TIMER("sparse_img_align");
		//		  SVO_LOG(img_align_n_tracked);
		//		  SVO_DEBUG_STREAM("Img Align:\t Tracked = " << img_align_n_tracked);

		// Map reprojection and feature alignment
		//		  SVO_START_TIMER("reproject");
		reprojector_.reprojectMap(new_frame_, overlap_kfs_);
		//		  SVO_STOP_TIMER("reproject");
		int repr_n_new_references = reprojector_.get_n_matches_();
		int repr_n_mps = reprojector_.get_n_trials_();
		//		  SVO_LOG2(repr_n_mps, repr_n_new_references);
		//		  SVO_DEBUG_STREAM("Reprojection:\t nPoints = "<<repr_n_mps<<"\t \t nMatches = "<<repr_n_new_references);
		if(repr_n_new_references < Config.getQualityMinFts())
		{
			//		    SVO_WARN_STREAM_THROTTLE(1.0, "Not enough matched features.");
			new_frame_.setT_f_w_(last_frame_.getT_f_w_()); // reset to avoid crazy pose jumps
			tracking_quality_ = TrackingQuality.TRACKING_INSUFFICIENT;
			return UpdateResult.RESULT_FAILURE;
		}
		
		// pose optimization
		//		  SVO_START_TIMER("pose_optimizer");
		
		double sfba_thresh, sfba_error_init, sfba_error_final;
		int sfba_n_edges_final;
		double[] optimizeGaussNewton_values = PoseOptimizer.optimizeGaussNewton( Config.getPoseoptim_thresh(), Config.getPoseoptim_num_iter(), false, new_frame_);
		sfba_thresh = optimizeGaussNewton_values[0];
		sfba_error_init = optimizeGaussNewton_values[1];
		sfba_error_final = optimizeGaussNewton_values[2];
		sfba_n_edges_final = (int) optimizeGaussNewton_values[3];
		// 			,sfba_thresh, sfba_error_init,sfba_error_final, sfba_n_edges_final);
		//		  SVO_STOP_TIMER("pose_optimizer");
		//		  SVO_LOG4(sfba_thresh, sfba_error_init, sfba_error_final, sfba_n_edges_final);
		//		  SVO_DEBUG_STREAM("PoseOptimizer:\t ErrInit = "<<sfba_error_init<<"px\t thresh = "<<sfba_thresh);
		//		  SVO_DEBUG_STREAM("PoseOptimizer:\t ErrFin. = "<<sfba_error_final<<"px\t nObsFin. = "<<sfba_n_edges_final);
		if(sfba_n_edges_final < 20)
			return UpdateResult.RESULT_FAILURE;
		
		// Structure optimization
		//		  SVO_START_TIMER("point_optimizer");
		optimizeStructure(new_frame_, Config.getStructureoptim_max_pts(), Config.getStructureoptim_num_iter());
		//		  SVO_STOP_TIMER("point_optimizer");
		
		// Select Keyframe
		core_kfs_.add(new_frame_);
		setTrackingQuality(sfba_n_edges_final);
		if(tracking_quality_ == TrackingQuality.TRACKING_INSUFFICIENT)
		{
			new_frame_.setT_f_w_(last_frame_.getT_f_w_());
			return UpdateResult.RESULT_FAILURE;
		}
		double depth_mean, depth_min;
		double[] min_and_mean = Frame_utils.getSceneDepth(last_frame_);
		depth_min = min_and_mean[0];
		depth_mean = min_and_mean[1];
		if(!needNewKf(depth_mean) || tracking_quality_ == TrackingQuality.TRACKING_BAD)
		{
			depth_filter_.addFrame(new_frame_);
			return UpdateResult.RESULT_NO_KEYFRAME;
		}
		new_frame_.setKeyframe();
		//		  SVO_DEBUG_STREAM("New keyframe selected.");

		// new keyframe selected
		for(Feature feat: new_frame_.getFts_())
		{
			if(feat.getPoint() != null)
			{
				feat.getPoint().addFrameRef(feat);
			}
		}
		map_.getPoint_Candidates().addCandidatePointToFrame(new_frame_);
		
		// Optional bundle adjustment
		//		#ifdef USE_BUNDLE_ADJUSTMENT
		//		  if(Config::lobaNumIter() > 0)
		//		  {
		//		    SVO_START_TIMER("local_ba");
		//		    setCoreKfs(Config::coreNKfs());
		//		    size_t loba_n_erredges_init, loba_n_erredges_fin;
		//		    double loba_err_init, loba_err_fin;
		//		    ba::localBA(new_frame_.get(), &core_kfs_, &map_,
		//		                loba_n_erredges_init, loba_n_erredges_fin,
		//		                loba_err_init, loba_err_fin);
		//		    SVO_STOP_TIMER("local_ba");
		//		    SVO_LOG4(loba_n_erredges_init, loba_n_erredges_fin, loba_err_init, loba_err_fin);
		//		    SVO_DEBUG_STREAM("Local BA:\t RemovedEdges {"<<loba_n_erredges_init<<", "<<loba_n_erredges_fin<<"} \t "
		//		                     "Error {"<<loba_err_init<<", "<<loba_err_fin<<"}");
		//		  }
		//		#endif
		
		// Init new depth-filters
		depth_filter_.addKeyFrame(new_frame_, depth_mean, 0.5*depth_min);
		
		// If limited number of keyframes, remove the one furthest apart
		if(Config.getMax_n_kfs() > 2 && map_.size() >= Config.getMax_n_kfs())
		{
			Frame furthest_frame = map_.getFurthestKeyframe(new_frame_.pos());
			depth_filter_.removeKeyframe(furthest_frame); // TODO this interrupts the mapper thread, maybe we can solve this better
			map_.safeDeleteFrame(furthest_frame);
		}
		
		// add keyframe to map
		map_.addKeyframe(new_frame_);
		
		return UpdateResult.RESULT_IS_KEYFRAME;

	}

	// Try relocalizing the frame at relative position to provided keyframe.
	private UpdateResult relocalizeFrame(Se3 T_cur_ref, Frame ref_keyFrame)
	{
		//		  SVO_WARN_STREAM_THROTTLE(1.0, "Relocalizing frame");
		if(ref_keyFrame == null)
		{
			//		    SVO_INFO_STREAM("No reference keyframe.");
			return UpdateResult.RESULT_FAILURE;
		}
		SparseImgAlign img_align = new SparseImgAlign(Config.getKltMaxLevel(), Config.getKltMinLevel(), 30, Method.GaussNewton, false, false);
		int img_align_n_tracked = img_align.run(ref_keyFrame, new_frame_);
		if(img_align_n_tracked > 30)
		{
			Se3 T_f_w_last = last_frame_.getT_f_w_();
			last_frame_ = ref_keyFrame;
			UpdateResult res = processFrame();
			if(res != UpdateResult.RESULT_FAILURE)
			{
				stage_ = Stage.STAGE_DEFAULT_FRAME;
				//			      SVO_INFO_STREAM("Relocalization successful.");
			}
			else
				new_frame_.setT_f_w_(T_f_w_last); // reset to last well localized pose
			return res;
		}
		return UpdateResult.RESULT_FAILURE;

	}

	// Reset the frame handler. Implement in derived class.
	private void resetAll()
	{
		super.resetCommon();
		last_frame_ = null;
		new_frame_ = null;
		core_kfs_.clear();
		overlap_kfs_.clear();
		depth_filter_ = null;
	}

	// Keyframe selection criterion.
	private boolean needNewKf (double scene_depth_mean)
	{
		for(int i=0; i < overlap_kfs_.size(); i++)
		{
			Vector3d relpos = new_frame_.w2f(overlap_kfs_.get(i).getFrame().pos()); 	//Vector3d relpos = new_frame_->w2f(it->first->pos());//This line may need altering
			if(Math.abs(relpos.get(0))/scene_depth_mean < Config.getKfSelectMinDist() &&
					Math.abs(relpos.get(1))/scene_depth_mean < Config.getKfSelectMinDist()*0.8 &&
					Math.abs(relpos.get(2))/scene_depth_mean < Config.getKfSelectMinDist()*1.3)
				return false;
		}
		return true;
	}
	// Partial sort removed for a simpler but less efficient Collections.sort. 
	// ToDo check that the sort is in the correct ascending/descending order!
	private void setCoreKfs(int n_closest)
	{
		int n = Math.min(n_closest, overlap_kfs_.size()-1);
		Collections.sort(overlap_kfs_);
		for(int i=0; i< n_closest; i++)
		{
			core_kfs_.add(overlap_kfs_.get(i).getFrame());
		}
	}
}

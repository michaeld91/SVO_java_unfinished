package SVO_1310;


/// Global configuration file of SVO.
/// Implements the Singleton design pattern to allow global access and to ensure
/// that only one instance exists.
public class Config {
//		createImgPyramid(img, max(nPyrLevels(),kltMaxLevel()+1), img_pyr_);
	private static Config instance;
	private static int nPyrLevels;
	private static int kltMaxLevel;
	private static int kltMinLevel;
	private static int gridSize;
	private static int qualityMinFts;
	private static int qualityMaxDropFts;
	private static int maxFts;
	private static double kfSelectMinDist;
	private static double poseoptim_thresh;
	private static int poseoptim_num_iter;
	private static int structureoptim_max_pts;
	private static int structureoptim_num_iter;
	private static int max_n_kfs;
	private static int triang_min_corner_score;	// ToDo changed from :private static double triang_min_corner_score;
	private static String trace_name;
	private static String trace_dir;
	private static boolean use_imu;
	private static int core_n_kfs;
	private static double map_scale;
	private static double init_min_disparity;
	private static int init_min_tracked;
	private static int init_min_inliers;
	private static double reproj_thresh;
	private static double loba_thresh;
	private static double loba_robust_huber_width;
	private static int loba_num_iter;
	private static int triang_half_patch_size;
	private static int subpix_n_iter;
	private static double img_imu_delay;
	
	private static int desired_number_of_corners_per_img;
	  
	private Config()
	{
		trace_name = "svo";
		trace_dir = "/tmp";
		nPyrLevels = 3;
		use_imu = false;
		core_n_kfs = 3;
		map_scale = 1.0;
		gridSize = 25;
		init_min_disparity = 50.0;
		init_min_tracked = 50;
		init_min_inliers = 40;
		kltMaxLevel = 4;
		kltMinLevel = 2;
		reproj_thresh = 2;
		poseoptim_thresh = 2;
		poseoptim_num_iter = 10;
		structureoptim_max_pts = 20;
		structureoptim_num_iter = 5;
		loba_thresh = 2.0;
		loba_robust_huber_width = 1.0;
		loba_num_iter = 0;
		kfSelectMinDist = 0.12;
		triang_min_corner_score = 20;
		triang_half_patch_size = 4;
		subpix_n_iter = 10;
		max_n_kfs = 0;
		img_imu_delay = 0.0;
		maxFts = 120;
		qualityMinFts = 50;
		qualityMaxDropFts = 40;
		desired_number_of_corners_per_img = 500;
	}
	public static Config getInstance()
	{
		if(instance == null)
		{
			instance = new Config();
		}
		return instance;
	}
	// Number of pyramid levels used for features.
	public static int getnPyrLevels() {
		return nPyrLevels;
	}
	// Maximum level of the Lucas Kanade tracker.
	public static int getKltMaxLevel() {
		return kltMaxLevel;
	}
	// Minimum level of the Lucas Kanade tracker.
	public static int getKltMinLevel() {
		return kltMinLevel;
	}
	public static void setkltMinLevel(int score) {
		kltMinLevel = score;
	}
	// Feature grid size of a cell in [px].
	public static int getGridSize() {
		return gridSize;
	}
	// If the number of tracked features drops below this threshold. Tracking quality is bad.
	public static int getQualityMinFts() {
		return qualityMinFts;
	}
	// If within one frame, this amount of features are dropped. Tracking quality is bad.
	public static int getQualityMaxDropFts() {
		return qualityMaxDropFts;
	}
	// Maximum number of features that should be tracked.
	public static int getMaxFts() {
		return maxFts;
	}
	// Minimum distance between two keyframes. Relative to the average height in the map.
	public static double getKfSelectMinDist() {
		return kfSelectMinDist;
	}
	// Reprojection threshold after pose optimization.
	public static double getPoseoptim_thresh() {
		return poseoptim_thresh;
	}
	// Number of iterations in local bundle adjustment.
	public static int getPoseoptim_num_iter() {
		return poseoptim_num_iter;
	}
	// Maximum number of points to optimize at every iteration.
	public static int getStructureoptim_max_pts() {
		return structureoptim_max_pts;
	}
	public static int getStructureoptim_num_iter() {
		return structureoptim_num_iter;
	}
	// Limit the number of keyframes in the map. This makes nslam essentially.
	// a Visual Odometry. Set to 0 if unlimited number of keyframes are allowed.
	// Minimum number of keyframes is 3.
	public static int getMax_n_kfs() {
		return max_n_kfs;
	}
	// Select only features with a minimum Harris corner score for triangulation.
	public static int getTriang_min_corner_score() {
		return triang_min_corner_score;
	}
	public static void setTriang_min_corner_score(int score) {
		triang_min_corner_score = score;
	}
	// Base-name of the tracefiles.
	public static String getTrace_name() {
		return trace_name;
	}
	// Directory where the tracefiles are saved.
	public static String getTrace_dir() {
		return trace_dir;
	}
	// Use the IMU to get relative rotations.
	public static boolean isUse_imu() {
		return use_imu;
	}
	// Number of keyframes in the core. The core-kfs are optimized through bundle adjustment.
	public static int getCore_n_kfs() {
		return core_n_kfs;
	}
	// Initial scale of the map. Depends on the distance the camera is moved for the initialization.
	public static double getMap_scale() {
		return map_scale;
	}
	// Initialization: Minimum required disparity between the first two frames.
	public static double getInit_min_disparity() {
		return init_min_disparity;
	}
	// Initialization: Minimum number of tracked features.
	public static int getInit_min_tracked() {
		return init_min_tracked;
	}
	// Initialization: Minimum number of inliers after RANSAC.
	public static int getInit_min_inliers() {
		return init_min_inliers;
	}
	// Reprojection threshold [px].
	public static double getReproj_thresh() {
		return reproj_thresh;
	}
	// Reprojection threshold after bundle adjustment.
	public static double getLoba_thresh() {
		return loba_thresh;
	}
	// Threshold for the robust Huber kernel of the local bundle adjustment.
	public static double getLoba_robust_huber_width() {
		return loba_robust_huber_width;
	}
	// Number of iterations in the local bundle adjustment.
	public static int getLoba_num_iter() {
		return loba_num_iter;
	}
	public static int getTriang_half_patch_size() {
		return triang_half_patch_size;
	}
	// Subpixel refinement of reprojection and triangulation. Set to 0 if no subpix refinement required!
	public static int getSubpix_n_iter() {
		return subpix_n_iter;
	}
	// How much (in milliseconds) is the camera delayed with respect to the imu.
	public static double getImg_imu_delay() {
		return img_imu_delay;
	}
	public static int getdesired_number_of_corners_per_img()
	{
		return desired_number_of_corners_per_img;
	}
	
}

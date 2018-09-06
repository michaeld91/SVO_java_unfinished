package SVO_1310;


import java.util.ArrayList;
import java.util.Iterator;
import matrix_types.*;
import Jama.Matrix;
import SVO_1310.Vikit.*;
import Sophus.Se3;

/*A 3D point on the surface of a scene.
 * Start date: 10/10/17
 *
 */
public class Point {
	//From C++ code, the variables are all public. I would prefer for these to be private so may change and add get/set methods.

	private static int point_counter_=0;
	private int id_;
	private Vector3d pos_;		//pos_ changed from GVector to Matrix
	private Vector3d normal_ = new Vector3d();	//normal_ changed from GVector to Matrix
	private Matrix3d normal_information_;	//normal_information_ changed from GMatrix to Matrix
	private Boolean normal_set_;
	private ArrayList<Feature> obs_= new ArrayList<Feature>();		//ArrayList, but could potentially be a class Features which is a list of Feature objects.
	private long n_obs_;
	//private g2oPoint* v_pt_;						//!< Temporary pointer to the point-vertex in g2o during bundle adjustment.
	private int last_published_ts_;					//!< Timestamp of last publishing.
	private int last_projected_kf_id_;				//!< Flag for the reprojection: don't reproject a pt twice.
	private PointType type_;						//!< Quality of the point.
	private int n_failed_reproj_;					//!< Number of failed reprojections. Used to assess the quality of the point.
	private int n_succeeded_reproj_;				//!< Number of succeeded reprojections. Used to assess the quality of the point.
	private int last_structure_optim_;				//!< Timestamp of last point optimization
	private static Matrix23d point_jac;// = new GMatrix(2,3);				// Moved from inside jacobian method fields.	//Changed point_jac from GMatrix to Matrix
	//Constructors

	//  Point(const Vector3d& pos){}  //Contains Vector2d reference in const form (similar to final in Java)
	public Point(Vector3d pos){	//Changed from Point(GVector pos)
		point_counter_++;
		this.id_ = point_counter_;
		this.pos_ = pos;
		normal_set_= false;
		this.n_obs_=0;
		//v_pt_				  v_pt_(NULL),				//commented
		this.last_published_ts_=0;
		this.last_projected_kf_id_=-1;
		this.type_ = PointType.TYPE_UNKNOWN;		
		this.n_failed_reproj_=0;
		n_succeeded_reproj_=0;
		last_structure_optim_=0;


	}
	//Point(const Vector3d& pos, Feature* ftr){}
	//Feature* is a pointer to ftr of type Feature.
	public Point(Vector3d pos, Feature ftr){		//Point(GVector pos, Feature ftr)
		point_counter_++;
		this.id_ = point_counter_;
		this.pos_ = pos;
		normal_set_=false;
		this.n_obs_=1;
		//v_pt_				  v_pt_(NULL),				//commented
		this.last_published_ts_=0;
		this.last_projected_kf_id_=-1;
		this.type_ = PointType.TYPE_UNKNOWN;		
		this.n_failed_reproj_=0;
		n_succeeded_reproj_=0;
		last_structure_optim_=0;

		//obs_.push_front(ftr);			TODO
		obs_.add(0,ftr);	//computationally costly compared to push_front.
		/*Why is push_front used? Is it viable to use add(ftr) and place at the back?*/
	}

	/// Add a reference to a frame.
	//Changed variable from Feature* to Feature.
	public void addFrameRef(Feature ftr)
	{
		obs_.add(0,ftr);
		n_obs_++;
	}

	// Remove reference to a frame.
	// bool deleteFrameRef(Frame* frame);
	// An iterator appears to be used in the C++, but i have used a simple for loop
	// In the c++ version, the method for retrieving a frame for evaluation is different.
	public boolean deleteFrameRef(Frame frame)
	{
		for(int i = 0;i<obs_.size(); i++){
			if(obs_.get(i).getFrame()==frame){
				obs_.remove(i);
				return true;
			}
		}
		return false;
	}

	// Check whether mappoint has reference to a frame.
	// Feature* findFrameRef(Frame* frame);
	public Feature findFrameRef(Frame frame)
	{
		for(int i = 0;i<obs_.size(); i++){
			if(obs_.get(i).getFrame()==frame){
				return obs_.get(i);
			}
		}
		return null; //no Key frame found
	}

	// Initialize point normal. The inital estimate will point towards the frame.
	//  void initNormal();
	// Might need to throw an exception to account for removing the assert().
	public void initNormal()
	{
		if(!obs_.isEmpty()){//ie is the obs_ non empty?
			Feature ftr = obs_.get(obs_.size()-1); //should return the last feature in the obs_ arraylist.
			if(ftr != null)
			{
//				System.out.println("Works up to here...");	.set(mat);
				Matrix mat = new Matrix(3, 3);
				normal_.set(ftr.getFrame().getT_f_w_().rotation_Matrix());//.transpose());//.times(ftr.getF().times(-1)));
				double[] diagonal_values = {Math.pow(20/(pos_.minus(ftr.getFrame().pos())).normF(), 2),0,0,0,1.0,0,0,0,1.0};
				normal_information_ = new Matrix3d(diagonal_values);
				normal_set_ = true;
			}
		}
	}

	// Get Frame with similar viewpoint.
	// bool getCloseViewObs(const Vector3d& framepos, Feature*& ftr) const;
	// Currently unsure as to whether or not this method does the same as the C++ version.
	// One area for concern is the variable ftr, why would a blank ftr be given to be altered?
	// Is this method meant to set this ftr variable in some other locations?
	public boolean getCloseViewObs(Vector3d framepos, Matcher match)		//Changed from getCloseViewObs(Vector3d framepos, Feature ftr)
	{
		if(obs_.size()!=0){		//initial check that there are Features stored for diagnosis.
			//double[] obs_dir_vals = {framepos.getX(), framepos.getY(), framepos.getZ()};

			//ToDo below can be shortened	
			Vector3d obs_dir = new Vector3d(framepos.getArray()); //Changed from GVector obs_dir = new GVector(framepos);
			obs_dir.minusEquals(pos_);	//obs_dir.sub(pos_);
			JamaUtils.normalize(obs_dir);
			//Feature min_it=obs_.get(0);
			//Iterator<Feature> min_it = obs_.iterator();
			Feature min_it = obs_.get(0);
			double min_cos_angle = 0;
			//for(int i=0;i<obs_.size();i++)
			Iterator<Feature> it=obs_.iterator();
			while(it.hasNext())							//ensure the .next() is in the correct place!
			{
				Feature ft = it.next();
				//double[] dir_vals = {ft.getFrame().Pos().getX(),ft.getFrame().Pos().getY(),ft.getFrame().Pos().getZ()};
				Vector3d dir = new Vector3d(ft.getFrame().pos().getArray());//GVector dir = new GVector(ft.getFrame().Pos());	//acquires values, not ref.
				JamaUtils.normalize(dir);

				double cos_angle = JamaUtils.dotproduct(obs_dir, dir);		//changed from double cos_angle = obs_dir.dot(dir);
				if(cos_angle > min_cos_angle)
				{
					min_cos_angle = cos_angle;
					min_it = ft;
				}
			}
			match.set_ref_ftr_(min_it); 
			if(min_cos_angle <0.5){return false;}

			return true;
		}
		System.out.println("The ArrayList obs_ is empty");
		return false;

	}

	//  Get number of observations.
	// 'const' not used.
	public int nRefs()
	{
		return obs_.size();
	}

	//Jacobian of point projection on unit plane (focal length = 1) in frame (f).
	//Need a check that vectors and matrices are of the correct dimensions!
	public static void jacobian_xyz2uv(Vector3d p_in_f, Matrix3d R_f_w)	//Fields changed from GVector p_in_f, GMatrix R_f_w
	{
		point_jac= new Matrix23d();	//changed from GMatrix
		double z_inv = 1.0/p_in_f.get(0,2);
		double z_inv_sq = z_inv*z_inv;
		//Changed all of these from setElement
		point_jac.set(0, 0, z_inv); //z_inv);		
		point_jac.set(0, 1, 0.0);
		point_jac.set(0, 2, -p_in_f.get(0,0) * z_inv_sq);
		point_jac.set(1, 0, 0.0);
		point_jac.set(1, 1, z_inv);
		point_jac.set(1, 2, -p_in_f.get(0,1) * z_inv_sq);
		//point_jac.print(0,0); 
//		System.out.println("==============================");
//		(point_jac.timesEquals(-1)).print(0, 0);
//		R_f_w.print(0, 0);		
//		System.out.println("==============================");
		point_jac.set((point_jac.timesEquals(-1)).times(R_f_w)); //returns Matrix types which are cast to matrix23d

	}
	public Matrix23d getPoint_Jac()
	{
		return point_jac;
	}
	public void optimize(int n_iter)
	{
		Vector3d old_point = pos_;		
		double chi2 = 0.0;
		Matrix3d A = new Matrix3d();	
		Vector3d b = new Vector3d();	

		for(int i=0;i< n_iter; i++)
		{
			//A.setZero(); Constructor sets all values to 0 so this is redundant
			//b.zero();	Also, this is set to 0 in constructor so this is redundant
			double new_chi2 = 0.0;

			//compute residuals
			Iterator<Feature> it = obs_.iterator();
			while(it.hasNext())
			{
				Matrix23d J = new Matrix23d(); //should be 2 rows 3 columns. changed from GMatrix J = new GMatrix(2,3)
				Feature it_feat = it.next();
				Vector3d p_in_f = new Vector3d();
				p_in_f.set(it_feat.getFrame().getT_f_w_().times(pos_)); //May need to transpose the pos_ vector


				/////// JACOBIAN LINE /////////////////////////

				//SE3 needs changing from GMatrix to Matrix
				jacobian_xyz2uv(p_in_f, ((Se3)(it_feat).getFrame().getT_f_w_()).rotation_Matrix());
				J=point_jac;

				//Not used Vikit for the e value here...
				//ALSO bad practice with prj
				Vector2d e = new Vector2d();		//Changed from GVector e = new GVector(2)
				Vector2d partA = new Vector2d(math_utils.project2d(((Feature)it).getF()).getArray());	//changed from GVector partA = new GVector(prj.project2d(((Feature)it).getF()));
				Vector2d partB = new Vector2d(math_utils.project2d(p_in_f).getArray());	// changed from  GVector partB = new GVector(prj.project2d(p_in_f));
				e = (Vector2d) partA.minus(partB);	//changed from e.sub(partA,partB);

				//	new_chi2 += e.dot(e); 
				new_chi2 += JamaUtils.dotproduct(e, e);	//since e is a Vector, the squaredNorm is just the dot product of e dot e. Tested.


				//Below is the "A.noalias() += J.transpose() * J;"	implementation. Too long!
				Matrix J_trans = new Matrix(3,2);		//changed from  GMatrix J_trans = new GMatrix(3,2);
				J_trans = J.transpose();
				Matrix3d J_trans_xJ = new Matrix3d(); //changed from GMatrix J_trans_xJ = new GMatrix(3,3);
				J_trans_xJ = (Matrix3d) J_trans.times(J);			//changed from J_trans_xJ.mul(J_trans, J);
				A.plusEquals(J_trans_xJ); 			//changed from A.add(J_trans_xJ);

				//Below is the implementation for  b.noalias() -= J.transpose() * e;	
				Matrix J_trans_xe = null;		//changed from GVector J_trans_xe = null;	
				J_trans_xe = J_trans.times(e);		//changed from J_trans_xe.mul(J_trans, e);
				b.minus(J_trans_xe);				//changed from b.sub(J_trans_xe);

			}
			// solve linear system
			Vector3d dp = new Vector3d();
			dp.set(A.chol().solve(b.transpose())); //May need to use b.transpose();

			if((i>0 && new_chi2 > chi2)) //removed the (bool) std::isnan((double)dp[0])) as i dont think it can occur
			{
				pos_ = old_point;	// roll back
				break;
			}

			//update the model
			Vector3d new_point = (Vector3d) pos_.plus(dp);
			old_point = pos_;
			pos_ = new_point;
			chi2 = new_chi2;



			//stop when converged
			//Still in C++ as vk norm_max has not been coded yet.
			if(math_utils.norm_max(dp) <= Global.EPS)
				break;

		}
	}
	public PointType getType()
	{
		return type_;
	}
	public void setType(PointType type)
	{
		type_ = type;
	}

	public ArrayList<Feature> getObs_()
	{
		return obs_;
	}
	public void set_n_failed_reproj_(int n_failed_rep)
	{
		n_failed_reproj_ = n_failed_rep;
	}
	public int get_n_failed_reproj_()
	{
		return n_failed_reproj_;
	}
	public void set_n_succeeded_reproj_(int n_succeeded_rep)
	{
		n_succeeded_reproj_ = n_succeeded_rep;
	}
	public int get_n_succeeded_reproj_()
	{
		return n_succeeded_reproj_;
	}
	//Below method needed for reprojector::reprojectPoint
	public Vector3d getPos()
	{
		return pos_;
	}
	public void setPos(Vector3d new_pos)
	{
		pos_ = new_pos;
	}
	public int get_last_projected_kf_id()
	{
		return last_projected_kf_id_;
	}
	public void set_last_projected_kf_id(int id_2) {
		last_projected_kf_id_=id_2;	
	}
	public int get_last_structure_optim_()
	{
		return last_structure_optim_;
	}
	public void set_last_structure_optim_(int id)
	{
		last_structure_optim_ = id;
	}
	public long get_n_obs_()
	{
		return n_obs_;
	}
	public Vector3d getNormal()
	{
		return normal_;
	}
	public Matrix3d getNormal_information_()
	{
		return normal_information_;
	}
	public int get_last_published_ts_()
	{
		return last_published_ts_;
	}
	public void set_last_published_ts_(int new_last_published_ts_)
	{
		last_published_ts_ = new_last_published_ts_;
	}
	public int getId()
	{
		return id_;
	}
}
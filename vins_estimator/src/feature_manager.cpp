#include "feature_manager.h"
#include "utility/utility.h"

Eigen::Matrix2d FeaturePerFrame::getOmega() // inverse of covariance matrix
{
    Eigen::Matrix2d sqrt_info = FOCAL_LENGTH / 1.5 * Matrix2d::Identity();

    if(getDepth() <= 0) {
        ROS_ERROR("feature_manager.cpp: something is wrong!"); 
        return sqrt_info; 
    }

    if(!g_use_stereo_correction)
        return sqrt_info; 

    if(!gc_succeed)
        return sqrt_info; 

    /*// Chapter 6 "Statistical Optimization for Geometric Computation"
    double epsilon = 1.5 / FOCAL_LENGTH; 
    
    Eigen::Matrix3d R = Rrl.transpose();
    Vector3d h = Trl;  
    Eigen::Matrix3d G = Utility::skewSymmetric(h); 
    Eigen::Matrix3d Gt = G.transpose(); 
    Eigen::Matrix3d Pk = Eigen::Matrix3d::Identity(); 
    Pk(2,2) = 0; 
    Eigen::Vector3d x1 = Pk*G*pointRight;
    Eigen::Matrix3d XX = x1*x1.transpose(); 
    Eigen::Vector3d x2 = Pk*Gt*point; 
    double de = x1.squaredNorm() + x2.squaredNorm(); 
    Eigen::Matrix2d XX2 = XX.block<2,2>(0,0)/de; 
    Eigen::Matrix2d cov = Eigen::Matrix2d::Identity() - XX2; 
    Eigen::Matrix2d cc = Eigen::Matrix2d::Zero(); 
    cc(0,0) = sqrt(cov(0,0)); 
    cc(1,1) = sqrt(cov(1,1)); 
    cov = epsilon * cc; 

    sqrt_info = cov.inverse();*/
    return sqrt_info; 
}

Eigen::Matrix2d FeaturePerFrame::getOmegaRight() // inverse of covariance matrix
{
    Eigen::Matrix2d sqrt_info = FOCAL_LENGTH / 1.5 * Matrix2d::Identity();

    if(getDepth() <= 0) {
        ROS_ERROR("feature_manager.cpp: something is wrong!"); 
        return sqrt_info; 
    }

    double depth = getDepth(); 
    double weighting = -SQ(depth - 1)/81. + 1.; 
    if(weighting <0 ) weighting = 0.01;

    if(!g_use_stereo_correction)
        return weighting*sqrt_info; 

    /*// Chapter 6 "Statistical Optimization for Geometric Computation"
    double epsilon = 1.5 / FOCAL_LENGTH; 
    
    Eigen::Matrix3d R = Rrl.transpose();
    Vector3d h = Trl;  
    Eigen::Matrix3d G = Utility::skewSymmetric(h); 
    Eigen::Matrix3d Gt = G.transpose(); 
    Eigen::Matrix3d Pk = Eigen::Matrix3d::Identity(); 
    Pk(2,2) = 0; 
    Eigen::Vector3d x1 = Pk*G*pointRight;
    Eigen::Vector3d x2 = Pk*Gt*point; 
    Eigen::Matrix3d XX = x2*x2.transpose(); 
    double de = x1.squaredNorm() + x2.squaredNorm(); 
    Eigen::Matrix2d XX2 = XX.block<2,2>(0,0)/de; 
    Eigen::Matrix2d cov = Eigen::Matrix2d::Identity() - XX2; 
    Eigen::Matrix2d cc = Eigen::Matrix2d::Zero(); 
    cc(0,0) = sqrt(cov(0,0)); 
    cc(1,1) = sqrt(cov(1,1)); 
    cov = epsilon * cc; 

    sqrt_info = cov.inverse(); */
    return weighting*sqrt_info; 
}

// triangulation to compute depth 
double FeaturePerFrame::getDepth()
{
    if(!is_stereo) return -1; 
    if(dpt > 0) return dpt; 
    if(point(0) < pointRight(0)){
        is_stereo = false; // this is false stereo match, since depth < 0 
        dpt = -1; 
        return -1; 
    }

    // check the triangulated depth 
    Eigen::Matrix<double, 3, 4> leftPose = Eigen::Matrix<double, 3, 4>::Zero();
    leftPose.leftCols<3>() = Eigen::Matrix3d::Identity(); 

    Eigen::Matrix<double, 3, 4> rightPose = Eigen::Matrix<double, 3, 4>::Zero(); 
    rightPose.leftCols<3>() = Rrl; 
    rightPose.rightCols<1>() = Trl; 

    Eigen::Vector2d point0, point1; 
    Eigen::Vector3d point3d; 
    point0 = point.head(2); 
    point1 = pointRight.head(2); 

    ((FeatureManager*)0)->triangulatePoint(leftPose, rightPose, point0, point1, point3d); 

    double depth = point3d.z(); 
    // if(depth <= 0.3 || depth >= 7){ // too small or too large depth, not reliable 
    // TODO: find out optimal threshold for distant point 
    if(depth <= 1. || depth >= 7.){ // barely no object so close to the camera 
        // ROS_ERROR("feature_manager.cpp: what? depth: %lf", depth); 
        // cout<<"feature_id: "<< feat_id <<"point0: "<<point0.transpose()<<" point1: "<<point1.transpose()<<" point3d: "<<point3d.transpose()<<endl;
        is_stereo = false; 
        dpt = -1; 

        return -1; 
    }

    // check reprojection error 
    Eigen::Vector3d proj_pt0, proj_pt1; 
    proj_pt0 = point3d / depth; 
    proj_pt1 = Rrl * point3d + Trl; 
    if(proj_pt1.z()<= 1.){
        is_stereo = false; 
        dpt = -1; 
        return -1; 
    }
    proj_pt1 = proj_pt1/proj_pt1.z(); 

    Eigen::Vector2d err_pt0 = proj_pt0.head(2) - point0; 
    Eigen::Vector2d err_pt1 = proj_pt1.head(2) - point1; 

    double ep0_norm = err_pt0.norm(); 
    double ep1_norm = err_pt1.norm(); 

    if((err_pt0.norm() > 2./FOCAL_LENGTH) || (err_pt1.norm() > 2./FOCAL_LENGTH)){
        is_stereo = false; 
        dpt = -1; 
   
        // for debug 
        // static int cnt =0; 
        // ROS_ERROR("really, we have %d wrong triangulations err_pt0: %lf  err_pt1: %lf", ++cnt, err_pt0.norm(), err_pt1.norm()); 
        return - 1; 
    }

    dpt = depth; 

    if(!g_use_stereo_correction) // don't apply geometric correction 
        return dpt; 

    // stereo correction, Chapter 6 "Statistical Optimization for Geometric Computation"
    Vector3d np0 = point; 
    Vector3d np1 = pointRight; 

    Eigen::Matrix3d R = Rrl.transpose();
    Vector3d h = Trl;  
    Eigen::Matrix3d G = Utility::skewSymmetric(h)*R; 
    Eigen::Matrix3d Gt = G.transpose(); 
    Eigen::Matrix3d Pk = Eigen::Matrix3d::Identity(); 
    Pk(2,2) = 0; 

    double fe = np0.transpose() * G * np1; 
    // double de1 = np1.transpose() * Gt * G * np1; 
    // double de2 = np0.transpose() * G * Gt * np0;
    // double de = de1 + de2; 
    Vector3d ve1 =  Pk*Gt*np0;
    Vector3d ve2 =  Pk*G*np1;
    double de = ve1.squaredNorm() + ve2.squaredNorm();
    Vector3d delta_np0 = fe * Pk * G * np1; 
    Vector3d delta_np1 = fe * Pk * Gt * np0; 

    // temporary solution, needs to improve 
    // TODO: update uv and uvRight as well 

    Vector3d new_p0 = np0 - delta_np0/de; 
    Vector3d new_p1 = np1 - delta_np1/de; 

    point0 = new_p0.head(2); 
    point1 = new_p1.head(2); 

    ((FeatureManager*)0)->triangulatePoint(leftPose, rightPose, point0, point1, point3d); 
    
    // check reprojection error 
    proj_pt0 = point3d / point3d.z(); 
    proj_pt1 = Rrl * point3d + Trl; 
    if(proj_pt1.z()<= 1.){
        is_stereo = false; 
        dpt = -1; 
        return -1; 
    }
    proj_pt1 = proj_pt1/proj_pt1.z(); 

    err_pt0 = proj_pt0.head(2) - point0; 
    err_pt1 = proj_pt1.head(2) - point1; 
    double new_ep0_norm = err_pt0.norm();
    double new_ep1_norm = err_pt1.norm(); 
    if(new_ep0_norm > ep0_norm || new_ep1_norm > ep1_norm){
        gc_succeed = false; 
        return depth ; 
    }

    point = new_p0; 
    pointRight = new_p1; 
    depth = point3d.z(); 
    dpt = depth; 
    gc_succeed = true; 
    return depth; 
}

int FeaturePerId::endFrame()
{
    return start_frame + feature_per_frame.size() - 1;
}

FeatureManager::FeatureManager(Matrix3d _Rs[])
    : Rs(_Rs)
{
    for (int i = 0; i < NUM_OF_CAM; i++)
        ric[i].setIdentity();
}

void FeatureManager::setRic(Matrix3d _ric[])
{
    for (int i = 0; i < NUM_OF_CAM; i++)
    {
        ric[i] = _ric[i];
    }
}

void FeatureManager::clearState()
{
    feature.clear();
}

int FeatureManager::getFeatureCount()
{
    int cnt = 0;
    for (auto &it : feature)
    {

        it.used_num = it.feature_per_frame.size();

        if (it.used_num >= 2 && it.start_frame < WINDOW_SIZE - 2)
        {
            cnt++;
        }
    }
    return cnt;
}


bool FeatureManager::addFeatureCheckParallax(int frame_count, const map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> &image, double td)
{
    ROS_DEBUG("input feature: %d", (int)image.size());
    // ROS_DEBUG("num of feature: %d", getFeatureCount());
    // ROS_WARN("before addFeatureCheckParallax num of feature: %d total feature: %d", getFeatureCount(), feature.size());
    double parallax_sum = 0;
    int parallax_num = 0;
    last_track_num = 0;
    last_average_parallax = 0;
    long_track_num = 0; 
    new_feature_num = 0; 
    for (auto &id_pts : image)
    {
        FeaturePerFrame f_per_fra(id_pts.second[0].second, td);
        f_per_fra.feat_id = id_pts.first; 
        if(id_pts.second[0].first != 0)
            ROS_ERROR("what? second[0].first = %d", id_pts.second[0].first );
        assert(id_pts.second[0].first == 0);
        if(id_pts.second.size() == 2)
        {
            f_per_fra.rightObservation(id_pts.second[1].second);
            assert(id_pts.second[1].first == 1);
        }

        int feature_id = id_pts.first;
        auto it = find_if(feature.begin(), feature.end(), [feature_id](const FeaturePerId &it)
                          {
            return it.feature_id == feature_id;
                          });

        if (it == feature.end())
        {
            feature.push_back(FeaturePerId(feature_id, frame_count));
            feature.back().feature_per_frame.push_back(f_per_fra);
            new_feature_num++;
        }
        else if (it->feature_id == feature_id)
        {
            it->feature_per_frame.push_back(f_per_fra);
            last_track_num++;
            if( it-> feature_per_frame.size() >= 4)
                long_track_num++;
        }
    }

    // ROS_INFO("after addFeatureCheckParallax num of feature: %d last_track_num: %d", getFeatureCount(), last_track_num);

    // if (frame_count < 2 || last_track_num < 20 || long_track_num < 40 || new_feature_num > 0.5 * last_track_num)
    //    return true;
    if (frame_count < 2 || last_track_num < 20)
        return true;

    for (auto &it_per_id : feature)
    {
        if (it_per_id.start_frame <= frame_count - 2 &&
            it_per_id.start_frame + int(it_per_id.feature_per_frame.size()) - 1 >= frame_count - 1)
        {
            parallax_sum += compensatedParallax2(it_per_id, frame_count);
            parallax_num++;
        }
    }

    if (parallax_num == 0)
    {
        return true;
    }
    else
    {
        ROS_DEBUG("parallax_sum: %lf, parallax_num: %d", parallax_sum, parallax_num);
        ROS_DEBUG("current parallax: %lf", parallax_sum / parallax_num * FOCAL_LENGTH);
        // last_average_parallax = parallax_sum / parallax_num * FOCAL_LENGTH;
        return parallax_sum / parallax_num >= MIN_PARALLAX;
    }
}

void FeatureManager::debugShow()
{
    ROS_DEBUG("debug show");
    for (auto &it : feature)
    {
        ROS_ASSERT(it.feature_per_frame.size() != 0);
        ROS_ASSERT(it.start_frame >= 0);
        ROS_ASSERT(it.used_num >= 0);

        ROS_DEBUG("%d,%d,%d ", it.feature_id, it.used_num, it.start_frame);
        int sum = 0;
        for (auto &j : it.feature_per_frame)
        {
            ROS_DEBUG("%d,", int(j.is_used));
            sum += j.is_used;
            printf("(%lf,%lf) ",j.point(0), j.point(1));
        }
        ROS_ASSERT(it.used_num == sum);
    }
}

vector<pair<Vector3d, Vector3d>> FeatureManager::getCorresponding(int frame_count_l, int frame_count_r)
{
    vector<pair<Vector3d, Vector3d>> corres;
    for (auto &it : feature)
    {
        if (it.start_frame <= frame_count_l && it.endFrame() >= frame_count_r)
        {
            Vector3d a = Vector3d::Zero(), b = Vector3d::Zero();
            int idx_l = frame_count_l - it.start_frame;
            int idx_r = frame_count_r - it.start_frame;

            a = it.feature_per_frame[idx_l].point;

            b = it.feature_per_frame[idx_r].point;

            corres.push_back(make_pair(a, b));
        }
    }
    return corres;
}

vector<pair<Vector3d, Vector3d>> FeatureManager::getCorrespondingWithDepth(int frame_count_l, int frame_count_r)
{
    vector<pair<Vector3d, Vector3d>> corres;
    for (auto &it : feature)
    {
        if (it.start_frame <= frame_count_l && it.endFrame() >= frame_count_r)
        {
            Vector3d a = Vector3d::Zero(), b = Vector3d::Zero();
            int idx_l = frame_count_l - it.start_frame;
            int idx_r = frame_count_r - it.start_frame;

            a = it.feature_per_frame[idx_l].point;
            a.z() = it.feature_per_frame[idx_l].getDepth(); 

            b = it.feature_per_frame[idx_r].point;
            b.z() = it.feature_per_frame[idx_r].getDepth();
            
            corres.push_back(make_pair(a, b));
        }
    }
    return corres;
}


void FeatureManager::setDepth(const VectorXd &x)
{
    int feature_index = -1;
    for (auto &it_per_id : feature)
    {
        it_per_id.used_num = it_per_id.feature_per_frame.size();
        if (!(it_per_id.used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
            continue;

        it_per_id.estimated_depth = 1.0 / x(++feature_index);
        //ROS_INFO("feature id %d , start_frame %d, depth %f ", it_per_id->feature_id, it_per_id-> start_frame, it_per_id->estimated_depth);
        if (it_per_id.estimated_depth < 0)
        {
            it_per_id.solve_flag = 2;
        }
        else
            it_per_id.solve_flag = 1;
    }
}

void FeatureManager::removeFailures()
{
    for (auto it = feature.begin(), it_next = feature.begin();
         it != feature.end(); it = it_next)
    {
        it_next++;
        if (it->solve_flag == 2)
            feature.erase(it);
    }
}

void FeatureManager::clearDepth(const VectorXd &x)
{
    int feature_index = -1;
    for (auto &it_per_id : feature)
    {
        it_per_id.used_num = it_per_id.feature_per_frame.size();
        if (!(it_per_id.used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
            continue;
        it_per_id.estimated_depth = 1.0 / x(++feature_index);
    }
}

VectorXd FeatureManager::getDepthVector()
{
    VectorXd dep_vec(getFeatureCount());
    int feature_index = -1;
    for (auto &it_per_id : feature)
    {
        it_per_id.used_num = it_per_id.feature_per_frame.size();
        if (!(it_per_id.used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
            continue;
#if 1
        dep_vec(++feature_index) = 1. / it_per_id.estimated_depth;
#else
        dep_vec(++feature_index) = it_per_id->estimated_depth;
#endif
    }
    return dep_vec;
}

void FeatureManager::triangulate(Vector3d Ps[], Vector3d tic[], Matrix3d ric[])
{
    for (auto &it_per_id : feature)
    {
        it_per_id.used_num = it_per_id.feature_per_frame.size();
        if (!(it_per_id.used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
            continue;

        if (it_per_id.estimated_depth > 0)
            continue;
        int imu_i = it_per_id.start_frame, imu_j = imu_i - 1;

        // ROS_ASSERT(NUM_OF_CAM == 1);
        Eigen::MatrixXd svd_A(2 * it_per_id.feature_per_frame.size(), 4);
        int svd_idx = 0;

        Eigen::Matrix<double, 3, 4> P0;
        Eigen::Vector3d t0 = Ps[imu_i] + Rs[imu_i] * tic[0];
        Eigen::Matrix3d R0 = Rs[imu_i] * ric[0];
        P0.leftCols<3>() = Eigen::Matrix3d::Identity();
        P0.rightCols<1>() = Eigen::Vector3d::Zero();

        for (auto &it_per_frame : it_per_id.feature_per_frame)
        {
            imu_j++;

            Eigen::Vector3d t1 = Ps[imu_j] + Rs[imu_j] * tic[0];
            Eigen::Matrix3d R1 = Rs[imu_j] * ric[0];
            Eigen::Vector3d t = R0.transpose() * (t1 - t0);
            Eigen::Matrix3d R = R0.transpose() * R1;
            Eigen::Matrix<double, 3, 4> P;
            P.leftCols<3>() = R.transpose();
            P.rightCols<1>() = -R.transpose() * t;
            Eigen::Vector3d f = it_per_frame.point.normalized();
            svd_A.row(svd_idx++) = f[0] * P.row(2) - f[2] * P.row(0);
            svd_A.row(svd_idx++) = f[1] * P.row(2) - f[2] * P.row(1);

            if (imu_i == imu_j)
                continue;
        }
        ROS_ASSERT(svd_idx == svd_A.rows());
        Eigen::Vector4d svd_V = Eigen::JacobiSVD<Eigen::MatrixXd>(svd_A, Eigen::ComputeThinV).matrixV().rightCols<1>();
        double svd_method = svd_V[2] / svd_V[3];
        //it_per_id->estimated_depth = -b / A;
        //it_per_id->estimated_depth = svd_V[2] / svd_V[3];

        it_per_id.estimated_depth = svd_method;
        //it_per_id->estimated_depth = INIT_DEPTH;

        if (it_per_id.estimated_depth < 0.1)
        {
            it_per_id.estimated_depth = INIT_DEPTH;
        }

    }
}

void FeatureManager::triangulatePoint(Eigen::Matrix<double, 3, 4> &Pose0, Eigen::Matrix<double, 3, 4> &Pose1,
                        Eigen::Vector2d &point0, Eigen::Vector2d &point1, Eigen::Vector3d &point_3d)
{
    Eigen::Matrix4d design_matrix = Eigen::Matrix4d::Zero();
    design_matrix.row(0) = point0[0] * Pose0.row(2) - Pose0.row(0);
    design_matrix.row(1) = point0[1] * Pose0.row(2) - Pose0.row(1);
    design_matrix.row(2) = point1[0] * Pose1.row(2) - Pose1.row(0);
    design_matrix.row(3) = point1[1] * Pose1.row(2) - Pose1.row(1);
    Eigen::Vector4d triangulated_point;
    triangulated_point =
              design_matrix.jacobiSvd(Eigen::ComputeFullV).matrixV().rightCols<1>();
    point_3d(0) = triangulated_point(0) / triangulated_point(3);
    point_3d(1) = triangulated_point(1) / triangulated_point(3);
    point_3d(2) = triangulated_point(2) / triangulated_point(3);
}

void FeatureManager::triangulateStereo()
{
    for (auto &it_per_id : feature)
    {
        if (it_per_id.estimated_depth > 0)
            continue;
        double depth = it_per_id.feature_per_frame[0].getDepth(); 
        if(depth > 0){
            it_per_id.estimated_depth = depth;
        }
    }
}

// this function not good 
void FeatureManager::triangulateWithDepth(Vector3d Ps[], Vector3d tic[], Matrix3d ric[])
{
    for (auto &it_per_id : feature)
    {
        it_per_id.used_num = it_per_id.feature_per_frame.size();
        if (!(it_per_id.used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
            continue;

        if (it_per_id.estimated_depth > 0)
            continue;

        int start_frame = it_per_id.start_frame;

        vector<double> verified_depths;

        Eigen::Vector3d tr = Ps[start_frame] + Rs[start_frame] * tic[0]; 
        Eigen::Matrix3d Rr = Rs[start_frame] * ric[0];                   

        for (int i=0; i < (int)it_per_id.feature_per_frame.size(); i++)
        {
            Eigen::Vector3d t0 = Ps[start_frame+i] + Rs[start_frame+i] * tic[0]; 
            Eigen::Matrix3d R0 = Rs[start_frame+i] * ric[0];
            double depth_threshold = 7; //for handheld and wheeled application. Since d435i <3 is quiet acc
            //double depth_threshold = 10; //for tracked application, since IMU quite noisy in this scene             
            // if (it_per_id.feature_per_frame[i].getDepth() < 0.3 || it_per_id.feature_per_frame[i].getDepth() >depth_threshold) 
            if (it_per_id.feature_per_frame[i].getDepth() < 0.3) 
                continue;
            Eigen::Vector3d point0(it_per_id.feature_per_frame[i].point * it_per_id.feature_per_frame[i].getDepth());

            // transform to reference frame
            Eigen::Vector3d t2r = Rr.transpose() * (t0 - tr);
            Eigen::Matrix3d R2r = Rr.transpose() * R0;        

            /*
            for (int j=0; j<(int)it_per_id.feature_per_frame.size(); j++)
            {
                if (i==j)
                    continue;
                Eigen::Vector3d t1 = Ps[start_frame+j] + Rs[start_frame+j] * tic[0];
                Eigen::Matrix3d R1 = Rs[start_frame+j] * ric[0];
                Eigen::Vector3d t20 = R0.transpose() * (t1 - t0); 
                Eigen::Matrix3d R20 = R0.transpose() * R1;        


                Eigen::Vector3d point1_projected = R20.transpose() * point0 - R20.transpose() * t20;
                Eigen::Vector2d point1_2d(it_per_id.feature_per_frame[j].pt.x(), it_per_id.feature_per_frame[j].pt.y());
                Eigen::Vector2d residual = point1_2d - Vector2d(point1_projected.x() / point1_projected.z(), point1_projected.y() / point1_projected.z());
                if (residual.norm() < 5.0 / 460) {//this can also be adjust to improve performance
                    Eigen::Vector3d point_r = R2r * point0 + t2r;
                    verified_depths.push_back(point_r.z());
                }
            }*/

            Eigen::Vector3d point1_projected = R2r * point0 + t2r;
            Eigen::Vector2d point1_2d(it_per_id.feature_per_frame[0].point.x(), it_per_id.feature_per_frame[0].point.y());
            Eigen::Vector2d residual;
            if(point1_projected.z() < 0.1)
                residual = Vector2d(10,10); 
            else
                residual = point1_2d - Vector2d(point1_projected.x() / point1_projected.z(), point1_projected.y() / point1_projected.z());
            if (residual.norm() < 2.0 / 460) {//this can also be adjust to improve performance
                // Eigen::Vector3d point_r = R2r * point0 + t2r;
                verified_depths.push_back(point1_projected.z());
            }
        }

        if (verified_depths.size() == 0)
            continue;
        double depth_sum = std::accumulate(std::begin(verified_depths),std::end(verified_depths),0.0);
        double depth_ave = depth_sum / verified_depths.size();
//        for (int i=0;i<(int)verified_depths.size();i++){
//            cout << verified_depths[i]<<"|";
//        }
//        cout << endl;        

        if (depth_ave < 0.1)
        {
            it_per_id.estimated_depth = -1; // INIT_DEPTH;
            it_per_id.solve_flag = 0;
            it_per_id.dpt_type = NO_DEPTH; 
        }else{
            it_per_id.estimated_depth = depth_ave;
            it_per_id.solve_flag = 1;
            it_per_id.dpt_type = DEPTH_MES; // actually averaged estimation
        }

    }
}


void FeatureManager::removeOutlier()
{
    ROS_BREAK();
    int i = -1;
    for (auto it = feature.begin(), it_next = feature.begin();
         it != feature.end(); it = it_next)
    {
        it_next++;
        i += it->used_num != 0;
        if (it->used_num != 0 && it->is_outlier == true)
        {
            feature.erase(it);
        }
    }
}

void FeatureManager::removeBackShiftDepth(Eigen::Matrix3d marg_R, Eigen::Vector3d marg_P, Eigen::Matrix3d new_R, Eigen::Vector3d new_P)
{
    int cnt_invalid_depth = 0;
    int cnt_deleted_feat = 0;
    for (auto it = feature.begin(), it_next = feature.begin();
         it != feature.end(); it = it_next)
    {
        it_next++;

        if (it->start_frame != 0)
            it->start_frame--;
        else
        {
            Eigen::Vector3d uv_i = it->feature_per_frame[0].point;
            it->feature_per_frame.erase(it->feature_per_frame.begin());
            if (it->feature_per_frame.size() < 2)
            {
                feature.erase(it);
		            ++cnt_deleted_feat;
                continue;
            }
            else
            {
                Eigen::Vector3d pts_i = uv_i * it->estimated_depth;
                Eigen::Vector3d w_pts_i = marg_R * pts_i + marg_P;
                Eigen::Vector3d pts_j = new_R.transpose() * (w_pts_i - new_P);
                double dep_j = pts_j(2);
                if (dep_j > 0)
                    it->estimated_depth = dep_j;
                else{
                    it->estimated_depth = INIT_DEPTH;
		                ++cnt_invalid_depth;
                }
            }
        }
        // remove tracking-lost feature after marginalize
        /*
        if (it->endFrame() < WINDOW_SIZE - 1)
        {
            feature.erase(it);
        }
        */
    }
   // ROS_WARN("feature_manager.cpp: number of feature depth invalid %d, number of feature to be deleted: %d", cnt_invalid_depth, cnt_deleted_feat);
}

void FeatureManager::removeBack()
{
    for (auto it = feature.begin(), it_next = feature.begin();
         it != feature.end(); it = it_next)
    {
        it_next++;

        if (it->start_frame != 0)
            it->start_frame--;
        else
        {
            it->feature_per_frame.erase(it->feature_per_frame.begin());
            if (it->feature_per_frame.size() == 0)
                feature.erase(it);
        }
    }
}

void FeatureManager::removeFront(int frame_count)
{
    for (auto it = feature.begin(), it_next = feature.begin(); it != feature.end(); it = it_next)
    {
        it_next++;

        if (it->start_frame == frame_count)
        {
            it->start_frame--;
        }
        else
        {
            int j = WINDOW_SIZE - 1 - it->start_frame;
            if (it->endFrame() < frame_count - 1)
                continue;
            it->feature_per_frame.erase(it->feature_per_frame.begin() + j);
            if (it->feature_per_frame.size() == 0)
                feature.erase(it);
        }
    }
}

double FeatureManager::compensatedParallax2(const FeaturePerId &it_per_id, int frame_count)
{
    //check the second last frame is keyframe or not
    //parallax betwwen seconde last frame and third last frame
    const FeaturePerFrame &frame_i = it_per_id.feature_per_frame[frame_count - 2 - it_per_id.start_frame];
    const FeaturePerFrame &frame_j = it_per_id.feature_per_frame[frame_count - 1 - it_per_id.start_frame];

    double ans = 0;
    Vector3d p_j = frame_j.point;

    double u_j = p_j(0);
    double v_j = p_j(1);

    Vector3d p_i = frame_i.point;
    Vector3d p_i_comp;

    //int r_i = frame_count - 2;
    //int r_j = frame_count - 1;
    //p_i_comp = ric[camera_id_j].transpose() * Rs[r_j].transpose() * Rs[r_i] * ric[camera_id_i] * p_i;
    p_i_comp = p_i;
    double dep_i = p_i(2);
    double u_i = p_i(0) / dep_i;
    double v_i = p_i(1) / dep_i;
    double du = u_i - u_j, dv = v_i - v_j;

    double dep_i_comp = p_i_comp(2);
    double u_i_comp = p_i_comp(0) / dep_i_comp;
    double v_i_comp = p_i_comp(1) / dep_i_comp;
    double du_comp = u_i_comp - u_j, dv_comp = v_i_comp - v_j;

    ans = max(ans, sqrt(min(du * du + dv * dv, du_comp * du_comp + dv_comp * dv_comp)));

    return ans;
}

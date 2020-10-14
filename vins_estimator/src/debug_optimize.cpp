/*
	10.12.2020 He Zhang, hzhang8@vcu.edu 

	find out the reason why add stereo in optimization diverge the result 

*/

#include <stdio.h>
#include <queue>
#include <map>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include "parameters.h"
#include "utility/utility.h"
#include "utility/tic_toc.h"
#include <std_msgs/Header.h>
#include <std_msgs/Float32.h>

#include <ceres/ceres.h>
#include "factor/imu_factor.h"
#include "factor/pose_local_parameterization.h"
#include "factor/projection_factor.h"
#include "factor/projection_td_factor.h"
#include "factor/marginalization_factor.h"
#include "factor/sampson_factor.h"
#include "factor/projectionOneFrameTwoCamFactor.h"
#include "factor/projectionTwoFrameOneCamFactor.h"
#include "factor/projectionTwoFrameTwoCamFactor.h"
#include <string>
#include <iostream>
#include <fstream>
#include <vector>
#include <map>

using namespace Eigen; 
using namespace std; 
#define SQ(x) (((x)*(x)))

struct FeatFactor{
	int imu_i, imu_j; 
	string LR; 
	Vector3d pts_i, pts_j; 
	Vector2d pi_vel, pj_vel; 
	double pi_td, pj_td; 
	int feat_id; 
	double inv_dpt; 
};

map<int, vector<FeatFactor> > featMeasure;


string debug_file(""); 

bool readMeasure(string line, FeatFactor& fac); 
bool readFactors(string); 
void optimize(); 

void afterOptimize(); 

double para_Pose[WINDOW_SIZE + 1][SIZE_POSE];
double para_SpeedBias[WINDOW_SIZE + 1][SIZE_SPEEDBIAS];
double para_Feature[NUM_OF_F][SIZE_FEATURE];
double para_Ex_Pose[2][SIZE_POSE];
double para_Td[1][1];
IntegrationBase *pre_integrations[(WINDOW_SIZE + 1)];

void printPoseBias()
{	
	for(int i=0; i<WINDOW_SIZE+1; i++){
		cout<<" pose "<<i<<" ";
		for(int j=0; j<3; j++){
			cout<<para_Pose[i][j]<<" ";
		}
		cout << endl;

		for(int j=0; j<3; j++)
			cout<<"tic1: "<<para_Ex_Pose[0][j]<<" "; 
		cout<<endl; 

		Quaterniond qic1(para_Ex_Pose[0][6], para_Ex_Pose[0][3], para_Ex_Pose[0][4], para_Ex_Pose[0][5]); 
		cout <<" Ric1: "<<endl<<qic1.toRotationMatrix()<<endl; 

		for(int j=0; j<3; j++)
			cout<<"tic2: "<<para_Ex_Pose[1][j]<<" "; 
		cout<<endl; 

		Quaterniond qic2(para_Ex_Pose[1][6], para_Ex_Pose[1][3], para_Ex_Pose[1][4], para_Ex_Pose[1][5]); 
		cout <<" Ric2: "<<endl<<qic2.toRotationMatrix()<<endl; 

		cout<<" bias: ba "; 
		for(int j=0; j<3; j++){
			cout<<para_SpeedBias[i][3+j]<<" ";
		} 
		cout<<endl; 
	}
}

int main(int argc, char* argv[])
{
	if(argc >= 2)
		debug_file = argv[1]; 

	if(!readFactors(debug_file))
		return 0; 

	optimize(); 
	afterOptimize();

	return 0; 
}

void optimize(){

	ofstream ouf("before_opt.txt");
	// ProjectionTwoFrameOneCamFactor::sqrt_info = FOCAL_LENGTH / 1.5 * Matrix2d::Identity();
    ProjectionTwoFrameTwoCamFactor::sqrt_info = FOCAL_LENGTH / 1.5 * Matrix2d::Identity();
    ProjectionOneFrameTwoCamFactor::sqrt_info = FOCAL_LENGTH / 1.5 * Matrix2d::Identity();

	ceres::Problem problem;
    ceres::LossFunction *loss_function;
    //loss_function = new ceres::HuberLoss(1.0);
    loss_function = new ceres::CauchyLoss(1.0);
    for (int i = 0; i < WINDOW_SIZE + 1; i++){
        ceres::LocalParameterization *local_parameterization = new PoseLocalParameterization();
        problem.AddParameterBlock(para_Pose[i], SIZE_POSE, local_parameterization);
        problem.AddParameterBlock(para_SpeedBias[i], SIZE_SPEEDBIAS);
    }
	for(int i=0; i<2; i++){ // extrinsic parameter 
    	ceres::LocalParameterization *local_parameterization = new PoseLocalParameterization();
    	problem.AddParameterBlock(para_Ex_Pose[i], SIZE_POSE, local_parameterization);
        problem.SetParameterBlockConstant(para_Ex_Pose[i]);
     
    }
  	problem.AddParameterBlock(para_Td[0], 1);
  	// problem.SetParameterBlockConstant(para_Td[0]);

  	double total_sq_norm = 0; 

    for (int i = 0; i < WINDOW_SIZE; i++)
    {
        int j = i + 1;
        IMUFactor* imu_factor = new IMUFactor(pre_integrations[j]);
        ceres::ResidualBlockId fid = problem.AddResidualBlock(imu_factor, NULL, para_Pose[i], para_SpeedBias[i], para_Pose[j], para_SpeedBias[j]);

        vector<double*>* para = new vector<double*>;  
        problem.GetParameterBlocksForResidualBlock(fid, para); 
        vector<double> res(15); 
        imu_factor->Evaluate(&para[0][0], &res[0], 0); 
        Eigen::Map<Eigen::Matrix<double, 15, 1>> residual(&res[0]);
        ouf<<"debug_optimize.cpp: IMU factor " <<j<<" has residual: "<< residual.squaredNorm()<<endl;
        total_sq_norm += residual.squaredNorm(); 
        // ouf << f_m_cnt<<" "<<pts_i.transpose()<<" "<<pts_j.transpose()<<" "<<para_Feature[feature_index][0]<<" "<< res[0]<<" "<<res[1]<<endl;
    }

    int feature_index = 0; 
    map<int, vector<FeatFactor> >::iterator it = featMeasure.begin(); 
    int L_cnt_vc = 0; 
    int R_cnt_vc = 0; 
    while(it!= featMeasure.end()){
    	
    	para_Feature[feature_index][0] = it->second[0].inv_dpt; 
    	// problem.AddParameterBlock(para_Feature[feature_index], 1);

    	for(int i=0; i<it->second.size(); i++){
    		FeatFactor& ft = it->second[i]; 

    		if(ft.LR == "L"){
    			ProjectionTwoFrameOneCamFactor *f_td = new ProjectionTwoFrameOneCamFactor(ft.pts_i, ft.pts_j, ft.pi_vel, ft.pj_vel, ft.pi_td, ft.pj_td); 
    			ceres::ResidualBlockId fid = problem.AddResidualBlock(f_td, loss_function, para_Pose[ft.imu_i], para_Pose[ft.imu_j], para_Ex_Pose[0], para_Feature[feature_index], para_Td[0]);
    			// ceres::ResidualBlockId fid = problem.AddResidualBlock(f_td, NULL, para_Pose[ft.imu_i], para_Pose[ft.imu_j], para_Ex_Pose[0], para_Feature[feature_index], para_Td[0]);

    			L_cnt_vc++;
    			vector<double*>* para = new vector<double*>;  
                problem.GetParameterBlocksForResidualBlock(fid, para); 
                vector<double> res(2); 
                f_td->Evaluate(&para[0][0], &res[0], 0); 
                double sn = SQ(res[0])+SQ(res[1]); 

                ouf<<"L feature " <<feature_index<<" "<< ft.imu_i<<" "<<ft.imu_j<<" depth: "<<1./para_Feature[feature_index][0]<<" residual: "<<res[0]<<" "<<res[1]<<" "<< sn <<endl;
                total_sq_norm += sn; 
                // ouf << f_m_cnt<<" "<<pts_i.transpose()<<" "<<pts_j.transpose()<<" "<<para_Feature[feature_index][0]<<" "<< res[0]<<" "<<res[1]<<endl;

    		}else{

                ProjectionTwoFrameTwoCamFactor *f = new ProjectionTwoFrameTwoCamFactor(ft.pts_i, ft.pts_j, ft.pi_vel, ft.pj_vel, ft.pi_td, ft.pj_td);
                ceres::ResidualBlockId fid = problem.AddResidualBlock(f, loss_function, para_Pose[ft.imu_i], para_Pose[ft.imu_j], para_Ex_Pose[0], para_Ex_Pose[1], para_Feature[feature_index], para_Td[0]);
                // ceres::ResidualBlockId fid = problem.AddResidualBlock(f, NULL, para_Pose[ft.imu_i], para_Pose[ft.imu_j], para_Ex_Pose[0], para_Ex_Pose[1], para_Feature[feature_index], para_Td[0]);

                R_cnt_vc++;

                vector<double*>* para = new vector<double*>;  
                problem.GetParameterBlocksForResidualBlock(fid, para); 
                vector<double> res(2); 
                f->Evaluate(&para[0][0], &res[0], 0);
                double sn = SQ(res[0])+SQ(res[1]);  
                ouf<<"R feature " <<feature_index<<" "<< ft.imu_i<<" "<<ft.imu_j<<" depth: "<<1./para_Feature[feature_index][0]<<" residual: "<<res[0]<<" "<<res[1]<<" "<<sn<<endl; 
                total_sq_norm += sn; 
    		}
    	}
    	it++;
    	feature_index++;
    }
    cout<<"visual measurements: "<<L_cnt_vc+R_cnt_vc<<" L_cnt: "<<L_cnt_vc<<" R_cnt: "<<R_cnt_vc<<endl; 
    cout <<"total squared norm is "<<total_sq_norm<<endl; 
    ceres::Solver::Options options;

    options.linear_solver_type = ceres::DENSE_SCHUR;
    //options.num_threads = 2;
    options.trust_region_strategy_type = ceres::DOGLEG;
    options.max_num_iterations = 10; // NUM_ITERATIONS;
    options.use_explicit_schur_complement = true;
    options.minimizer_progress_to_stdout = true;
    //options.use_nonmonotonic_steps = true;
    // options.max_solver_time_in_seconds = SOLVER_TIME;
    TicToc t_solver;
    ceres::Solver::Summary summary;
    printPoseBias(); 
    ceres::Solve(options, &problem, &summary);
    printPoseBias();




    return ; 

}

void afterOptimize()
{
	ofstream ouf("after_opt.txt"); 
	ceres::Problem problem;
    ceres::LossFunction *loss_function;
    //loss_function = new ceres::HuberLoss(1.0);
    loss_function = new ceres::CauchyLoss(1.0);
    for (int i = 0; i < WINDOW_SIZE + 1; i++){
        ceres::LocalParameterization *local_parameterization = new PoseLocalParameterization();
        problem.AddParameterBlock(para_Pose[i], SIZE_POSE, local_parameterization);
        problem.AddParameterBlock(para_SpeedBias[i], SIZE_SPEEDBIAS);
    }
	for(int i=0; i<2; i++){ // extrinsic parameter 
    	ceres::LocalParameterization *local_parameterization = new PoseLocalParameterization();
    	problem.AddParameterBlock(para_Ex_Pose[i], SIZE_POSE, local_parameterization);
        problem.SetParameterBlockConstant(para_Ex_Pose[i]);
     
    }
  	problem.AddParameterBlock(para_Td[0], 1);
  	double total_sq_norm = 0; 
  	// problem.SetParameterBlockConstant(para_Td[0]);
    for (int i = 0; i < WINDOW_SIZE; i++)
    {
        int j = i + 1;
        IMUFactor* imu_factor = new IMUFactor(pre_integrations[j]);
        ceres::ResidualBlockId fid = problem.AddResidualBlock(imu_factor, NULL, para_Pose[i], para_SpeedBias[i], para_Pose[j], para_SpeedBias[j]);

        vector<double*>* para = new vector<double*>;  
        problem.GetParameterBlocksForResidualBlock(fid, para); 
        vector<double> res(15); 
        imu_factor->Evaluate(&para[0][0], &res[0], 0); 
        Eigen::Map<Eigen::Matrix<double, 15, 1>> residual(&res[0]);
        ouf<<"debug_optimize.cpp: IMU factor " <<j<<" has residual: "<< residual.norm()<<endl;
        total_sq_norm += residual.squaredNorm(); 
        // ouf << f_m_cnt<<" "<<pts_i.transpose()<<" "<<pts_j.transpose()<<" "<<para_Feature[feature_index][0]<<" "<< res[0]<<" "<<res[1]<<endl;
    }

    int feature_index = 0; 
    map<int, vector<FeatFactor> >::iterator it = featMeasure.begin(); 
    int L_cnt_vc = 0; 
    int R_cnt_vc = 0; 
    while(it!= featMeasure.end()){
    	
    	para_Feature[feature_index][0] = it->second[0].inv_dpt; 
    	// problem.AddParameterBlock(para_Feature[feature_index], 1);

    	for(int i=0; i<it->second.size(); i++){
    		FeatFactor& ft = it->second[i]; 

    		if(ft.LR == "L"){
    			ProjectionTwoFrameOneCamFactor *f_td = new ProjectionTwoFrameOneCamFactor(ft.pts_i, ft.pts_j, ft.pi_vel, ft.pj_vel, ft.pi_td, ft.pj_td); 
    			ceres::ResidualBlockId fid = problem.AddResidualBlock(f_td, loss_function, para_Pose[ft.imu_i], para_Pose[ft.imu_j], para_Ex_Pose[0], para_Feature[feature_index], para_Td[0]);
    			L_cnt_vc++;
    			vector<double*>* para = new vector<double*>;  
                problem.GetParameterBlocksForResidualBlock(fid, para); 
                vector<double> res(2); 
                f_td->Evaluate(&para[0][0], &res[0], 0); 
                double sn = SQ(res[0])+SQ(res[1]);  
                ouf<<"L feature " <<feature_index<<" "<< ft.imu_i<<" "<<ft.imu_j<<" depth: "<<1./para_Feature[feature_index][0]<<" residual: "<<res[0]<<" "<<res[1]<<" "<<sn<<endl;
                total_sq_norm += sn; 
                // ouf << f_m_cnt<<" "<<pts_i.transpose()<<" "<<pts_j.transpose()<<" "<<para_Feature[feature_index][0]<<" "<< res[0]<<" "<<res[1]<<endl;

    		}else{

                ProjectionTwoFrameTwoCamFactor *f = new ProjectionTwoFrameTwoCamFactor(ft.pts_i, ft.pts_j, ft.pi_vel, ft.pj_vel, ft.pi_td, ft.pj_td);
                ceres::ResidualBlockId fid = problem.AddResidualBlock(f, loss_function, para_Pose[ft.imu_i], para_Pose[ft.imu_j], para_Ex_Pose[0], para_Ex_Pose[1], para_Feature[feature_index], para_Td[0]);
                R_cnt_vc++;

                vector<double*>* para = new vector<double*>;  
                problem.GetParameterBlocksForResidualBlock(fid, para); 
                vector<double> res(2); 
                f->Evaluate(&para[0][0], &res[0], 0); 
                double sn = SQ(res[0])+SQ(res[1]);  
                ouf<<"R feature " <<feature_index<<" "<< ft.imu_i<<" "<<ft.imu_j<<" depth: "<<1./para_Feature[feature_index][0]<<" residual: "<<res[0]<<" "<<res[1]<<" "<<sn<<endl;
                total_sq_norm += sn;  
    		}
    	}
    	it++;
    	feature_index++;
    }

    cout <<"total_sq_norm is "<<total_sq_norm<<endl; 

}


bool readFactors(string debug_file)
{
	ifstream inf(debug_file.c_str()); 
	if(!inf.is_open()){
		cerr << "debug_optimize.cpp: fail to load file: "<<debug_file<<endl; 
		return false; 
	}

	for(int i=0; i<WINDOW_SIZE+1; i++){
		for(int j=0; j<SIZE_POSE; j++){
			inf>>para_Pose[i][j]; 
			// cout<<para_Pose[i][j]<<" "; 
		}
		// cout <<endl; 
		for(int j=0; j<SIZE_SPEEDBIAS; j++){
			inf>>para_SpeedBias[i][j]; 
			// cout << para_SpeedBias[i][j]<<" ";
		}
		// cout << endl; 
	}	

	for(int j=0; j<2; j++)
		for(int i=0; i<SIZE_POSE; i++){
			inf >> para_Ex_Pose[j][i]; 
		// cout << para_Ex_Pose[0][i]<<" ";
		}
	// cout<<endl; 

	inf >> para_Td[0][0]; 
	// cout << para_Td[0][0] << endl; 
	Vector3d acc, gyr, ba, bg; 
	for(int j=1; j<=WINDOW_SIZE; j++){
		pre_integrations[j] = new IntegrationBase(acc, gyr, ba, bg); 
		pre_integrations[j]->input(inf); 
		// pre_integrations[j]->output(cout); 
		// cout<<endl; 
	}

	// get line 
	string line; 
	getline(inf, line);
	while(!inf.eof()){
		getline(inf, line); 
		// cout<<"read line: "<<line<<endl; 
		if(line.empty())
			break; 
		FeatFactor fac; 
		readMeasure(line, fac); 
		if(featMeasure.find(fac.feat_id) == featMeasure.end()){
			vector<FeatFactor> vf; 
			vf.push_back(fac); 
			featMeasure[fac.feat_id] = vf; 
		}else{
			featMeasure[fac.feat_id].push_back(fac); 
		}
	}

	cout<<"read "<<featMeasure.size()<<" visual measurements!"<<endl; 
	return true; 

}


bool readMeasure(string line, FeatFactor& fac)
{
	stringstream ss; 
	ss << line; 
	ss >> fac.feat_id >> fac.LR >> fac.imu_i >> fac.imu_j; 
	for(int i=0; i<3; i++)
		ss >> fac.pts_i(i); 
	for(int i=0; i<3; i++)
		ss >> fac.pts_j(i); 
	for(int i=0; i<2; i++)
		ss >> fac.pi_vel(i); 
	for(int i=0; i<2; i++)
		ss >> fac.pj_vel(i);
	ss >> fac.pi_td >> fac.pj_td; 
	ss >> fac.inv_dpt; 
	// cout<<" given line: "<<line<<endl;
	// cout <<"read feat_id: "<<fac.feat_id<<" LR: "<<fac.LR<<" imu_i: "<<fac.imu_i<<" imu_j: "<<fac.imu_j<<endl; 
	// cout<<" read pi_td: "<<fac.pi_td<<" pj_td: "<<fac.pj_td<<" inv_dpt: "<<fac.inv_dpt<<endl;
	return true;  
}
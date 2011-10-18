/*
 * ecp_bcl_t_test.cpp
 *
 *  Created on: 09-10-2011
 *      Author: szkudi
 */

#include "ecp_bcl_t_test.h"

#include "robot/irp6ot_m/ecp_r_irp6ot_m.h"
#include "robot/irp6p_m/ecp_r_irp6p_m.h"

#include "ecp_mp_st_scan_move.h"
#include "ecp_mp_st_position_move.h"

#include "ecp_st_scan_move.h"
#include "ecp_st_position_move.h"

using boost::shared_ptr;

namespace mrrocpp {

namespace ecp {

namespace common {

namespace task {

ecp_bcl_t_test::ecp_bcl_t_test(mrrocpp::lib::configurator& configurator):common::task::task(configurator){

	std::cout << "TWORZE BCL TEST" << std::endl;

#ifdef IRP6_OT
	ecp_m_robot = (boost::shared_ptr<robot_t>) new ecp::irp6ot_m::robot(*this);
#endif//IRP6_OT

#ifdef IRP6_P
	ecp_m_robot = (boost::shared_ptr<robot_t) new ecp::irp6p_m::robot(*this);
#endif//IRP6_P


	this->bcl_gen = shared_ptr<generator::ecp_bcl_gen> (new common::generator::ecp_bcl_gen(*this));
//	this->bcl_gen = shared_ptr<generator::newsmooth> (new common::generator::newsmooth(*this, lib::ECP_XYZ_ANGLE_AXIS, VEC_SIZE));
//	this->bcl_gen = shared_ptr<generator::constant_velocity> (new common::generator::constant_velocity(*this, lib::ECP_XYZ_EULER_ZYZ, VEC_SIZE));

	std::cout << "STWORZYLEM BCL TEST" << std::endl;

	//Adding additional subtasks
	sub_task::sub_task* ecpst;
	ecpst = new sub_task::ecp_st_scan_move(*this);
	subtask_m[ecp_mp::task::ECP_ST_SCAN_MOVE] = ecpst;

	ecpst = new sub_task::ecp_st_position_move(*this);
	subtask_m[ecp_mp::task::ECP_ST_POSITION_MOVE] = ecpst;

}

ecp_bcl_t_test::~ecp_bcl_t_test(){
}

void ecp_bcl_t_test::main_task_algorithm(void){
	std::vector<double> vec;
	vec.clear();
	vec.assign(ecp::common::task::left, ecp::common::task::left + VEC_SIZE);

	sr_ecp_msg->message("MOVE left");

	this->makeMove(vec);

	int i = 0;

	//Move robot between three control points continuously
	while(1){
		std::cout << "SEND" << std::endl;
		switch(i){
			case 0:
				vec.clear();
				vec.assign(ecp::common::task::left, ecp::common::task::left + VEC_SIZE);
				sr_ecp_msg->message("RIGHT send");
				break;
			case 1:
				vec.clear();
				vec.assign(ecp::common::task::right, ecp::common::task::right + VEC_SIZE);
				sr_ecp_msg->message("LEFT send");
				break;
			case 2:
				vec.clear();
				vec.assign(ecp::common::task::start, ecp::common::task::start + VEC_SIZE);
				sr_ecp_msg->message("START send");
				break;
		}

		i++;
		i = i % 3;

		this->makeMove(vec);

		sr_ecp_msg->message("MP end loop");

	}

////	double tmp[] = { 0.0, 0.5, -1.87, 0.100, -0.040, 4.627, -1.57, 0.0};
//	std::vector<double> vec(left, left + VEC_SIZE);
//	std::cout << "LOAD TRAJECTORY" << " Vec size: " <<  vec.size() << std::endl;
//
//	bcl_gen->reset();
////	bcl_gen->set_absolute();
//
////	bcl_gen->load_absolute_joint_trajectory_pose(vec);
////	if(!bcl_gen->load_absolute_euler_zyz_trajectory_pose(vec))
//	if(!bcl_gen->load_absolute_angle_axis_trajectory_pose(vec))
//		return;
//	std::cout << "CALCUATE INTERPOLATE" << std::endl;
//	if(bcl_gen->calculate_interpolate()){
//		std::cout << "MOVE" << std::endl;
//		bcl_gen->Move();
//		std::cout << "RESET" << std::endl;
//		bcl_gen->reset();
//		std::cout << "NOTICE" << std::endl;
//	}
//	termination_notice();
//	std::cout << "BLC END" << std::endl;

}

void ecp_bcl_t_test::makeMove(std::vector<double> &vec){
	bcl_gen->reset();

	if(!bcl_gen->load_absolute_angle_axis_trajectory_pose(vec))
		return;
	std::cout << "CALCUATE INTERPOLATE" << std::endl;
	if(bcl_gen->calculate_interpolate()){
		std::cout << "MOVE" << std::endl;
		bcl_gen->Move();
		std::cout << "RESET" << std::endl;
		bcl_gen->reset();
		std::cout << "NOTICE" << std::endl;
	}
	termination_notice();
}

task_base *return_created_ecp_task(lib::configurator & _config){
	return new ecp_bcl_t_test(_config);
}

}

}

}

}

/*
 * ecp_bcl_t_test.cpp
 *
 *  Created on: 09-10-2011
 *      Author: szkudi
 */

#include "ecp_bcl_t_main.h"

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

ecp_bcl_t_main::ecp_bcl_t_main(mrrocpp::lib::configurator& configurator):common::task::task(configurator){

	std::cout << "TWORZE BCL TEST" << std::endl;

#ifdef IRP6_OT
	ecp_m_robot = (boost::shared_ptr<robot_t>) new ecp::irp6ot_m::robot(*this);
#endif//IRP6_OT

#ifdef IRP6_P
	ecp_m_robot = (boost::shared_ptr<robot_t) new ecp::irp6p_m::robot(*this);
#endif//IRP6_P

	mrrocpp::ecp_mp::sensor::discode::discode_sensor *ds = new mrrocpp::ecp_mp::sensor::discode::discode_sensor(this->config, "[vsp_discode_sensor]");
	sensor_m["my_discode_sensor"] = ds;
	sensor_m["my_discode_sensor"]->configure_sensor();


	this->bcl_gen = shared_ptr<generator::ecp_bcl_gen> (new common::generator::ecp_bcl_gen(*this, ds));
	this->smooth_gen = shared_ptr<generator::newsmooth> (new common::generator::newsmooth(*this, lib::ECP_XYZ_ANGLE_AXIS, VEC_SIZE));
//	this->bcl_gen = shared_ptr<generator::constant_velocity> (new common::generator::constant_velocity(*this, lib::ECP_XYZ_EULER_ZYZ, VEC_SIZE));

	std::cout << "STWORZYLEM BCL TEST" << std::endl;

	//Adding additional subtasks
	sub_task::sub_task* ecpst;
	ecpst = new sub_task::ecp_st_scan_move(*this);
	subtask_m[ecp_mp::task::ECP_ST_SCAN_MOVE] = ecpst;

	ecpst = new sub_task::ecp_st_position_move(*this);
	subtask_m[ecp_mp::task::ECP_ST_POSITION_MOVE] = ecpst;

}

ecp_bcl_t_main::~ecp_bcl_t_main(){
}


void ecp_bcl_t_main::scanningMove(std::vector<double> &vec){
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

void ecp_bcl_t_main::positionMove(std::vector<double> &vec){
	smooth_gen->reset();

	if(!smooth_gen->load_absolute_angle_axis_trajectory_pose(vec))
		return;
	std::cout << "CALCUATE INTERPOLATE" << std::endl;
	if(smooth_gen->calculate_interpolate()){
		std::cout << "MOVE" << std::endl;
		smooth_gen->Move();
		std::cout << "RESET" << std::endl;
		smooth_gen->reset();
		std::cout << "NOTICE" << std::endl;
	}
	termination_notice();
}

}

}

}

}

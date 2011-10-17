/*
 * ecp_bcl_t_test.cpp
 *
 *  Created on: 09-10-2011
 *      Author: szkudi
 */

#include "ecp_bcl_t_test.h"

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


//	this->bcl_gen = shared_ptr<generator::ecp_bcl_gen> (new common::generator::ecp_bcl_gen(*this));
	this->bcl_gen = shared_ptr<generator::newsmooth> (new common::generator::newsmooth(*this, lib::ECP_XYZ_ANGLE_AXIS, VEC_SIZE));
//	this->bcl_gen = shared_ptr<generator::constant_velocity> (new common::generator::constant_velocity(*this, lib::ECP_XYZ_EULER_ZYZ, VEC_SIZE));

	std::cout << "STWORZYLEM BCL TEST" << std::endl;

}

ecp_bcl_t_test::~ecp_bcl_t_test(){
}

void ecp_bcl_t_test::main_task_algorithm(void){
//	double tmp[] = { 0.0, 0.5, -1.87, 0.100, -0.040, 4.627, -1.57, 0.0};
	std::vector<double> vec(left, left + VEC_SIZE);
	std::cout << "LOAD TRAJECTORY" << " Vec size: " <<  vec.size() << std::endl;

	bcl_gen->reset();
//	bcl_gen->set_absolute();

//	bcl_gen->load_absolute_joint_trajectory_pose(vec);
//	if(!bcl_gen->load_absolute_euler_zyz_trajectory_pose(vec))
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
	std::cout << "BLC END" << std::endl;

}

task_base *return_created_ecp_task(lib::configurator & _config){
	return new ecp_bcl_t_test(_config);
}

}

}

}

}

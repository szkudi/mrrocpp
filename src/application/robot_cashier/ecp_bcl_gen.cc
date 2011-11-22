/**
 * \file bclike_gen.cc
 * \brief Scanning subtask generator class methods definition file
 * \date 02.09.2010
 * \author Kacper Szkudlarek
 */

#include "ecp_bcl_gen.h"
#include "ecp_bcl_t_test.h"
#include "bcl_t_switcher.h"
#include "BCLReading.h"
#include <cstring>

#include "base/lib/logger.h"

#include <iostream>
#include <stdexcept>
#include <algorithm>

namespace mrrocpp {

namespace ecp {

namespace common {

namespace generator {

//Rotation 180 x, 90 z
const double rotation[3][3] = {{-1,    0,   0},
							   { 0,   -1,   0},
							   { 0,    0,   1}};

//Camera translation matrix
const double translation[3] = {0, 0, 0};


#ifdef IRP6_OT
const int joint_num = 7;
#endif//IRP6_OT

#ifdef IRP6_P
const int joint_num = 6;
#endif //IRP6_P

#ifdef JOINT
const  lib::ECP_POSE_SPECIFICATION move_type = lib::ECP_JOINT;
const lib::POSE_SPECIFICATION return_pos_type = lib::FRAME;
#endif //JOINT

#ifdef EULER
const lib::ECP_POSE_SPECIFICATION move_type = lib::ECP_XYZ_EULER_ZYZ;
const lib::POSE_SPECIFICATION return_pos_type = lib::FRAME;
#endif //EULER



ecp_bcl_gen::ecp_bcl_gen(mrrocpp::ecp::common::task::task & ecp_task) :
//		mrrocpp::ecp::common::generator::constant_velocity(ecp_task, move_type, joint_num),
		mrrocpp::ecp::common::generator::newsmooth(ecp_task, move_type, joint_num),
		bcl_ecp((task::ecp_bcl_t_test &)ecp_t), num_send(0){

	this->debug = false;

	vsp_discode = NULL;
	no_discode = false;
}


//ecp_bcl_gen::ecp_bcl_gen(mrrocpp::ecp::common::task::ecp_bcl_t_test & task):
//				common::generator::constant_velocity((mrrocpp::ecp::common::task::task &)task, move_type, joint_num),
//				bcl_ecp((task::ecp_bcl_t_test &)ecp_t), num_send(0){
//
//	std::cout << "DISCODE VERSION" << std::endl;
//	no_discode = false;
//	vsp_discode = bcl_ecp.get_vsp_discode();
//
//
//	if(vsp_discode != NULL){
//		sensor_m["my_discode_sensor"] = vsp_discode;
//		vsp_discode->base_period = 1;
//
//		std::cout << "SENSOR ADD no vsp constructor" << std::endl;
//	}
//
//}


ecp_bcl_gen::ecp_bcl_gen(mrrocpp::ecp::common::task::task & ecp_task, mrrocpp::ecp_mp::sensor::discode::discode_sensor* ds):
						mrrocpp::ecp::common::generator::newsmooth(ecp_task, move_type, joint_num),
						bcl_ecp((task::ecp_bcl_t_test &)ecp_t),
						vsp_discode(ds), num_send(0){

	this->debug = false;

	no_discode = false;

	if(vsp_discode != NULL){
		sensor_m["my_discode_sensor"] = vsp_discode;
		vsp_discode->base_period = 1;

		std::cout << "SENSOR ADD vsp constructor" << std::endl;
	}
}

ecp_bcl_gen::~ecp_bcl_gen() {
}


bool ecp_bcl_gen::first_step(){

	the_robot->ecp_command.instruction_type = lib::GET;
	the_robot->ecp_command.get_type = ARM_DEFINITION;
	the_robot->ecp_command.get_arm_type = return_pos_type;

	if(no_discode){
		std::cout << "ERROR: no discode == TRUE" << std::endl;
		return false;
	}

	return newsmooth::first_step();
//	return constant_velocity::first_step();
}

bool ecp_bcl_gen::next_step(){

//	//Get FraDIA reading
//	reading = bcl_ecp.vsp_discode->get_reading_message();

	try {
		if (vsp_discode->get_state() == mrrocpp::ecp_mp::sensor::discode::discode_sensor::DSS_READING_RECEIVED) {
			//			log_dbg("pb_visual_servo::retrieve_reading(): sensor->get_state() == discode_sensor::DSS_READING_RECEIVED.\n");
			reading = vsp_discode->retreive_reading <Types::Mrrocpp_Proxy::BCLReading > ();
		}
	} catch (std::exception &ex) {
		logger::log("ecp_bcl_gen::retrieve_reading(): %s\n", ex.what());
	}
//
	//Get actual robot's position
	actual_pos = the_robot->reply_package.arm.pf_def.arm_frame;
	std::vector<double> vec;
	vec.assign(the_robot->reply_package.arm.pf_def.arm_coordinates, the_robot->reply_package.arm.pf_def.arm_coordinates + VEC_SIZE);

//	If there are new bar code like areas translate their positions and check existance in vector
	if(reading.codeFound()){
//		double t[3];
//		actual_pos.get_translation_vector(t);
		translateToRobotPositionAndSave(reading);
//		std::cout << "KODOW: " << readings.size() << std::endl;
	}

//	//If there is nothing to send and robot is still moving, go on
	if(newsmooth::next_step()){
		return true;
	}

//	if(constant_velocity::next_step()){
//		return true;
//	}

//	//End everything, when there is nothing to send and robot stops
//	readings.clear();
	return false;

}


void ecp_bcl_gen::translateToRobotPositionAndSave(Types::Mrrocpp_Proxy::BCLReading& regs){

	lib::K_vector u_translation(0, 0, 0);
	lib::Homog_matrix u_rotation;
	Eigen::Matrix <double, 6, 1> e;
	Eigen::Matrix <double, 3, 1> e_translation;

	Eigen::Matrix <double, 6, 1> control;
	Eigen::Matrix <double, 6, 6> Kp;

	Eigen::Matrix <double, 3, 1> camera_to_object_translation;

	lib::Homog_matrix e_T_c_position;

	lib::Homog_matrix delta_position;

	lib::Xyz_Euler_Zyz_vector new_pos;

	Types::ImagePosition region;


	for(int i = 0; i < regs.getCount(); ++i){

		std::cout << "Kolejny odczyt" << std::endl;

		region = regs.getElementAt(i);

//		std::cout << "Region nr: " << i << " x = " << region.elements[0] << " y = " << region.elements[1] << " w = " << region.elements[2] << " h = " << region.elements[3] << std::endl;

		e.setZero();
		Kp.setZero();
		e_translation.setZero();
		control.setZero();
		camera_to_object_translation.setZero();

		e(0,0) = region.elements[0]; //x
		e(1,0) = region.elements[1]; //y

		e(2, 0) = 0;
		e(3, 0) = 0;


		e_translation(0, 0) = e(0, 0);
		e_translation(1, 0) = e(1, 0);
		e_translation(2, 0) = e(2, 0);

	//		[0.00004 0 0 0 0 0; 0 0.00004 0 0 0 0; 0 0 0.001 0 0 0; 0 0 0 0.1 0 0; 0 0 0 0 0 0; 0 0 0 0 0 0]

		Kp(0,0) = 0.000476;
		Kp(1,1) = 0.000476;
		Kp(2,2) = 0.001;
		Kp(3,3) = 0.1;

		control = Kp * e;

		camera_to_object_translation(0, 0) = control(0, 0);
		camera_to_object_translation(1, 0) = control(1, 0);
		camera_to_object_translation(2, 0) = control(2, 0);

		e_T_c_position.remove_translation();
		e_T_c_position.remove_rotation();

		e_T_c_position.set_rotation_matrix(rotation);
		e_T_c_position.set_translation_vector(translation);

		u_translation = e_T_c_position * camera_to_object_translation;

		u_rotation.set_from_xyz_angle_axis(lib::Xyz_Angle_Axis_vector(0, 0, 0, 0, 0, control(3, 0)));

		delta_position.set_rotation_matrix(u_rotation);
		delta_position.set_translation_vector(u_translation);

		tmp_pos = actual_pos * delta_position;

		tmp_pos.get_xyz_euler_zyz(new_pos);

//		regs.getElementAt(i).elements[0] = new_pos(0,0); //x
//		regs.getElementAt(i).elements[1] = new_pos(1,0); //y
//		regs.getElementAt(i).elements[2] *= (0.1 / 210); //w
//		regs.getElementAt(i).elements[3] *= (0.1 / 210); //h

		task::mrrocpp_regions mr_region;

		mr_region.x = new_pos(0,0);
		mr_region.y = new_pos(1,0);

		double ratio = mr_region.x/region.elements[0];

		mr_region.w = region.elements[2] * ratio;//(0.1 / 210);
		mr_region.h = region.elements[3] * ratio;//(0.1 / 210);

//		std::cout << "Region nr: " << i << " x = " << mr_region.x << " y = " << mr_region.y << " w = " << mr_region.w << " h = " << mr_region.h << std::endl;

		if(!checkIfCodeBeenRead(mr_region))
			readings.push_back(std::pair<task::mrrocpp_regions, bool>(mr_region, false));

	}

}



bool ecp_bcl_gen::checkIfCodeBeenRead(task::mrrocpp_regions& code){

	//TODO: Dodać opcję wysokosci do sprawdzania czy kody się przecinają

	std::vector<std::pair<task::mrrocpp_regions, bool> >::iterator it;

	for(it = readings.begin(); it != readings.end(); ++ it){
		if(codesIntersect(code, (*it).first)){
			(*it).first.x = std::min((*it).first.x,code.x);
			(*it).first.y = std::max((*it).first.y, code.y);

			(*it).first.w = std::fabs(std::max((*it).first.x + (*it).first.w, code.x + code.w) - (*it).first.x);
			(*it).first.h = std::fabs(std::min((*it).first.y + (*it).first.h, code.y + code.h) - (*it).first.h);

			return true;
		}
	}

	return false;
}

bool ecp_bcl_gen::codesIntersect(task::mrrocpp_regions& c1, task::mrrocpp_regions& c2){

	task::mrrocpp_regions inter = codesIntersectionArea(c1, c2);

	double area = std::min(c1.w * c1.h, c2.w * c2.h);

//	std::cout << "Intersect w: " << inter.w << " h: " << inter.h << std::endl;
	if(area / (inter.w * inter.h) > this->intersect_area)
		return true;

	return false;


}


task::mrrocpp_regions ecp_bcl_gen::codesIntersectionArea(task::mrrocpp_regions& c1, task::mrrocpp_regions& c2){

	task::mrrocpp_regions intersection;

	intersection.x = std::max(c1.x, c2.x);
	intersection.y = std::min(c1.y, c2.y);

	double tmp = std::abs(std::min(c1.x + c1.w, c2.x + c2.w) - intersection.x);

	intersection.w = tmp > 0.0f ? tmp : 0.0f;

	tmp = std::abs(std::max(c1.y + c1.h, c2.y + c2.h) - intersection.y);

	intersection.h = tmp > 0.0f ? tmp : 0.0f;

	return intersection;
}

std::vector<std::pair<task::mrrocpp_regions, bool> > ecp_bcl_gen::getReadings(){
	return readings;
}

}

}

}

}

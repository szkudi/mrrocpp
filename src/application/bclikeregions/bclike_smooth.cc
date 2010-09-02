/*
 * bclike_smooth.cpp
 *
 *  Created on: 05-07-2010
 *      Author: kszkudla
 */

#include "bclike_smooth.h"
#include "bclikeregions_task.h"
#include "bcl_t_switcher.h"
#include <cstring>

namespace mrrocpp {

namespace ecp {

namespace common {

namespace generator {


//Kamera jest obrocona wzgledem chwytaka o 90 stopni wzdloz osi z
// x = -y
// y =  x
const double x_param = 373/200; //x translation from camera to robot position (px/mm)
const double y_param = 184/100; //y translation from camera to robot position (px/mm)

//Rotation 90 deg z
//const double rotation[3][3] = {{-0.4481,   -0.8940,         0},
//							   { 0.8940,   -0.4481,         0},
//							   {      0,         0,    1.0000}};
//Rotation -90 deg z
//const double rotation[3][3] = {{-0.4481,   -0.8940,         0},
//							   {-0.8940,   -0.4481,         0},
//							   {      0,         0,    1.0000}};
//Rotation 180 deg x
//const double rotation[3][3] = {{1.0000,         0,         0},
//							   {	 0,   -0.5985,    0.8012},
//							   {	 0,   -0.8012,   -0.5985}};
//No rotation
//const double rotation[3][3] = {{1.0000,         0,         0},
//							   {	 0,    	  1.0,    	   0},
//							   {	 0,     	0,   	   1}};

//Rotation 180 x, 90 z
//const double rotation[3][3] = {{-0.4481,    0.5350,   -0.7162},
//							   { 0.8940,    0.2682,   -0.3590},
//							   { 	  0,   -0.8012,   -0.5985}};

//Rotation 180 x, 90 z
//const double rotation[3][3] = {{-0.4481,    -0.5350,   0.7162},
//							   { -0.8940,    0.2682,   -0.3590},
//							   { 	  0,   -0.8012,   -0.5985}};

//Rotation 180 x, 90 z
const double rotation[3][3] = {{-1,    0,   0},
							   { 0,   -1,   0},
							   { 0,    0,   1}};

//Camera translation matrix
const double translation[3] = {0, 0, 0};


#ifdef IRP6_OT
const int joint_num = 6;
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


/**
 * Class constructor without creating FraDIA sensor
 * @param ecp_task parent task
 */
bclike_smooth::bclike_smooth(mrrocpp::ecp::common::task::task & ecp_task) :
//		common::generator::newsmooth(ecp_task, move_type, joint_num),
		common::generator::constant_velocity(ecp_task, move_type, joint_num),
		bcl_ecp((task::bcl_t_switcher &)ecp_t), num_send(0){

	vsp_fradia = NULL;
	no_fradia = true;

}

/**
 * Class constructor, FraDIA sensor is acquired from parent task
 * @param task parent task
 */
bclike_smooth::bclike_smooth(mrrocpp::ecp::common::task::bcl_t_switcher & task):
//				common::generator::newsmooth((mrrocpp::ecp::common::task::task &)task, move_type, joint_num),
				common::generator::constant_velocity((mrrocpp::ecp::common::task::task &)task, move_type, joint_num),
				bcl_ecp((task::bcl_t_switcher &)ecp_t), num_send(0){

	std::cout << "FRADIA VERSION" << std::endl;
	no_fradia = false;
	vsp_fradia = bcl_ecp.get_vsp_fradia();


	if(vsp_fradia != NULL){
		sensor_m[ecp_mp::sensor::SENSOR_FRADIA] = vsp_fradia;
		vsp_fradia->base_period = 1;

		std::cout << "SENSOR ADD no vsp constructor" << std::endl;
	}

}


/**
 * Class constructor, FraDIA sensor is given as a parameter
 * @param task parent task
 * @param fr pointer to FraDIA sensor structure
 */
bclike_smooth::bclike_smooth(mrrocpp::ecp::common::task::bcl_t_switcher & task, task::bcl_fradia_sensor* fr):
						common::generator::constant_velocity((mrrocpp::ecp::common::task::task &)task, move_type, joint_num),
//						common::generator::newsmooth((mrrocpp::ecp::common::task::task &)task, move_type, joint_num),
						bcl_ecp((task::bcl_t_switcher &)ecp_t),
						vsp_fradia(fr), num_send(0){

	no_fradia = false;

	if(vsp_fradia != NULL){
		sensor_m[ecp_mp::sensor::SENSOR_FRADIA] = vsp_fradia;
		vsp_fradia->base_period = 1;

		std::cout << "SENSOR ADD vsp constructor" << std::endl;
	}
}

bclike_smooth::~bclike_smooth() {
}

/**
 * Set necessary instructions, and other data for preparing the robot to move
 */
bool bclike_smooth::first_step(){

//	std::cout << "FIRST STEP" << std::endl;

	the_robot->ecp_command.instruction.instruction_type = lib::GET;
	the_robot->ecp_command.instruction.get_type = ARM_DEFINITION;
	the_robot->ecp_command.instruction.get_arm_type = return_pos_type;

	if(no_fradia){
		std::cout << "ERROR: no fradia == TRUE" << std::endl;
		return false;
	}

//	return newsmooth::first_step();
	return constant_velocity::first_step();
}

bool bclike_smooth::next_step(){

	//Get FraDIA reading
	reading = bcl_ecp.vsp_fradia->get_reading_message();

	//Get actual robot's position
	actual_pos.set_from_frame_tab(the_robot->reply_package.arm.pf_def.arm_frame);
	std::vector<double> vec;
	vec.assign(the_robot->reply_package.arm.pf_def.arm_coordinates, the_robot->reply_package.arm.pf_def.arm_coordinates + VEC_SIZE);

	//If there are new bar code like areas translate their positions and check existance in vector
	if(reading.code_found){
		reading.code_found = false;
		double t[3];
		actual_pos.get_translation_vector(t);
		translateToRobotPosition(reading);
		addCodesToVector(reading);
//		std::cout << "KODOW DO WYSANIA: " << readings.size() << std::endl;
	}

	//If there is something to send, do it
	if(sendNextPart()){
//		newsmooth::reset();
		return false;
	}

	//If there is nothing to send and robot is still moving, go on
//	if(newsmooth::next_step()){
//		return true;
//	}

	if(constant_velocity::next_step()){
		return true;
	}

	//End everything, when there is nothing to send and robot stops
	strcpy(ecp_t.ecp_reply.ecp_2_mp_string, "KONIEC");
	readings.clear();
	return false;


}

/**
 * Translating code positions from local image position, to global robot positon
 * @param regs packet received from FraDIA
 */
void bclike_smooth::translateToRobotPosition(task::fradia_regions& regs){

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

//	std::cout << "REGS_FOUND = " << regs.num_found << std::endl;
//	if(regs.num_found > 1){
//		switch(regs.num_found){
//			case 4:
//				std::cout << "KOD 4: x = " << regs.x_k3 << " y = " << regs.y_k3  <<  " promien = " << regs.w_k3 << std::endl;
//			case 3:
//				std::cout << "KOD 3: x = " << regs.x_k2 << " y = " << regs.y_k2  <<  " promien = " << regs.w_k2 << std::endl;
//			case 2:
//				std::cout << "KOD 2: x = " << regs.x_k1 << " y = " << regs.y_k1  <<  " promien = " << regs.w_k1 << std::endl;
//			case 1:
//				std::cout << "KOD 1: x = " << regs.x_k0 << " y = " << regs.y_k0  <<  " promien = " << regs.w_k0 << std::endl;
//		}
//	}

	for(int i = 0; i < regs.num_found; ++i){

		e.setZero();
		Kp.setZero();
		e_translation.setZero();
		control.setZero();
		camera_to_object_translation.setZero();

		switch(i){
			case 0:
				e(0, 0) = regs.x_k0;
				e(1, 0) = regs.y_k0;
				break;
			case 1:
				e(0, 0) = regs.x_k1;
				e(1, 0) = regs.y_k1;
				break;
			case 2:
				e(0, 0) = regs.x_k2;
				e(1, 0) = regs.y_k2;
				break;
			case 3:
				e(0, 0) = regs.x_k3;
				e(1, 0) = regs.y_k3;
				break;

		}
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

		switch(i){
			case 0:
				regs.x_k0 = new_pos(0,0);//- regs.y_k0 * 0.2/470 - 0.04;
				regs.y_k0 = new_pos(1,0);// + regs.x_k0 * 0.2/470;
				regs.r_k0 = regs.r_k0 * 0.1 / 210;//373;
//				regs.w_k0 = regs.w_k0 * 0.1 / 210;//373;
//				regs.h_k0 = regs.h_k0 * 0.1 / 210;//373;
				break;
			case 1:
				regs.x_k1 = new_pos(0,0);// - regs.y_k1 * 0.2/470 - 0.04;
				regs.y_k1 = new_pos(1,0);// + regs.x_k1 * 0.2/470;
				regs.r_k1 = regs.r_k1 * 0.1 / 210;//373;
//				regs.w_k1 = regs.w_k1 * 0.1 / 210;//373;
//				regs.h_k1 = regs.h_k1 * 0.1 / 210;//373;
				break;
			case 2:
				regs.x_k2 = new_pos(0,0);// - regs.y_k2 * 0.2/470 - 0.04;
				regs.y_k2 = new_pos(1,0);//+ regs.x_k2 * 0.2/470;
				regs.r_k2 = regs.r_k2 * 0.1 / 210;//373;
//				regs.w_k2 = regs.w_k2 * 0.1 / 210;//373;
//				regs.h_k2 = regs.h_k2 * 0.1 / 210;//373;
				break;
			case 3:
				regs.x_k3 = new_pos(0,0);// - regs.y_k3 * 0.2/470 - 0.04;
				regs.y_k3 = new_pos(1,0);//regs.x_k3 * 0.2/470;
				regs.r_k3 = regs.r_k3 * 0.1 / 210;//373;
//				regs.w_k3 = regs.w_k3 * 0.1 / 210;//373;
//				regs.h_k3 = regs.h_k3 * 0.1 / 210;//373;
				break;
		}
	}

//	if(regs.num_found > 1){
//		switch(regs.num_found){
//			case 4:
//				std::cout << "KOD 4: x = " << regs.x_k3 << " y = " << regs.y_k3  <<  " promien = " << regs.w_k3 << std::endl;
//			case 3:
//				std::cout << "KOD 3: x = " << regs.x_k2 << " y = " << regs.y_k2  <<  " promien = " << regs.w_k2 << std::endl;
//			case 2:
//				std::cout << "KOD 2: x = " << regs.x_k1 << " y = " << regs.y_k1  <<  " promien = " << regs.w_k1 << std::endl;
//			case 1:
//				std::cout << "KOD 1: x = " << regs.x_k0 << " y = " << regs.y_k0  <<  " promien = " << regs.w_k0 << std::endl;
//		}
//	}

}

/**
 * Function rewriting codes received from FraDIA to local container if they haven't been
 * there earlier
 * @param reading packet received from FraDIA framework
 */
void bclike_smooth::addCodesToVector(task::fradia_regions reading){

	task::mrrocpp_regions tmp;


//	std::cout << "REGS TO ADD: " << reading.num_found << std::endl;

	switch(reading.num_found){
		case 4:
			tmp.x = reading.x_k3;
			tmp.y = reading.y_k3;
			tmp.r = reading.r_k3;
//			tmp.h = reading.h_k3;
//			tmp.w = reading.w_k3;
			if(!checkIfCodeBeenRead(tmp))
				readings.push_back(std::pair<task::mrrocpp_regions, bool>(tmp, false));
		case 3:
			tmp.x = reading.x_k2;
			tmp.y = reading.y_k2;
			tmp.r = reading.r_k2;
//			tmp.h = reading.h_k2;
//			tmp.w = reading.w_k2;
			if(!checkIfCodeBeenRead(tmp))
				readings.push_back(std::pair<task::mrrocpp_regions, bool>(tmp, false));
		case 2:
			tmp.x = reading.x_k1;
			tmp.y = reading.y_k1;
			tmp.r = reading.r_k1;
//			tmp.h = reading.h_k1;
//			tmp.w = reading.w_k1;
			if(!checkIfCodeBeenRead(tmp))
				readings.push_back(std::pair<task::mrrocpp_regions, bool>(tmp, false));
		case 1:
			tmp.x = reading.x_k0;
			tmp.y = reading.y_k0;
			tmp.r = reading.r_k0;
//			tmp.h = reading.h_k0;
//			tmp.w = reading.w_k0;
			if(!checkIfCodeBeenRead(tmp))
				readings.push_back(std::pair<task::mrrocpp_regions, bool>(tmp, false));
			break;
		case 0:
			break;
	}

//	std::cout << "AKTUALNIE KODOW: " << readings.size() << std::endl;
}

/**
 * Function to check if found code isn't already in memory vector
 * @param code Code which will be check if it intersect with any other code in vector
 * @return true if code is in vector, false otherwise
 */
bool bclike_smooth::checkIfCodeBeenRead(task::mrrocpp_regions& code){

	std::vector<std::pair<task::mrrocpp_regions, bool> >::iterator it;

	for(it = readings.begin(); it != readings.end(); ++ it){
		if(codesIntersect(code, (*it).first)){
			(*it).first.x = ((*it).first.x + code.x)/2;
			(*it).first.y = ((*it).first.y + code.y)/2;
			(*it).first.r = sqrt(((*it).first.x - code.x)*((*it).first.x - code.x) + ((*it).first.y - code.y)*((*it).first.y - code.y))/2 + ((*it).first.r + code.r)/2;

//			std::cout << "x = " << (*it).first.x << " y = " << (*it).first.y << " w = " <<(*it).first.w << " h = " << (*it).first.h << std::endl;

			return true;
		}
	}

	return false;
}
/**
 * Check if two given code areas intersects
 * @param c1 first of codes to be checked
 * @param c2 second of codes to be checked
 * @return true if codes intersect, false otherwise
 */
bool bclike_smooth::codesIntersect(task::mrrocpp_regions& c1, task::mrrocpp_regions& c2){

	if(sqrt((c1.x - c2.x)*(c1.x - c2.x) + (c1.y - c2.y)*(c1.y - c2.y)) < (c1.r + c2.r)){
		return true;
	}
	return false;
}


/**
 * Rewriting data from vector to buffer to send to MP
 */
bool bclike_smooth::sendNextPart(){

	char* ret = new char[MP_2_ECP_STRING_SIZE];
	double *tab = reinterpret_cast<double*>(ret);

	//Write to matrix number of elements which will be written to
	lib::Xyz_Euler_Zyz_vector new_pos;
	std::vector<double> vec;
	actual_pos.get_xyz_euler_zyz(new_pos);
	new_pos.to_vector(vec);
	tab[0] = vec.size();

	int i = 1;
	//Rewrite vector elements to matrix
	for(std::vector<double>::iterator it = vec.begin(); (it != vec.end()) && (i < MP_2_ECP_STRING_SIZE/sizeof(double)); ++it, ++i){
		tab[i] = *it;
	}

	std::vector<std::pair<task::mrrocpp_regions, bool> >::iterator it;// = readings.begin();

	tab[i] = 0;

	for(it = readings.begin(); it != readings.end() && ((i + 5 * tab[i] + 1) * sizeof(double) < MP_2_ECP_STRING_SIZE); ++it){
//		std::cout << "ADING TO SEND: " << (*it).second << std::endl;
		if(!(*it).second){
			tab[i + 3 * (int)tab[i] + 1] = (*it).first.x;
			tab[i + 3 * (int)tab[i] + 2] = (*it).first.y;
			tab[i + 3 * (int)tab[i] + 3] = (*it).first.r;
//			tab[i + 4 * (int)tab[i] + 3] = (*it).first.w;
//			tab[i + 4 * (int)tab[i] + 4] = (*it).first.h;
			tab[i]++;
			(*it).second = true;
		}
	}


	if(tab[i] > 0){
//		std::cout << "SENDING DATA " << tab[i] << std::endl;
		memcpy(ecp_t.ecp_reply.ecp_2_mp_string, ret, sizeof(char) * MP_2_ECP_STRING_SIZE);
		delete(ret);
		return true;
	}else{
		delete(ret);
		return false;
	}
}

}

}

}

}
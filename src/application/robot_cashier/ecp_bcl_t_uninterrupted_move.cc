/*
 * ecp_bcl_t_test.cpp
 *
 *  Created on: 09-10-2011
 *      Author: szkudi
 */

#include "ecp_bcl_t_uninterrupted_move.h"

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

ecp_bcl_t_uninterrupted_move::ecp_bcl_t_uninterrupted_move(mrrocpp::lib::configurator& configurator):
		common::task::ecp_bcl_t_main(configurator){

}

ecp_bcl_t_uninterrupted_move::~ecp_bcl_t_uninterrupted_move(){
}

void ecp_bcl_t_uninterrupted_move::main_task_algorithm(void){
	sr_ecp_msg->message("MP start");

	std::vector<double> vec;

//	Set robot to start position (left)
	vec.clear();
	vec.assign(ecp::common::task::left, ecp::common::task::left + VEC_SIZE);

	this->positionMove(vec);

	//Setup end position (right)
	vec.clear();
	vec.assign(ecp::common::task::right, ecp::common::task::right + VEC_SIZE);
	this->scanningMove(vec);

	//Get found areas
	readings = bcl_gen->getReadings();

	mrrocpp::ecp::common::task::mrrocpp_regions region;

	std::cout << "POBIERAM ODCZYTY" << std::endl;

	for(int i = 0; i < readings.size(); ++i){
		region = readings[i].first;
		std::cout << "Region nr: " << i << " x = " << region.x << " y = " << region.y << " w = " << region.w << " h = " << region.h << std::endl;

	}


//	std::vector<std::pair<ecp::common::task::mrrocpp_regions, bool> >::iterator it;
//
//	while(strcmp(robot_m[actual_robot]->ecp_reply_package.recognized_command, "KONIEC") != 0){
//
//		pos = msg.stringToECPOrder(robot_m[actual_robot]->ecp_reply_package.recognized_command, regions);
//
//		for(it = regions.begin(); it!= regions.end(); ++it){
//			if(!(*it).second){
//				std::cout << "WYKONANIE ODCZYTU" << std::endl;
//				vec[0] = (*it).first.x;
//				vec[1] = (*it).first.y;
//				//TODO: Przelaczyc Task we FraDIA
//				//TODO: Wywolac subtask Marcina
//				(*it).second = true;
//
//				//Go to code
//				tmp = msg.robotPositionToString(vec);
//				set_next_ecp_state (ecp_mp::task::ECP_ST_POSITION_MOVE, 0, tmp, lib::ECP_2_MP_STRING_SIZE, actual_robot);
//				wait_for_task_termination(false, 1,  actual_robot.c_str());
//
//				//Get back to previous position
//				tmp = msg.robotPositionToString(pos);
//				set_next_ecp_state (ecp_mp::task::ECP_ST_POSITION_MOVE, 0, tmp, lib::ECP_2_MP_STRING_SIZE, actual_robot);
//				wait_for_task_termination(false, 1,  actual_robot.c_str());
//			}
//		}
//
//		set_next_ecp_state (ecp_mp::task::ECP_ST_SCAN_MOVE, 0, tab, lib::ECP_2_MP_STRING_SIZE, actual_robot);
//		sr_ecp_msg->message("MOVE right");
//		wait_for_task_termination(false, 1, actual_robot.c_str());
//	}

	sr_ecp_msg->message("KONIEC");

}

task_base *return_created_ecp_task(lib::configurator & _config){
	return new ecp_bcl_t_uninterrupted_move(_config);
}

}

}

}

}

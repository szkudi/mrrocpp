/*
 * generator/ecp_g_vis_ib_eih_wrist_turner_irp6ot.cc
 *
 *  Created on: DEC 10, 2009
 *      Author: rtulwin
 */

#include "ecp/irp6_on_track/generator/ecp_g_vis_ib_eih_wrist_turner_irp6ot.h"
#include <math.h>


namespace mrrocpp {
namespace ecp {
namespace irp6ot {

	ecp_vis_ib_eih_wrist_turner_irp6ot::ecp_vis_ib_eih_wrist_turner_irp6ot(common::task::task& _ecp_task) :
	common::ecp_visual_servo(_ecp_task) {

}

bool ecp_vis_ib_eih_wrist_turner_irp6ot::first_step() {
/*
	printf("first step\n");
	flushall();
	vsp_fradia = sensor_m[lib::SENSOR_CVFRADIA];

	the_robot->ecp_command.instruction.instruction_type = lib::GET;
	the_robot->ecp_command.instruction.get_type = ARM_DV;
	the_robot->ecp_command.instruction.get_arm_type = lib::XYZ_ANGLE_AXIS;
	the_robot->ecp_command.instruction.motion_type = lib::RELATIVE;
	the_robot->ecp_command.instruction.set_type = ARM_DV;
	the_robot->ecp_command.instruction.set_arm_type = lib::XYZ_ANGLE_AXIS;
	the_robot->ecp_command.instruction.interpolation_type = lib::TCIM;
	the_robot->ecp_command.instruction.motion_steps = MOTION_STEPS;
	the_robot->ecp_command.instruction.value_in_step_no = MOTION_STEPS - 1;

	for (int i=0; i<6; i++)
	{
		the_robot->ecp_command.instruction.arm.pf_def.behaviour[i] = lib::UNGUARDED_MOTION;
	}

	for (int i = 0; i < 7; i++) {//ustawianie next_position dla wszystkich osi (lacznie z chwytakiem) na 0
		next_position[i] = 0;
	}
	printf("dochodzi tutaj\n");
	return true;
	*/
}

bool ecp_vis_ib_eih_wrist_turner_irp6ot::next_step_without_constraints() {

	printf("poczatek next_step\n");
	the_robot->ecp_command.instruction.instruction_type = lib::SET_GET;//TODO sprawdzic czy to moze byc robione tylko raz

	printf("next step\n");

	lib::VSP_REPORT vsp_report = vsp_fradia->from_vsp.vsp_report;
	if (vsp_report == lib::VSP_REPLY_OK) {

		tracking = vsp_fradia->from_vsp.comm_image.sensor_union.object_tracker.tracking;
		reached = vsp_fradia->from_vsp.comm_image.sensor_union.object_tracker.reached;
		printf("tracking: %d\t reached: %d \n", tracking, reached);
		flushall();

		if(tracking == true) {
			next_position[5] = 0.001;
		}

		if(reached == true) {
			return false;
		}
	}

	memcpy(the_robot->ecp_command.instruction.arm.pf_def.arm_coordinates, next_position,
			6 * sizeof(double)); //zapisanie pozycji w angle axes

	the_robot->ecp_command.instruction.arm.pf_def.gripper_coordinate = next_position[6]; //zapisanie pozycji grippera

	for (int i = 0; i <= 7; i++) {//ustawianie next_position dla wszystkich osi (lacznie z chwytakiem) na 0
		next_position[i] = 0;
	}

	return true;
}

void ecp_vis_ib_eih_wrist_turner_irp6ot::entertain_constraints() {

}

} // namespace irp6ot
} // namespace ecp
} // namespace mrrocpp
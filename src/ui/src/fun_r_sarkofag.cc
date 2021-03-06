/* Y o u r   D e s c r i p t i o n                       */
/*                            AppBuilder Photon Code Lib */
/*                                         Version 2.01  */

/* Standard headers */
#include <cstdio>
#include <cstdlib>
#include <unistd.h>
#include <cstring>
#include <strings.h>
#include <iostream>
#include <fstream>
#include <dirent.h>
#include <sys/types.h>
#include <csignal>
#include <sys/netmgr.h>
#include <cerrno>
#include <process.h>
#include <cmath>

#include <boost/bind.hpp>

#include "base/lib/sr/srlib.h"

#include "ui/src/ui_class.h"
// #include "ui/src/ui.h"
// Konfigurator.
#include "base/lib/configurator.h"
#include "ui/src/ui_ecp_r_tfg_and_conv.h"

/* Local headers */
#include "ablibs.h"
#include "abimport.h"
#include "proto.h"

extern ui::common::Interface interface;

int close_wind_sarkofag_moves(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	if (interface.sarkofag->is_wind_sarkofag_moves_open) {
		PtDestroyWidget(ABW_wnd_sarkofag_moves);
	}

	return (Pt_CONTINUE);

}

int close_wnd_sarkofag_servo_algorithm(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	if (interface.sarkofag->is_wind_sarkofag_servo_algorithm_open) {
		PtDestroyWidget(ABW_wnd_sarkofag_servo_algorithm);
	}

	return (Pt_CONTINUE);

}

int EDP_sarkofag_create(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	interface.sarkofag->edp_create();

	return (Pt_CONTINUE);

}

int EDP_sarkofag_slay(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	interface.sarkofag->EDP_slay_int();

	return (Pt_CONTINUE);

}

int EDP_sarkofag_synchronise(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	interface.sarkofag->synchronise();
	return (Pt_CONTINUE);

}

int start_wind_sarkofag_moves(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	if (!interface.sarkofag->is_wind_sarkofag_moves_open) // otworz okno
	{
		ApCreateModule(ABM_wnd_sarkofag_moves, widget, cbinfo);
		interface.sarkofag->is_wind_sarkofag_moves_open = true;
	} else { // przelacz na okno
		PtWindowToFront(ABW_wnd_sarkofag_moves);
	}

	return (Pt_CONTINUE);

}

int start_wnd_sarkofag_servo_algorithm(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	if (!interface.sarkofag->is_wind_sarkofag_servo_algorithm_open) // otworz okno
	{
		ApCreateModule(ABM_wnd_sarkofag_servo_algorithm, widget, cbinfo);
		interface.sarkofag->is_wind_sarkofag_servo_algorithm_open = 1;
	} else { // przelacz na okno
		PtWindowToFront(ABW_wnd_sarkofag_servo_algorithm);
	}

	return (Pt_CONTINUE);

}

int init_wnd_sarkofag_servo_algorithm(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	uint8_t servo_alg_no[interface.sarkofag->number_of_servos];
	uint8_t servo_par_no[interface.sarkofag->number_of_servos];

	// wychwytania ew. bledow ECP::robot
	try {
		if (interface.sarkofag->state.edp.pid != -1) {
			if (interface.sarkofag->state.edp.is_synchronised) // Czy robot jest zsynchronizowany?
			{
				interface.sarkofag->ui_ecp_robot->get_servo_algorithm(servo_alg_no, servo_par_no);

				PtSetResource(ABW_PtNumericInteger_wnd_sarkofag_servo_algorithm_read_alg_1, Pt_ARG_NUMERIC_VALUE, servo_alg_no[0], 0);

				PtSetResource(ABW_PtNumericInteger_wnd_sarkofag_servo_algorithm_read_par_1, Pt_ARG_NUMERIC_VALUE, servo_par_no[0], 0);

			} else {

			}
		}
	} // end try
	CATCH_SECTION_UI

	return (Pt_CONTINUE);

}

int sarkofag_servo_algorithm_set(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	uint8_t *servo_alg_no_tmp[interface.sarkofag->number_of_servos];
	uint8_t servo_alg_no_output[interface.sarkofag->number_of_servos];
	uint8_t *servo_par_no_tmp[interface.sarkofag->number_of_servos];
	uint8_t servo_par_no_output[interface.sarkofag->number_of_servos];

	// wychwytania ew. bledow ECP::robot
	try {
		if (interface.sarkofag->state.edp.is_synchronised) {

			PtGetResource(ABW_PtNumericInteger_wnd_sarkofag_servo_algorithm_alg_1, Pt_ARG_NUMERIC_VALUE, &servo_alg_no_tmp[0], 0);

			PtGetResource(ABW_PtNumericInteger_wnd_sarkofag_servo_algorithm_par_1, Pt_ARG_NUMERIC_VALUE, &servo_par_no_tmp[0], 0);

			for (int i = 0; i < interface.sarkofag->number_of_servos; i++) {
				servo_alg_no_output[i] = *servo_alg_no_tmp[i];
				servo_par_no_output[i] = *servo_par_no_tmp[i];
			}

			// zlecenie wykonania ruchu
			interface.sarkofag->ui_ecp_robot->set_servo_algorithm(servo_alg_no_output, servo_par_no_output);

		} else {
		}
	} // end try
	CATCH_SECTION_UI

	return (Pt_CONTINUE);

}

int wind_sarkofag_moves_init(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	// wychwytania ew. bledow ECP::robot
	try {
		if ((interface.sarkofag->state.edp.pid != -1) && (interface.sarkofag->is_wind_sarkofag_moves_open)) {
			if (interface.sarkofag->state.edp.is_synchronised) // Czy robot jest zsynchronizowany?
			{
				interface.unblock_widget(ABW_PtNumericFloat_wind_sarkofag_moves_inc_pos);
				interface.unblock_widget(ABW_PtButton_wind_sarkofag_moves_inc_exec);
				interface.unblock_widget(ABW_PtButton_wind_sarkofag_moves_int_left);
				interface.unblock_widget(ABW_PtButton_wind_sarkofag_moves_int_right);
				interface.unblock_widget(ABW_PtNumericFloat_wind_sarkofag_moves_int_step);
				interface.unblock_widget(ABW_PtNumericFloat_wind_sarkofag_moves_int_pos);
				interface.unblock_widget(ABW_PtButton_wind_sarkofag_moves_int_exec);

				interface.sarkofag->ui_ecp_robot->read_motors(interface.sarkofag->current_pos); // Odczyt polozenia walow silnikow

				PtSetResource(ABW_PtNumericFloat_wind_sarkofag_moves_read_motor_pos, Pt_ARG_NUMERIC_VALUE, &interface.sarkofag->current_pos[0], 0);

				interface.sarkofag->ui_ecp_robot->read_joints(interface.sarkofag->current_pos);

				PtSetResource(ABW_PtNumericFloat_wind_sarkofag_moves_read_int_pos, Pt_ARG_NUMERIC_VALUE, &interface.sarkofag->current_pos[0], 0);

			} else {
				interface.block_widget(ABW_PtNumericFloat_wind_sarkofag_moves_inc_pos);
				interface.block_widget(ABW_PtButton_wind_sarkofag_moves_inc_exec);

				interface.block_widget(ABW_PtButton_wind_sarkofag_moves_int_left);
				interface.block_widget(ABW_PtButton_wind_sarkofag_moves_int_right);
				interface.block_widget(ABW_PtNumericFloat_wind_sarkofag_moves_int_step);
				interface.block_widget(ABW_PtNumericFloat_wind_sarkofag_moves_int_pos);
				interface.block_widget(ABW_PtButton_wind_sarkofag_moves_int_exec);
			}
			PtDamageWidget(ABW_wnd_sarkofag_moves);
		}
	} // end try
	CATCH_SECTION_UI

	return (Pt_CONTINUE);

}

int clear_wind_sarkofag_moves_flag(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	interface.sarkofag->is_wind_sarkofag_moves_open = false;

	return (Pt_CONTINUE);

}

int wind_sarkofag_moves_move(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	double *wektor_ptgr, desired_pos_motors[6], desired_pos_int[6];
	double *step1;

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	// wychwytania ew. bledow ECP::robot
	try {

		if (interface.sarkofag->state.edp.pid != -1) {

			// incremental
			if ((widget == ABW_PtButton_wind_sarkofag_moves_inc_left) || (widget
					== ABW_PtButton_wind_sarkofag_moves_inc_right) || (widget
					== ABW_PtButton_wind_sarkofag_moves_inc_exec)) {

				if (interface.sarkofag->state.edp.is_synchronised) {
					PtGetResource(ABW_PtNumericFloat_wind_sarkofag_moves_inc_pos, Pt_ARG_NUMERIC_VALUE, &wektor_ptgr, 0);
					desired_pos_motors[0] = (*wektor_ptgr);
				} else {
					desired_pos_motors[0] = 0.0;
				}

				PtGetResource(ABW_PtNumericFloat_wind_sarkofag_moves_inc_step, Pt_ARG_NUMERIC_VALUE, &step1, 0);

				if (widget == ABW_PtButton_wind_sarkofag_moves_inc_left) {
					desired_pos_motors[0] -= (*step1);
				} else if (widget == ABW_PtButton_wind_sarkofag_moves_inc_right) {
					desired_pos_motors[0] += (*step1);
				}

				interface.sarkofag->ui_ecp_robot->move_motors(desired_pos_motors);

			}

			// internal
			if ((widget == ABW_PtButton_wind_sarkofag_moves_int_left) || (widget
					== ABW_PtButton_wind_sarkofag_moves_int_right) || (widget
					== ABW_PtButton_wind_sarkofag_moves_int_exec)) {
				if (interface.sarkofag->state.edp.is_synchronised) {
					PtGetResource(ABW_PtNumericFloat_wind_sarkofag_moves_int_pos, Pt_ARG_NUMERIC_VALUE, &wektor_ptgr, 0);
					desired_pos_int[0] = (*wektor_ptgr);
				}

				PtGetResource(ABW_PtNumericFloat_wind_sarkofag_moves_int_step, Pt_ARG_NUMERIC_VALUE, &step1, 0);

				if (widget == ABW_PtButton_wind_sarkofag_moves_int_left) {
					desired_pos_int[0] -= (*step1);
				} else if (widget == ABW_PtButton_wind_sarkofag_moves_int_right) {
					desired_pos_int[0] += (*step1);
				}
				interface.sarkofag->ui_ecp_robot->move_joints(desired_pos_int);
			}

			// odswierzenie pozycji robota
			if ((interface.sarkofag->state.edp.is_synchronised) && (interface.sarkofag->is_wind_sarkofag_moves_open)) {

				PtSetResource(ABW_PtNumericFloat_wind_sarkofag_moves_inc_pos, Pt_ARG_NUMERIC_VALUE, &desired_pos_motors[0], 0);
				PtSetResource(ABW_PtNumericFloat_wind_sarkofag_moves_int_pos, Pt_ARG_NUMERIC_VALUE, &desired_pos_int[0], 0);

			}
		}
	} // end try
	CATCH_SECTION_UI

	return (Pt_CONTINUE);

}

int clear_wnd_sarkofag_servo_algorithm_flag(PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo)

{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	interface.sarkofag->is_wind_sarkofag_servo_algorithm_open = false;

	return (Pt_CONTINUE);

}

int
sarkofag_move_to_synchro_position( PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo )

	{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	return( Pt_CONTINUE );

	}


int
sarkofag_move_to_preset_position_0( PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo )

	{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	return( Pt_CONTINUE );

	}


int
sarkofag_move_to_preset_position_1( PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo )

	{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	return( Pt_CONTINUE );

	}


int
sarkofag_move_to_preset_position_2( PtWidget_t *widget, ApInfo_t *apinfo, PtCallbackInfo_t *cbinfo )

	{

	/* eliminate 'unreferenced' warnings */
	widget = widget, apinfo = apinfo, cbinfo = cbinfo;

	return( Pt_CONTINUE );

	}


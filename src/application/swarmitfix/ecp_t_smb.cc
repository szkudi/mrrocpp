#include <cstdio>

#include "base/lib/typedefs.h"
#include "base/lib/impconst.h"
#include "base/lib/com_buf.h"

#include "base/lib/sr/srlib.h"
#include "ecp_mp_t_swarmitfix.h"

#include "robot/smb/ecp_r_smb.h"
#include "generator/ecp/ecp_g_newsmooth.h"
#include "generator/ecp/ecp_g_sleep.h"
#include "ecp_g_epos.h"
#include "ecp_g_smb.h"
#include "ecp_t_smb.h"
#include "generator/ecp/ecp_mp_g_transparent.h"
#include "generator/ecp/ecp_mp_g_newsmooth.h"
#include "generator/ecp/ecp_mp_g_sleep.h"
#include "ecp_mp_g_smb.h"
#include "ecp_mp_g_epos.h"

namespace mrrocpp {
namespace ecp {
namespace smb {
namespace task {

// KONSTRUKTORY
swarmitfix::swarmitfix(lib::configurator &_config) :
	task(_config)
{
	// the robot is choose dependendat on the section of configuration file sent as argv[4]
	ecp_m_robot = new robot(*this);

	gt = new common::generator::transparent(*this);
	//sg = new common::generator::smooth(*this, true);
	g_sleep = new common::generator::sleep(*this);
	g_epos_cubic = new common::generator::epos_cubic(*this);
	g_epos_trapezoidal = new common::generator::epos_trapezoidal(*this);
	g_pin_lock = new generator::pin_lock(*this);
	g_pin_unlock = new generator::pin_unlock(*this);
	g_pin_rise = new generator::pin_rise(*this);
	g_pin_lower = new generator::pin_lower(*this);

	sr_ecp_msg->message("ecp smb loaded");
}

void swarmitfix::mp_2_ecp_next_state_string_handler(void)
{

	if (mp_2_ecp_next_state_string == ecp_mp::generator::ECP_GEN_TRANSPARENT) {
		gt->throw_kinematics_exceptions = (bool) mp_command.ecp_next_state.mp_2_ecp_next_state_variant;
		gt->Move();

	} else if (mp_2_ecp_next_state_string == ecp_mp::generator::ECP_GEN_NEWSMOOTH) {
		std::string path(mrrocpp_network_path);
		path += mp_command.ecp_next_state.get_mp_2_ecp_next_state_string();

		switch ((ecp_mp::task::SMOOTH_MOTION_TYPE) mp_command.ecp_next_state.mp_2_ecp_next_state_variant)
		{
			case ecp_mp::task::RELATIVE:
			//	sg->set_relative();
				break;
			case ecp_mp::task::ABSOLUTE:
			//	sg->set_absolute();
				break;
			default:
				break;
		}

	//	sg->load_file_with_path(path.c_str());
	//	sg->Move();
	} else if (mp_2_ecp_next_state_string == ecp_mp::generator::ECP_GEN_SLEEP) {
		g_sleep->init_time(mp_command.ecp_next_state.mp_2_ecp_next_state_variant);
		g_sleep->Move();
	} else if (mp_2_ecp_next_state_string == ecp_mp::generator::ECP_GEN_EPOS_CUBIC) {
		g_epos_cubic->Move();
	} else if (mp_2_ecp_next_state_string == ecp_mp::generator::ECP_GEN_EPOS_TRAPEZOIDAL) {
		g_epos_trapezoidal->Move();
	} else if (mp_2_ecp_next_state_string == ecp_mp::smb::generator::ECP_GEN_PIN_LOCK) {
		g_pin_lock->Move();
	} else if (mp_2_ecp_next_state_string == ecp_mp::smb::generator::ECP_GEN_PIN_UNLOCK) {
		g_pin_unlock->Move();
	} else if (mp_2_ecp_next_state_string == ecp_mp::smb::generator::ECP_GEN_PIN_RISE) {
		g_pin_rise->Move();
	} else if (mp_2_ecp_next_state_string == ecp_mp::smb::generator::ECP_GEN_PIN_LOWER) {
		g_pin_lower->Move();
	}

}

}
} // namespace smb

namespace common {
namespace task {

task* return_created_ecp_task(lib::configurator &_config)
{
	return new smb::task::swarmitfix(_config);
}

}
} // namespace common
} // namespace ecp
} // namespace mrrocpp

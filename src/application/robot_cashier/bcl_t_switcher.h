/**
 * \file bcl_t_switcher.h
 * \brief Main ECP task file header
 * \date 02.09.2010
 * \author Kacper Szkudlarek
 */

#ifndef BCL_T_SWITCHER_H_
#define BCL_T_SWITCHER_H_

#include "base/ecp/ecp_task.h"
#include "base/ecp/ecp_sub_task.h"
#include <boost/shared_ptr.hpp>
#include "ecp_mp_bclike.h"
#include "ecp_mp_message.h"

#include "bcl_types.h"

using boost::shared_ptr;

namespace mrrocpp {

namespace ecp {

namespace common {

namespace task {

enum BCL_MOTION_DIR{
	LEFT, RIGHT, START
};

class bcl_t_switcher: public common::task::task {
public:
	/**
	 * Class constructor, creating FraDIA sensor, smooth generator, insance
	 * of robot object. Also take care about creating subtasks
	 * @param _config reference to configuration file parser object
	 */
	bcl_t_switcher(lib::configurator &_config);
	/**
	 * Class destructor
	 */
	virtual ~bcl_t_switcher();

	/**
	 * Method for handling communication with MP
	 */
    void mp_2_ecp_next_state_string_handler(void);

    /**
     * Method used by subtask to get access to FraDIA sensor
     * @return pointer to fradia sensor structure
     */
    virtual mrrocpp::ecp_mp::sensor::discode::discode_sensor* get_vsp_discode();

    /**
     * VSP Fradia object pointer
     */
    mrrocpp::ecp_mp::sensor::discode::discode_sensor* vsp_discode;

private:
	ecp_mp_message msg;
};
}

}

}

}

#endif /* BCL_T_SWITCHER_H_ */

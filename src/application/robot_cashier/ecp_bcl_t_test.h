/*
 * ecp_bcl_t_test.h
 *
 *  Created on: 09-10-2011
 *      Author: szkudi
 */

#ifndef ECP_BCL_T_TEST_H_
#define ECP_BCL_T_TEST_H_

#include <boost/shared_ptr.hpp>
#include "base/ecp/ecp_task.h"
#include "ecp_bcl_gen.h"
#include "robot/irp6ot_m/ecp_r_irp6ot_m.h"
#include "robot/irp6p_m/ecp_r_irp6p_m.h"

#include "bcl_types.h"

using boost::shared_ptr;

namespace mrrocpp {

namespace ecp {

namespace common {

namespace task {


class ecp_bcl_t_test: public common::task::task {

public:

	ecp_bcl_t_test(mrrocpp::lib::configurator& configurator);
	virtual ~ecp_bcl_t_test();
	void main_task_algorithm(void);

//	task_base* return_created_ecp_task(lib::configurator &_config);

private:
//	shared_ptr<generator::ecp_bcl_gen> bcl_gen;
	shared_ptr<generator::newsmooth> bcl_gen;
//	shared_ptr<generator::constant_velocity> bcl_gen;

};

}

}

}

}

#endif /* ECP_BCL_T_TEST_H_ */

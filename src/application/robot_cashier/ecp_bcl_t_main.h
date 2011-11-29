/*
 * ecp_bcl_t_test.h
 *
 *  Created on: 09-10-2011
 *      Author: szkudi
 */

#ifndef ECP_BCL_T_MAIN_H_
#define ECP_BCL_T_MAIN_H_

#include <boost/shared_ptr.hpp>
#include "base/ecp/ecp_task.h"
#include "ecp_bcl_gen.h"


#include "bcl_types.h"

using boost::shared_ptr;

namespace mrrocpp {

namespace ecp {

namespace common {

namespace task {


class ecp_bcl_t_main: public common::task::task {

public:

	ecp_bcl_t_main(mrrocpp::lib::configurator& configurator);
	virtual ~ecp_bcl_t_main();
	virtual void main_task_algorithm(void) = 0;

private:
	shared_ptr<generator::newsmooth> smooth_gen;
//	shared_ptr<generator::constant_velocity> bcl_gen;
	subtasks_t sub;

protected:
	void scanningMove(std::vector<double> &vec);
	void positionMove(std::vector<double> &vec);

	std::vector<std::pair<mrrocpp::ecp::common::task::mrrocpp_regions, bool> > readings;
	shared_ptr<generator::ecp_bcl_gen> bcl_gen;



};

}

}

}

}

#endif /* ECP_BCL_T_TEST_H_ */

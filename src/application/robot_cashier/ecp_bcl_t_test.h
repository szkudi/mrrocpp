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

private:
	shared_ptr<generator::ecp_bcl_gen> bcl_gen;
//	shared_ptr<generator::newsmooth> bcl_gen;
//	shared_ptr<generator::constant_velocity> bcl_gen;

	void makeMove(std::vector<double> &vec);

	subtasks_t sub;

};

}

}

}

}

#endif /* ECP_BCL_T_TEST_H_ */

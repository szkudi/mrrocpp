/**
 * \file bclike_gen.h
 * \brief Scanning subtask generator class header file
 * \date 02.09.2010
 * \author Kacper Szkudlarek
 */

#ifndef BCLIKE_GEN_H_
#define BCLIKE_GEN_H_

#include "../../generator/ecp/ecp_g_newsmooth.h"
#include "sensor/discode/discode_sensor.h"
#include "generator/ecp/ecp_g_constant_velocity.h"
#include "base/ecp_mp/ecp_mp_sensor.h"
#include "bcl_types.h"
#include "ecp_mp_message.h"

using boost::shared_ptr;

namespace mrrocpp {

namespace ecp {

namespace common {

namespace task{
class ecp_bcl_t_test;
}

namespace generator {

class ecp_bcl_gen: public mrrocpp::ecp::common::generator::newsmooth {
//class ecp_bcl_gen: public mrrocpp::ecp::common::generator::constant_velocity {
public:
	/**
	 * Class constructor without creating DisCODe sensor
	 * @param ecp_task parent task
	 */
	ecp_bcl_gen(mrrocpp::ecp::common::task::task & ecp_task);
	/**
	 * Class constructor, DisCODe sensor is acquired from parent task
	 * @param task parent task
	 */
//	ecp_bcl_gen(mrrocpp::ecp::common::task::ecp_bcl_t_test & task);
	/**
	 * Class constructor, DisCODe sensor is given as a parameter
	 * @param task parent task
	 * @param fr pointer to DisCODe sensor structure
	 */
//	ecp_bcl_gen(mrrocpp::ecp::common::task::ecp_bcl_t_test & task, mrrocpp::ecp_mp::sensor::discode::discode_sensor* dc);

	/**
	 * Class destructor
	 */
	virtual ~ecp_bcl_gen();

	/**
	 * Set necessary instructions, and other data for preparing the robot to move
	 *  @return true if everything is ok
	 */
	virtual bool next_step();
	/**
	 * Method called in every step of movement, here all code areas computatuons have pleace
	 * @return true if movment is not finished
	 */
	virtual bool first_step();

private:
	task::fradia_regions reading;
	std::vector<std::pair<task::mrrocpp_regions, bool> > readings;
	bool no_discode;
	task::ecp_bcl_t_test & bcl_ecp;

	ecp_mp_message msg;

	mrrocpp::ecp_mp::sensor::discode::discode_sensor* vsp_discode;

	int num_send;

	lib::Homog_matrix actual_pos;
	lib::Homog_matrix tmp_pos;

	/**
	 * Translating code positions from local image position, to global robot positon
	 * @param regs packet received from DisCODe
	 */
	void translateToRobotPosition(task::fradia_regions& regs);
	/**
	 * Function rewriting codes received from DisCODe to local container if they haven't been
	 * there earlier
	 * @param reading packet received from DisCODe framework
	 */
	void addCodesToVector(task::fradia_regions reading);
	/**
	 * Function to check if found code isn't already in memory vector
	 * @param code Code which will be check if it intersect with any other code in vector
	 * @return true if code is in vector, false otherwise
	 */
	bool checkIfCodeBeenRead(task::mrrocpp_regions& code);
	/**
	 * Check if two given code areas intersects
	 * @param c1 first of codes to be checked
	 * @param c2 second of codes to be checked
	 * @return true if codes intersect, false otherwise
	 */
	bool codesIntersect(task::mrrocpp_regions& c1, task::mrrocpp_regions& c2);
	/**
	 * Rewriting data from vector to buffer to send to MP
	 */
	bool sendNextPart();
};

}

}

}

}

#endif /* BCLIKE_GEN_H_ */

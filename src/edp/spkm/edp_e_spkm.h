/*!
 * \file edp_e_spkm.h
 * \brief File containing the declaration of edp::spkm::effector class.
 *
 * \author yoyek
 * \date 2009
 *
 */


#ifndef __EDP_E_SPKM_H
#define __EDP_E_SPKM_H

#include "edp/common/edp_e_manip.h"
#include "lib/spkm_const.h"

namespace mrrocpp {
namespace edp {
namespace spkm {

// Klasa reprezentujaca robota IRp-6 na postumencie.
/*!
 * \brief class of EDP SwarmItFix parallel kinematic manipulator
 *
 * It is the base of the head mounted on the mobile base.
 */
class effector: public common::manip_effector
{
protected:

	/*!
	 * \brief method,  creates a list of available kinematic models for spkm effector.
	 *
	 * Here it is parallel manipulator direct and inverse kinematic transform
	 * and motor to joint transform
	 */
	virtual void create_kinematic_models_for_given_robot(void);

public:

	/*!
	 * \brief class constructor
	 *
	 * The attributes are initialized here.
	 */
	effector(lib::configurator &_config);

	/*!
	 * \brief method to create threads other then EDP master thread.
	 *
	 * Here there is only one extra thread - reader_thread.
	 */
	void create_threads();

	/*!
	 * \brief method to move robot arm
	 *
	 * it chooses the single thread variant from the manip_effector
	 */
	void move_arm(lib::c_buffer &instruction); // przemieszczenie ramienia

	/*!
	 * \brief method to get position of the arm
	 *
	 * Here it calls common::manip_effector::get_arm_position_get_arm_type_switch
	 */
	void get_arm_position(bool read_hardware, lib::c_buffer &instruction); // odczytanie pozycji ramienia

	/*!
	 * \brief method to choose master_order variant
	 *
	 * IHere the single thread variant is chosen
	 */
	void master_order(common::MT_ORDER nm_task, int nm_tryb);

};

} // namespace spkm
} // namespace edp
} // namespace mrrocpp


#endif

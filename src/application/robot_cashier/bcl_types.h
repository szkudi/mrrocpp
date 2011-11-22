/**
 * \file bcl_types.h
 * \brief Declaration of types used in project
 * \date 02.09.2010
 * \author Kacper Szkudlarek
 */

#ifndef BCL_TYPES_H_
#define BCL_TYPES_H_

#include "sensor/discode/discode_sensor.h"

//#include "../servovision/visual_servo_types.h"
#include "../../base/lib/impconst.h"

namespace mrrocpp {

namespace ecp {

namespace common {

namespace task {


#define IRP6_OT
//#define IRP6_P
//#define JOINT
#define EULER


///Definition of movement positions
#ifdef IRP6_OT
#ifdef JOINT
#define VEC_SIZE 7
const double left[] = { 0.0, 0.5, -1.87, 0.100, -0.040, 4.627, -1.57};
	const double right[] = { 0.0, -0.55, -1.37, 0.100, -0.040, 4.627, -1.57};
	const double start[] = { 0.0, 0.0, -1.37, 0.100, -0.040, 4.627, 0.0};
#endif//JOINT

#ifdef EULER
#define VEC_SIZE 6
	const double left[] = { 0.83, 0.5, 0.250, -0.014, 3.100, -0.019, 0.0};
	const double right[] = {0.83, -0.5, 0.250, 0.007, 3.100, 0.002, 0.0};
	const double start[] = {0.83, 0.0, 0.250, 0.017, 3.100, 0.012, 0.0};
#endif //EULER

#endif//IRP6_OT

#ifdef IRP6_P
#define VEC_SIZE 6
#ifdef JOINT
	const double left[] = { 0.5, -1.87, 0.100, -0.040, 4.627, -1.57};
	const double right[] = {-0.55, -1.37, 0.100, -0.040, 4.627, -1.57};
	const double start[] = {0.0, -1.37, 0.100, -0.040, 4.627, 0.0};
#endif //JOINT

#ifdef EULER
	const double left[] = { 0.83, 2.45, 0.250, 0.0, 3.100, 0.0};
	const double right[] = {0.83, 1.45, 0.250, 0.0, 3.100, 0.0};
	const double start[] = {0.83, 1.95, 0.250, 0.0, 3.100, 0.0};
#endif //EULER

#endif//IRP6_P


//#define MP_2_ECP_STRING_SIZE 300
#define VEC_POS 27


/**
 * Structure used in communication between MRROC++ and DisCoDe
 */
typedef struct {
	bool code_found;
	int num_found;
	double x_k0;//Code center X
	double y_k0;//Code center Y
	double r_k0;//Code radius
	double x_k1;
	double y_k1;
	double r_k1;
	double x_k2;
	double y_k2;
	double r_k2;
	double x_k3;
	double y_k3;
	double r_k3;
} fradia_regions;


/**
 * Structure used in communication between ECP and MP
 */
typedef struct {
	double x;
	double y;
	double w;
	double h;

} mrrocpp_regions;

}

}

}

}

#endif /* BCL_TYPES_H_ */

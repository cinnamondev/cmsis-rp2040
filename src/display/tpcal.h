/**
 * @file tpcal.h
 *
 */

#ifndef TPCAL_H
#define TPCAL_H

#ifdef __cplusplus
extern "C" {
#endif

/*********************
 *      INCLUDES
 *********************/

/*********************
 *      DEFINES
 *********************/

#ifndef TPCAL_POS_X1
#define TPCAL_POS_X1 20
#endif
#ifndef TPCAL_POS_Y1
#define TPCAL_POS_Y1 40
#endif

#ifndef TPCAL_POS_X2
#define TPCAL_POS_X2 120
#endif
#ifndef TPCAL_POS_Y2
#define TPCAL_POS_Y2 150
#endif

#ifndef TPCAL_POS_X3
#define TPCAL_POS_X3 280
#endif
#ifndef TPCAL_POS_Y3
#define TPCAL_POS_Y3 200
#endif

/**********************
 *      TYPEDEFS
 **********************/
struct point_t {
  uint16_t x;
  uint16_t y;
};

// tpcal callback - 3 touch and 3 dispay expected.
typedef void (*tpcal_cb_t)(const struct point_t t[], const struct point_t d[]);

/**********************
 * GLOBAL PROTOTYPES
 **********************/

/**
 * Create a touch pad calibration screen
 */
void tpcal_create(tpcal_cb_t cb, void (*f)(void));
void tpcal_destroy();
/**********************
 *      MACROS
 **********************/


#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /*TP_CAL_H*/

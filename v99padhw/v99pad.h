#ifndef V99PAD_H
#define V99OAD_H

#include "quantum.h"
//#include "v99slim.h"
//#include "analog.h"
#include "pointing_device.h"

void process_mouse(report_mouse_t* mouse_report);
void process_mouse_user(report_mouse_t* mouse_report, int16_t x, int16_t y);
void process_wheel(report_mouse_t* mouse_report);
void process_wheel_user(report_mouse_t* mouse_report, int16_t h, int16_t v);

#define LAYOUT( \
	K00, K01, K02, K03, K04, \
	K10, K11, K12, K13, K14, \
	K20, K21, K22, K23, K24, \
	K30, K31, K32, K33, K34, \
	K40, K41, K42, K43, K44, \
	K50, K51, K52, K53, K54, \
	K60, K61, K62, K63, K64  \
) { \
	{ K00,   K01,   K02,   K03,   K04 }, \
	{ K10,   K11,   K12,   K13,   K14 }, \
	{ K20,   K21,   K22,   K23,   K24 }, \
	{ K30,   K31,   K32,   K33,   K34 }, \
	{ K40,   K41,   K42,   K43,   K44 }, \
	{ K50,   K51,   K52,   K53,   K54 }, \
	{ K60,   K61,   K62,   K63,   K64 }  \
}

#endif

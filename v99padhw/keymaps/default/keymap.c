#include "v99pad.h"
#include "wait.h"
//#include "debug.h"
#include "v99i2chw.h"

//#define V99PAD_DRAGSCROLL_MOMENTARY
//const static float V99_X_TRANSFORM = -1.0;
//const static float V99_Y_TRANSFORM = 1.0;
const static float V99_X_TRANSFORM = -1;//-2.5;
const static float V99_Y_TRANSFORM = -1;//2.5;
const static float V99_WHEEL_H_DIV = 10;
const static float V99_WHEEL_V_DIV = 20;

static bool is_drag_scroll = 0;

const uint16_t PROGMEM keymaps[][MATRIX_ROWS][MATRIX_COLS] = {

	KEYMAP(
		KC_G, KC_E, KC_G, KC_I, KC_K,
		KC_H, KC_A, KC_B, KC_C, KC_D,
		KC_I, KC_F, KC_H, KC_J, KC_L,
		KC_M, KC_N, KC_O, KC_P, KC_Q,
		KC_R, KC_S, KC_T, KC_U, KC_V,
		KC_W, KC_X, KC_Y, KC_Z, KC_A,
		KC_B, KC_C, KC_D, KC_E, KC_F),

	KEYMAP(
		KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS,
		KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS,
		KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS,
		KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS,
		KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS,
		KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS,
		KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS),

	KEYMAP(
		KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS,
		KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS,
		KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS,
		KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS,
		KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS,
		KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS,
		KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS),

	KEYMAP(
		KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS,
		KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS,
		KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS,
		KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS,
		KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS,
		KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS,
		KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS),

	KEYMAP(
		KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS,
		KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS,
		KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS,
		KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS,
		KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS,
		KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS,
		KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS),

	KEYMAP(
		KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS,
		KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS,
		KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS,
		KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS,
		KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS,
		KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS,
		KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS),

	KEYMAP(
		KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS,
		KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS,
		KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS,
		KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS,
		KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS,
		KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS,
		KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS),

	KEYMAP(
		KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS,
		KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS,
		KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS,
		KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS,
		KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS,
		KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS,
		KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS),

	KEYMAP(
		KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS,
		KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS,
		KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS,
		KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS,
		KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS,
		KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS,
		KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS),

	KEYMAP(
		KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS,
		KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS,
		KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS,
		KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS,
		KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS,
		KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS,
		KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS),
#if 0
	KEYMAP(
		KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS,
		KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS,
		KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS,
		KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS,
		KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS,
		KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS,
		KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS),
#endif
	KEYMAP(
		KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS,
		KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS,
		KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS,
		KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS,
		KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS,
		KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS,
		KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS)

};

const macro_t *action_get_macro(keyrecord_t *record, uint8_t id, uint8_t opt) {
	//keyevent_t event = record->event;

	switch (id) {

	}
	return MACRO_NONE;
}

void matrix_init_user(void) {
}

void matrix_scan_user(void) {
}

bool process_record_user(uint16_t keycode, keyrecord_t *record) {
	return true;
}

void led_set_user(uint8_t usb_led) {

	if (usb_led & (1 << USB_LED_NUM_LOCK)) {

	} else {

	}

	if (usb_led & (1 << USB_LED_CAPS_LOCK)) {

	} else {

	}

	if (usb_led & (1 << USB_LED_SCROLL_LOCK)) {

	} else {

	}

	if (usb_led & (1 << USB_LED_COMPOSE)) {

	} else {

	}

	if (usb_led & (1 << USB_LED_KANA)) {

	} else {

	}

}
#if 0
const uint16_t PROGMEM test_combo1[] = {KC_J, KC_L, COMBO_END};
//const uint16_t PROGMEM test_combo2[] = {KC_C, KC_D, COMBO_END};
combo_t key_combos[COMBO_COUNT] = {
    COMBO(test_combo1, KC_ESC),
    //COMBO(test_combo2, LCTL(KC_Z)), // keycodes with modifiers are possible too!
};
#endif
//encoder padA PD1 2
//encoder padB PC7 13
//#define ENCODER_LOCATION {0,1},{1,1},{0,2},{1,2},{0,3},{1,3},{0,4},{1,4}
//static keypos_t encoder_location[NUMBER_OF_ENCODERS] = ENCODER_LOCATION;
//static keypos_t encoder_location[4][2] = {ENCODER_LOCATION};
#if 0
static keypos_t encoder_location[4][2] = {
	{{.col = 0, .row = 1},{.col = 1, .row = 1}},
  {{.col = 0, .row = 2},{.col = 1, .row = 2}},
	{{.col = 0, .row = 3},{.col = 1, .row = 3}},
	{{.col = 0, .row = 4},{.col = 1, .row = 4}}
};
#endif
static keypos_t encoder_location[4][2] = {
	{{.col = 1, .row = 1},{.col = 1, .row = 0}},
  {{.col = 2, .row = 1},{.col = 2, .row = 0}},
	{{.col = 3, .row = 1},{.col = 3, .row = 0}},
	{{.col = 4, .row = 1},{.col = 4, .row = 0}}
};


#if 1
bool encoder_update_user(uint8_t index, bool clockwise) {
    //uint8_t  layer   = get_highest_layer(layer_state);
		//uint8_t  layer = layer_state;
    //uint16_t keycode = pgm_read_word(&encoder_map[layer][index][clockwise]);
    //uint16_t keycode = dynamic_keymap_get_keycode(biton32(layer_state), 0, 1);


    keyevent_t  encoder_event = (keyevent_t){.key = encoder_location[index][clockwise], .pressed = true, .time = (timer_read() | 1)};
    keyrecord_t record        = {.event = encoder_event};
    						//record.keycode = keycode;

		//uint16_t keycode = dynamic_keymap_get_keycode(biton32(layer_state), encoder_event.key.col, encoder_event.key.row);
		uint16_t keycode = dynamic_keymap_get_keycode(biton32(layer_state), encoder_event.key.row, encoder_event.key.col);
    //record.keycode = keycode;
    //while (keycode == KC_TRANSPARENT && layer > 0) {
    //    if (layer_state_is(layer--)) { keycode = pgm_read_word(&encoder_map[layer][index][clockwise]); }
    //}

    if (keycode != KC_TRANSPARENT && keycode != KC_NO) {
        //if (process_record_quantum_helper(keycode, &record)) {
				if (process_record_quantum(&record)) {
            record.event.pressed = false;
            //if (process_record_quantum_helper(keycode, &record)) {
						if (process_record_quantum(&record)) {
                if (keycode <= QK_MODS_MAX)
                tap_code16(keycode);
            }
        }
    }
		return true;
}
#endif
#if 0
bool encoder_update_user(uint8_t index, bool clockwise) {
#if 1
	if (index == 0){
		if(clockwise){
			//rgblight_step();
			//tap_code((uint8_t)RGB_HUI);
			tap_code16(RGB_HUI);
		}else{
			//rgblight_step_reverse();
			//tap_code((uint8_t)RGB_HUD);
			tap_code16(RGB_HUD);
		}
#endif
#if 0
  if (index == 0) { /* First encoder */
    if (clockwise) {
      //tap_code(dynamic_keymap_get_keycode(layer_state,0,1));
			uint16_t keycode = dynamic_keymap_get_keycode(biton32(layer_state), 0, 1);
			if (keycode >= MACRO00 && keycode <= MACRO15) {
            dynamic_keymap_macro_send(keycode - MACRO00);
      } else {
            register_code16(keycode);
            wait_ms(10);
            unregister_code16(keycode);
      }
    } else {
      //tap_code(dynamic_keymap_get_keycode(layer_state,1,1));
			uint16_t keycode = dynamic_keymap_get_keycode(biton32(layer_state), 1, 1);
			if (keycode >= MACRO00 && keycode <= MACRO15) {
            dynamic_keymap_macro_send(keycode - MACRO00);
      } else {
            register_code16(keycode);
            wait_ms(10);
            unregister_code16(keycode);
      }
    }
#endif
  } else if (index == 1) { /* Second encoder */
    if (clockwise) {
      tap_code(dynamic_keymap_get_keycode(layer_state,0,2));
    } else {
      tap_code(dynamic_keymap_get_keycode(layer_state,1,2));
    }
  } else if (index == 2) { /* Second encoder */
    if (clockwise) {
      tap_code(dynamic_keymap_get_keycode(layer_state,0,3));
    } else {
      tap_code(dynamic_keymap_get_keycode(layer_state,1,3));
    }
  } else if (index == 3) { /* Second encoder */
    if (clockwise) {
      tap_code(dynamic_keymap_get_keycode(layer_state,0,4));
    } else {
      tap_code(dynamic_keymap_get_keycode(layer_state,1,4));
    }
  }
	return true;
}
#endif

//=============================================================================

#if 0
__attribute__((weak)) void process_wheel_user(report_mouse_t* mouse_report, int16_t h, int16_t v) {
    mouse_report->h = h;
    mouse_report->v = v;
}


__attribute__((weak)) void process_wheel(report_mouse_t* mouse_report) {
    // If the mouse wheel was just released, do not scroll.
    //if (timer_elapsed(lastMidClick) < SCROLL_BUTT_DEBOUNCE)
    //    return;

    // Limit the number of scrolls per unit time.
    //if (timer_elapsed(lastScroll) < OPT_DEBOUNCE)
    //    return;

    // Don't scroll if the middle button is depressed.
    //if (is_scroll_clicked) {
//#ifndef IGNORE_SCROLL_CLICK
//        return;
//#endif
//    }

//    lastScroll  = timer_read();
    //uint16_t p1 = adc_read(OPT_ENC1_MUX);
    //uint16_t p2 = adc_read(OPT_ENC2_MUX);

    //if (debug_encoder)
    //    dprintf("OPT1: %d, OPT2: %d\n", p1, p2);

    //uint8_t dir = opt_encoder_handler(p1, p2);

    //if (dir == 0)
    //    return;

    process_wheel_user(mouse_report, mouse_report->h, (int)(mouse_report->v + (dir * OPT_SCALE)));
}
#endif

__attribute__((weak)) void process_mouse_user(report_mouse_t* mouse_report, int16_t x, int16_t y) {
    mouse_report->x = x;
    mouse_report->y = y;
}

__attribute__((weak)) void process_mouse(report_mouse_t* mouse_report) {

	  float xt = 0;
	  float yt = 0;

    //report_v99_t data = v99_slim_read_burst();
		report_v99_t data;
		//data.dx = v99_read(REG_PRODUCT_ID);
		data.dx = v99_read(REG_DELTA_X);
		data.dy = v99_read(REG_DELTA_Y);



    if (data.dx != 0 || data.dy != 0) {
        //if (debug_mouse)
        //    dprintf("Raw ] X: %d, Y: %d\n", data.dx, data.dy);

        // Apply delta-X and delta-Y transformations.
				if(abs(data.dx)>1 || abs(data.dy)>1){
				  xt = (float) data.dx * V99_X_TRANSFORM;
          yt = (float) data.dy * V99_Y_TRANSFORM;
			  }else{
					xt = (float) data.dx * -1;
          yt = (float) data.dy * -1;
				}

        int16_t xti = (int16_t)xt;
        int16_t yti = (int16_t)yt;

        process_mouse_user(mouse_report, xti, yti);
    }
}

#if 1
bool process_record_kb(uint16_t keycode, keyrecord_t* record) {
    //xprintf("KL: kc: %u, col: %u, row: %u, pressed: %u\n", keycode, record->event.key.col, record->event.key.row, record->event.pressed);

/*
    // Update Timer to prevent accidental scrolls
    if ((record->event.key.col == 1) && (record->event.key.row == 0)) {
        lastMidClick = timer_read();
        is_scroll_clicked = record->event.pressed;
    }

    //if (!process_record_user(keycode, record))
    //    return false;

    if (keycode == DPI_CONFIG && record->event.pressed) {
        keyboard_config.dpi_config = (keyboard_config.dpi_config + 1) % DPI_OPTION_SIZE;
        eeconfig_update_kb(keyboard_config.raw);
        adns_set_cpi(dpi_array[keyboard_config.dpi_config]);
    }
*/

    //if (keycode == DRAG_SCROLL) {
		if (keycode == KC_M) {
#ifndef V99PAD_DRAGSCROLL_MOMENTARY
        if (record->event.pressed)
#endif
        {
            is_drag_scroll ^= 1;
        }
        //adns_set_cpi(is_drag_scroll ? PLOOPY_DRAGSCROLL_DPI : dpi_array[keyboard_config.dpi_config]);
    }


    return true;
}
#endif

void pointing_device_init(void) {
    //v99_slim_init();
		v99_init();
    //v99_slim_write(REG_CHIP_RESET, 0x5A);

    // wait maximum time before v99 sensor is ready.
    // this ensures that the v99 sensor is actuall ready after reset.
    wait_ms(55);

    // read a burst from the v99 sensor and then discard it.
    // gets the v99 sensor ready for write commands
    // (for example, setting the powerdown mode).
    //v99_slim_read_burst();

    //wait_us(30);
    // set the powerdown mode.
    //v99_slim_write(REG_MOUSE_CONTROL, 0x01); // default 0x00
}

void pointing_device_task(void) {

    static int16_t wheelh = 0;
		static int16_t wheelv = 0;

    report_mouse_t mouse_report = pointing_device_get_report();
    //process_wheel(&mouse_report);
    process_mouse(&mouse_report);

    if (is_drag_scroll) {
        //mouse_report.h = -mouse_report.x/V99_X_TRANSFORM;
				//mouse_report.h = mouse_report.x/4;

        wheelh += mouse_report.x;
				if(wheelh >= V99_WHEEL_H_DIV ){
					mouse_report.h = 1;
					wheelh-=V99_WHEEL_H_DIV;
				}else if(wheelh <= -V99_WHEEL_H_DIV){
					mouse_report.h = -1;
					wheelh+= V99_WHEEL_H_DIV;
				}
//#ifdef PLOOPY_DRAGSCROLL_INVERT
#ifdef V99PAD_DRAGSCROLL_INVERT
        // Invert vertical scroll direction
        //mouse_report.v = mouse_report.y/V99_Y_TRANSFORM;
				mouse_report.v = mouse_report.y/4;
#else
        //mouse_report.v = -mouse_report.y/V99_Y_TRANSFORM;
				//mouse_report.v = -mouse_report.y/4;

				wheelv += mouse_report.y;
				if(wheelv >= V99_WHEEL_V_DIV ){
					mouse_report.v = -1;
					wheelv-=V99_WHEEL_V_DIV;
				}else if(wheelv <= -V99_WHEEL_V_DIV){
					mouse_report.v = 1;
					wheelv+= V99_WHEEL_V_DIV;
				}
#endif
        if(abs(mouse_report.y) >= abs(mouse_report.x))
				{
					mouse_report.h = 0;
					wheelh = 0;
				}else{
					mouse_report.v = 0;
					wheelv = 0;
				}


        mouse_report.x = 0;
        mouse_report.y = 0;
    }

    pointing_device_set_report(mouse_report);
    pointing_device_send();
}

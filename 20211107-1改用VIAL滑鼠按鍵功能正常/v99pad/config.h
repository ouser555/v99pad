#ifndef CONFIG_H
#define CONFIG_H

#include "config_common.h"

/* USB Device descriptor parameter */
#define VENDOR_ID       0x1917
#define PRODUCT_ID      0x0001
#define DEVICE_VER      0x0001
#define MANUFACTURER    touchsens
#define PRODUCT         v99pad
#define DESCRIPTION     v99pad Keyboard

#define VIAL_KEYBOARD_UID {0x33, 0xA1, 0x98, 0xE6, 0x21, 0x62, 0x93, 0x05}

//#define DYNAMIC_KEYMAP_LAYER_COUNT 10
#define DYNAMIC_KEYMAP_LAYER_COUNT 4

//#define COMBO_COUNT 1
#define VIAL_COMBO_ENTRIES 1

/* key matrix size */
#define MATRIX_ROWS 7
#define MATRIX_COLS 5

/* key matrix pins */
//#define MATRIX_ROW_PINS { E6, D2, D6, D7, B4, B5, B6 }
#define MATRIX_ROW_PINS { E7, E5, D6, D7, B4, B5, B6 }
#define MATRIX_COL_PINS { B7, B0, B1, D4, D5 }
#define UNUSED_PINS

/* COL2ROW or ROW2COL */
#define DIODE_DIRECTION COL2ROW

/* encoder */

#define ENCODER1A D1
#define ENCODER1B C7
#define ENCODER2A F0
#define ENCODER2B F1
#define ENCODER3A F4
#define ENCODER3B F5
#define ENCODER4A F6
#define ENCODER4B F7

#define ENCODERS_PAD_A { ENCODER1A, ENCODER2A, ENCODER3A, ENCODER4A }
#define ENCODERS_PAD_B { ENCODER1B, ENCODER2B, ENCODER3B, ENCODER4B }

/* number of backlight levels */

#ifdef BACKLIGHT_PIN
#define BACKLIGHT_LEVELS 8
#endif

/* Set 0 if debouncing isn't needed */
#define DEBOUNCING_DELAY 30

//#define MOUSEKEY_DELAY 16

/* Mechanical locking support. Use KC_LCAP, KC_LNUM or KC_LSCR instead in keymap */
#define LOCKING_SUPPORT_ENABLE

/* Locking resynchronize hack */
#define LOCKING_RESYNC_ENABLE

/* key combination for command */
#define IS_COMMAND() ( \
    keyboard_report->mods == (MOD_BIT(KC_LSHIFT) | MOD_BIT(KC_RSHIFT)) \
)

/* prevent stuck modifiers */
#define PREVENT_STUCK_MODIFIERS

#define RGB_DI_PIN C6
#ifdef RGB_DI_PIN
#define RGBLIGHT_ANIMATIONS
#define RGBLED_NUM 19
#define RGBLIGHT_HUE_STEP 8
#define RGBLIGHT_SAT_STEP 8
#define RGBLIGHT_VAL_STEP 8
#endif

#endif

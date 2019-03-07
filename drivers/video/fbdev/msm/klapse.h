#ifndef _LINUX_KLAPSE_H
#define _LINUX_KLAPSE_H

/* Required variables for external access. Change as per use */
extern void set_rgb_slider(u32 bl_lvl);
void kcal_ext_apply_values(int red, int green, int blue);
int kcal_ext_get_value(int color);

enum rgb {
	KCAL_RED = 0,
	KCAL_GREEN,
	KCAL_BLUE,
};

#define KCAL_COLOR(x) (kcal_ext_get_value(x))

// This file uses generalised K_### defines
// The interpretation (right argument) should be the respective color's var
#define K_RED KCAL_COLOR(KCAL_RED)
#define K_GREEN KCAL_COLOR(KCAL_GREEN)
#define K_BLUE KCAL_COLOR(KCAL_BLUE)

/* DEFAULT_ENABLE values :
 * 0 = off
 * 1 = time-based scaling
 * 2 = brightness-based scaling
 */
#define DEFAULT_ENABLE  0

// MAX_SCALE : Maximum value of RGB possible
#define MAX_SCALE       256

// SCALE_VAL_MIN : Minimum value of RGB recommended
#define SCALE_VAL_MIN   20

// MAX_BRIGHTNESS : Maximum value of the display brightness/backlight
#define MAX_BRIGHTNESS  255

// MIN_BRIGHTNESS : Minimum value of the display brightness/backlight
#define MIN_BRIGHTNESS  2

/* UPPER_BL_LVL : Initial upper limit for brightness-dependent mode. 
 * Value <= MAX_BRIGHTNESS && > LOWER_BL_LVL (MUST)
 */
#define UPPER_BL_LVL  200

/* LOWER_BL_LVL : Initial lower limit for brightness-dependent mode. 
 * Value < UPPER_BL_LVL (MUST)
 */
#define LOWER_BL_LVL 2

#endif  /* _LINUX_KLAPSE_H */

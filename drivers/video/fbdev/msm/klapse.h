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
// The interpretation (right argument) should be the respective color's var that you
// include as extern via the CUSTOM_HEADER above
#define K_RED KCAL_COLOR(KCAL_RED)
#define K_GREEN KCAL_COLOR(KCAL_GREEN)
#define K_BLUE KCAL_COLOR(KCAL_BLUE)

#endif  /* _LINUX_KLAPSE_H */

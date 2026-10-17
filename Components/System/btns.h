#ifndef BTNS_H_
#define BTNS_H_

#include "main.h"
#include <stdbool.h>

/* Call this once before the main loop (optional, for state reset) */
void BTNS_Init(void);

/* Call this continuously inside your main while(1) loop */
void BTNS_Update(void);

/* Edge Detection: Returns true ONLY ONCE per physical button press.
 * Ideal for changing GUI pages or toggling states. */
bool BTNS_IsK0_Pressed(void);
bool BTNS_IsK1_Pressed(void);

/* State Detection: Returns true continuously as long as the button is held down.
 * Ideal for manual driving controls (e.g., holding a button to spin the vacuum). */
bool BTNS_IsK0_Held(void);
bool BTNS_IsK1_Held(void);

#endif /* BTNS_H_ */

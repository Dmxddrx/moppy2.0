#ifndef OLED_GUI_H
#define OLED_GUI_H

#include "general.h"
#include "oled.h"
#include "side_brushes.h"
#include "chassis_control.h"
#include "roller_brush.h"
#include "lidar_uart.h"
#include "lidar_pid.h"
#include "imu_manager.h"

/* Global variable to control the current page (0=Wheels, 1=Vacuum, 2=Brushes) */
extern uint8_t current_gui_page;

/* Primary dashboard rendering function */
void render_dashboard_page(void);

#endif /* OLED_GUI_H */

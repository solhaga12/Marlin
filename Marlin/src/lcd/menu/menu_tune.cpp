/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2019 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Based on Sprinter and grbl.
 * Copyright (c) 2011 Camiel Gubbels / Erik van der Zalm
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

//
// Tune Menu
//

#include "../../inc/MarlinConfigPre.h"

#if HAS_LCD_MENU

#include "menu.h"
#include "../../module/motion.h"
#include "../../module/planner.h"
#include "../../module/voltages.h"
#include "../../Marlin.h"

#if ENABLED(BABYSTEPPING)

  #include "../../feature/babystep.h"
  #include "../lcdprint.h"
  #if HAS_GRAPHICAL_LCD
    #include "../dogm/ultralcd_DOGM.h"
  #endif

  void _lcd_babystep(const AxisEnum axis, PGM_P const msg) {
    if (ui.use_click()) return ui.goto_previous_screen_no_defer();
    if (ui.encoderPosition) {
      const int16_t steps = int16_t(ui.encoderPosition) * (BABYSTEP_MULTIPLICATOR);
      ui.encoderPosition = 0;
      ui.refresh(LCDVIEW_REDRAW_NOW);
      babystep.add_steps(axis, steps);
    }
    if (ui.should_draw()) {
      const float spm = planner.steps_to_mm[axis];
      draw_edit_screen(msg, ftostr54sign(spm * babystep.accum));
      #if ENABLED(BABYSTEP_DISPLAY_TOTAL)
        const bool in_view = (true
          #if HAS_GRAPHICAL_LCD
            && PAGE_CONTAINS(LCD_PIXEL_HEIGHT - MENU_FONT_HEIGHT, LCD_PIXEL_HEIGHT - 1)
          #endif
        );
        if (in_view) {
          #if HAS_GRAPHICAL_LCD
            ui.set_font(FONT_MENU);
            lcd_moveto(0, LCD_PIXEL_HEIGHT - MENU_FONT_DESCENT);
          #else
            lcd_moveto(0, LCD_HEIGHT - 1);
          #endif
          lcd_put_u8str_P(PSTR(MSG_BABYSTEP_TOTAL ":"));
          lcd_put_u8str(ftostr54sign(spm * babystep.axis_total[BS_TOTAL_AXIS(axis)]));
        }
      #endif
    }
  }

  inline void _lcd_babystep_go(const screenFunc_t screen) {
    ui.goto_screen(screen);
    ui.defer_status_screen();
    babystep.accum = 0;
  }

  #if ENABLED(BABYSTEP_XY)
    void _lcd_babystep_x() { _lcd_babystep(X_AXIS, PSTR(MSG_BABYSTEP_X)); }
    void _lcd_babystep_y() { _lcd_babystep(Y_AXIS, PSTR(MSG_BABYSTEP_Y)); }
    void lcd_babystep_x() { _lcd_babystep_go(_lcd_babystep_x); }
    void lcd_babystep_y() { _lcd_babystep_go(_lcd_babystep_y); }
  #endif

  #if DISABLED(BABYSTEP_ZPROBE_OFFSET)
    void _lcd_babystep_z() { _lcd_babystep(Z_AXIS, PSTR(MSG_BABYSTEP_Z)); }
    void lcd_babystep_z() { _lcd_babystep_go(_lcd_babystep_z); }
  #endif

#endif // BABYSTEPPING

void menu_tune() {
  START_MENU();
  MENU_BACK(MSG_MAIN);

  //
  // Speed:
  //
  MENU_ITEM_EDIT(int3, MSG_SPEED, &feedrate_percentage, 10, 999);


  //
  // Babystep X:
  // Babystep Y:
  // Babystep Z:
  //
  #if ENABLED(BABYSTEPPING)
    #if ENABLED(BABYSTEP_XY)
      MENU_ITEM(submenu, MSG_BABYSTEP_X, lcd_babystep_x);
      MENU_ITEM(submenu, MSG_BABYSTEP_Y, lcd_babystep_y);
    #endif
    #if ENABLED(BABYSTEP_ZPROBE_OFFSET)
      MENU_ITEM(submenu, MSG_ZPROBE_ZOFFSET, lcd_babystep_zoffset);
    #else
      MENU_ITEM(submenu, MSG_BABYSTEP_Z, lcd_babystep_z);
    #endif
  #endif

  END_MENU();
}

#endif // HAS_LCD_MENU

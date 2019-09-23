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

#include "../../inc/MarlinConfigPre.h"
#include "../../core/serial.h"

#if HAS_CHARACTER_LCD

/**
 * ultralcd_HD44780.cpp
 *
 * LCD display implementations for Hitachi HD44780.
 * These are the most common LCD character displays.
 */

#include "ultralcd_HD44780.h"
#include "../ultralcd.h"
#include "../../libs/numtostr.h"

#include "../../sd/cardreader.h"
#include "../../module/voltages.h"
#include "../../module/printcounter.h"
#include "../../module/planner.h"
#include "../../module/motion.h"
#include "../../libs/numtostr.h"

#if DISABLED(LCD_PROGRESS_BAR) && BOTH(FILAMENT_LCD_DISPLAY, SDSUPPORT)
  #include "../../feature/filwidth.h"
  #include "../../gcode/parser.h"
#endif

#if ENABLED(AUTO_BED_LEVELING_UBL)
  #include "../../feature/bedlevel/ubl/ubl.h"
#endif

//
// Create LCD instance and chipset-specific information
//

#if ENABLED(LCD_I2C_TYPE_PCF8575)

  LCD_CLASS lcd(LCD_I2C_ADDRESS, LCD_I2C_PIN_EN, LCD_I2C_PIN_RW, LCD_I2C_PIN_RS, LCD_I2C_PIN_D4, LCD_I2C_PIN_D5, LCD_I2C_PIN_D6, LCD_I2C_PIN_D7);

#elif EITHER(LCD_I2C_TYPE_MCP23017, LCD_I2C_TYPE_MCP23008)

  LCD_CLASS lcd(LCD_I2C_ADDRESS
    #ifdef DETECT_DEVICE
      , 1
    #endif
  );

#elif ENABLED(LCD_I2C_TYPE_PCA8574)

  LCD_CLASS lcd(LCD_I2C_ADDRESS, LCD_WIDTH, LCD_HEIGHT);

#elif ENABLED(SR_LCD_2W_NL)

  // 2 wire Non-latching LCD SR from:
  // https://bitbucket.org/fmalpartida/new-liquidcrystal/wiki/schematics#!shiftregister-connection

  LCD_CLASS lcd(SR_DATA_PIN, SR_CLK_PIN
    #if PIN_EXISTS(SR_STROBE)
      , SR_STROBE_PIN
    #endif
  );

#elif ENABLED(SR_LCD_3W_NL)

  // NewLiquidCrystal was not working
  // https://github.com/mikeshub/SailfishLCD
  // uses the code directly from Sailfish

  LCD_CLASS lcd(SR_STROBE_PIN, SR_DATA_PIN, SR_CLK_PIN);

#elif ENABLED(LCM1602)

  LCD_CLASS lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);

#else

  // Standard direct-connected LCD implementations
  LCD_CLASS lcd(LCD_PINS_RS, LCD_PINS_ENABLE, LCD_PINS_D4, LCD_PINS_D5, LCD_PINS_D6, LCD_PINS_D7);

#endif

static void createChar_P(const char c, const byte * const ptr) {
  byte temp[8];
  for (uint8_t i = 0; i < 8; i++)
    temp[i] = pgm_read_byte(&ptr[i]);
  lcd.createChar(c, temp);
}

#if ENABLED(LCD_PROGRESS_BAR)
  #define LCD_STR_PROGRESS  "\x03\x04\x05"
#endif

void MarlinUI::set_custom_characters(const HD44780CharSet screen_charset/*=CHARSET_INFO*/) {
  #if NONE(LCD_PROGRESS_BAR, SHOW_BOOTSCREEN)
    UNUSED(screen_charset);
  #endif

  // CHARSET_BOOT

  // CHARSET_INFO
  const static PROGMEM byte bedTemp[8] = {
    B00000,
    B11111,
    B10101,
    B10001,
    B10101,
    B11111,
    B00000,
    B00000
  };

  const static PROGMEM byte degree[8] = {
    B01100,
    B10010,
    B10010,
    B01100,
    B00000,
    B00000,
    B00000,
    B00000
  };

  const static PROGMEM byte thermometer[8] = {
    B00100,
    B01010,
    B01010,
    B01010,
    B01010,
    B10001,
    B10001,
    B01110
  };

  const static PROGMEM byte uplevel[8] = {
    B00100,
    B01110,
    B11111,
    B00100,
    B11100,
    B00000,
    B00000,
    B00000
  };

  const static PROGMEM byte feedrate[8] = {
    #if LCD_INFO_SCREEN_STYLE == 1
      B00000,
      B00100,
      B10010,
      B01001,
      B10010,
      B00100,
      B00000,
      B00000
    #else
      B11100,
      B10000,
      B11000,
      B10111,
      B00101,
      B00110,
      B00101,
      B00000
    #endif
  };

  const static PROGMEM byte clock[8] = {
    B00000,
    B01110,
    B10011,
    B10101,
    B10001,
    B01110,
    B00000,
    B00000
  };

  #if ENABLED(LCD_PROGRESS_BAR)

    // CHARSET_INFO
    const static PROGMEM byte progress[3][8] = { {
      B00000,
      B10000,
      B10000,
      B10000,
      B10000,
      B10000,
      B10000,
      B00000
    }, {
      B00000,
      B10100,
      B10100,
      B10100,
      B10100,
      B10100,
      B10100,
      B00000
    }, {
      B00000,
      B10101,
      B10101,
      B10101,
      B10101,
      B10101,
      B10101,
      B00000
    } };

  #endif // LCD_PROGRESS_BAR

  #if ENABLED(SDSUPPORT)

    // CHARSET_MENU
    const static PROGMEM byte refresh[8] = {
      B00000,
      B00110,
      B11001,
      B11000,
      B00011,
      B10011,
      B01100,
      B00000,
    };
    const static PROGMEM byte folder[8] = {
      B00000,
      B11100,
      B11111,
      B10001,
      B10001,
      B11111,
      B00000,
      B00000
    };

  #endif // SDSUPPORT

    { // Info Screen uses 5 special characters
      createChar_P(LCD_STR_BEDTEMP[0], bedTemp);
      createChar_P(LCD_STR_DEGREE[0], degree);
      createChar_P(LCD_STR_THERMOMETER[0], thermometer);
      createChar_P(LCD_STR_FEEDRATE[0], feedrate);
      createChar_P(LCD_STR_CLOCK[0], clock);

      #if ENABLED(LCD_PROGRESS_BAR)
        if (screen_charset == CHARSET_INFO) { // 3 Progress bar characters for info screen
          for (int16_t i = 3; i--;)
            createChar_P(LCD_STR_PROGRESS[i], progress[i]);
        }
        else
      #endif
        {
          createChar_P(LCD_STR_UPLEVEL[0], uplevel);
          #if ENABLED(SDSUPPORT)
            // SD Card sub-menu special characters
            createChar_P(LCD_STR_REFRESH[0], refresh);
            createChar_P(LCD_STR_FOLDER[0], folder);
          #endif
        }
    }

}

void MarlinUI::init_lcd() {

  #if ENABLED(LCD_I2C_TYPE_PCF8575)
    lcd.begin(LCD_WIDTH, LCD_HEIGHT);
    #ifdef LCD_I2C_PIN_BL
      lcd.setBacklightPin(LCD_I2C_PIN_BL, POSITIVE);
      lcd.setBacklight(HIGH);
    #endif

  #elif ENABLED(LCD_I2C_TYPE_MCP23017)
    lcd.setMCPType(LTI_TYPE_MCP23017);
    lcd.begin(LCD_WIDTH, LCD_HEIGHT);
    update_indicators();

  #elif ENABLED(LCD_I2C_TYPE_MCP23008)
    lcd.setMCPType(LTI_TYPE_MCP23008);
    lcd.begin(LCD_WIDTH, LCD_HEIGHT);

  #elif ENABLED(LCD_I2C_TYPE_PCA8574)
    lcd.init();
    lcd.backlight();

  #else
    lcd.begin(LCD_WIDTH, LCD_HEIGHT);
  #endif

  set_custom_characters(on_status_screen() ? CHARSET_INFO : CHARSET_MENU);

  lcd.clear();
}

void MarlinUI::clear_lcd() { lcd.clear(); }

void MarlinUI::draw_kill_screen() {
  lcd_moveto(0, 0);
  lcd_put_u8str(status_message);
  #if LCD_HEIGHT < 4
    lcd_moveto(0, 2);
  #else
    lcd_moveto(0, 2);
    lcd_put_u8str_P(PSTR(MSG_HALTED));
    lcd_moveto(0, 3);
  #endif
  lcd_put_u8str_P(PSTR(MSG_PLEASE_RESET));
}

//
// Before homing, blink '123' <-> '???'.
// Homed but unknown... '123' <-> '   '.
// Homed and known, display constantly.
//
FORCE_INLINE void _draw_axis_value(const AxisEnum axis, const char *value, const bool blink) {
  lcd_put_wchar('X' + uint8_t(axis));
  if (blink)
    lcd_put_u8str(value);
  else {
    if (!TEST(axis_homed, axis))
      while (const char c = *value++) lcd_put_wchar(c <= '.' ? c : '?');
    else {
      #if NONE(HOME_AFTER_DEACTIVATE, DISABLE_REDUCED_ACCURACY_WARNING)
        if (!TEST(axis_known_position, axis))
          lcd_put_u8str_P(axis == Z_AXIS ? PSTR("       ") : PSTR("    "));
        else
      #endif
          lcd_put_u8str(value);
    }
  }
}


#if HAS_PRINT_PROGRESS

  FORCE_INLINE void _draw_print_progress() {
    const uint8_t progress = ui.get_progress();
    lcd_put_u8str_P(PSTR(
      #if ENABLED(SDSUPPORT)
        "SD"
      #elif ENABLED(LCD_SET_PROGRESS_MANUALLY)
        "P:"
      #endif
    ));
    if (progress)
      lcd_put_u8str(ui8tostr3(progress));
    else
      lcd_put_u8str_P(PSTR("---"));
    lcd_put_wchar('%');
  }

#endif

#if ENABLED(LCD_PROGRESS_BAR)

  void MarlinUI::draw_progress_bar(const uint8_t percent) {
    const int16_t tix = (int16_t)(percent * (LCD_WIDTH) * 3) / 100,
              cel = tix / 3,
              rem = tix % 3;
    uint8_t i = LCD_WIDTH;
    char msg[LCD_WIDTH + 1], b = ' ';
    msg[LCD_WIDTH] = '\0';
    while (i--) {
      if (i == cel - 1)
        b = LCD_STR_PROGRESS[2];
      else if (i == cel && rem != 0)
        b = LCD_STR_PROGRESS[rem - 1];
      msg[i] = b;
    }
    lcd_put_u8str(msg);
  }

#endif // LCD_PROGRESS_BAR

void MarlinUI::draw_status_message(const bool blink) {

  lcd_moveto(0, LCD_HEIGHT - 1);

  #if ENABLED(LCD_PROGRESS_BAR)

    // Draw the progress bar if the message has shown long enough
    // or if there is no message set.
    if (ELAPSED(millis(), progress_bar_ms + PROGRESS_BAR_MSG_TIME) || !has_status()) {
      const uint8_t progress = get_progress();
      if (progress > 2) return draw_progress_bar(progress);
    }

  #elif BOTH(FILAMENT_LCD_DISPLAY, SDSUPPORT)

    // Alternate Status message and Filament display
    if (ELAPSED(millis(), next_filament_display)) {
      lcd_put_u8str_P(PSTR("Dia "));
      lcd_put_u8str(ftostr12ns(filament_width_meas));
      lcd_put_u8str_P(PSTR(" V"));
      lcd_put_u8str(i16tostr3(100.0 * (
          parser.volumetric_enabled
            ? planner.volumetric_area_nominal / planner.volumetric_multiplier[FILAMENT_SENSOR_EXTRUDER_NUM]
            : planner.volumetric_multiplier[FILAMENT_SENSOR_EXTRUDER_NUM]
        )
      ));
      lcd_put_wchar('%');
      return;
    }

  #endif // FILAMENT_LCD_DISPLAY && SDSUPPORT

  #if ENABLED(STATUS_MESSAGE_SCROLLING)
    static bool last_blink = false;

    // Get the UTF8 character count of the string
    uint8_t slen = utf8_strlen(status_message);

    // If the string fits into the LCD, just print it and do not scroll it
    if (slen <= LCD_WIDTH) {

      // The string isn't scrolling and may not fill the screen
      lcd_put_u8str(status_message);

      // Fill the rest with spaces
      while (slen < LCD_WIDTH) { lcd_put_wchar(' '); ++slen; }
    }
    else {
      // String is larger than the available space in screen.

      // Get a pointer to the next valid UTF8 character
      // and the string remaining length
      uint8_t rlen;
      const char *stat = status_and_len(rlen);
      lcd_put_u8str_max(stat, LCD_WIDTH);     // The string leaves space

      // If the remaining string doesn't completely fill the screen
      if (rlen < LCD_WIDTH) {
        lcd_put_wchar('.');                   // Always at 1+ spaces left, draw a dot
        uint8_t chars = LCD_WIDTH - rlen;     // Amount of space left in characters
        if (--chars) {                        // Draw a second dot if there's space
          lcd_put_wchar('.');
          if (--chars)
            lcd_put_u8str_max(status_message, chars); // Print a second copy of the message
        }
      }
      if (last_blink != blink) {
        last_blink = blink;
        advance_status_scroll();
      }
    }
  #else
    UNUSED(blink);

    // Get the UTF8 character count of the string
    uint8_t slen = utf8_strlen(status_message);

    // Just print the string to the LCD
    lcd_put_u8str_max(status_message, LCD_WIDTH);

    // Fill the rest with spaces if there are missing spaces
    while (slen < LCD_WIDTH) {
      lcd_put_wchar(' ');
      ++slen;
    }
  #endif
}

/**
 *  LCD_INFO_SCREEN_STYLE 0 : Classic Status Screen
 *
 *  16x2   |000/000 B000/000|
 *         |0123456789012345|
 *
 *  16x4   |000/000 B000/000|
 *         |SD---%  Z 000.00|
 *         |F---%     T--:--|
 *         |0123456789012345|
 *
 *  20x2   |T000/000° B000/000° |
 *         |01234567890123456789|
 *
 *  20x4   |T000/000° B000/000° |
 *         |X 000 Y 000 Z000.000|
 *         |F---%  SD---% T--:--|
 *         |01234567890123456789|
 *
 *  LCD_INFO_SCREEN_STYLE 1 : Průša-style Status Screen
 *
 *  |T000/000°  Z 000.00 |
 *  |B000/000°  F---%    |
 *  |SD---%     T--:--   |
 *  |01234567890123456789|
 *
 *  |T000/000°  Z 000.00 |
 *  |T000/000°  F---%    |
 *  |B000/000°  SD---%   |
 *  |01234567890123456789|
 */

void MarlinUI::draw_status_screen() {

  const bool blink = true;  // Don't blink when not auto squared
  char buffer[14];

  lcd_moveto(0, 0);

    // ========== Line 1 ==========

  // Show actual/wanted Divider Voltage

  lcd_put_wchar('T');
  lcd_put_wchar('H');
  lcd_put_wchar('C');
  lcd_put_wchar(' ');
  lcd_put_u8str(ftostr41nosign(Voltage::getActualThcVoltage()/SLOPE));
  lcd_put_wchar(' ');
  if (Voltage::getActualThcVoltage() == Voltage::getWantedThcVoltage()) lcd_put_wchar('=');
  else if (Voltage::getActualThcVoltage() < Voltage::getWantedThcVoltage()) lcd_put_wchar('<');
  else if (Voltage::getActualThcVoltage() > Voltage::getWantedThcVoltage()) lcd_put_wchar('>');
  lcd_put_wchar(' ');
  lcd_put_u8str(ftostr41nosign(Voltage::getWantedThcVoltage()/SLOPE));

    // ========== Line 2 ==========

  lcd_moveto(0, 1);

  _draw_axis_value(X_AXIS, ftostr4sign(LOGICAL_X_POSITION(current_position[X_AXIS])), blink);
  lcd_put_wchar(' ');
  _draw_axis_value(Y_AXIS, ftostr4sign(LOGICAL_Y_POSITION(current_position[Y_AXIS])), blink);
  lcd_moveto(LCD_WIDTH - 8, 1);
  _draw_axis_value(Z_AXIS, ftostr52sp(LOGICAL_Z_POSITION(current_position[Z_AXIS])), blink);

    // ========== Line 3 ==========

  lcd_moveto(0, 2);

  // Put Plasma status here. Idle, Start, Transfer and Cutting

  duration_t elapsed = print_job_timer.duration();
  const uint8_t len = elapsed.toDigital(buffer),
                    timepos = LCD_WIDTH - len - 1;
  lcd_moveto(timepos, 2);
  lcd_put_wchar(LCD_STR_CLOCK[0]);
  lcd_put_u8str(buffer);


  // ========= Line 4 ========

  //
  // Status Message (which may be a Progress Bar or Filament display)
  //
  draw_status_message(blink);
}

#if HAS_LCD_MENU

  void draw_menu_item_static(const uint8_t row, PGM_P pstr, const bool center/*=true*/, const bool invert/*=false*/, const char *valstr/*=nullptr*/) {
    UNUSED(invert);
    int8_t n = LCD_WIDTH;
    lcd_moveto(0, row);
    if (center && !valstr) {
      int8_t pad = (LCD_WIDTH - utf8_strlen_P(pstr)) / 2;
      while (--pad >= 0) { lcd_put_wchar(' '); n--; }
    }
    n -= lcd_put_u8str_max_P(pstr, n);
    if (valstr) n -= lcd_put_u8str_max(valstr, n);
    for (; n > 0; --n) lcd_put_wchar(' ');
  }

  void draw_menu_item(const bool sel, const uint8_t row, PGM_P pstr, const char pre_char, const char post_char) {
    uint8_t n = LCD_WIDTH - 2;
    lcd_moveto(0, row);
    lcd_put_wchar(sel ? pre_char : ' ');
    n -= lcd_put_u8str_max_P(pstr, n);
    for (; n; --n) lcd_put_wchar(' ');
    lcd_put_wchar(post_char);
  }

  void _draw_menu_item_edit(const bool sel, const uint8_t row, PGM_P pstr, const char* const data, const bool pgm) {
    uint8_t n = LCD_WIDTH - 2 - (pgm ? utf8_strlen_P(data) : utf8_strlen(data));
    lcd_moveto(0, row);
    lcd_put_wchar(sel ? LCD_STR_ARROW_RIGHT[0] : ' ');
    n -= lcd_put_u8str_max_P(pstr, n);
    lcd_put_wchar(':');
    for (; n; --n) lcd_put_wchar(' ');
    if (pgm) lcd_put_u8str_P(data); else lcd_put_u8str(data);
  }

  void draw_edit_screen(PGM_P const pstr, const char* const value/*=nullptr*/) {
    lcd_moveto(0, 1);
    lcd_put_u8str_P(pstr);
    if (value != nullptr) {
      lcd_put_wchar(':');
      int len = utf8_strlen(value);
      const uint8_t valrow = (utf8_strlen_P(pstr) + 1 + len + 1) > (LCD_WIDTH - 2) ? 2 : 1;   // Value on the next row if it won't fit
      lcd_moveto((LCD_WIDTH - 1) - (len + 1), valrow);                                        // Right-justified, padded by spaces
      lcd_put_wchar(' ');                                                                     // Overwrite char if value gets shorter
      lcd_put_u8str(value);
    }
  }

  void draw_select_screen(PGM_P const yes, PGM_P const no, const bool yesno, PGM_P const pref, const char * const string, PGM_P const suff) {
    ui.draw_select_screen_prompt(pref, string, suff);
    SETCURSOR(0, LCD_HEIGHT - 1);
    lcd_put_wchar(yesno ? ' ' : '['); lcd_put_u8str_P(no); lcd_put_wchar(yesno ? ' ' : ']');
    SETCURSOR_RJ(utf8_strlen_P(yes) + 2, LCD_HEIGHT - 1);
    lcd_put_wchar(yesno ? '[' : ' '); lcd_put_u8str_P(yes); lcd_put_wchar(yesno ? ']' : ' ');
  }

  #if ENABLED(SDSUPPORT)

    void draw_sd_menu_item(const bool sel, const uint8_t row, PGM_P const pstr, CardReader &theCard, const bool isDir) {
      UNUSED(pstr);

      lcd_moveto(0, row);
      lcd_put_wchar(sel ? LCD_STR_ARROW_RIGHT[0] : ' ');
      constexpr uint8_t maxlen = LCD_WIDTH - 2;
      uint8_t n = maxlen - lcd_put_u8str_max(ui.scrolled_filename(theCard, maxlen, row, sel), maxlen);
      for (; n; --n) lcd_put_wchar(' ');
      lcd_put_wchar(isDir ? LCD_STR_FOLDER[0] : ' ');
    }

  #endif // SDSUPPORT

#endif // HAS_LCD_MENU

#endif // HAS_CHARACTER_LCD

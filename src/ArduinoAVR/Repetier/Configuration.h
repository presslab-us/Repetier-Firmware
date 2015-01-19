/*
    This file is part of Repetier-Firmware.

    Repetier-Firmware is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    Repetier-Firmware is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with Repetier-Firmware.  If not, see <http://www.gnu.org/licenses/>.

*/

#ifndef CONFIGURATION_H
#define CONFIGURATION_H

/**************** READ FIRST ************************

   This configuration file was created with the configuration tool. For that
   reason, it does not contain the same informations as the original Configuration.h file.
   It misses the comments and unused parts. Open this file file in the config tool
   to see and change the data. You can also upload it to newer/older versions. The system
   will silently add new options, so compilation continues to work.

   This file is optimized for version 0.91
   generator: http://www.repetier.com/firmware/v091/

   If you are in doubt which named functions use which pins on your board, please check the
   pins.h for the used name->pin assignments and your board documentation to verify it is
   as you expect.

*/

#define NUM_EXTRUDER 2
#define MOTHERBOARD 301

#include "pins.h"

// ################## EDIT THESE SETTINGS MANUALLY ################
// Microstepping mode of your RAMBO board
#define MICROSTEP_MODES { 16,16,16,16,16 } // [1,2,4,8,16]
// Motor Current setting (Only functional when motor driver current ref pins are connected to a digital trimpot on supported boards)
#define MOTOR_CURRENT { 230,230,230,160,160 } // Values 0-255 (RAMBO 135 = ~0.75A, 185 = ~1A)

// ################ END MANUAL SETTINGS ##########################

#define FAN_BOARD_PIN -1

//#define EXTERNALSERIAL  use Arduino serial library instead of build in. Requires more ram, has only 63 byte input buffer.
// Uncomment the following line if you are using arduino compatible firmware made for Arduino version earlier then 1.0
// If it is incompatible you will get compiler errors about write functions not beeing compatible!
//#define COMPAT_PRE1
#define MIXING_EXTRUDER 1

#define DRIVE_SYSTEM 3
#define XAXIS_STEPS_PER_MM 80
#define YAXIS_STEPS_PER_MM 80
#define ZAXIS_STEPS_PER_MM 80
#define EXTRUDER_FAN_COOL_TEMP -50
#define PDM_FOR_EXTRUDER 0
#define PDM_FOR_COOLER 0
#define DECOUPLING_TEST_MAX_HOLD_VARIANCE 25
#define DECOUPLING_TEST_MIN_TEMP_RISE 1
#define EXT0_X_OFFSET 0
#define EXT0_Y_OFFSET 0
#define EXT0_STEPS_PER_MM 93
#define EXT0_TEMPSENSOR_TYPE 8
#define EXT0_TEMPSENSOR_PIN TEMP_0_PIN
#define EXT0_HEATER_PIN HEATER_0_PIN
#define EXT0_STEP_PIN ORIG_E0_STEP_PIN
#define EXT0_DIR_PIN ORIG_E0_DIR_PIN
#define EXT0_INVERSE 1
#define EXT0_ENABLE_PIN E0_ENABLE_PIN
#define EXT0_ENABLE_ON 0
#define EXT0_MAX_FEEDRATE 160
#define EXT0_MAX_START_FEEDRATE 20
#define EXT0_MAX_ACCELERATION 1000
#define EXT0_HEAT_MANAGER 3
#define EXT0_WATCHPERIOD 1
#define EXT0_PID_INTEGRAL_DRIVE_MAX 180
#define EXT0_PID_INTEGRAL_DRIVE_MIN 40
#define EXT0_PID_PGAIN_OR_DEAD_TIME 3
#define EXT0_PID_I 0
#define EXT0_PID_D 0
#define EXT0_PID_MAX 255
#define EXT0_ADVANCE_K 0
#define EXT0_ADVANCE_L 40.0
#define EXT0_ADVANCE_BACKLASH_STEPS 0
#define EXT0_WAIT_RETRACT_TEMP 150
#define EXT0_WAIT_RETRACT_UNITS 0
#define EXT0_SELECT_COMMANDS ""
#define EXT0_DESELECT_COMMANDS ""
#define EXT0_EXTRUDER_COOLER_PIN HEATER_2_PIN
#define EXT0_EXTRUDER_COOLER_SPEED 255
#define EXT0_DECOUPLE_TEST_PERIOD 18000
#define EXT1_X_OFFSET 0
#define EXT1_Y_OFFSET 0
#define EXT1_STEPS_PER_MM 93
#define EXT1_TEMPSENSOR_TYPE 8
#define EXT1_TEMPSENSOR_PIN TEMP_0_PIN
#define EXT1_HEATER_PIN -1
#define EXT1_STEP_PIN ORIG_E1_STEP_PIN
#define EXT1_DIR_PIN ORIG_E1_DIR_PIN
#define EXT1_INVERSE 1
#define EXT1_ENABLE_PIN E1_ENABLE_PIN
#define EXT1_ENABLE_ON 0
#define EXT1_MAX_FEEDRATE 160
#define EXT1_MAX_START_FEEDRATE 20
#define EXT1_MAX_ACCELERATION 1000
#define EXT1_HEAT_MANAGER 3
#define EXT1_WATCHPERIOD 1
#define EXT1_PID_INTEGRAL_DRIVE_MAX 180
#define EXT1_PID_INTEGRAL_DRIVE_MIN 40
#define EXT1_PID_PGAIN_OR_DEAD_TIME 3
#define EXT1_PID_I 0
#define EXT1_PID_D 0
#define EXT1_PID_MAX 255
#define EXT1_ADVANCE_K 0
#define EXT1_ADVANCE_L 40.0
#define EXT1_ADVANCE_BACKLASH_STEPS 0
#define EXT1_WAIT_RETRACT_TEMP 150
#define EXT1_WAIT_RETRACT_UNITS 0
#define EXT1_SELECT_COMMANDS ""
#define EXT1_DESELECT_COMMANDS ""
#define EXT1_EXTRUDER_COOLER_PIN -1
#define EXT1_EXTRUDER_COOLER_SPEED 255

/** Allow retraction with G10/G11 removing requirement for retraction setting in slicer. Also allows filament change if lcd is configured. */
#define FEATURE_RETRACTION 1
/** autoretract converts pure axtrusion moves into retractions. Beware that 
 simple extrusion e.g. over Repetier-Host will then not work! */
#define AUTORETRACT_ENABLED 0
#define RETRACTION_LENGTH 3
#define RETRACTION_LONG_LENGTH 13
#define RETRACTION_SPEED 40
#define RETRACTION_Z_LIFT 0
#define RETRACTION_UNDO_EXTRA_LENGTH 0
#define RETRACTION_UNDO_EXTRA_LONG_LENGTH 0
#define RETRACTION_UNDO_SPEED 20

/**
If you have a lcd display, you can do a filament switch with M600.
It will change the current extruders filament and temperature must already be high enough.
*/
#define FILAMENTCHANGE_X_POS 0
#define FILAMENTCHANGE_Y_POS 0
#define FILAMENTCHANGE_Z_ADD 1
/** Does a homing procedure after a filament change. This is good in case
you moved the extruder while changing filament during print.
0 = no homing, 1 = xy homing, 2 = xyz homing
*/
#define FILAMENTCHANGE_REHOME 1
/** Will first retract short distance, go to change position and then retract longretract.
Retractions speeds are taken from RETRACTION_SPEED and RETRACTION_UNDO_SPEED
*/
#define FILAMENTCHANGE_SHORTRETRACT 30
#define FILAMENTCHANGE_LONGRETRACT 30

/** PID control only works target temperature +/- PID_CONTROL_RANGE.
If you get much overshoot at the first temperature set, because the heater is going full power too long, you
need to increase this value. For one 6.8 Ohm heater 10 is ok. With two 6.8 Ohm heater use 15.
*/
#define EXT1_DECOUPLE_TEST_PERIOD 18000
#define RETRACT_DURING_HEATUP true
#define PID_CONTROL_RANGE 20
#define SKIP_M109_IF_WITHIN 2
#define SCALE_PID_TO_MAX 0
#define TEMP_HYSTERESIS 0
#define EXTRUDE_MAXLENGTH 160
#define NUM_TEMPS_USERTHERMISTOR0 0
#define USER_THERMISTORTABLE0 {}
#define NUM_TEMPS_USERTHERMISTOR1 0
#define USER_THERMISTORTABLE1 {}
#define NUM_TEMPS_USERTHERMISTOR2 0
#define USER_THERMISTORTABLE2 {}
#define GENERIC_THERM_VREF 5
#define GENERIC_THERM_NUM_ENTRIES 33
#define HEATER_PWM_SPEED 0

// ############# Heated bed configuration ########################

#define HAVE_HEATED_BED 1
#define HEATED_BED_MAX_TEMP 120
#define SKIP_M190_IF_WITHIN 5
#define HEATED_BED_SENSOR_TYPE 8
#define HEATED_BED_SENSOR_PIN TEMP_1_PIN
#define HEATED_BED_HEATER_PIN HEATER_1_PIN
#define HEATED_BED_SET_INTERVAL 5000
#define HEATED_BED_HEAT_MANAGER 3
#define HEATED_BED_PID_INTEGRAL_DRIVE_MAX 192
#define HEATED_BED_PID_INTEGRAL_DRIVE_MIN 10
#define HEATED_BED_PID_PGAIN_OR_DEAD_TIME 20
#define HEATED_BED_PID_IGAIN   0
#define HEATED_BED_PID_DGAIN 0
#define HEATED_BED_PID_MAX 255
#define HEATED_BED_DECOUPLE_TEST_PERIOD 120000
#define MIN_EXTRUDER_TEMP 150
#define MAXTEMP 280
#define MIN_DEFECT_TEMPERATURE -10
#define MAX_DEFECT_TEMPERATURE 290

// ################ Endstop configuration #####################

#define ENDSTOP_PULLUP_X_MIN true
#define ENDSTOP_X_MIN_INVERTING false
#define MIN_HARDWARE_ENDSTOP_X false
#define ENDSTOP_PULLUP_Y_MIN true
#define ENDSTOP_Y_MIN_INVERTING false
#define MIN_HARDWARE_ENDSTOP_Y false
#define ENDSTOP_PULLUP_Z_MIN true
#define ENDSTOP_Z_MIN_INVERTING false
#define MIN_HARDWARE_ENDSTOP_Z false
#define ENDSTOP_PULLUP_X_MAX true
#define ENDSTOP_X_MAX_INVERTING false
#define MAX_HARDWARE_ENDSTOP_X true
#define ENDSTOP_PULLUP_Y_MAX true
#define ENDSTOP_Y_MAX_INVERTING false
#define MAX_HARDWARE_ENDSTOP_Y true
#define ENDSTOP_PULLUP_Z_MAX true
#define ENDSTOP_Z_MAX_INVERTING false
#define MAX_HARDWARE_ENDSTOP_Z true
#define max_software_endstop_r true

#define min_software_endstop_x true
#define min_software_endstop_y true
#define min_software_endstop_z true
#define max_software_endstop_x false
#define max_software_endstop_y false
#define max_software_endstop_z false
#define ENDSTOP_X_BACK_MOVE 10
#define ENDSTOP_Y_BACK_MOVE 10
#define ENDSTOP_Z_BACK_MOVE 10
#define ENDSTOP_X_RETEST_REDUCTION_FACTOR 3
#define ENDSTOP_Y_RETEST_REDUCTION_FACTOR 3
#define ENDSTOP_Z_RETEST_REDUCTION_FACTOR 3
#define ENDSTOP_X_BACK_ON_HOME 5
#define ENDSTOP_Y_BACK_ON_HOME 5
#define ENDSTOP_Z_BACK_ON_HOME 5
#define ALWAYS_CHECK_ENDSTOPS 1

// ################# XYZ movements ###################

#define X_ENABLE_ON 0
#define Y_ENABLE_ON 0
#define Z_ENABLE_ON 0
#define DISABLE_X 0
#define DISABLE_Y 0
#define DISABLE_Z 0
#define DISABLE_E 0
#define INVERT_X_DIR 1
#define INVERT_Y_DIR 0
#define INVERT_Z_DIR 1
#define X_HOME_DIR -1
#define Y_HOME_DIR -1
#define Z_HOME_DIR 1
#define X_MAX_LENGTH 140
#define Y_MAX_LENGTH 140
#define Z_MAX_LENGTH 370
#define X_MIN_POS -140
#define Y_MIN_POS -140
#define Z_MIN_POS 0

// ##########################################################################################
// ##                           Movement settings                                          ##
// ##########################################################################################

#define FEATURE_BABYSTEPPING 0
#define BABYSTEP_MULTIPLICATOR 1

#define DELTA_SEGMENTS_PER_SECOND_PRINT 200 // Move accurate setting for print moves
#define DELTA_SEGMENTS_PER_SECOND_MOVE 120 // Less accurate setting for other moves

// Delta settings
#define DELTA_DIAGONAL_ROD 293 // mm
#define DELTA_ALPHA_A 210
#define DELTA_ALPHA_B 330
#define DELTA_ALPHA_C 90
#define DELTA_RADIUS_CORRECTION_A 0
#define DELTA_RADIUS_CORRECTION_B 0
#define DELTA_RADIUS_CORRECTION_C 0
#define DELTA_DIAGONAL_CORRECTION_A 0
#define DELTA_DIAGONAL_CORRECTION_B 0
#define DELTA_DIAGONAL_CORRECTION_C 0
#define END_EFFECTOR_HORIZONTAL_OFFSET 0
#define CARRIAGE_HORIZONTAL_OFFSET 0
#define DELTA_MAX_RADIUS 140
#define ROD_RADIUS 130
#define PRINTER_RADIUS 130
#define DELTA_HOME_ON_POWER 1
#define STEP_COUNTER
#define DELTA_X_ENDSTOP_OFFSET_STEPS 0
#define DELTA_Y_ENDSTOP_OFFSET_STEPS 0
#define DELTA_Z_ENDSTOP_OFFSET_STEPS 0
#define DELTA_FLOOR_SAFETY_MARGIN_MM 10
//#define SOFTWARE_LEVELING

#define DELTASEGMENTS_PER_PRINTLINE 18
#define STEPPER_INACTIVE_TIME 360L
#define MAX_INACTIVE_TIME 0L
#define MAX_FEEDRATE_X 250
#define MAX_FEEDRATE_Y 250
#define MAX_FEEDRATE_Z 250
#define HOMING_FEEDRATE_X 120
#define HOMING_FEEDRATE_Y 120
#define HOMING_FEEDRATE_Z 120
#define HOMING_ORDER HOME_ORDER_ZXY
#define ENABLE_BACKLASH_COMPENSATION 0
#define X_BACKLASH 0
#define Y_BACKLASH 0
#define Z_BACKLASH 0
#define RAMP_ACCELERATION 1
#define STEPPER_HIGH_DELAY 0
#define DIRECTION_DELAY 0
#define STEP_DOUBLER_FREQUENCY 12000
#define ALLOW_QUADSTEPPING 1
#define DOUBLE_STEP_DELAY 1 // time in microseconds
#define MAX_HALFSTEP_INTERVAL 1999
#define MAX_ACCELERATION_UNITS_PER_SQ_SECOND_X 1500
#define MAX_ACCELERATION_UNITS_PER_SQ_SECOND_Y 1500
#define MAX_ACCELERATION_UNITS_PER_SQ_SECOND_Z 1500
#define MAX_TRAVEL_ACCELERATION_UNITS_PER_SQ_SECOND_X 2200
#define MAX_TRAVEL_ACCELERATION_UNITS_PER_SQ_SECOND_Y 2200
#define MAX_TRAVEL_ACCELERATION_UNITS_PER_SQ_SECOND_Z 2200
#define MAX_JERK 30
#define MAX_ZJERK 0.3
#define PRINTLINE_CACHE_SIZE 21
#define MOVE_CACHE_LOW 13
#define LOW_TICKS_PER_MOVE 250000
#define FEATURE_TWO_XSTEPPER 0
#define X2_STEP_PIN   ORIG_E1_STEP_PIN
#define X2_DIR_PIN    ORIG_E1_DIR_PIN
#define X2_ENABLE_PIN ORIG_E1_ENABLE_PIN
#define FEATURE_TWO_YSTEPPER 0
#define Y2_STEP_PIN   ORIG_E1_STEP_PIN
#define Y2_DIR_PIN    ORIG_E1_DIR_PIN
#define Y2_ENABLE_PIN ORIG_E1_ENABLE_PIN
#define FEATURE_TWO_ZSTEPPER 0
#define Z2_STEP_PIN   ORIG_E1_STEP_PIN
#define Z2_DIR_PIN    ORIG_E1_DIR_PIN
#define Z2_ENABLE_PIN ORIG_E1_ENABLE_PIN
#define FEATURE_DITTO_PRINTING 0
#define USE_ADVANCE 1
#define ENABLE_QUADRATIC_ADVANCE 0


// ################# Misc. settings ##################

#define BAUDRATE 115200
#define ENABLE_POWER_ON_STARTUP 1
#define POWER_INVERTING 0
#define KILL_METHOD 1
#define GCODE_BUFFER_SIZE 2
#define ACK_WITH_LINENUMBER 1
#define WAITING_IDENTIFIER "wait"
#define ECHO_ON_EXECUTE 0
#define EEPROM_MODE 2
#define PS_ON_PIN -1

/* ======== Servos =======
Control the servos with
M340 P<servoId> S<pulseInUS>   / ServoID = 0..3  pulseInUs = 500..2500
Servos are controlled by a pulse width normally between 500 and 2500 with 1500ms in center position. 0 turns servo off.
WARNING: Servos can draw a considerable amount of current. Make sure your system can handle this or you may risk your hardware!
*/
#define FEATURE_SERVO 0
#define SERVO0_PIN 11
#define SERVO1_PIN -1
#define SERVO2_PIN -1
#define SERVO3_PIN -1
#define FEATURE_WATCHDOG 1

// #################### Z-Probing #####################

#define FEATURE_Z_PROBE 1
#define Z_PROBE_BED_DISTANCE 5
#define Z_PROBE_PIN ORIG_Z_MIN_PIN
#define Z_PROBE_PULLUP 1
#define Z_PROBE_ON_HIGH 0
#define Z_PROBE_X_OFFSET 0
#define Z_PROBE_Y_OFFSET 0
#define Z_PROBE_WAIT_BEFORE_TEST 0
#define Z_PROBE_SPEED 2
#define Z_PROBE_XY_SPEED 150
#define Z_PROBE_SWITCHING_DISTANCE 1
#define Z_PROBE_REPETITIONS 2
#define Z_PROBE_HEIGHT 0
#define Z_PROBE_START_SCRIPT ""
#define Z_PROBE_FINISHED_SCRIPT ""

/* DISTORTION_CORRECTION compensates the distortion caused by mechanical imprecisions of nonlinear (i.e. DELTA) printers
 * assumes that the floor is plain (i.e. glass plate)
 *     and that it is perpendicular to the towers
 *     and that the (0,0) is in center
 * requires z-probe
 * G29 measures the Z offset in matrix NxN points (due to nature of the delta printer, the corners are extrapolated instead of measured)
 * and compensate the distortion
 * more points means better compensation, but consumes more memory and takes more time
 * DISTORTION_CORRECTION_R is the distance of last row or collumn from center
 */

#define DISTORTION_CORRECTION         0
#define DISTORTION_CORRECTION_POINTS  5
#define DISTORTION_CORRECTION_R       80
/** Uses eeprom instead of ram. Allows bigger matrix (up to 22x22) without any ram cost.
  Especially on arm based systems with cached eeprom it is good, on AVR it has a small
  performance penalty.
*/
#define DISTORTION_PERMANENT          1
/** Correction computation is not a cheap operation and changes are only small. So it
is not necessary to update it for every subline computed. For example lets take DELTA_SEGMENTS_PER_SECOND_PRINT = 150
and fastest print speed 100 mm/s. So we have a maximum segment length of 100/150 = 0.66 mm.
Now lats say our point field is 200 x 200 mm with 9 x 9 points. So between 2 points we have
200 / (9-1) = 25 mm. So we need at least 25 / 0.66 = 37 lines to move to the next measuring
point. So updting correction every 15 calls gives us at least 2 updates between the
measured points.
NOTE: Explicit z changes will always trigger an update!
*/
#define DISTORTION_UPDATE_FREQUENCY   15
/** z distortion degrades to 0 from this height on. You should start after the first layer to get
best bonding with surface. */
#define DISTORTION_START_DEGRADE 0.5
/** z distortion correction gets down to 0 at this height. */
#define DISTORTION_END_HEIGHT 1.5
/** If your corners measurement points are not measureable with given radius, you can
set this to 1. It then omits the outer measurement points allowing a larger correction area.*/
#define DISTORTION_EXTRAPOLATE_CORNERS 0

/* If your printer is not exactly square but is more like a parallelogramm, you can
use this to compensate the effect of printing squares like parallelogramms. Set the
parameter to then tangens of the deviation from 90° when you print a square object.
E.g. if you angle is 91° enter tan(1) = 0.017. If error doubles you have the wrong sign.
Always hard to say since the other angle is 89° in this case!
*/
#define FEATURE_AUTOLEVEL 1
#define Z_PROBE_X1 -86.6
#define Z_PROBE_Y1 -50
#define Z_PROBE_X2 86.6
#define Z_PROBE_Y2 -50
#define Z_PROBE_X3 0
#define Z_PROBE_Y3 100

#define FEATURE_AXISCOMP 0
#define AXISCOMP_TANXY 0
#define AXISCOMP_TANYZ 0
#define AXISCOMP_TANXZ 0

#ifndef SDSUPPORT  // Some boards have sd support on board. These define the values already in pins.h
#define SDSUPPORT 0
#define SDCARDDETECT 81
#define SDCARDDETECTINVERTED 0
#endif
#define SD_EXTENDED_DIR 1 /** Show extended directory including file length. Don't use this with Pronterface! */
#define ARC_SUPPORT 0
#define FEATURE_MEMORY_POSITION 1
#define FEATURE_CHECKSUM_FORCED 0
#define FEATURE_FAN_CONTROL 1
#define FEATURE_CONTROLLER 13
#define UI_LANGUAGE 0
#define UI_PRINTER_NAME "Albertus Magnus"
#define UI_PRINTER_COMPANY "SeeMeCNC"
#define UI_PAGES_DURATION 4000
#define UI_ANIMATION 0
#define UI_SPEEDDEPENDENT_POSITIONING 1
#define UI_DISABLE_AUTO_PAGESWITCH 1
#define UI_AUTORETURN_TO_MENU_AFTER 30000
#define FEATURE_UI_KEYS 0
#define UI_ENCODER_SPEED 1
#define UI_KEY_BOUNCETIME 10
#define UI_KEY_FIRST_REPEAT 500
#define UI_KEY_REDUCE_REPEAT 50
#define UI_KEY_MIN_REPEAT 50
#define FEATURE_BEEPER 1
#define CASE_LIGHTS_PIN -1
#define CASE_LIGHT_DEFAULT_ON 1
#define UI_START_SCREEN_DELAY 1000
/**
Beeper sound definitions for short beeps during key actions
and longer beeps for important actions.
Parameter is delay in microseconds and the secons is the number of repetitions.
Values must be in range 1..255
*/
#define BEEPER_SHORT_SEQUENCE 1,1
#define BEEPER_LONG_SEQUENCE 200,16
#define UI_SET_PRESET_HEATED_BED_TEMP_PLA 60
#define UI_SET_PRESET_EXTRUDER_TEMP_PLA   180
#define UI_SET_PRESET_HEATED_BED_TEMP_ABS 90
#define UI_SET_PRESET_EXTRUDER_TEMP_ABS   210
#define UI_SET_MIN_HEATED_BED_TEMP  30
#define UI_SET_MAX_HEATED_BED_TEMP 120
#define UI_SET_MIN_EXTRUDER_TEMP   150
#define UI_SET_MAX_EXTRUDER_TEMP   300
#define UI_SET_EXTRUDER_FEEDRATE 2
#define UI_SET_EXTRUDER_RETRACT_DISTANCE 4.5

#endif

/* Below you will find the configuration string, that created this Configuration.h

========== Start configuration string ==========
{
    "editMode": 2,
    "processor": 0,
    "baudrate": 115200,
    "xStepsPerMM": 80,
    "yStepsPerMM": 80,
    "zStepsPerMM": 80,
    "xInvert": "1",
    "xInvertEnable": "0",
    "eepromMode": 2,
    "yInvert": 0,
    "yInvertEnable": "0",
    "zInvert": "1",
    "zInvertEnable": "0",
    "extruder": [
        {
            "id": 0,
            "heatManager": 3,
            "pidDriveMin": 40,
            "pidDriveMax": 180,
            "pidMax": 255,
            "sensorType": 8,
            "sensorPin": "TEMP_0_PIN",
            "heaterPin": "HEATER_0_PIN",
            "maxFeedrate": 160,
            "startFeedrate": 20,
            "invert": "1",
            "invertEnable": "0",
            "acceleration": 1000,
            "watchPeriod": 1,
            "pidP": 3,
            "pidI": 0,
            "pidD": 0,
            "advanceK": 0,
            "advanceL": 40.0,
            "waitRetractTemp": 150,
            "waitRetractUnits": 0,
            "waitRetract": 0,
            "stepsPerMM": 93,
            "coolerPin": "HEATER_2_PIN",
            "coolerSpeed": 255,
            "selectCommands": "",
            "deselectCommands": "",
            "xOffset": 0,
            "yOffset": 0,
            "xOffsetSteps": 0,
            "yOffsetSteps": 0,
            "stepper": {
                "name": "Extruder 0",
                "step": "ORIG_E0_STEP_PIN",
                "dir": "ORIG_E0_DIR_PIN",
                "enable": "E0_ENABLE_PIN"
            },
            "advanceBacklashSteps": 0,
            "decoupleTestPeriod": 15
        },
        {
            "id": 1,
            "heatManager": 3,
            "pidDriveMin": 40,
            "pidDriveMax": 180,
            "pidMax": 255,
            "sensorType": 8,
            "sensorPin": "TEMP_0_PIN",
            "heaterPin": "-1",
            "maxFeedrate": 160,
            "startFeedrate": 20,
            "invert": "1",
            "invertEnable": "0",
            "acceleration": 1000,
            "watchPeriod": 1,
            "pidP": 3,
            "pidI": 0,
            "pidD": 0,
            "advanceK": 0,
            "advanceL": 40.0,
            "waitRetractTemp": 150,
            "waitRetractUnits": 0,
            "waitRetract": 0,
            "stepsPerMM": 93,
            "coolerPin": "-1",
            "coolerSpeed": 255,
            "selectCommands": "",
            "deselectCommands": "",
            "xOffset": 0,
            "yOffset": 0,
            "xOffsetSteps": 0,
            "yOffsetSteps": 0,
            "stepper": {
                "name": "Extruder 1",
                "step": "ORIG_E1_STEP_PIN",
                "dir": "ORIG_E1_DIR_PIN",
                "enable": "E1_ENABLE_PIN"
            },
            "advanceBacklashSteps": 0,
            "decoupleTestPeriod": 15
        }
    ],
    "uiLanguage": 0,
    "uiController": 0,
    "xMinEndstop": 0,
    "yMinEndstop": 0,
    "zMinEndstop": 0,
    "xMaxEndstop": 2,
    "yMaxEndstop": 2,
    "zMaxEndstop": 2,
    "motherboard": 301,
    "driveSystem": 3,
    "xMaxSpeed": 250,
    "xHomingSpeed": 120,
    "xTravelAcceleration": 2200,
    "xPrintAcceleration": 1500,
    "yMaxSpeed": 250,
    "yHomingSpeed": 120,
    "yTravelAcceleration": 2200,
    "yPrintAcceleration": 1500,
    "zMaxSpeed": 250,
    "zHomingSpeed": 120,
    "zTravelAcceleration": 2200,
    "zPrintAcceleration": 1500,
    "xMotor": {
        "name": "X motor",
        "step": "ORIG_X_STEP_PIN",
        "dir": "ORIG_X_DIR_PIN",
        "enable": "ORIG_X_ENABLE_PIN"
    },
    "yMotor": {
        "name": "Y motor",
        "step": "ORIG_Y_STEP_PIN",
        "dir": "ORIG_Y_DIR_PIN",
        "enable": "ORIG_Y_ENABLE_PIN"
    },
    "zMotor": {
        "name": "Z motor",
        "step": "ORIG_Z_STEP_PIN",
        "dir": "ORIG_Z_DIR_PIN",
        "enable": "ORIG_Z_ENABLE_PIN"
    },
    "enableBacklash": "0",
    "backlashX": 0,
    "backlashY": 0,
    "backlashZ": 0,
    "stepperInactiveTime": 360,
    "maxInactiveTime": 0,
    "xMinPos": 0,
    "yMinPos": 0,
    "zMinPos": 0,
    "xLength": 200,
    "yLength": 200,
    "zLength": 380,
    "alwaysCheckEndstops": "1",
    "disableX": "0",
    "disableY": "0",
    "disableZ": "0",
    "disableE": "0",
    "xHomeDir": "-1",
    "yHomeDir": "-1",
    "zHomeDir": 1,
    "xEndstopBack": 5,
    "yEndstopBack": 5,
    "zEndstopBack": 5,
    "deltaSegmentsPerSecondPrint": 200,
    "deltaSegmentsPerSecondTravel": 120,
    "deltaDiagonalRod": 293,
    "deltaHorizontalRadius": 130,
    "deltaAlphaA": 210,
    "deltaAlphaB": 330,
    "deltaAlphaC": 90,
    "deltaDiagonalCorrA": 0,
    "deltaDiagonalCorrB": 0,
    "deltaDiagonalCorrC": 0,
    "deltaMaxRadius": 140,
    "deltaFloorSafetyMarginMM": 10,
    "deltaRadiusCorrA": 0,
    "deltaRadiusCorrB": 0,
    "deltaRadiusCorrC": 0,
    "deltaXOffsetSteps": 0,
    "deltaYOffsetSteps": 0,
    "deltaZOffsetSteps": 0,
    "deltaSegmentsPerLine": 18,
    "stepperHighDelay": 0,
    "directionDelay": 0,
    "stepDoublerFrequency": 12000,
    "allowQuadstepping": "1",
    "doubleStepDelay": 1,
    "maxHalfstepInterval": 1999,
    "maxJerk": 30,
    "maxZJerk": 0.3,
    "moveCacheSize": 21,
    "moveCacheLow": 13,
    "lowTicksPerMove": 250000,
    "enablePowerOnStartup": "1",
    "echoOnExecute": "0",
    "sendWaits": "1",
    "ackWithLineNumber": "1",
    "killMethod": 1,
    "useAdvance": "1",
    "useQuadraticAdvance": "0",
    "powerInverting": 0,
    "mirrorX": 0,
    "mirrorXMotor": {
        "name": "Extruder 1",
        "step": "ORIG_E1_STEP_PIN",
        "dir": "ORIG_E1_DIR_PIN",
        "enable": "ORIG_E1_ENABLE_PIN"
    },
    "mirrorY": 0,
    "mirrorYMotor": {
        "name": "Extruder 1",
        "step": "ORIG_E1_STEP_PIN",
        "dir": "ORIG_E1_DIR_PIN",
        "enable": "ORIG_E1_ENABLE_PIN"
    },
    "mirrorZ": 0,
    "mirrorZMotor": {
        "name": "Extruder 1",
        "step": "ORIG_E1_STEP_PIN",
        "dir": "ORIG_E1_DIR_PIN",
        "enable": "ORIG_E1_ENABLE_PIN"
    },
    "dittoPrinting": "0",
    "featureServos": "0",
    "servo0Pin": 11,
    "servo1Pin": -1,
    "servo2Pin": -1,
    "servo3Pin": -1,
    "featureWatchdog": "1",
    "hasHeatedBed": "1",
    "enableZProbing": "1",
    "extrudeMaxLength": 160,
    "homeOrder": "HOME_ORDER_ZXY",
    "featureController": 13,
    "uiPrinterName": "Albertus Magnus",
    "uiPrinterCompany": "SeeMeCNC",
    "uiPagesDuration": 4000,
    "uiAnimation": "0",
    "uiDisablePageswitch": "1",
    "uiAutoReturnAfter": 30000,
    "featureKeys": "0",
    "uiEncoderSpeed": 1,
    "uiKeyBouncetime": 10,
    "uiKeyFirstRepeat": 500,
    "uiKeyReduceRepeat": 50,
    "uiKeyMinRepeat": 50,
    "featureBeeper": "1",
    "uiPresetBedTempPLA": 60,
    "uiPresetBedABS": 90,
    "uiPresetExtruderPLA": 180,
    "uiPresetExtruderABS": 210,
    "uiMinHeatedBed": 30,
    "uiMaxHeatedBed": 120,
    "uiMinEtxruderTemp": 150,
    "uiMaxExtruderTemp": 300,
    "uiExtruderFeedrate": 2,
    "uiExtruderRetractDistance": 4.5,
    "uiSpeeddependentPositioning": "1",
    "maxBedTemperature": 120,
    "bedSensorType": 8,
    "bedSensorPin": "TEMP_1_PIN",
    "bedHeaterPin": "HEATER_1_PIN",
    "bedHeatManager": 3,
    "bedUpdateInterval": 5000,
    "bedPidDriveMin": 10,
    "bedPidDriveMax": 192,
    "bedPidP": 20,
    "bedPidI": 0,
    "bedPidD": 0,
    "bedPidMax": 255,
    "bedDecoupleTestPeriod": 120,
    "caseLightPin": -1,
    "caseLightDefaultOn": "1",
    "bedSkipIfWithin": 5,
    "gen1T0": 25,
    "gen1R0": 100000,
    "gen1Beta": 4036,
    "gen1MinTemp": -20,
    "gen1MaxTemp": 300,
    "gen1R1": 0,
    "gen1R2": 4700,
    "gen2T0": 25,
    "gen2R0": 100000,
    "gen2Beta": 4036,
    "gen2MinTemp": -20,
    "gen2MaxTemp": 300,
    "gen2R1": 0,
    "gen2R2": 4700,
    "gen3T0": 25,
    "gen3R0": 100000,
    "gen3Beta": 4036,
    "gen3MinTemp": -20,
    "gen3MaxTemp": 300,
    "gen3R1": 0,
    "gen3R2": 4700,
    "userTable0": {
        "r1": 0,
        "r2": 4700,
        "temps": [

        ]
    },
    "userTable1": {
        "r1": 0,
        "r2": 4700,
        "temps": [

        ]
    },
    "userTable2": {
        "r1": 0,
        "r2": 4700,
        "temps": [

        ]
    },
    "tempHysteresis": 0,
    "pidControlRange": 20,
    "skipM109Within": 2,
    "extruderFanCoolTemp": -50,
    "minTemp": 150,
    "maxTemp": 280,
    "minDefectTemp": -10,
    "maxDefectTemp": 290,
    "arcSupport": "0",
    "featureMemoryPositionWatchdog": "1",
    "forceChecksum": "0",
    "sdExtendedDir": "1",
    "featureFanControl": "1",
    "fanPin": "ORIG_FAN_PIN",
    "scalePidToMax": 0,
    "zProbePin": "ORIG_Z_MIN_PIN",
    "zProbeBedDistance": 5,
    "zProbePullup": "1",
    "zProbeOnHigh": "0",
    "zProbeXOffset": 0,
    "zProbeYOffset": 0,
    "zProbeWaitBeforeTest": "0",
    "zProbeSpeed": 2,
    "zProbeXYSpeed": 150,
    "zProbeHeight": 0,
    "zProbeStartScript": "",
    "zProbeFinishedScript": "",
    "featureAutolevel": "1",
    "zProbeX1": -86.6,
    "zProbeY1": -50,
    "zProbeX2": 86.6,
    "zProbeY2": -50,
    "zProbeX3": 0,
    "zProbeY3": 100,
    "zProbeSwitchingDistance": 1,
    "zProbeRepetitions": 2,
    "sdSupport": "0",
    "sdCardDetectPin": 81,
    "sdCardDetectInverted": "0",
    "uiStartScreenDelay": 1000,
    "xEndstopBackMove": 10,
    "yEndstopBackMove": 10,
    "zEndstopBackMove": 10,
    "xEndstopRetestFactor": 3,
    "yEndstopRetestFactor": 3,
    "zEndstopRetestFactor": 3,
    "xMinPin": "ORIG_X_MIN_PIN",
    "yMinPin": "ORIG_Y_MIN_PIN",
    "zMinPin": "ORIG_Z_MIN_PIN",
    "xMaxPin": "ORIG_X_MAX_PIN",
    "yMaxPin": "ORIG_Y_MAX_PIN",
    "zMaxPin": "ORIG_Z_MAX_PIN",
    "deltaHomeOnPower": "1",
    "fanBoardPin": -1,
    "heaterPWMSpeed": 0,
    "featureBabystepping": "0",
    "babystepMultiplicator": 1,
    "pdmForHeater": "0",
    "pdmForCooler": "0",
    "psOn": -1,
    "mixingExtruder": "1",
    "decouplingTestMaxHoldVariance": 15,
    "decouplingTestMinTempRise": 1,
    "featureAxisComp": "0",
    "axisCompTanXY": 0,
    "axisCompTanXZ": 0,
    "axisCompTanYZ": 0,
    "hasMAX6675": false,
    "hasMAX31855": false,
    "hasGeneric1": false,
    "hasGeneric2": false,
    "hasGeneric3": false,
    "hasUser0": false,
    "hasUser1": false,
    "hasUser2": false,
    "numExtruder": 2,
    "version": 92
}
========== End configuration string ==========

*/

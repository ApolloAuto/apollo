/**
 * @file BDCoreTypes.h
 * @author MapAuto Linux Team
 *
 * Copyright (c) 2017  Baidu MapAuto Company,,
 * All Rights Reserved.
 *
 * Use and copying of this software and preparation of derivative works
 * based upon this software are permitted. Any copy of this software or
 * of any derivative work must include the above copyright notice, this
 * paragraph and the one after it.  Any distribution of this software or
 * derivative works must comply with all aplicable laws.
 *
 * This software is made available AS IS, and COPYRIGHT OWNERS DISCLAIMS
 * ALL WARRANTIES, EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE, AND NOTWITHSTANDING ANY OTHER PROVISION CONTAINED HEREIN, ANY
 * LIABILITY FOR DAMAGES RESULTING FROM THE SOFTWARE OR ITS USE IS
 * EXPRESSLY DISCLAIMED, WHETHER ARISING IN CONTRACT, TORT (INCLUDING
 * NEGLIGENCE) OR STRICT LIABILITY, EVEN IF COPYRIGHT OWNERS ARE ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGES.
 */

#ifndef BAIDU_MAPAUTO_BDKEYTYPES_H
#define BAIDU_MAPAUTO_BDKEYTYPES_H

namespace baidu{
namespace mapauto {
/**
 * @brief The Key Type enum
 */
enum class BDKeyType
{    
    UP,                 /**< Up Event (KEYBOARD_NUM_8) */
    DOWN,               /**< Down Event (KEYBOARD_NUM_2) */
    LEFT_UP,            /**< Left-Up Event (KEYBOARD_NUM_7) */
    LEFT,               /**< Left Event (KEYBOARD_NUM_4) */
    LEFT_DOWN,          /**< Left-Down Event (KEYBOARD_NUM_1) */
    RIGHT_UP,           /**< Right-Up Event (KEYBOARD_NUM_9) */
    RIGHT,              /**< Right Event (KEYBOARD_NUM_6) */
    RIGHT_DOWN,         /**< Right-Down Event (KEYBOARD_NUM_3) */
    DIAL,               /**< Dial-Push Event (KEYBOARD_NUM_5) */
    BACK,               /**< Back Key Event (KEYBOARD_BACK) */
    MENU,               /**< Menu Key Event (KEYBOARD_LEFT_WINDOW) */
    MAX,                /**< MAX Enum */
};
	
/**
 * @brief The System Key Type enum
 */
enum class BDSystemKeyType
{
    SEEK_UP,                    	/**< Seek Up Key Event (KEYBOARD_PAGE_UP) */
    SEEK_DOWN,                      /**< Seel Down Key Event (KEYBOARD_PAGE_DOWN) */
    VOLUME_UP,                      /**< RRC: Vol Up Key Event */
    VOLUME_DOWN,                    /**< RRC: Vol Down Key Event */
    VOLUME_MUTE,                    /**< Volume Mute Key Event (KEYBOARD_END) */
    HOME,                           /**< Home Key Event (KEYBOARD_HOME) */
    MAP,                            /**< Map Key Event (KEYBOARD_M) */
    RADIO,                          /**< Radio Key Event (KEYBOARD_R) */
    MEDIA,                          /**< Media Key Event (KEYBOARD_E) */
    POWER,                          /**< Power Key Event (KEYBOARD_P) */
    NAV,                            /**< NAV Key Event (KEYBOARD_N) */
    AVM,                            /**< AVM Key Event (KEYBOARD_V) */
    PHONE,                          /**< Phone Key Event (KEYBOARD_INSERT) */
    BLUELINK,                       /**< Bluelink Key Event (KEYBOARD_B) */
    VOL_PUSH,                       /**< not use */
    TUNE_PUSH,                      /**< not use */
    EJECT,                          /**< not use */
    DISP_ONOFF,                     /**< not use */
    DMB,                            /**< not use */
    PHONE_HANGUP,                   /**< Phone Hangup Key Event (KEYBOARD_DELETE) */
    VR,                             /**< VR Key Event (KEYBOARD_T) */
    MODE,                           /**< not use */
    RADIO_FM,                       /**< not use */
    RADIO_AM,                       /**< not use */
    CH_UP,                          /**< not use */
    CH_DOWN,                        /**< not use */
    CUSTOM_1,                       /**< not use */
    CUSTOM_2,                       /**< not use */
    CUSTOM_3,                       /**< not use */
    CUSTOM_4,                       /**< not use */
    /* more to define */
	MAX,
};

/**
 * @brief The KeyStatus enum
 */
enum class BDKeyStatus
{
    PRESSED,     /**< Key Pressed */
    RELEASED,    /**< Key released */
    LONG,        /**< Key pressed and hold for more than 300ms */
    MAX,         /**< Max value of Enum */
};

/**
 * @brief The Knob Dial Type enum
 */
enum class BDKnobType
{
	JOGDIAL,    /**< Dial-Rotate Event (KEYBOARD_PLUS) : Value range 1 ~ 5, Normal Press: 1, Long Press: 2 //KEYBOARD_MINUS : Value range -1 ~ -5, Normal Press: 1, Long Press: 2 */
	MAX,        /**< Max value of Enum */
};

/**
 * @brief The System Knob Dial Type enum
 */
enum class BDSystemKnobType
{
	VOLUME,    /**< : Value range +-1 ~ +-5, */
	TUNE,      /**< : Value range +-1 ~ +-5, */
	MAX,       /**< Max value of Enum */
};

/**
 * @brief The Knob Rotation Type enum
 */
enum class BDKnobRotationType
{
	CLOCKWISE,         /**< Clockwise rotation */
	ANTI_CLOCKWISE,    /**< Anti-clockwise rotation */
	MAX,               /**< Max value of Enum */
};

/**
 * @brief [Not Defined Yet] The Gesture Type enum
 * @todo This enum type will be used after baidu::mapauto 0.8
 */
enum class BDGestureType
{
    SWIPE_RIGHT_TO_LEFT,            /**< Swipe right to left */
    SWIPE_LEFT_TO_RIGHT,            /**< Swipe left to right */
    SWIPE_UP_TO_DOWN,               /**< Swipe up to down */
    SWIPE_DOWN_TO_UP,               /**< Swipe down to up */
    TWO_FINGER_SWIPE_RIGHT_TO_LEFT, /**< Two finger swipe right to left */
    TWO_FINGER_SWIPE_LEFT_TO_RIGHT, /**< Two finger swipe left to right */
    TWO_FINGER_SWIPE_UP_TO_DOWN,    /**< Two finger swipe up to down */
    TWO_FINGER_SWIPE_DOWN_TO_UP,    /**< Two finger swipe down to up */
    PINCH_IN,                       /**< Pinch In */
    PINCH_OUT,                      /**< Pinch Out */
    //DRAG,                         /**< Drag */
    TWO_FINGER_DRAG,                /**< Two Finger Drag */
    PANNING,                        /**< Panning */
    //TAP,                          /**< Tap */
    DOUBLE_TAP,                     /**< Double Tap */
    //TWO_FINGER_TAP,               /**< Two Finger Tap */
    //HOLD,                         /**< Hold */
    MAX,                            /**< Max value of Enum */
};

} // mapauto
} // baidu
#endif // BAIDU_MAPAUTO_BDKEYTYPES_H

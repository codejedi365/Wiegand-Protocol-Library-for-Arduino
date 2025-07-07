/*********************************
 * FILE: Wiegand.h
 *********************************/

#ifndef _WIEGAND_H
#define _WIEGAND_H

#if defined(ARDUINO) && ARDUINO >= 100
    #include "Arduino.h"
#else
    #include "WProgram.h"
#endif

/**
 * Defines the timeout for Wiegand data. If no new bits are received within this
 * time, it's assumed the card read is complete.
 * 25ms is a common value, but increasing it slightly for safety is fine.
 */
#define WIEGAND_RECEIVE_TIMEOUT_MS      25

/**
 * Defines the maximum amount of time that the Wiegand library will maintain the last received code
 */
#define WIEGAND_CODE_LIFETIME_MS        1000  // 1 second

/**
 * Default pins for Wiegand DATA0
 * Must be hardware interrupt capable pins, most Arduino boards have these on pins 2 and 3
 */
#define WIEGAND_DEFAULT_PIN_D0          2

/**
 * Default pins for Wiegand DATA1
 * Must be hardware interrupt capable pins, most Arduino boards have these on pins 2 and 3
 */
#define WIEGAND_DEFAULT_PIN_D1          3

#define ASCII_ESCAPE_KEY                0x1b
#define ASCII_ENTER_KEY                 0x0d
#define WIEGAND_KEYPAD_ASTERISK_KEY     0x0b
#define WIEGAND_KEYPAD_OCTOTHORPE_KEY   0x0a

/**
 * Wiegand Protocol Library for Arduino
 */
class Wiegand {

    public:

        /**
         * Constructor
         */
        Wiegand();

        /**
         * Initializes the Wiegand Reader, sets input pins, & adds interrupt listeners.
         * Uses WIEGAND_DEFAULT_PIN_D0 and WIEGAND_DEFAULT_PIN_D1 as input pins.
         *
         * If you want to use different pins, use the begin(pinD0, pinD1) overload instead.
         */
        void begin();

        /**
         * Initializes the Wiegand Reader, sets input pins, & adds interrupts to the falling edge.
         *
         * @param pinD0 Pin number for Wiegand DATA0
         * @param pinD1 Pin number for Wiegand DATA1
         *
         * This function should only be called once, generally during setup().
         */
        void begin(int pinD0, int pinD1);

        /**
         * Checks if a Wiegand code is available.
         *
         * A non-blocking method that will return true when a full Wiegand code has been
         * received and validated.
         *
         * @return true if a Wiegand code is available, false otherwise.
         */
        bool available();

        /**
         * Resets the Wiegand state machine and clears any stored data.
         */
        static void resetBuffersState();

        /**
         * Clears the last received Wiegand code.
         */
        static void clearCodeState();

        /**
         * Gets the value of the last received Wiegand code.
         *
         * @return The last received Wiegand code as an unsigned long.
         */
        unsigned long getCode();

        /**
         * Gets the type of the last received Wiegand code.
         *
         * @return The type of the last received Wiegand code as an integer.
         */
        int getWiegandType();

        /**
         * Gets the card ID from the last received Wiegand code.
         *
         * @return The card ID as an integer.
         */
        int getBitCount() { return _bitCount; }

    private:
        static void readDATA0();
        static void readDATA1();
        static unsigned long parseCardData (
            volatile unsigned long codehigh,
            volatile unsigned long codelow,
            byte bitlength
        );
        static char parseKeyPress(char originalKeyPress);
        static bool processReceivedData();
        static bool processCardData();
        static bool processKeyPress();
        static bool validateKeyPress8Bit(volatile unsigned long data);
        static bool validateDataParity(
            volatile unsigned long *data,
            byte leadingParityBit,
            byte leadingParityBitLength,
            byte trailingParityBit,
            byte trailingParityBitLength
        );

        static volatile unsigned long     _bitBufferHigh;
        static volatile unsigned long     _bitBufferLow;
        static volatile unsigned long     _lastBitReceivedTimeMS;
        static volatile int               _bitCount;
        static int                        _wiegandType;
        static unsigned long              _code;
        static volatile unsigned long     _lastValidDataProcessedTimeMS;
};

/**
 * Wiegand Data Packet Sizes
 */
enum WiegandDataPacketSizes {
    // Wiegand Keypress 4-bit type
    KEYPRESS_4BIT = 4,
    // Wiegand Keypress 8-bit type (4-bit key with integrity check)
    KEYPRESS_8BIT = 8,
    // Wiegand 26 type (without parity bits)
    DATA_24BIT = 24,
    // Wiegand 26 type (with parity bits)
    DATA_26BIT = 26,
    // Wiegand 34 type (without parity bits)
    DATA_32BIT = 32,
    // Wiegand 34 type (with parity bits)
    DATA_34BIT = 34,
};

#endif

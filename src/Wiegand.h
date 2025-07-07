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
         * Simple Constructor
         *
         * This constructor initializes the Wiegand Reader with default pins
         *
         * @param d0_isr_callback global or static callback function for Wiegand DATA0 ISR
         * @param d1_isr_callback global or static callback function for Wiegand DATA1 ISR
         */
        Wiegand(void (*d0_isr_callback)(), void (*d1_isr_callback)());

        /**
         * Constructor
         *
         * @param pinD0 Pin number for Wiegand DATA0
         * @param pinD1 Pin number for Wiegand DATA1
         * @param d0_isr_callback global or static callback function for Wiegand DATA0 ISR
         * @param d1_isr_callback global or static callback function for Wiegand DATA1 ISR
         */
        Wiegand(
            uint8_t pinD0,
            uint8_t pinD1,
            void (*d0_isr_callback)(),
            void (*d1_isr_callback)()
        );

        /**
         * Initializes the Wiegand Reader, sets input pins, & adds interrupt listeners.
         * Uses WIEGAND_DEFAULT_PIN_D0 and WIEGAND_DEFAULT_PIN_D1 as input pins.
         *
         * If you want to use different pins, use the begin(pinD0, pinD1) overload instead.
         */
        bool begin();

        /**
         * Initializes the Wiegand Reader, sets input pins, & adds interrupts to the falling edge.
         *
         * @param pinD0 Pin number for Wiegand DATA0
         * @param pinD1 Pin number for Wiegand DATA1
         *
         * This function should only be called once, generally during setup().
         */
        bool begin(uint8_t pinD0, uint8_t pinD1);

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
         * Clears the last received Wiegand code.
         */
        void clearCodeState();

        /**
         * Wiegand DATA0 Interrupt Service Routine (ISR)
         *
         * @param reader Pointer to the Wiegand object
         */
        static void readDATA0(Wiegand* reader);

        /**
         * Wiegand DATA1 Interrupt Service Routine (ISR)
         *
         * @param reader Pointer to the Wiegand object
         */
        static void readDATA1(Wiegand* reader);

        /**
         * Gets the value of the last received Wiegand code.
         *
         * @return The last received Wiegand code as an unsigned long.
         */
        unsigned long getCode() const { return _code; }

        /**
         * Gets the type of the last received Wiegand code.
         *
         * @return The type of the last received Wiegand code as an integer.
         */
        uint8_t getWiegandType() const { return _wiegandType; }

        /**
         * Gets the configured pin for Wiegand DATA0.
         *
         * @return The pin number for Wiegand DATA0.
         */
        uint8_t getPinDATA0() const { return _pinD0; }

        /**
         * Gets the configured pin for Wiegand DATA1.
         *
         * @return The pin number for Wiegand DATA1.
         */
        uint8_t getPinDATA1() const { return _pinD1; }

    private:
        bool            _started;                       // Flag to indicate if the Wiegand reader has begun
        uint8_t         _pinD0;                         // Pin for Wiegand DATA0
        uint8_t         _pinD1;                         // Pin for Wiegand DATA1
        unsigned long   _code;                          // Last received valid Wiegand code
        uint8_t         _wiegandType;                   // Type of the last received Wiegand code
        unsigned long   _lastValidDataProcessedTimeMS;  // Timestamp of the last valid data processed

        /**
         * Callback function for Wiegand DATA0 ISR
         * This function is called when a Wiegand DATA0 bit is received.
         * It will be attached to the pin interrupt for DATA0 in the begin() method.
         */
        void (*_d0_isr_callback)();

        /**
         * Callback function for Wiegand DATA1 ISR
         * This function is called when a Wiegand DATA1 bit is received.
         * It will be attached to the pin interrupt for DATA1 in the begin() method.
         */
        void (*_d1_isr_callback)();

        // All variables that are modified by the ISR must be declared as volatile
        // to prevent the compiler from optimizing them out or caching their values.

        volatile unsigned long _bitBufferHigh;
        volatile unsigned long _bitBufferLow;
        volatile uint8_t _bitCount;
        volatile unsigned long _lastBitReceivedTimeMS;

        /**
         * Resets the Wiegand state machine and clears any stored data.
         */
        void resetBuffersState();

        static unsigned long parseCardData (
            unsigned long codehigh,
            unsigned long codelow,
            uint8_t bitlength
        );
        static char parseKeyPress(char originalKeyPress);
        bool processReceivedData();
        bool processCardData();
        bool processKeyPress();
        static bool validateKeyPress8Bit(unsigned long data);
        static bool validateDataParity(
            unsigned long data,
            byte leadingParityBit,
            uint8_t leadingParityBitLength,
            byte trailingParityBit,
            uint8_t trailingParityBitLength
        );
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

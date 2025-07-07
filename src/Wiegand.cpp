/*********************************
 * FILE: Wiegand.cpp
 *********************************/

#include "Wiegand.h"

#if defined(ESP8266)
    #define INTERRUPT_ATTR ICACHE_RAM_ATTR
#elif defined(ESP32)
    #define INTERRUPT_ATTR IRAM_ATTR
#else
    #define INTERRUPT_ATTR
#endif

#define     IS_EVEN(x) ((x % 2) == 0)
#define     IS_ODD(x)  ((x % 2) == 1)

static const WiegandDataPacketSizes dataSizes[] = {
    KEYPRESS_4BIT, KEYPRESS_8BIT, DATA_24BIT, DATA_26BIT, DATA_32BIT, DATA_34BIT
};

uint16_t Wiegand::_nextId = 1; // Initialize static variable for unique ID assignment

Wiegand::Wiegand(void (*d0_isr_callback)(), void (*d1_isr_callback)()) {
    Wiegand(WIEGAND_DEFAULT_PIN_D0, WIEGAND_DEFAULT_PIN_D1, d0_isr_callback, d1_isr_callback);
}

Wiegand::Wiegand(String name, void (*d0_isr_callback)(), void (*d1_isr_callback)()) {
    Wiegand(name, WIEGAND_DEFAULT_PIN_D0, WIEGAND_DEFAULT_PIN_D1, d0_isr_callback, d1_isr_callback);
}

Wiegand::Wiegand(
    uint8_t pinD0, uint8_t pinD1, void (*d0_isr_callback)(), void (*d1_isr_callback)()
) {
    Wiegand("Wiegand Reader " + String(_id), pinD0, pinD1, d0_isr_callback, d1_isr_callback);
}

Wiegand::Wiegand(
    String name, uint8_t pinD0, uint8_t pinD1, void (*d0_isr_callback)(), void (*d1_isr_callback)()
) {
    _id = Wiegand::_nextId++;
    _name = name;
    _pinD0 = pinD0;
    _pinD1 = pinD1;
    _d0_isr_callback = d0_isr_callback;
    _d1_isr_callback = d1_isr_callback;
    _started = false;
    _code = 0;
    _bitCount = 0;
    _wiegandType = 0;
    _lastBitReceivedTimeMS = 0;
    _lastValidDataProcessedTimeMS = 0;
    _bitBufferLow = 0;
    _bitBufferHigh = 0;
}

bool Wiegand::available() {
    unsigned long elapsedTime;
    bool timeoutReached;
    bool isAvailable = false;
    unsigned long currentTime = millis();

    // Prevent interrupts from modifying the current state while checking if a code is available
    noInterrupts();

    if (_lastValidDataProcessedTimeMS > 0) {
        // valid card was processed and the cached value is still within its lifetime
        elapsedTime = currentTime - _lastValidDataProcessedTimeMS;
        timeoutReached = (elapsedTime > WIEGAND_CODE_LIFETIME_MS);

        if (timeoutReached) {
            // The last valid data processed time has exceeded the maximum memory timeout
            clearCodeState();

        } else if (_lastValidDataProcessedTimeMS - _lastBitReceivedTimeMS > 0) {
            // No new data received since last valid data processed, user can safely read the last valid data
            interrupts();
            return true;
        }
    }

    bool dataReceived = (_bitCount > 0);
    elapsedTime = currentTime - _lastBitReceivedTimeMS;
    timeoutReached = (elapsedTime > WIEGAND_RECEIVE_TIMEOUT_MS);

    if (dataReceived && timeoutReached) {
        if ((isAvailable = processReceivedData()) == true) {
            // valid data was processed, update the last valid data processed timestamp
            _lastValidDataProcessedTimeMS = currentTime;
        }
        // Reset the buffer after processing data regardless of validity
        resetBuffersState();
    }

    interrupts();
    return isAvailable;
}

bool Wiegand::begin() {
    if (_started) {
        // If the interrupts are already attached, return false
        return false;
    }

    // Reset the state of the Wiegand reader
    clearCodeState();
    resetBuffersState();

    // Set D0 & D1 pins as input pins
    pinMode(_pinD0, INPUT);
    pinMode(_pinD1, INPUT);

    // Set Hardware interrupts on DATA0 & DATA1 - high to low pulse
    attachInterrupt(digitalPinToInterrupt(_pinD0), _d0_isr_callback, FALLING);
    attachInterrupt(digitalPinToInterrupt(_pinD1), _d1_isr_callback, FALLING);

    _started = true;
    return _started;
}

bool Wiegand::begin(uint8_t pinD0, uint8_t pinD1) {
    if (_started) {
        // If the interrupts are already attached, return false
        return false;
    }

    _pinD0 = pinD0;
    _pinD1 = pinD1;
    return begin();
}

/*
 * Interrupt Service Routine for Data 0 (binary 0)
 */
void Wiegand::readDATA0(Wiegand* reader) {
    reader->_lastBitReceivedTimeMS = millis();

    // If bit count is more than 31, then process high bits
    if (reader->_bitCount >= DATA_32BIT) {
        reader->_bitBufferHigh <<= 1;
        reader->_bitBufferHigh |= ((reader->_bitBufferLow & 0x80000000) >> 31);
    }

    // Shift the current card data left by 1 bit
    reader->_bitBufferLow <<= 1;

    // Increment the bit count
    reader->_bitCount++;

    // --- ISR DEBUG PRINT START ---
    // String bitCountStr = "bit=" + String(_bitCount);
    // String bitBufferHighStr = "High=0x" + String(_cardTempHigh, HEX);
    // String bitBufferLowStr = "Low=0x" + String(_cardTemp, HEX);
    // String debugMessage = "D0: " + bitCountStr + " " + bitBufferHighStr + " " + bitBufferLowStr;
    // Serial.println(debugMessage);
    // --- ISR DEBUG PRINT END ---
}

void Wiegand::readDATA1(Wiegand* reader) {
    reader->_lastBitReceivedTimeMS = millis();

    if (reader->_bitCount >= DATA_32BIT) {
        reader->_bitBufferHigh <<= 1;
        reader->_bitBufferHigh |= ((reader->_bitBufferLow & 0x80000000) >> 31);
    }

    // Shift the current card data left by 1 bit
    reader->_bitBufferLow <<= 1;

    // Set the least significant bit to 1
    reader->_bitBufferLow |= 1;

    // Increment the bit count
    reader->_bitCount++;

    // --- ISR DEBUG PRINT START (COMMENTED OUT FOR ACCURACY TEST) ---
    // String bitCountStr = "bit=" + String(_bitCount);
    // String bitBufferHighStr = "High=0x" + String(_cardTempHigh, HEX);
    // String bitBufferLowStr = "Low=0x" + String(_cardTemp, HEX);
    // String debugMessage = "D1: " + bitCountStr + " " + bitBufferHighStr + " " + bitBufferLowStr;
    // Serial.println(debugMessage);
    // --- ISR DEBUG PRINT END ---
}

bool Wiegand::validateDataParity(
    unsigned long data,
    byte leadingParityBit,
    uint8_t leadingParityBitLength,
    byte trailingParityBit,
    uint8_t trailingParityBitLength
) {
    // Initialize parity count to start with the trailing parity bit
    byte trailingParity = trailingParityBit;

    // Initialize parity count for leading parity bit
    byte leadingParity = leadingParityBit;

    // Compute the total data length
    uint8_t total_data_length = leadingParityBitLength + trailingParityBitLength;

    // Calculate parity bits
    for (uint8_t i = 0; i < total_data_length; i++) {
        if (data & (1UL << i)) {
            if (i < trailingParityBitLength) {
                // Count bits for trailing parity
                trailingParity++;
            } else {
                // Count bits for leading parity
                leadingParity++;
            }
        }
    }

    // Validate the captured data based on the parity bits
    return (IS_EVEN(leadingParity) && IS_ODD(trailingParity)) ? true : false;
}

unsigned long Wiegand::parseCardData(
    unsigned long codeHigh, unsigned long codeLow, uint8_t bitLength
) {
    switch (bitLength) {
        case DATA_24BIT:
            return codeLow & 0xFFFFFF;

        case DATA_26BIT:
            return (codeLow >> 1) & 0xFFFFFF;

        case DATA_32BIT:
            // Assumes no parity for raw 32-bit data
            return codeLow;

        case DATA_34BIT:
            // Return the full 32-bit code (combination of the high and low buffers without parity bits)
            // To remove the high parity bit, shift the lowest-significant bit of high code to the most-significant bit
            // To remove the low parity bit, shift the low code to the right by 1 bit
            // Then combine the two parts into a single 32-bit value
            return ((codeHigh << 31) | (codeLow >> 1));

        default:
            // This will return _codelow directly for any unhandled bitlength (like 33-bit)
            return codeLow;
    }
}

bool Wiegand::processCardData() {
    // Process the card data based on the bit count
    unsigned long parsedCode = parseCardData(_bitBufferHigh, _bitBufferLow, _bitCount);

    byte leadingParityBit, trailingParityBit, standardParityBitLength;

    switch (_bitCount) {
        case DATA_26BIT:
            leadingParityBit = (byte) (_bitBufferLow >> (DATA_26BIT - 1)) & 0x1;
            trailingParityBit = (byte) (_bitBufferLow & 0x1);
            standardParityBitLength = (byte) (DATA_26BIT - 2) / 2;

            // Verify 26-bit Wiegand data parity
            if (
                !Wiegand::validateDataParity(
                    parsedCode, leadingParityBit, standardParityBitLength, trailingParityBit, standardParityBitLength
                )
            ) {
                // Parity check failed
                return false;
            }
            break;

        case DATA_34BIT:
            leadingParityBit = (byte) (_bitBufferHigh >> (DATA_34BIT % sizeof(unsigned long) - 1)) & 0x1;
            trailingParityBit = (byte) (_bitBufferLow & 0x1);

            // TODO: 34-bit Wiegand is not standardized and can have a customizable parity bit length, will need to handle that later
            // For now, we assume the parity bit length splits the message into two equal parts
            standardParityBitLength = (byte) (DATA_34BIT - 2) / 2;

            // Verify 34-bit Wiegand data parity
            if (
                !Wiegand::validateDataParity(
                    parsedCode, leadingParityBit, standardParityBitLength, trailingParityBit, standardParityBitLength
                )
            ) {
                // Parity check failed
                return false;
            }
            break;
    }

    // If we reach here, the data is valid and we can set the code
    _code = parsedCode;

    // Set the type based on the bit count
    _wiegandType = _bitCount;

    // Indicate that valid data has been processed
    return true;
}

bool Wiegand::validateKeyPress8Bit(unsigned long data) {
    // keypress wiegand with integrity
    // 8-bit Wiegand keyboard data, high nibble is the "NOT" of low nibble
    // eg if key 1 pressed, data=E1 in binary 11100001 , high nibble=1110 , low nibble = 0001
    char highNibble = (data & 0xf0) >> 4;
    char lowNibble = (data & 0x0f);

    // Data Integrity check
    if (lowNibble != (~highNibble & 0x0f)) {
        // Low nibble does not match the inverse of the high nibble!
        return false;
    }
    return true;
}

char Wiegand::parseKeyPress(char originalKeyPress) {
    // Translate the Reset or Submit keypresses to ASCII values
    switch (originalKeyPress) {
        case WIEGAND_KEYPAD_ASTERISK_KEY: return ASCII_ENTER_KEY;
        case WIEGAND_KEYPAD_OCTOTHORPE_KEY: return ASCII_ESCAPE_KEY;
        default: return originalKeyPress;
    }
}

bool Wiegand::processKeyPress() {
    switch (_bitCount) {
        case KEYPRESS_8BIT:
            if (!Wiegand::validateKeyPress8Bit(_bitBufferLow)){
                return false;
            }
            // ALLOW FALLTHROUGH LOGIC - process validated value 4-bit keypress

        case KEYPRESS_4BIT:
            // For 4-bit and 8-bit keypresses, extract the low nibble
            _code = (unsigned long) Wiegand::parseKeyPress(_bitBufferLow & 0x0F);

            // Set the type based on the bit count
            _wiegandType = _bitCount;

            // Indicate that valid data has been processed
            return true;

        default:
            // If the bit length is not recognized, return false
            return false;
    }
}

void Wiegand::resetBuffersState() {
    // To prevent data corruption, disable interrupts briefly when modifying
    // variables that are also modified by the ISR
    noInterrupts();
    _lastBitReceivedTimeMS = 0;
    _bitCount = 0;
    _bitBufferLow = 0;
    _bitBufferHigh = 0;
    interrupts();
}

void Wiegand::clearCodeState() {
    _code = 0;
    _wiegandType = 0;
    _lastValidDataProcessedTimeMS = 0;
}

bool Wiegand::processReceivedData() {

    bool validDataSize = false;
    for (WiegandDataPacketSizes packetSize : dataSizes) {
        if (packetSize == _bitCount) {
            validDataSize = true;
            break;
        }
    }

    if (!validDataSize) {
        // Invalid/unrecognized bit counts
        return false;
    }

    if (_bitCount < DATA_24BIT) {
        // If the bit count is less than 24, it must be a keypress
        return processKeyPress();
    }

    // Handle rest of the Wiegand cases (26 & 34)
    return processCardData();
}

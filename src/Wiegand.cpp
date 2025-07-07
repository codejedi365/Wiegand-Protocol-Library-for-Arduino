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

volatile unsigned long Wiegand::_bitBufferHigh = 0;
volatile unsigned long Wiegand::_bitBufferLow = 0;
volatile unsigned long Wiegand::_lastBitReceivedTimeMS = 0;
unsigned long Wiegand::_code = 0;
volatile int Wiegand::_bitCount = 0;
int Wiegand::_wiegandType = 0;
volatile unsigned long Wiegand::_lastValidDataProcessedTimeMS = 0;

Wiegand::Wiegand() {
    // Constructor - no specific initialization needed here as begin() does it
}

unsigned long Wiegand::getCode() {
    return _code;
}

int Wiegand::getWiegandType() {
    return _wiegandType;
}

bool Wiegand::available() {
    bool ret = false;

    // Prevent interrupts from modifying the current state while checking if a code is available
    noInterrupts();

    unsigned long elapsedTime;
    bool timeoutReached;
    unsigned long currentTime = millis();

    if (_lastValidDataProcessedTimeMS > 0) {
        // valid card was processed and the cached value is still within its lifetime
        elapsedTime = currentTime - _lastValidDataProcessedTimeMS;
        timeoutReached = (elapsedTime > WIEGAND_CODE_LIFETIME_MS);

        if (timeoutReached) {
            // The last valid data processed time has exceeded the maximum memory timeout
            Wiegand::clearCodeState();

        } else if (_lastValidDataProcessedTimeMS - _lastBitReceivedTimeMS > 0) {
            // No new data received since last valid data processed, user can safely read the last valid data
            interrupts();
            return true;
        }
    }

    elapsedTime = currentTime - _lastBitReceivedTimeMS;
    timeoutReached = (elapsedTime > WIEGAND_RECEIVE_TIMEOUT_MS);
    bool dataReceived = (_bitCount > 0);

    if (dataReceived && timeoutReached) {
        if ((ret = processReceivedData()) == true) {
            // valid data was processed, update the last valid data processed timestamp
            _lastValidDataProcessedTimeMS = currentTime;
        }
        // Reset the buffer after processing data regardless of validity
        Wiegand::resetBuffersState();
    }

    interrupts();
    return ret;
}

void Wiegand::begin() {
    begin(WIEGAND_DEFAULT_PIN_D0, WIEGAND_DEFAULT_PIN_D1);
}

void Wiegand::begin(int pinD0, int pinD1) {
    Wiegand::clearCodeState();
    Wiegand::resetBuffersState();

    // Set D0 pin as input
    pinMode(pinD0, INPUT);
    // Set D1 pin as input
    pinMode(pinD1, INPUT);

    // Hardware interrupt - high to low pulse
    attachInterrupt(digitalPinToInterrupt(pinD0), readDATA0, FALLING);
    // Hardware interrupt - high to low pulse
    attachInterrupt(digitalPinToInterrupt(pinD1), readDATA1, FALLING);
}

/*
 * Interrupt Service Routine for Data 0 (binary 0)
 */
INTERRUPT_ATTR void Wiegand::readDATA0 () {
    _lastBitReceivedTimeMS = millis();

    // If bit count is more than 31, then process high bits
    if (_bitCount >= DATA_32BIT) {
        _bitBufferHigh <<= 1;
        _bitBufferHigh |= ((_bitBufferLow & 0x80000000) >> 31);
    }

    // Shift the current card data left by 1 bit
    _bitBufferLow <<= 1;

    // Increment the bit count
    _bitCount++;

    // --- ISR DEBUG PRINT START ---
    // String bitCountStr = "bit=" + String(_bitCount);
    // String bitBufferHighStr = "High=0x" + String(_cardTempHigh, HEX);
    // String bitBufferLowStr = "Low=0x" + String(_cardTemp, HEX);
    // String debugMessage = "D0: " + bitCountStr + " " + bitBufferHighStr + " " + bitBufferLowStr;
    // Serial.println(debugMessage);
    // --- ISR DEBUG PRINT END ---
}

// Interrupt Service Routine for Data 1 (binary 1)
INTERRUPT_ATTR void Wiegand::readDATA1() {
    _lastBitReceivedTimeMS = millis();

    if (_bitCount >= DATA_32BIT) {
        _bitBufferHigh <<= 1;
        _bitBufferHigh |= ((_bitBufferLow & 0x80000000) >> 31);
    }

    // Shift the current card data left by 1 bit
    _bitBufferLow <<= 1;

    // Set the least significant bit to 1
    _bitBufferLow |= 1;

    // Increment the bit count
    _bitCount++;

    // --- ISR DEBUG PRINT START (COMMENTED OUT FOR ACCURACY TEST) ---
    // String bitCountStr = "bit=" + String(_bitCount);
    // String bitBufferHighStr = "High=0x" + String(_cardTempHigh, HEX);
    // String bitBufferLowStr = "Low=0x" + String(_cardTemp, HEX);
    // String debugMessage = "D1: " + bitCountStr + " " + bitBufferHighStr + " " + bitBufferLowStr;
    // Serial.println(debugMessage);
    // --- ISR DEBUG PRINT END ---
}

bool Wiegand::validateDataParity(
    volatile unsigned long *data,
    byte leadingParityBit,
    byte leadingParityBitLength,
    byte trailingParityBit,
    byte trailingParityBitLength
) {
    // Initialize parity count to start with the trailing parity bit
    byte trailingParity = trailingParityBit;

    // Initialize parity count for leading parity bit
    byte leadingParity = leadingParityBit;

    // Compute the total data length
    byte total_data_length = leadingParityBitLength + trailingParityBitLength;

    // Calculate parity bits
    for (byte i = 0; i < total_data_length; i++) {
        if (*data & (1UL << i)) {
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
    volatile unsigned long codeHigh, volatile unsigned long codeLow, byte bitLength
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
                    &parsedCode, leadingParityBit, standardParityBitLength, trailingParityBit, standardParityBitLength
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
                    &parsedCode, leadingParityBit, standardParityBitLength, trailingParityBit, standardParityBitLength
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

bool Wiegand::validateKeyPress8Bit(volatile unsigned long data) {
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
    _lastBitReceivedTimeMS = 0;
    _bitCount = 0;
    _bitBufferLow = 0;
    _bitBufferHigh = 0;
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

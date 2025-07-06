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

static const WiegandDataPacketSizes dataSizes[] = {
    KEYPRESS_4BIT, KEYPRESS_8BIT, DATA_24BIT, DATA_26BIT, DATA_32BIT, DATA_34BIT
};

volatile unsigned long Wiegand::_cardTempHigh = 0;
volatile unsigned long Wiegand::_cardTemp = 0;
volatile unsigned long Wiegand::_lastBitReceivedTimeMS = 0;
unsigned long Wiegand::_code = 0;
volatile int Wiegand::_bitCount = 0;
int Wiegand::_wiegandType = 0;

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

    if (_cardDataReady) {
        interrupts();
        return true;
    }

    unsigned long currentTime = millis();
    unsigned long elapsedTime = currentTime - _lastBitReceivedTimeMS;
    bool timeoutReached = (elapsedTime > WIEGAND_RECEIVE_TIMEOUT_MS);
    bool dataReceived = (_bitCount > 0);

    if (dataReceived && timeoutReached) {
        _cardDataReady = processReceivedData();
        ret = _cardDataReady;
        // Reset the buffer after processing data regardless of validity
        Wiegand::reset();
    }

    interrupts();
    return ret;
}

void Wiegand::begin() {
    begin(WIEGAND_DEFAULT_PIN_D0, WIEGAND_DEFAULT_PIN_D1);
}

void Wiegand::begin(int pinD0, int pinD1) {
    _code = 0;
    _wiegandType = 0;
    Wiegand::reset();

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
    _cardDataReady = false;

    // If bit count is more than 31, then process high bits
    if (_bitCount >= DATA_32BIT) {
        _cardTempHigh <<= 1;
        _cardTempHigh |= ((_cardTemp & 0x80000000) >> 31);
    }

    // Shift the current card data left by 1 bit
    _cardTemp <<= 1;

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
    _cardDataReady = false;

    if (_bitCount >= DATA_32BIT) {
        _cardTempHigh <<= 1;
        _cardTempHigh |= ((_cardTemp & 0x80000000) >> 31);
    }

    // Shift the current card data left by 1 bit
    _cardTemp <<= 1;

    // Set the least significant bit to 1
    _cardTemp |= 1;

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

unsigned long Wiegand::parseCardCode(
    volatile unsigned long *codehigh, volatile unsigned long *codelow, char bitlength
) {
    switch (bitlength) {
        // EM tags
        case DATA_24BIT:
            return (*codelow & 0x7FFFFE) >> 1;
        case DATA_26BIT:
            return (*codelow & 0x1FFFFFE) >> 1;

        // MiFare
        case DATA_34BIT:
            // only need the 2 LSB of the codehigh
            *codehigh = *codehigh & 0x03;
            // shift 2 LSB to MSB
            *codehigh <<= 30;
            *codelow >>= 1;
            return *codehigh | *codelow;

        case DATA_32BIT:
            return (*codelow & 0x7FFFFFFE) >> 1;

        default:
            // This will return _codelow directly for any unhandled bitlength (like 33-bit)
            return *codelow;
    }
}

char Wiegand::translateEnterEscapeKeyPress(char originalKeyPress) {
    switch (originalKeyPress) {
        case KEYPAD_ASTERISK_KEY: return ASCII_ENTER_KEY;
        case KEYPAD_OCTOTHORPE_KEY: return ASCII_ESCAPE_KEY;
        default: return originalKeyPress;
    }
}

void Wiegand::reset() {
    _lastBitReceivedTimeMS = millis();
    _bitCount = 0;
    _cardTemp = 0;
    _cardTempHigh = 0;
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

    switch (_bitCount) {
        case KEYPRESS_8BIT:
            // keypress wiegand with integrity
            // 8-bit Wiegand keyboard data, high nibble is the "NOT" of low nibble
            // eg if key 1 pressed, data=E1 in binary 11100001 , high nibble=1110 , low nibble = 0001
            char highNibble = (_cardTemp & 0xf0) >> 4;
            char lowNibble = (_cardTemp & 0x0f);

            // Data Integrity check
            if (lowNibble != (~highNibble & 0x0f)) {
                // Low nibble does not match the inverse of the high nibble!
                return false;
            }

            // Update card code to only include the single code
            _cardTemp = lowNibble;
            // FALLTHROUGH LOGIC to be processed as a 4 bit

        case KEYPRESS_4BIT:
            // 4-bit Wiegand codes have no data integrity check so we just
            // read the LOW nibble.
            _code = (int) translateEnterEscapeKeyPress(_cardTemp & 0x0000000F);
            break;

        default:
            // Handle rest of the Wiegand cases (26 & 34)
            _code = parseCardCode(&_cardTempHigh, &_cardTemp, _bitCount);
    }

    // Set the type based on the bit count
    _wiegandType = _bitCount;

    // Indicate that valid data has been processed
    return true;
}

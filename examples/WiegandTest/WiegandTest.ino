#include <Wiegand.h>

// Update these pin definitions if you are using different pins for Wiegand DATA0 and DATA1.
#define PIN_DATA0  WIEGAND_DEFAULT_PIN_D0
#define PIN_DATA1  WIEGAND_DEFAULT_PIN_D1

// Set the baud rate for serial communication
// Need a high baud rate to avoid data loss because of the high frequency of Wiegand data
#define BAUD_RATE  115200

#define LAST_SCANNED_CODE_LIFETIME_MS  WIEGAND_CODE_LIFETIME_MS

// Globals
static Wiegand wiegandReader1;
static unsigned long wiegandReader1LastCode = 0;
static unsigned long wiegandReader1LastCodeTimeMS = 0;

String formatWiegandData(unsigned long cardCode, int wiegandType) {
    // Format the Wiegand data into a human-readable string
    // Ex: "Wiegand WG26 Code: 123456789 [0x75bcd15]"
    String word_parts[] = {
        "Wiegand WG" + String(wiegandType) + " Code:",
        String(cardCode),
        "[0x" + String(cardCode, HEX) + "]"
    };

    String formattedData = "";
    for (String word_part : word_parts) {
        formattedData += word_part + " ";
    }
    formattedData.trim();

    return formattedData;
}

void checkWiegandData(Wiegand* wiegandReader, unsigned long* cachedCode, unsigned long* lastTimestamp) {
    if (wiegandReader->available()) {
        unsigned long cardCode = wiegandReader->getCode();
        unsigned long currentTime = millis();
        if (cardCode == *cachedCode) {
            unsigned long elapsedTime = currentTime - *lastTimestamp;
            if (elapsedTime > LAST_SCANNED_CODE_LIFETIME_MS) {
                *cachedCode = 0;
                *lastTimestamp = 0;
            }
            return;
        }

        // Cache the timestamp of the last code so we can only keep it for a limited lifetime
        *lastTimestamp = currentTime;

        // Cache the last card so it doesn't repeat the print for the lifetime of the wiegand internal cache
        *cachedCode = cardCode;

        // Print the formatted Wiegand data to the serial monitor
        Serial.println(formatWiegandData(cardCode, wiegandReader->getWiegandType()));
    }
}

void setup() {
    Serial.begin(BAUD_RATE);

    // Wait for serial to be ready
    while (!Serial);
    Serial.println("Wiegand Reader Ready for User Input");

    // Initialize the Wiegand reader with your defined pins
    wiegandReader1.begin(PIN_DATA0, PIN_DATA1);
}

void loop() {
    checkWiegandData(&wiegandReader1, &wiegandReader1LastCode, &wiegandReader1LastCodeTimeMS);
}

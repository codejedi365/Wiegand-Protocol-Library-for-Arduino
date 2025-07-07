#include <Wiegand.h>

// Update these pin definitions if you are using different pins for Wiegand DATA0 and DATA1.
#define READER_1_PIN_DATA0  WIEGAND_DEFAULT_PIN_D0
#define READER_1_PIN_DATA1  WIEGAND_DEFAULT_PIN_D1

// Set the baud rate for serial communication
// Need a high baud rate to avoid data loss because of the high frequency of Wiegand data
#define BAUD_RATE  115200


// Function prototypes
void check4WiegandData(Wiegand* wiegandReader);
String formatWiegandData(unsigned long cardCode, int wiegandType);
void reader1_data0_isr_callback();
void reader1_data1_isr_callback();


// Globals
static Wiegand wiegandReader1 = Wiegand(
    READER_1_PIN_DATA0, READER_1_PIN_DATA1, reader1_data0_isr_callback, reader1_data1_isr_callback
);


void setup() {
    Serial.begin(BAUD_RATE);
    while (!Serial);  // Wait for serial to be ready

    // Initialize the Wiegand reader with your defined pins
    if (!wiegandReader1.begin()) {
        String err = (
            "Failed to start interrupt input for '" + wiegandReader1.getName() + "' using pins "
            + String(wiegandReader1.getPinDATA0()) + " and " + String(wiegandReader1.getPinDATA1())
        );
        Serial.println(err);
        return;
    }
    Serial.println(wiegandReader1.getName() + " Ready for User Input");
}


void loop() {
    check4WiegandData(&wiegandReader1);

    // Small delay to prevent busy-waiting and allow other non-interrupt tasks to run
    delay(1);
}


void check4WiegandData(Wiegand* wiegandReader) {
    if (wiegandReader->available()) {
        unsigned long cardCode = wiegandReader->getCode();
        wiegandReader->clearCodeState();

        // Print the formatted Wiegand data to the serial monitor
        Serial.println(
            formatWiegandData(
                wiegandReader->getName(), cardCode, wiegandReader->getWiegandType()
            )
        );
    }
}


String formatWiegandData(String prefix, unsigned long cardCode, int wiegandType) {
    // Format the Wiegand data into a human-readable string
    // Ex: "Wiegand Reader 1 Code: (WG26) 123456789 [0x75bcd15]"
    String word_parts[] = {
        prefix,
        "Code:",
        "(WG" + String(wiegandType) + ")",
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


void reader1_data0_isr_callback() { Wiegand::readDATA0(&wiegandReader1); }

void reader1_data1_isr_callback() { Wiegand::readDATA1(&wiegandReader1); }

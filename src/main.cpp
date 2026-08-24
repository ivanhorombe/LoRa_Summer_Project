#include <Arduino.h>
#include <RadioLib.h>
#include <U8g2lib.h>

// HARDWARE CONFIGURATION & PINOUTS (Heltec WiFi LoRa 32 V3)
#define LORA_NSS    8
#define LORA_DIO1   14
#define LORA_NRST   12
#define LORA_BUSY   13
#define VEXT_CTRL   36 
#define OLED_SDA    17
#define OLED_SCL    18
#define OLED_RST    21

// Toggle NODE_IS_TRANSMITTER to true for Node A, false for Node B
#define NODE_IS_TRANSMITTER true

// Hardware Objects
SX1262 radio = new Module(LORA_NSS, LORA_DIO1, LORA_NRST, LORA_BUSY);
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, OLED_RST, OLED_SCL, OLED_SDA);

// __attribute__((packed)) disables memory padding alignment by the compiler.
enum FrameType : uint8_t {
  FRAME_DATA = 0x01,
  FRAME_ACK  = 0x02
};

typedef struct __attribute__((packed)) {
  uint8_t  type;        // FRAME_DATA or FRAME_ACK
  uint16_t packetId;    // Sequence counter to correlate ACKs & perform deduplication
  float    temperature; // Payload metric 1
  float    humidity;    // Payload metric 2
} TelemetryFrame;

// FREERTOS SYNCHRONIZATION PRIMITIVES
// Event Groups allow tasks to block until hardware events occur without CPU polling.
EventGroupHandle_t radioEvents;
#define EVT_TX_DONE (1 << 0)
#define EVT_RX_DONE (1 << 1)

// Queue for passing deduplicated, verified data frames to the display task safely
QueueHandle_t displayQueue;

// ISR: Must be stored in IRAM to execute immediately during hardware interrupts
void IRAM_ATTR onRadioInterrupt(void) {
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  // Notify FreeRTOS event group from ISR context safely
  xEventGroupSetBitsFromISR(radioEvents, EVT_RX_DONE | EVT_TX_DONE, &xHigherPriorityTaskWoken);
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

// TRANSMITTER LOGIC
#if NODE_IS_TRANSMITTER

#define MAX_RETRIES    3
#define ACK_TIMEOUT_MS 350

// ARQ Reliable Transmission with Backoff Jitter
bool sendWithAck(TelemetryFrame &frame) {
  TelemetryFrame ackFrame;
  frame.type = FRAME_DATA;

  for (int attempt = 0; attempt < MAX_RETRIES; attempt++) {
    // Transmit Data Frame
    xEventGroupClearBits(radioEvents, EVT_TX_DONE | EVT_RX_DONE);
    radio.startTransmit((uint8_t*)&frame, sizeof(TelemetryFrame));

    // Block task until TX hardware interrupt fires (1000ms safety limit)
    EventBits_t bits = xEventGroupWaitBits(
      radioEvents, EVT_TX_DONE, pdTRUE, pdFALSE, pdMS_TO_TICKS(1000)
    );
    if (!(bits & EVT_TX_DONE)) {
      Serial.println(F("[TX Error] Hardware TX timeout. Retrying..."));
      continue;
    }

    // Switch radio into RX mode immediately to listen for incoming ACK
    xEventGroupClearBits(radioEvents, EVT_RX_DONE);
    radio.startReceive();

    // Block task waiting for ACK response or software timeout
    bits = xEventGroupWaitBits(
      radioEvents, EVT_RX_DONE, pdTRUE, pdFALSE, pdMS_TO_TICKS(ACK_TIMEOUT_MS)
    );

    if (bits & EVT_RX_DONE) {
      int state = radio.readData((uint8_t*)&ackFrame, sizeof(TelemetryFrame));
      
      if (state == RADIOLIB_ERR_NONE) {
        // Confirm frame is an ACK and sequence ID matches current outgoing frame
        if (ackFrame.type == FRAME_ACK && ackFrame.packetId == frame.packetId) {
          radio.standby();
          Serial.printf("[TX Success] ACK received for Packet #%d on attempt %d\n", frame.packetId, attempt + 1);
          return true; // Packet successfully delivered
        }
      } else if (state == RADIOLIB_ERR_CRC_MISMATCH) {
        Serial.println(F("[TX Warning] Corrupted ACK received (CRC Error)."));
      }
    }

    // Adding a random delay (e.g., random(20, 80) ms) prevents "lockstep collisions"
    // where two transmitters collide, retry on the exact same schedule, and collide forever.
    uint32_t jitterMs = random(20, 80) + (attempt * 100);
    Serial.printf("[TX Retry] ACK timeout for #%d. Retrying in %d ms (Attempt %d/%d)\n", 
                  frame.packetId, jitterMs, attempt + 1, MAX_RETRIES);
    
    vTaskDelay(pdMS_TO_TICKS(jitterMs));
  }

  radio.standby();
  Serial.printf("[TX Failure] Packet #%d lost after %d attempts.\n", frame.packetId, MAX_RETRIES);
  return false;
}

void transmitterTask(void *pvParameters) {
  uint16_t sequenceCounter = 0;

  while (1) {
    TelemetryFrame packet;
    packet.packetId = ++sequenceCounter;
    packet.temperature = 24.5f + (random(-10, 10) / 10.0f);
    packet.humidity = 55.0f + (random(-20, 20) / 10.0f);

    Serial.printf("\n[TX Task] Initiating transfer for Packet #%d...\n", packet.packetId);
    
    // Execute reliable ARQ transfer block
    bool delivered = sendWithAck(packet);

    // Pass status to display pipeline via Queue
    xQueueSend(displayQueue, &packet, 0);

    // Read sensor / send metrics every 5 seconds
    vTaskDelay(pdMS_TO_TICKS(5000));
  }
}

#else

// RECEIVER LOGIC

void receiverTask(void *pvParameters) {
  TelemetryFrame rxFrame;
  TelemetryFrame ackFrame;
  ackFrame.type = FRAME_ACK;

  // Tracks last successfully processed sequence ID to maintain idempotency.
  static uint16_t lastProcessedPacketId = 0;
  static bool hasReceivedFirstPacket = false;

  radio.startReceive();

  while (1) {
    // Block indefinitely until radio ISR flags EVT_RX_DONE
    EventBits_t bits = xEventGroupWaitBits(
      radioEvents, EVT_RX_DONE, pdTRUE, pdFALSE, portMAX_DELAY
    );

    if (bits & EVT_RX_DONE) {
      // The SX1262 automatically evaluates payload checksums. RadioLib flags corruption.
      int state = radio.readData((uint8_t*)&rxFrame, sizeof(TelemetryFrame));

      if (state == RADIOLIB_ERR_CRC_MISMATCH) {
        Serial.println(F("[RX Error] Hardware CRC Mismatch! Corrupted frame dropped."));
      } 
      else if (state == RADIOLIB_ERR_NONE && rxFrame.type == FRAME_DATA) {
      
        // If an ACK was destroyed in flight on a previous try, Node A will retransmit.
        // Node B must recognize the duplicate, drop the payload, but RE-SEND the ACK.
        bool isDuplicate = hasReceivedFirstPacket && (rxFrame.packetId == lastProcessedPacketId);

        if (isDuplicate) {
          Serial.printf("[RX Deduplication] Duplicate Packet #%d detected. Re-sending ACK.\n", rxFrame.packetId);
          // DO NOT push to displayQueue (prevents double-processing side effects)
        } else {
          // New valid frame: Update state and forward to application queue
          lastProcessedPacketId = rxFrame.packetId;
          hasReceivedFirstPacket = true;
          
          xQueueSend(displayQueue, &rxFrame, 0);
          Serial.printf("[RX Success] Valid Packet #%d received & queued.\n", rxFrame.packetId);
        }

        // Always re-transmit ACK for valid data frames (new or duplicate)
        ackFrame.packetId = rxFrame.packetId;

        xEventGroupClearBits(radioEvents, EVT_TX_DONE);
        radio.startTransmit((uint8_t*)&ackFrame, sizeof(TelemetryFrame));

        // Wait for ACK transmit completion before returning to RX mode
        xEventGroupWaitBits(radioEvents, EVT_TX_DONE, pdTRUE, pdFALSE, pdMS_TO_TICKS(500));
      }

      // Re-arm radio to receive mode
      radio.startReceive();
    }
  }
}

#endif

// DISPLAY TASK
void displayTask(void *pvParameters) {
  TelemetryFrame displayBuffer;
  char line[32];

  while (1) {
    // Block until new telemetry arrives in queue
    if (xQueueReceive(displayQueue, &displayBuffer, portMAX_DELAY)) {
      u8g2.clearBuffer();
      
      snprintf(line, sizeof(line), "Packet ID: #%d", displayBuffer.packetId);
      u8g2.drawStr(0, 15, line);

      snprintf(line, sizeof(line), "Temp: %.1f C", displayBuffer.temperature);
      u8g2.drawStr(0, 35, line);

      snprintf(line, sizeof(line), "Hum:  %.1f %%", displayBuffer.humidity);
      u8g2.drawStr(0, 55, line);

      u8g2.sendBuffer();
    }
  }
}

// SETUP & INITIALIZATION
void setup() {
  Serial.begin(115200);

  // Power on VEXT peripheral rail (Heltec V3 requirement)
  pinMode(VEXT_CTRL, OUTPUT);
  digitalWrite(VEXT_CTRL, LOW);
  delay(100);

  // Initialize Display
  u8g2.begin();
  u8g2.setFont(u8g2_font_ncenB08_tr);

  // Initialize FreeRTOS Primitives
  radioEvents = xEventGroupCreate();
  displayQueue = xQueueCreate(5, sizeof(TelemetryFrame));

  // Initialize SX1262 Radio
  int state = radio.begin(915.0);
  if (state == RADIOLIB_ERR_NONE) {
    radio.setDio1Action(onRadioInterrupt); // Assign ISR callback
    Serial.println(F("SX1262 LoRa Radio Initialized Successfully."));
  } else {
    Serial.printf("SX1262 Initialization Failed, code: %d\n", state);
    while (1);
  }

  // Launch FreeRTOS Tasks
#if NODE_IS_TRANSMITTER
  xTaskCreate(transmitterTask, "TX_Task", 4096, NULL, 2, NULL);
#else
  xTaskCreate(receiverTask, "RX_Task", 4096, NULL, 2, NULL);
#endif

  xTaskCreate(displayTask, "Display_Task", 3072, NULL, 1, NULL);
}

void loop() {
  // Main thread yields indefinitely; FreeRTOS scheduler takes full control
  vTaskDelay(portMAX_DELAY);
}
/*
 * FreeRTOS Sensor Monitoring System
 * ---------------------------------
 * Board   : ESP32 DevKit-C (simulated on Wokwi)
 * Sensor  : MPU6050 accelerometer + temperature sensor (I2C address 0x68)
 * Display : SSD1306 128x64 OLED (I2C address 0x3C)
 * Output  : Serial monitor (UART, 115200 baud) and an alarm LED
 *
 * Four FreeRTOS tasks (higher number = higher priority):
 *
 *   buttonTask  (4)  Woken by the button ISR through a binary semaphore.
 *                    Acknowledges (silences) the temperature alarm.
 *   sensorTask  (3)  Reads the MPU6050 over I2C every 100 ms and sends
 *                    each raw sample to rawQueue.
 *   filterTask  (2)  Takes samples from rawQueue, applies an 8-sample
 *                    moving average, checks the alarm limit, drives the
 *                    LED and sends the result to displayQueue.
 *   displayTask (1)  Takes results from displayQueue and, every 500 ms,
 *                    draws the latest one on the OLED and prints it on
 *                    the serial port.
 *
 * Shared resources:
 *   i2cMutex     The MPU6050 and the OLED sit on the same I2C bus, and
 *                two tasks (sensorTask, displayTask) use it. The mutex
 *                makes sure only one task talks on the bus at a time.
 *   serialMutex  Several tasks print to Serial. The mutex stops their
 *                lines from getting mixed together.
 */

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

/* ------------------------------------------------------------------ */
/* Pins and settings                                                   */
/* ------------------------------------------------------------------ */
#define I2C_SDA_PIN        21
#define I2C_SCL_PIN        22
#define BUTTON_PIN         4
#define ALARM_LED_PIN      2

#define MPU6050_ADDR       0x68
#define MPU_REG_PWR_MGMT_1 0x6B   /* write 0 here to wake the sensor up  */
#define MPU_REG_ACCEL_XOUT 0x3B   /* first of the accel + temp registers */

#define OLED_ADDR          0x3C
#define SCREEN_WIDTH       128
#define SCREEN_HEIGHT      64

#define SAMPLE_PERIOD_MS   100    /* sensor is read 10 times per second  */
#define AVG_WINDOW         8      /* moving average length (samples)     */
#define TEMP_ALARM_C       40.0f  /* alarm when average temp goes above  */
#define DEBOUNCE_MS        200    /* ignore button presses closer than this */
#define DISPLAY_PERIOD_MS  500    /* redraw OLED at most twice a second  */

/* ------------------------------------------------------------------ */
/* Data sent between tasks through queues                             */
/* ------------------------------------------------------------------ */
typedef struct {
    float    ax, ay, az;      /* acceleration in g                     */
    float    tempC;           /* sensor temperature in degrees Celsius */
    uint32_t timestampMs;
} RawSample;

typedef struct {
    float    avgTempC;        /* moving average of temperature         */
    float    avgAccelG;       /* moving average of |acceleration|      */
    bool     overLimit;       /* average temperature above the limit   */
    bool     alarmActive;     /* over the limit and not acknowledged   */
    uint32_t timestampMs;
} FilteredData;

/* ------------------------------------------------------------------ */
/* FreeRTOS objects                                                    */
/* ------------------------------------------------------------------ */
static QueueHandle_t     rawQueue;         /* sensorTask -> filterTask  */
static QueueHandle_t     displayQueue;     /* filterTask -> displayTask */
static SemaphoreHandle_t i2cMutex;         /* protects the I2C bus      */
static SemaphoreHandle_t serialMutex;      /* protects Serial           */
static SemaphoreHandle_t buttonSemaphore;  /* button ISR -> buttonTask  */

/* Written by buttonTask, read by filterTask. volatile so the compiler
 * always reads the real value instead of a cached copy. */
static volatile bool alarmAcknowledged = false;

/* 400 kHz I2C during and after display transfers */
static Adafruit_SSD1306 oled(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1,
                             400000UL, 400000UL);

/* ------------------------------------------------------------------ */
/* Helpers                                                             */
/* ------------------------------------------------------------------ */

/* Print one line on Serial without other tasks cutting in. */
static void logLine(const char *msg)
{
    if (xSemaphoreTake(serialMutex, portMAX_DELAY) == pdTRUE) {
        Serial.println(msg);
        xSemaphoreGive(serialMutex);
    }
}

/* Write one byte to an MPU6050 register. Returns true on success. */
static bool mpuWriteRegister(uint8_t reg, uint8_t value)
{
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(reg);
    Wire.write(value);
    return Wire.endTransmission() == 0;   /* 0 means the sensor sent ACK */
}

/*
 * Read acceleration (X, Y, Z) and temperature from the MPU6050.
 * Registers 0x3B..0x42 hold 8 bytes: AX, AY, AZ, TEMP, each 16-bit
 * signed, high byte first (big-endian).
 */
static bool mpuReadSample(RawSample *s)
{
    uint8_t buf[8];

    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(MPU_REG_ACCEL_XOUT);
    if (Wire.endTransmission(false) != 0) {     /* false = repeated start */
        return false;
    }
    if (Wire.requestFrom(MPU6050_ADDR, 8) != 8) {
        return false;
    }
    for (int i = 0; i < 8; i++) {
        buf[i] = Wire.read();
    }

    int16_t rawAx   = (int16_t)((buf[0] << 8) | buf[1]);
    int16_t rawAy   = (int16_t)((buf[2] << 8) | buf[3]);
    int16_t rawAz   = (int16_t)((buf[4] << 8) | buf[5]);
    int16_t rawTemp = (int16_t)((buf[6] << 8) | buf[7]);

    /* Default range is +/-2 g, which is 16384 counts per g. */
    s->ax = rawAx / 16384.0f;
    s->ay = rawAy / 16384.0f;
    s->az = rawAz / 16384.0f;
    /* Temperature formula from the MPU6050 datasheet. */
    s->tempC = rawTemp / 340.0f + 36.53f;
    return true;
}

/* ------------------------------------------------------------------ */
/* Interrupt service routine                                           */
/* ------------------------------------------------------------------ */

/*
 * Runs when the button pin goes from HIGH to LOW (button pressed).
 * An ISR must be short, so it only gives the semaphore. The real work
 * happens in buttonTask. IRAM_ATTR places the ISR in internal RAM so it
 * runs even while the flash cache is busy.
 */
static void IRAM_ATTR buttonISR()
{
    BaseType_t higherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(buttonSemaphore, &higherPriorityTaskWoken);
    if (higherPriorityTaskWoken == pdTRUE) {
        portYIELD_FROM_ISR();   /* switch to buttonTask straight away */
    }
}

/* ------------------------------------------------------------------ */
/* Tasks                                                               */
/* ------------------------------------------------------------------ */

/* Priority 4: handle button presses signalled by the ISR. */
static void buttonTask(void *params)
{
    TickType_t lastPress = 0;

    for (;;) {
        /* Sleep here (using no CPU) until the ISR gives the semaphore. */
        if (xSemaphoreTake(buttonSemaphore, portMAX_DELAY) != pdTRUE) {
            continue;
        }

        /* Software debounce: a real button "bounces" and can fire the
         * interrupt several times for a single press. */
        TickType_t now = xTaskGetTickCount();
        if ((now - lastPress) < pdMS_TO_TICKS(DEBOUNCE_MS)) {
            continue;
        }
        lastPress = now;

        alarmAcknowledged = true;
        logLine("[button] Alarm acknowledged");
    }
}

/* Priority 3: read the sensor at a fixed rate. */
static void sensorTask(void *params)
{
    RawSample  sample;
    TickType_t lastWake = xTaskGetTickCount();

    for (;;) {
        bool ok = false;

        /* The OLED uses the same I2C bus, so take the mutex first. */
        if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
            ok = mpuReadSample(&sample);
            xSemaphoreGive(i2cMutex);
        }

        if (ok) {
            sample.timestampMs = millis();
            /* Timeout 0: never block the sensor task if the queue is full. */
            if (xQueueSend(rawQueue, &sample, 0) != pdTRUE) {
                logLine("[sensor] rawQueue full, sample dropped");
            }
        } else {
            logLine("[sensor] I2C read failed");
        }

        /* vTaskDelayUntil keeps an exact 100 ms period. A plain vTaskDelay
         * would add the time spent reading the sensor to every cycle. */
        vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(SAMPLE_PERIOD_MS));
    }
}

/* Priority 2: moving average and alarm check. */
static void filterTask(void *params)
{
    float tempWindow[AVG_WINDOW]  = {0};
    float accelWindow[AVG_WINDOW] = {0};
    float tempSum  = 0.0f;
    float accelSum = 0.0f;
    int   index    = 0;
    int   count    = 0;
    bool  previousAlarm = false;

    RawSample    in;
    FilteredData out;

    for (;;) {
        /* Block until the sensor task sends a sample. */
        if (xQueueReceive(rawQueue, &in, portMAX_DELAY) != pdTRUE) {
            continue;
        }

        float accelMagnitude = sqrtf(in.ax * in.ax + in.ay * in.ay + in.az * in.az);

        /* Circular buffer with running sums: subtract the oldest value,
         * add the newest, so each update costs O(1) instead of O(N). */
        tempSum  -= tempWindow[index];
        accelSum -= accelWindow[index];
        tempWindow[index]  = in.tempC;
        accelWindow[index] = accelMagnitude;
        tempSum  += in.tempC;
        accelSum += accelMagnitude;
        index = (index + 1) % AVG_WINDOW;
        if (count < AVG_WINDOW) {
            count++;
        }

        out.avgTempC  = tempSum / count;
        out.avgAccelG = accelSum / count;
        out.overLimit = out.avgTempC > TEMP_ALARM_C;

        /* Once the temperature is back to normal, re-arm the alarm. */
        if (!out.overLimit) {
            alarmAcknowledged = false;
        }
        out.alarmActive = out.overLimit && !alarmAcknowledged;

        digitalWrite(ALARM_LED_PIN, out.alarmActive ? HIGH : LOW);
        if (out.alarmActive && !previousAlarm) {
            logLine("[filter] ALARM: temperature above limit");
        }
        previousAlarm = out.alarmActive;

        out.timestampMs = in.timestampMs;
        /* If the display falls behind, drop this result rather than
         * slowing down the filter. */
        xQueueSend(displayQueue, &out, 0);
    }
}

/* Priority 1 (lowest): update the OLED and the serial output. */
static void displayTask(void *params)
{
    FilteredData data;
    char line[80];
    TickType_t lastDraw = 0;

    for (;;) {
        if (xQueueReceive(displayQueue, &data, portMAX_DELAY) != pdTRUE) {
            continue;
        }

        /* A full OLED frame is about 1 KB over I2C. Redrawing 10 times a
         * second would keep the bus busy, so redraw at most every 500 ms
         * and skip the results in between. */
        TickType_t now = xTaskGetTickCount();
        if ((now - lastDraw) < pdMS_TO_TICKS(DISPLAY_PERIOD_MS)) {
            continue;
        }
        lastDraw = now;

        const char *status = data.alarmActive ? "ALARM!"
                           : data.overLimit   ? "ACKNOWLEDGED"
                                              : "OK";

        /* The sensor task also uses I2C, so take the mutex first. */
        if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            oled.clearDisplay();
            oled.setTextSize(1);
            oled.setCursor(0, 0);
            oled.println("FreeRTOS Monitor");
            oled.drawLine(0, 10, SCREEN_WIDTH - 1, 10, SSD1306_WHITE);

            oled.setCursor(0, 16);
            snprintf(line, sizeof(line), "Temp : %.1f C", data.avgTempC);
            oled.println(line);
            snprintf(line, sizeof(line), "Accel: %.2f g", data.avgAccelG);
            oled.println(line);
            snprintf(line, sizeof(line), "State: %s", status);
            oled.println(line);
            snprintf(line, sizeof(line), "Up   : %lu s", (unsigned long)(data.timestampMs / 1000));
            oled.println(line);

            oled.display();   /* sends the frame to the OLED over I2C */
            xSemaphoreGive(i2cMutex);
        }

        snprintf(line, sizeof(line), "[display] t=%lu ms  temp=%.2f C  accel=%.3f g  %s",
                 (unsigned long)data.timestampMs, data.avgTempC, data.avgAccelG, status);
        logLine(line);
    }
}

/* ------------------------------------------------------------------ */
/* Setup                                                               */
/* ------------------------------------------------------------------ */
void setup()
{
    Serial.begin(115200);
    Serial.println("FreeRTOS Sensor Monitor starting...");

    pinMode(ALARM_LED_PIN, OUTPUT);
    pinMode(BUTTON_PIN, INPUT_PULLUP);   /* reads HIGH, LOW when pressed */

    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
    Wire.setClock(400000);               /* 400 kHz "fast mode" I2C */

    /* Create queues and semaphores before any task can use them. */
    rawQueue        = xQueueCreate(10, sizeof(RawSample));
    displayQueue    = xQueueCreate(4,  sizeof(FilteredData));
    i2cMutex        = xSemaphoreCreateMutex();
    serialMutex     = xSemaphoreCreateMutex();
    buttonSemaphore = xSemaphoreCreateBinary();

    if (!rawQueue || !displayQueue || !i2cMutex || !serialMutex || !buttonSemaphore) {
        Serial.println("ERROR: could not create RTOS objects (out of memory)");
        for (;;) { delay(1000); }
    }

    /* The MPU6050 starts in sleep mode. Clearing PWR_MGMT_1 wakes it. */
    if (!mpuWriteRegister(MPU_REG_PWR_MGMT_1, 0x00)) {
        Serial.println("ERROR: MPU6050 not found on I2C bus");
    }

    if (!oled.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
        Serial.println("ERROR: SSD1306 OLED not found on I2C bus");
    }
    oled.setTextColor(SSD1306_WHITE);
    oled.clearDisplay();
    oled.display();

    attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), buttonISR, FALLING);

    /*                 function     name       stack  param priority handle */
    xTaskCreate(buttonTask,  "button",  4096, NULL, 4, NULL);
    xTaskCreate(sensorTask,  "sensor",  4096, NULL, 3, NULL);
    xTaskCreate(filterTask,  "filter",  4096, NULL, 2, NULL);
    xTaskCreate(displayTask, "display", 4096, NULL, 1, NULL);

    Serial.println("All tasks started.");
}

/* Arduino's loop() runs in its own task. Everything is done by the four
 * tasks above, so delete the loop task to free its memory. */
void loop()
{
    vTaskDelete(NULL);
}

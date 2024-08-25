#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

/* Driver Header files */
#include <ti/drivers/GPIO.h>
#include <ti/drivers/I2C.h>
#include <ti/drivers/UART.h>
#include <ti/drivers/Timer.h>

/* Driver configuration */
#include "ti_drivers_config.h"

// I2C Global Variables
static const struct {
    uint8_t address;
    uint8_t resultReg;
    char *id;
} sensors[3] = {
    { 0x48, 0x0000, "11X" },
    { 0x49, 0x0000, "116" },
    { 0x41, 0x0001, "006" }
};

uint8_t txBuffer[1];
uint8_t rxBuffer[2];
I2C_Transaction i2cTransaction;
I2C_Handle i2c;

// UART Global Variables
char output[64];
UART_Handle uart;

// Timer Global Variables
Timer_Handle timer0;
volatile unsigned char TimerFlag = 0;

// Task Scheduler Counters
int counter200ms = 0;
int counter500ms = 0;
int counter1s = 0;

// Application Variables
int16_t temperature = 25;  // Default starting temperature
int16_t setPoint = 25;     // Default set-point temperature
bool heaterOn = false;
int secondsElapsed = 0;

/*
 *  ======== initUART ========
 *  Initializes the UART driver for serial communication.
 */
void initUART(void) {
    UART_Params uartParams;

    // Init the driver
    UART_init();

    // Configure the driver
    UART_Params_init(&uartParams);
    uartParams.writeDataMode = UART_DATA_BINARY;
    uartParams.readDataMode = UART_DATA_BINARY;
    uartParams.readReturnMode = UART_RETURN_FULL;
    uartParams.baudRate = 115200;

    // Open the driver
    uart = UART_open(CONFIG_UART_0, &uartParams);
    if (uart == NULL) {
        // UART_open() failed
        while (1);
    }
}

#define DISPLAY(x) UART_write(uart, &output, x);

/*
 *  ======== initI2C ========
 *  Initializes the I2C driver and detects the connected temperature sensor.
 */
void initI2C(void) {
    int8_t i, found;
    I2C_Params i2cParams;

    DISPLAY(snprintf(output, 64, "Initializing I2C Driver - "));

    // Init the driver
    I2C_init();

    // Configure the driver
    I2C_Params_init(&i2cParams);
    i2cParams.bitRate = I2C_400kHz;

    // Open the driver
    i2c = I2C_open(CONFIG_I2C_0, &i2cParams);
    if (i2c == NULL) {
        DISPLAY(snprintf(output, 64, "Failed\n\r"));
        while (1);
    }
    DISPLAY(snprintf(output, 32, "Passed\n\r"));

    // Attempt to detect the temperature sensor
    i2cTransaction.writeBuf = txBuffer;
    i2cTransaction.writeCount = 1;
    i2cTransaction.readBuf = rxBuffer;
    i2cTransaction.readCount = 0;
    found = 0;

    for (i = 0; i < 3; ++i) {
        i2cTransaction.slaveAddress = sensors[i].address;
        txBuffer[0] = sensors[i].resultReg;
        DISPLAY(snprintf(output, 64, "Is this %s? ", sensors[i].id));
        if (I2C_transfer(i2c, &i2cTransaction)) {
            DISPLAY(snprintf(output, 64, "Found\n\r"));
            found = 1;
            break;
        }
        DISPLAY(snprintf(output, 64, "No\n\r"));
    }
    // Report sensor detection status
    if (found) {
        DISPLAY(snprintf(output, 64, "Detected TMP%s I2C address: %x\n\r", sensors[i].id, i2cTransaction.slaveAddress));
    } else {
        DISPLAY(snprintf(output, 64, "Temperature sensor not found, contact professor\n\r"));
    }
}

/*
 *  ======== readTemp ========
 *  Reads the temperature from the detected I2C sensor.
 *  Returns the temperature as an integer in degrees Celsius.
 */
int16_t readTemp(void) {
    int16_t temp = 0;
    i2cTransaction.readCount = 2;
    if (I2C_transfer(i2c, &i2cTransaction)) {
        // Extract the temperature value from the sensor data
        temp = (rxBuffer[0] << 8) | (rxBuffer[1]);
        temp *= 0.0078125;

        // Handle negative temperatures
        if (rxBuffer[0] & 0x80) {
            temp |= 0xF000;
        }
    } else {
        DISPLAY(snprintf(output, 64, "Error reading temperature sensor (%d)\n\r", i2cTransaction.status));
        DISPLAY(snprintf(output, 64, "Please power cycle your board by unplugging USB and plugging back in.\n\r"));
    }
    return temp;
}

/*
 *  ======== timerCallback ========
 *  Callback function for the Timer interrupt. Sets the TimerFlag.
 */
void timerCallback(Timer_Handle myHandle, int_fast16_t status) {
    TimerFlag = 1;
}

/*
 *  ======== initTimer ========
 *  Initializes the Timer driver with a 200ms period.
 */
void initTimer(void) {
    Timer_Params params;

    Timer_init();

    Timer_Params_init(&params);
    params.period = 200000;  // 200ms timer period
    params.periodUnits = Timer_PERIOD_US;
    params.timerMode = Timer_CONTINUOUS_CALLBACK;
    params.timerCallback = timerCallback;

    timer0 = Timer_open(CONFIG_TIMER_0, &params);
    if (timer0 == NULL) {
        while (1) {}
    }
    if (Timer_start(timer0) == Timer_STATUS_ERROR) {
        while (1) {}
    }
}

/*
 *  ======== gpioButtonFxn0 ========
 *  Callback function for the GPIO interrupt on CONFIG_GPIO_BUTTON_0.
 *  Increases the set-point temperature by 1 degree.
 */
void gpioButtonFxn0(uint_least8_t index) {
    setPoint++;
}

/*
 *  ======== gpioButtonFxn1 ========
 *  Callback function for the GPIO interrupt on CONFIG_GPIO_BUTTON_1.
 *  Decreases the set-point temperature by 1 degree.
 */
void gpioButtonFxn1(uint_least8_t index) {
    setPoint--;
}

/*
 *  ======== mainThread ========
 */
void *mainThread(void *arg0) {
    // Initialize the drivers
    GPIO_init();
    initUART();
    initI2C();
    initTimer();

    // Configure GPIO for the LED and buttons
    GPIO_setConfig(CONFIG_GPIO_LED_0, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);
    GPIO_setConfig(CONFIG_GPIO_BUTTON_0, GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_FALLING);
    GPIO_setConfig(CONFIG_GPIO_BUTTON_1, GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_FALLING);

    // Initialize LED state
    GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_OFF);

    // Set up button callbacks
    GPIO_setCallback(CONFIG_GPIO_BUTTON_0, gpioButtonFxn0);
    GPIO_setCallback(CONFIG_GPIO_BUTTON_1, gpioButtonFxn1);

    // Enable GPIO interrupts for the buttons
    GPIO_enableInt(CONFIG_GPIO_BUTTON_0);
    GPIO_enableInt(CONFIG_GPIO_BUTTON_1);

    // Main loop with task scheduling based on timer intervals
    while (1) {
        if (TimerFlag) {
            TimerFlag = 0;

            // Increment task counters
            counter200ms++;
            counter500ms++;
            counter1s++;

            // 200ms Tasks: Handle button press logic
            if (counter200ms >= 1) {
                counter200ms = 0;
                // Button press actions are handled by interrupts
            }

            // 500ms Tasks: Read temperature and control LED
            if (counter500ms >= 2) {
                counter500ms = 0;
                temperature = readTemp();

                // Control the LED based on the temperature and set-point
                if (temperature > setPoint) {
                    GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_ON);
                    heaterOn = true;
                } else {
                    GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_OFF);
                    heaterOn = false;
                }
            }

            // 1 second Tasks: Send data via UART
            if (counter1s >= 5) {
                counter1s = 0;
                secondsElapsed++;
                DISPLAY(snprintf(output, 64, "<%02d,%02d,%d,%04d>\n\r", temperature, setPoint, heaterOn, secondsElapsed));
            }

            // Wait for the timer period (200ms)
            while (!TimerFlag) {}
        }
    }
}

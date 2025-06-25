// Code written by Yash Panchal - 400506904

#include <stdint.h>
#include "PLL.h"                // Clock configuration (24 MHz setup)
#include "SysTick.h"            // SysTick-based delay functions
#include "uart.h"               // UART serial communication functions
#include "onboardLEDs.h"        // LED control functions
#include "tm4c1294ncpdt.h"      // Register-level access for Tiva C MCU
#include "VL53L1X_api.h"        // ToF sensor API from ST

// === Constants ===
#define I2C_ADDRESS     0x29    // Default I2C address for VL53L1X sensor
#define TOTAL_STEPS     512     // Number of steps in a full motor rotation (for 360°)
#define DEGREE_INTERVAL 16      // Perform sensor measurement every 16 steps (~11.25°)
#define STEP_WAIT       1       // Delay between stepper motor steps in 10ms units

// === Global Variables ===
int tofStatus = 0;              // Used to track status/error codes from ToF sensor functions
uint32_t stepDelay = STEP_WAIT;// Stepper motor delay used in SysTick_Wait10ms
uint16_t rangeVal = 0;         // Stores the measured distance value from VL53L1X

// === I2C Peripheral Setup for VL53L1X ===
void ConfigureI2C(void){
  SYSCTL_RCGCI2C_R |= SYSCTL_RCGCI2C_R0;    // Enable I2C0 module clock
  SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R1;  // Enable clock for Port B (I2C pins)

  while((SYSCTL_PRGPIO_R & 0x02) == 0){};   // Wait until Port B is ready

  GPIO_PORTB_AFSEL_R |= 0x0C;              // Enable alternate function for PB2 (SCL) & PB3 (SDA)
  GPIO_PORTB_ODR_R |= 0x08;                // Enable open-drain for SDA (PB3)
  GPIO_PORTB_DEN_R |= 0x0C;                // Enable digital I/O on PB2 & PB3
  GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R & 0xFFFF00FF) | 0x00002200; // Configure I2C function

  I2C0_MCR_R = 0x10;                       // Master mode enabled
  I2C0_MTPR_R = 0x3B;                      // Set I2C clock speed (~100 kHz)
}

// === Initialize Stepper Motor Control Pins (PM0–PM3) ===
void InitStepperMotorPort(void){
  SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R11; // Enable Port M clock
  while((SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R11) == 0){}; // Wait for ready

  GPIO_PORTM_DIR_R |= 0x0F;       // Set PM0–PM3 as output
  GPIO_PORTM_DEN_R |= 0x0F;       // Enable digital functionality
  GPIO_PORTM_AFSEL_R &= ~0x0F;    // Disable alternate function
  GPIO_PORTM_AMSEL_R &= ~0x0F;    // Disable analog function
}

// === Rotate Stepper Motor Clockwise (Forward Rotation) ===
void RotateMotorClockwise(void){
  GPIO_PORTM_DATA_R = 0x03; SysTick_Wait10ms(stepDelay);
  GPIO_PORTM_DATA_R = 0x06; SysTick_Wait10ms(stepDelay);
  GPIO_PORTM_DATA_R = 0x0C; SysTick_Wait10ms(stepDelay);
  GPIO_PORTM_DATA_R = 0x09; SysTick_Wait10ms(stepDelay);
}

// === Rotate Stepper Motor Counterclockwise (Reverse Rotation) ===
void RotateMotorCounterClockwise(void){
  GPIO_PORTM_DATA_R = 0x09; SysTick_Wait10ms(stepDelay);
  GPIO_PORTM_DATA_R = 0x0C; SysTick_Wait10ms(stepDelay);
  GPIO_PORTM_DATA_R = 0x06; SysTick_Wait10ms(stepDelay);
  GPIO_PORTM_DATA_R = 0x03; SysTick_Wait10ms(stepDelay);
}

// === Setup Button Input on PJ1 (used as scan trigger) ===
void SetupButtonPort(void){
  SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R8;   // Enable clock to Port J
  while ((SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R8) == 0) {}; // Wait until ready

  GPIO_PORTJ_DIR_R &= ~0x02;    // Set PJ1 as input
  GPIO_PORTJ_DEN_R |= 0x02;     // Enable digital functionality
  GPIO_PORTJ_PCTL_R &= ~0x000000F0; // Clear alternate functions
  GPIO_PORTJ_AMSEL_R &= ~0x02;  // Disable analog mode
  GPIO_PORTJ_PUR_R |= 0x02;     // Enable pull-up resistor
}

// === Setup Sensor Reset Line (XSHUT on PG0) ===
void SetupSensorResetPort(void){
  SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R6;   // Enable Port G
  while((SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R6) == 0){}; // Wait until ready

  GPIO_PORTG_DIR_R &= ~0x01;     // Set PG0 as input (XSHUT)
  GPIO_PORTG_AFSEL_R &= ~0x01;   // Disable alternate functions
  GPIO_PORTG_DEN_R |= 0x01;      // Enable digital functionality
  GPIO_PORTG_AMSEL_R &= ~0x01;   // Disable analog mode
}

// === Perform XSHUT Reset Sequence for ToF Sensor ===
void ResetToFSensor(void){
  GPIO_PORTG_DIR_R |= 0x01;      // Set PG0 as output (manual control)
  GPIO_PORTG_DATA_R &= ~0x01;    // Drive PG0 low (reset)
  FlashAllLEDs();                // Blink LEDs during reset
  SysTick_Wait10ms(10);          // Wait 100ms
  GPIO_PORTG_DIR_R &= ~0x01;     // Release PG0 (back to input)
}

// === MAIN FUNCTION ===
int main(void){
  uint8_t isReady = 0, dataAvailable = 0;   // Flags for sensor boot/data readiness

  // === System Initialization ===
  PLL_Init();                  // Set system clock to 24 MHz
  SysTick_Init();              // Initialize SysTick timer
  onboardLEDs_Init();          // Initialize onboard LEDs (PF, PN)
  ConfigureI2C();              // Setup I2C interface for ToF sensor
  UART_Init();                 // Initialize UART for serial output
  InitStepperMotorPort();      // Setup stepper motor control (PM0–PM3)
  SetupButtonPort();           // Configure PJ1 push button
  SetupSensorResetPort();      // Configure PG0 (XSHUT line if needed)

  UART_printf("Program Begins\r\n");

  // === Sensor Startup ===
  tofStatus = VL53L1X_GetSensorId(I2C_ADDRESS, &rangeVal);
  UART_printf("Sensor ID read\r\n");

  // Wait for ToF sensor to boot up
  while(isReady == 0){
    tofStatus = VL53L1X_BootState(I2C_ADDRESS, &isReady);
    SysTick_Wait10ms(10);  // Wait 100ms between checks
  }

  FlashAllLEDs();
  UART_printf("ToF Ready\r\n");

  VL53L1X_ClearInterrupt(I2C_ADDRESS);   // Clear any initial interrupts
  VL53L1X_SensorInit(I2C_ADDRESS);       // Initialize ToF sensor configuration
  VL53L1X_StartRanging(I2C_ADDRESS);     // Begin continuous distance measurement

  // === Main Loop ===
  while(1){
    if((GPIO_PORTJ_DATA_R & 0x02) == 0){  // Button PJ1 pressed (active low)
      
      // === Forward Stepper Rotation + Data Collection ===
      for(int step = 0; step < TOTAL_STEPS; step++){
        RotateMotorClockwise();  // Perform 1 motor step

        // Take a reading at each DEGREE_INTERVAL (e.g., every 11.25°)
        if(step % DEGREE_INTERVAL == 0){
          // Wait until data is ready
          while(dataAvailable == 0){
            VL53L1X_CheckForDataReady(I2C_ADDRESS, &dataAvailable);
            VL53L1_WaitMs(I2C_ADDRESS, 5); // Sensor-friendly delay
          }

          dataAvailable = 0;
          VL53L1X_GetDistance(I2C_ADDRESS, &rangeVal);  // Get distance in mm

          FlashLED4(1); // Indicator: Measurement taken
          FlashLED2(1); // Indicator: UART transmission

          sprintf(printf_buffer, "%u\r\n", rangeVal);  // Format data
          UART_printf(printf_buffer);                  // Send over UART
          VL53L1X_ClearInterrupt(I2C_ADDRESS);          // Reset sensor state
        }
      }

      // === Return to Start Position (Reverse Motor, No Data Capture) ===
      for(int step = 0; step < TOTAL_STEPS; step++){
        RotateMotorCounterClockwise();  // Reverse motor 1 step
      }

      FlashLED1(1);  // Indicator: Full scan completed
    }
  }

  // === Program should never reach here, but if it does... ===
  VL53L1X_StopRanging(I2C_ADDRESS);  // Stop sensor
}

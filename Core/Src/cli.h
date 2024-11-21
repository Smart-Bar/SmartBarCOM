
#ifndef CLI_H
#define CLI_H

#include <stdlib.h>
#include <string.h>

// Length of the receive buffer
#define RX_BUFFER_SIZE          24

// Length of the circular buffer
#define CIRC_BUFFER_SIZE        8

// Length of the command vector
#define CMD_VCTR_SIZE           6

// Length of the command parameters
#define ARG_LENGTH              4

// Minimum duty cycle for the servo motor (2.5% or 0°)
#define DUTY_CYCLE_LOWER_BOUND  2.5

// Maximum duty cycle for the servo motor (12.5% or 180°)
#define DUTY_CYCLE_UPPER_BOUND  12.5

// Minimum pulse width for the servo motor
#define MIN_PULSE_WIDTH         6.375

// Maximum pulse width for the servo motor
#define MAX_PULSE_WIDTH         31.875

// Buffer for storing received data from UART
char rxBuffer[RX_BUFFER_SIZE];

/**
 * @brief Structure for storing command arguments
 * 
 * This structure stores the name of the argument and its value.
 * 
 * @param name: Name of the argument
 * @param value: Value of the argument
 */
typedef struct {
    char name[ARG_LENGTH];
    char value[ARG_LENGTH];
} CommandArgs;

/**
 * @brief Structure for storing commands
 * 
 * This structure stores the name of the command, the function to be executed
 * and the arguments of the command.
 * 
 * @param cmdName: Name of the command
 * @param function: Function to be executed
 * @param args: Arguments of the command
 */
typedef struct {
    char cmdName[ARG_LENGTH];
    void (*function)(void);
    CommandArgs args[CMD_VCTR_SIZE];
} CommandVector;

/**
 * @brief Structure for storing commands
 * 
 * This structure stores the name of the command and the function to be executed.
 * 
 * @param name: Name of the command
 * @param function: Function to be executed
 */
typedef struct {
  const char *name;
  void (*function)(void);
} Command;

// Circular buffer for storing commands
CommandVector circBuffer[CIRC_BUFFER_SIZE];

volatile uint8_t cmdWriteIdx = 0;
volatile uint8_t cmdReadIdx = 0;
volatile uint8_t cmdCount = 0;

extern TIM_HandleTypeDef htim3;

/* Function protoypes */

void ParseCommand(void);
void ExecutePWM(CommandVector*);
void SelectInputChannel(uint8_t);
void SelectOutputChannel(uint8_t);
void ComputePID(void);
float clamp(float, float, float);
float map(float, float, float, float, float);

/**
 * @brief Parses the received command and stores it in the command vector.
 *  
 * @retval None
 */
void ParseCommand(void) {
  // Parse the command to get its parameters
  char *input = (char*) rxBuffer;
  char *token = strtok(input, " ");
  int argIndex = 0;
  CommandVector command;

  // Initialize the command structure
  memset(&command, 0, sizeof(CommandVector));

  // Get the command name
  strcpy(command.cmdName, token);
  token = strtok(NULL, " ");

  // Get the command arguments
  while (token != NULL && argIndex < CMD_VCTR_SIZE) {
    if (token[0] == '-') {
      // Get the argument name
      strcpy(command.args[argIndex].name, token);
      token = strtok(NULL, " ");

      // Get the argument value, if provided
      if (token != NULL) {
        strcpy(command.args[argIndex].value, token);
        argIndex++;
      }
    }

    token = strtok(NULL, " ");
  }

  // Store the command in the circular buffer if there is space available
  if (cmdCount < CIRC_BUFFER_SIZE) {
    circBuffer[cmdWriteIdx] = command;
    cmdWriteIdx = (cmdWriteIdx + 1) % CIRC_BUFFER_SIZE;
    cmdCount++;
  }
}

/**
 * @brief Executes the PWM command.
 * 
 * This function executes the PWM command by setting the input and output channels,
 * calculating the pulse width and setting the proper duty cycle of the signal.
 * 
 * @param cmdVctr 
 */
void ExecutePWM(CommandVector *cmdVctr) {
  int inputChnl = -1;
  int outputChnl = -1;
  float dutyCycle = -1;

  // Get the command arguments
  for (int i = 0; i < CMD_VCTR_SIZE; i++) {
    // Input channel
    if (strcmp(cmdVctr->args[i].name, "-i") == 0) {
      inputChnl = atoi(cmdVctr->args[i].value);
    }
    // Output channel
    else if (strcmp(cmdVctr->args[i].name, "-o") == 0) {
      outputChnl = atoi(cmdVctr->args[i].value);
    }
    // Duty cycle (as a percentage)
    else if (strcmp(cmdVctr->args[i].name, "-d") == 0) {
      dutyCycle = atof(cmdVctr->args[i].value);
      dutyCycle = map(dutyCycle, 0, 100, DUTY_CYCLE_LOWER_BOUND, DUTY_CYCLE_UPPER_BOUND);
    }
    // ARR value (as an integer)
    else if (strcmp(cmdVctr->args[i].name, "-a") == 0) {
      dutyCycle = atof(cmdVctr->args[i].value);
      dutyCycle = clamp(dutyCycle, DUTY_CYCLE_LOWER_BOUND, DUTY_CYCLE_UPPER_BOUND);
    }
  }

  // Select the MUX input channel in zero-index notation
  SelectInputChannel((uint8_t) inputChnl - 1);

  // Select the DMux output channel in zero-index notation
  SelectOutputChannel((uint8_t) outputChnl - 1);

  // Calculate the pulse width and set the duty cycle of the signal
  uint8_t pulse = (uint8_t)((dutyCycle / 100.0) * 255);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, pulse);
}

void ExecutePID(CommandVector *cmdVctr) {

}

void ComputePID() {

}

/**
 * @brief Clamps a value between a lower and upper bound.
 * 
 * @param value Value to be clamped
 * @param lowerBound Lower bound
 * @param upperBound Upper bound
 * 
 * @retval Clamped value
 */
float clamp(float input, float lowerBound, float upperBound) {
  return input < lowerBound
    ? lowerBound
    : input > upperBound
      ? upperBound
      : input;
}

/**
 * @brief Maps a value from one range to another.
 * 
 * @param value Value to be mapped
 * @param fromLower Lower bound of the input range
 * @param fromUpper Upper bound of the input range
 * @param toLower Lower bound of the output range
 * @param toUpper Upper bound of the output range
 * 
 * @retval Mapped value
 * 
 * @ref https://docs.arduino.cc/language-reference/en/functions/math/map/
 */
float map(float value, float fromLower, float fromUpper, float toLower, float toUpper) {
  return (value - fromLower) * (toUpper - toLower) / (fromUpper - fromLower) + toLower;
}

#endif // CLI_H

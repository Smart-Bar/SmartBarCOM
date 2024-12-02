
#ifndef CLI_H
#define CLI_H

#include <stdlib.h>
#include <string.h>

// Length of the receive buffer
#define RX_BUFFER_SIZE                24

// Length of the circular buffer
#define CIRC_BUFFER_SIZE              16

// Length of the command vector
#define CMD_VCTR_SIZE                 8

// Length of the command parameters
#define ARG_LENGTH                    4

// Minimum duty cycle for the servo motor (2.5% or 0°)
#define SERVO_DUTY_CYCLE_LOWER_BOUND  2.5

// Maximum duty cycle for the servo motor (12.5% or 180°)
#define SERVO_DUTY_CYCLE_UPPER_BOUND  12.5

// Threshold for the PID controller
#define PID_THRESHOLD                 0.90

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

double target = 0.0;
double input = 0.0;
double output = 0.0;
volatile long encoderPos;
double pulsesPerRev = 580.0;
uint32_t interval = 20000;

double Kp = 0.55273;
double Ki = 0.30508;
double Kd = 0.05222;

double lastInput = 0.0;
double integral = 0.0;
double integralMax = 100.0;
double integralMin = -100.0;
uint32_t lastTime = 0;
uint8_t motorSpeed = 0;

extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim15;
extern TIM_HandleTypeDef htim16;
extern TIM_HandleTypeDef htim17;

/* Function protoypes */

void ParseCommand(void);
void ExecutePWM(CommandVector*);
void ExecutePID(CommandVector *);
void SelectInputChannel(uint8_t);
void SelectOutputChannel(uint8_t);
void ComputePID(void);
void setPWMPulse(int, float);
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
  float time = -1;

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
    }
    else if (strcmp(cmdVctr->args[i].name, "-t") == 0) {
      time = atof(cmdVctr->args[i].value);
    }
  }

  // Select the MUX input channel in zero-index notation
  SelectInputChannel((uint8_t) inputChnl - 1);

  // Select the DMux output channel in zero-index notation
  SelectOutputChannel((uint8_t) outputChnl - 1);

  // Calculate the pulse width and set the duty cycle of the signal
  setPWMPulse(inputChnl - 1, dutyCycle);

  // Timer for the PWM signal
  if (time > 0) {
    uint32_t target = time * 1000000;
    uint32_t lastTime = __HAL_TIM_GET_COUNTER(&htim16);
    uint32_t previousCounter = lastTime;
    uint32_t overflow = 0;
    uint32_t elapsed = 0;

    // Wait until the target time is reached
    while (elapsed <= target) {
      uint32_t currentCounter = __HAL_TIM_GET_COUNTER(&htim16);

      if (currentCounter < previousCounter) {
        overflow++;
      }

      // Calculate the elapsed time by adding the overflow (65535 ticks per overflow)
      // to the current counter value and subtracting the last time
      elapsed = (overflow * 0x10000 + currentCounter - lastTime);
      previousCounter = currentCounter;
    }

    // Stop the PWM signal
    setPWMPulse(inputChnl - 1, 0.0);
  }
}

/**
 * @brief Executes the PID command.
 * 
 * This function executes the instruction to move the axis to a specific position, as the number
 * of revolutions to be made by the motor.
 * 
 * @param cmdVctr 
 */
void ExecutePID(CommandVector *cmdVctr) {
  target = atof(cmdVctr->args[0].value) * pulsesPerRev;
  uint32_t lastTime = __HAL_TIM_GET_COUNTER(&htim16);

  while (encoderPos != (target * PID_THRESHOLD)) {
    uint32_t now = __HAL_TIM_GET_COUNTER(&htim16);
    uint32_t elapsed = now >= lastTime ? (now - lastTime) : ((0xFFFF - lastTime) + now + 1);

    if (elapsed >= interval) {
      lastTime = now;
      input = encoderPos;

      ComputePID();

      if (output > 0) {
        HAL_GPIO_WritePin(PH1_GPIO_Port, PH1_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(PH2_GPIO_Port, PH2_Pin, GPIO_PIN_RESET);
        motorSpeed = output;
      } else {
        HAL_GPIO_WritePin(PH1_GPIO_Port, PH1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(PH2_GPIO_Port, PH2_Pin, GPIO_PIN_SET);
        motorSpeed = -output;
      }

      motorSpeed = clamp(motorSpeed, 0, 255);
      __HAL_TIM_SET_COMPARE(&htim17, TIM_CHANNEL_1, motorSpeed);  // Ajusta el PWM
    }
  }

  HAL_GPIO_WritePin(PH1_GPIO_Port, PH1_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(PH2_GPIO_Port, PH2_Pin, GPIO_PIN_RESET);
  __HAL_TIM_SET_COMPARE(&htim17, TIM_CHANNEL_1, 0);
}

void ComputePID() {
  uint32_t now = __HAL_TIM_GET_COUNTER(&htim16);
  uint32_t deltaMicros = now - lastTime;
  lastTime = now;

  double deltaTime = deltaMicros / 1000000.0;
  double error = target - input;
  integral += error * deltaTime;

  if (integral > integralMax) integral = integralMax;
  if (integral < integralMin) integral = integralMin;

  double derivative = (input - lastInput) / deltaTime;

  output = Kp * error + Ki * integral + Kd * derivative;

  if (output > 255) output = 255;
  if (output < -255) output = -255;

  lastInput = input;
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

/**
 * @brief Sets the PWM pulse width.
 * 
 * This function sets the pulse width of the PWM signal and adjusts the duty cycle of the signal
 * based on the input channel and the desired duty cycle.
 * 
 * @param inputChannel 
 * @param dutyCycle 
 * @return float 
 */
void setPWMPulse(int inputChannel, float dutyCycle) {
  // Maps the duty cycle from a percentage to a value between 2.5 and 12.5, corresponding to 0° and 180°
  // in the servo motor
  if (inputChannel == 0) {
    dutyCycle = map(dutyCycle, 0, 100, SERVO_DUTY_CYCLE_LOWER_BOUND, SERVO_DUTY_CYCLE_UPPER_BOUND);
  }

  uint8_t pulse = (uint8_t)((dutyCycle / 100.0) * 255);

  // Selectes the proper timer and channel to set the duty cycle of the signal
  if (inputChannel == 0) {
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, pulse);
  } else if (inputChannel == 1) {
    __HAL_TIM_SET_COMPARE(&htim15, TIM_CHANNEL_1, pulse);
  }
}

#endif // CLI_H

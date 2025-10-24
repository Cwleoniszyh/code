/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "i2c.h"
#include "rtc.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "mpu6500.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* USER CODE BEGIN PV */

// Global variable declaration
uint8_t rx_buffer[1];           // USART1 receive buffer
uint8_t led_state = 0;          // LED state: 0-off, 1-on
unsigned int speed = 300;       // Motor speed (for open-loop control)
uint16_t rx_len = 0;            // Length of received data
uint8_t rx_complete = 0;        // Receive completion flag
unsigned char buffer[100];      // General-purpose buffer
///////////////////////////////////////////////////////
#define USART3_RX_BUFFER_SIZE 1024  // USART3 receive buffer size

#include <stdio.h>
#include <string.h>

// Laser radar related variables
static uint8_t uart3_rx_con = 0;       // Receive counter
static uint8_t uart3_rx_chksum;        // Checksum
static uint8_t uart3_rx_buf[100];      // Receive buffer
static uint8_t uart3_tx_buf[10];       // Transmit buffer
LaserPointTypeDef ax_ls_point[250];    // Laser radar data for one full scan
uint8_t uart3_rx_idx = 0;

// Laser radar A/B point data
uint16_t a_angle = 0;    // Angle of point A (around 0 degrees)
uint16_t a_distance = 0; // Distance of point A
uint16_t b_angle = 0;    // Angle of point B (around 90 degrees)
uint16_t b_distance = 0; // Distance of point B

// Encoder and timer related variables
unsigned int encoder_L_count = 0;
unsigned int encoder_R_count = 0;
unsigned int ms_count = 0;
unsigned int ms_200_count = 0;
unsigned int ms_200 = 0;
unsigned int s_time = 0;

// MPU6500 attitude angles (extern declaration)
extern float angle_pitch;
extern float angle_roll;
extern float angle_yaw;

// ------------------- Added: Closed-loop control related variables -------------------
#define PID_SPEED_KP 2.0f    // Speed PID proportional gain (needs tuning)
#define PID_SPEED_KI 0.5f    // Speed PID integral gain (needs tuning)
#define PID_SPEED_KD 0.1f    // Speed PID derivative gain (needs tuning)
#define TARGET_SPEED 500     // Target speed (encoder counts/second, needs tuning)

// Intermediate variables for left and right motor PID calculation
float left_pid_error;    // Current error of left motor (target - actual)
float left_pid_integral; // Integral term of left motor
float left_pid_deriv;    // Derivative term of left motor
float left_pid_last_err; // Last error of left motor
float left_pwm_output;   // Final PWM output of left motor

float right_pid_error;   // Current error of right motor (target - actual)
float right_pid_integral;// Integral term of right motor
float right_pid_deriv;   // Derivative term of right motor
float right_pid_last_err;// Last error of right motor
float right_pwm_output;  // Final PWM output of right motor

// Auxiliary variables for speed calculation
uint32_t left_encoder_last = 0;  // Last count of left encoder
uint32_t right_encoder_last = 0; // Last count of right encoder
float left_actual_speed = 0;     // Actual speed of left motor (counts/second)
float right_actual_speed = 0;    // Actual speed of right motor (counts/second)



// Direction control enumeration
typedef enum {
    STOP,
    FORWARD,
    BACKWARD,
    LEFT,
    RIGHT
} Direction_TypeDef;

Direction_TypeDef current_dir = STOP;  // Current movement direction

void PID_Reset(void)
{
    left_pid_integral = 0;
    right_pid_integral = 0;
    left_pid_last_err = 0;
    right_pid_last_err = 0;
    left_encoder_last = __HAL_TIM_GET_COUNTER(&htim3);
    right_encoder_last = __HAL_TIM_GET_COUNTER(&htim4);
}
///////////////////////////////////////////////////////

/* USER CODE END PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

// Function declarations

void LED_Control(uint8_t state);
void Motor_Control(uint32_t left_pwm_1, uint32_t left_pwm_2, uint32_t right_pwm_1, uint32_t right_pwm_2);

void AX_LASER_Start(void);   
void AX_LASER_Stop(void);
void LS_DataHandle(void);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


// Laser radar start command
void AX_LASER_Start(void)   
{
  uint8_t i;  
  
  uart3_tx_buf[0] = 0xA5;  // Frame header
  uart3_tx_buf[1] = 0x82;  // Start scanning command
  uart3_tx_buf[2] = 0x05;    
  uart3_tx_buf[3] = 0x00;    
  uart3_tx_buf[4] = 0x00;    
  uart3_tx_buf[5] = 0x00;    
  uart3_tx_buf[6] = 0x00;    
  uart3_tx_buf[7] = 0x00;    
  uart3_tx_buf[8] = 0x22;  // Checksum
  
  // Send start command via USART3
  for(i = 0; i < 9; i++)
  {
    HAL_UART_Transmit(&huart3, &uart3_tx_buf[i], 1, 100);
  }

  HAL_Delay(1000);
}




// Laser radar stop command
void AX_LASER_Stop(void)   
{
  uint8_t i;  

  uart3_tx_buf[0] = 0xA5;       // Frame header
  uart3_tx_buf[1] = 0x25;       // Stop command
  uart3_tx_buf[2] = 0xA5 + 0x25;  // Checksum
  
  // Send stop command via USART3
  for(i = 0; i < 3; i++)
  {
    HAL_UART_Transmit(&huart3, &uart3_tx_buf[i], 1, 100);
  }
}




// Laser radar data processing (maintain original logic)
void LS_DataHandle(void)
{
  uint8_t i;
  float temp;
  static uint16_t cnt = 0;
  static float angle_last = 0;
  static LaserPointTypeDef point[250];
  float angle_new = (((uint16_t)((uart3_rx_buf[3] & 0x7F) << 8)) + uart3_rx_buf[2]) / 64.0;
  float angle_area;

  if(angle_new > angle_last)
  {
    angle_area = (angle_new - angle_last) / 20;
    for(i = 0; i < 20; i++)
    {
      temp = angle_new + angle_area * i;
      point[cnt + i].angle = (temp > 360) ? (temp - 360) * 100 : temp * 100;
      point[cnt + i].distance = ((uint16_t)(uart3_rx_buf[5 + i * 4] << 8)) + uart3_rx_buf[4 + i * 4];
    }
  }
  else
  {
    angle_area = (angle_new + 360 - angle_last) / 20;
    for(i = 0; i < 20; i++)
    {
      temp = angle_new + angle_area * i;
      point[cnt + i].angle = (temp > 360) ? (temp - 360) * 100 : temp * 100;
      point[cnt + i].distance = ((uint16_t)(uart3_rx_buf[5 + i * 4] << 8)) + uart3_rx_buf[4 + i * 4];
    }
  }

  angle_last = angle_new;
  cnt += 20;

  // Complete data collection for one full circle
  if(cnt > 260)
  {
    memcpy(ax_ls_point, point, sizeof(point));
    cnt = 0;
  }
}


/**
  * @brief  Read and print the current time and date of RTC
  * @param  None
  * @retval None
  */
void PrintDateTime(void)
{
    RTC_TimeTypeDef sTime;
    RTC_DateTypeDef sDate;
    char datetime_str[64];
    
    // Get time (must get time first, then date)
    HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
    // Get date
    HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);
    
    // Format datetime string
    sprintf(datetime_str, "Date: 20%02d-%02d-%02d Time: %02d:%02d:%02d\r\n",
            sDate.Year, sDate.Month, sDate.Date,
            sTime.Hours, sTime.Minutes, sTime.Seconds);
    
    // Transmit via USART2
    HAL_UART_Transmit(&huart1, (uint8_t *)datetime_str, strlen(datetime_str), HAL_MAX_DELAY);
}


/**
  * @brief  LED control function
  * @param  state: LED state (0-off, 1-on)
  * @retval None
  */
void LED_Control(uint8_t state)
{
    if(state)
    {
        led_state = 1;
        uint8_t msg[] = "LED on\r\n";
        HAL_UART_Transmit(&huart1, msg, sizeof(msg)-1, 0xFFFF);
    }
    else
    {
        led_state = 0;
        uint8_t msg[] = "LED off\r\n";
        HAL_UART_Transmit(&huart1, msg, sizeof(msg)-1, 0xFFFF);
    }
}
// ------------------- Motor control (open-loop) -------------------
/**
  * @brief  Motor control function
  * @param  left_pwm_1:  PWM1 value of left motor
  * @param  left_pwm_2:  PWM2 value of left motor
  * @param  right_pwm_1: PWM1 value of right motor
  * @param  right_pwm_2: PWM2 value of right motor
  * @retval None
  */
void Motor_Control(uint32_t left_pwm_1, uint32_t left_pwm_2, uint32_t right_pwm_1, uint32_t right_pwm_2)
{
    // Limit PWM value range, assuming 0-1000
    if(left_pwm_1 > 1000) left_pwm_1 = 900;
    if(left_pwm_2 > 1000) left_pwm_2 = 900;
    if(right_pwm_1 > 1000) right_pwm_1 = 900;
    if(right_pwm_2 > 1000) right_pwm_2 = 900;

    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, left_pwm_1);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, left_pwm_2);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, right_pwm_1);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, right_pwm_2);
}



// ------------------- Core functions for closed-loop control -------------------
// 1. Calculate actual speed of left and right motors (encoder counts/second)
void Calculate_Actual_Speed(void)
{
    uint32_t left_encoder_now = __HAL_TIM_GET_COUNTER(&htim3);
    uint32_t right_encoder_now = __HAL_TIM_GET_COUNTER(&htim4);
    
    // Handle encoder count overflow (32-bit counter)
    if (left_encoder_now >= left_encoder_last)
    {
        left_actual_speed = left_encoder_now - left_encoder_last;
    }
    else
    {
        left_actual_speed = left_encoder_now + (0xFFFFFFFF - left_encoder_last);
    }
    
    if (right_encoder_now >= right_encoder_last)
    {
        right_actual_speed = right_encoder_now - right_encoder_last;
    }
    else
    {
        right_actual_speed = right_encoder_now + (0xFFFFFFFF - right_encoder_last);
    }
    
    // Update last count
    left_encoder_last = left_encoder_now;
    right_encoder_last = right_encoder_now;
    
    // Print actual speed via UART (for debugging)
    sprintf(buffer, "Left Speed:%0.0f, Right Speed:%0.0f\r\n", left_actual_speed, right_actual_speed);
    HAL_UART_Transmit(&huart1, (uint8_t *)buffer, strlen(buffer), HAL_MAX_DELAY);
}
float PID_Controller(float target, float actual, float kp, float ki, float kd, 
                     float *integral, float *last_err)
{
    float error = target - actual;  // Current error
    float output;
    
    // Proportional term
    float p_out = kp * error;
    
    // Integral term (limit integral saturation)
    *integral += error;
    if (*integral > 1000) *integral = 1000;
    if (*integral < -1000) *integral = -1000;
    float i_out = ki * (*integral);
    
    // Derivative term
    float deriv = error - *last_err;
    float d_out = kd * deriv;
    
    // Total output (limit PWM range, support positive and negative directions)
    output = p_out + i_out + d_out;
    if (output > 900) output = 900;
    if (output < -900) output = -900;  // Negative output represents reverse direction
    
    // Update last error
    *last_err = error;
    
    return output;
}

void Vehicle_ClosedLoop_Control(void)
{
    // 1. Calculate current actual speed (encoder difference)
    Calculate_Actual_Speed();
    
    // 2. Set target speeds for left and right motors according to current direction
    float left_target = 0, right_target = 0;
    switch(current_dir)
    {
        case FORWARD:
            left_target = TARGET_SPEED;   // Forward: left and right motors rotate forward in the same direction
            right_target = TARGET_SPEED;
            break;
        case BACKWARD:
            left_target = -TARGET_SPEED;  // Backward: left and right motors rotate backward in the same direction
            right_target = -TARGET_SPEED;
            break;
        case LEFT:
            left_target = -TARGET_SPEED/2;  // Left turn: left motor reverses, right motor forward (differential steering)
            right_target = TARGET_SPEED/2;
            break;
        case RIGHT:
            left_target = TARGET_SPEED/2;   // Right turn: left motor forward, right motor reverses (differential steering)
            right_target = -TARGET_SPEED/2;
            break;
        case STOP:
        default:
            left_target = 0;
            right_target = 0;
            break;
    }
    
    // 3. Calculate PWM outputs for left and right motors (with direction)
    left_pwm_output = PID_Controller(left_target, left_actual_speed, 
                                     PID_SPEED_KP, PID_SPEED_KI, PID_SPEED_KD,
                                     &left_pid_integral, &left_pid_last_err);
    
    right_pwm_output = PID_Controller(right_target, right_actual_speed, 
                                      PID_SPEED_KP, PID_SPEED_KI, PID_SPEED_KD,
                                      &right_pid_integral, &right_pid_last_err);
    
    // 4. Set motor direction and duty cycle according to PWM positive/negative values
    // Left motor: positive output→PWM1 active, negative output→PWM2 active
    uint32_t left_pwm1 = (left_pwm_output > 0) ? (uint32_t)left_pwm_output : 0;
    uint32_t left_pwm2 = (left_pwm_output < 0) ? (uint32_t)(-left_pwm_output) : 0;
    
    // Right motor: positive output→PWM1 active, negative output→PWM2 active
    uint32_t right_pwm1 = (right_pwm_output > 0) ? (uint32_t)right_pwm_output : 0;
    uint32_t right_pwm2 = (right_pwm_output < 0) ? (uint32_t)(-right_pwm_output) : 0;
    
    // 5. Drive the motors
    Motor_Control(left_pwm1, left_pwm2, right_pwm1, right_pwm2);
}
///////////////////////////////////////////////
/**
  * @brief  UART receive completion callback function
  * @param  huart: Pointer to UART handle
  * @retval None
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart == &huart1)
    {
        HAL_UART_Transmit(&huart1, rx_buffer, 1, 0xFFFF);
        
        // Control LED and motor according to received character
        switch(rx_buffer[0])
        {
            case '1':
                LED_Control(1);  // Turn on LED
                break;
            case '0':
                LED_Control(0);  // Turn off LED
                break;
            case 'F':
                sprintf(buffer, "forward\r\n");
                HAL_UART_Transmit(&huart1, (uint8_t *)buffer, strlen(buffer), HAL_MAX_DELAY);
                Motor_Control(speed, 0, speed, 0);
                break;
            case 'B':
                sprintf(buffer, "back\r\n");
                HAL_UART_Transmit(&huart1, (uint8_t *)buffer, strlen(buffer), HAL_MAX_DELAY);
                Motor_Control(0, speed, 0, speed);
                break;
            case 'L':
                sprintf(buffer, "left\r\n");
                HAL_UART_Transmit(&huart1, (uint8_t *)buffer, strlen(buffer), HAL_MAX_DELAY);
                Motor_Control(speed, 0, 0, speed);
                break;
            case 'R':
                sprintf(buffer, "right\r\n");
                HAL_UART_Transmit(&huart1, (uint8_t *)buffer, strlen(buffer), HAL_MAX_DELAY);
                Motor_Control(0, speed, speed, 0);
                break;
            case '+':
                speed += 100;
                if(speed > 900) speed = 900;
                sprintf(buffer, "speed:%d\r\n", speed);
                HAL_UART_Transmit(&huart1, (uint8_t *)buffer, strlen(buffer), HAL_MAX_DELAY);
                break;
            case '-':
                speed -= 100;
                if(speed < 0) speed = 0;
                sprintf(buffer, "speed:%d\r\n", speed);
                HAL_UART_Transmit(&huart1, (uint8_t *)buffer, strlen(buffer), HAL_MAX_DELAY);
                break;
            case 'S':
                sprintf(buffer, "stop\r\n");
                HAL_UART_Transmit(&huart1, (uint8_t *)buffer, strlen(buffer), HAL_MAX_DELAY);
                Motor_Control(0, 0, 0, 0);
                break;        
 
						
            // Added closed-loop control commands
            case 'c':  // Closed-loop forward (lowercase c to distinguish from open-loop)
                sprintf(buffer, "Closed-loop forward (Target:%.0f)\r\n", TARGET_SPEED);
                HAL_UART_Transmit(&huart1, (uint8_t *)buffer, strlen(buffer), HAL_MAX_DELAY);
                current_dir = FORWARD;
                PID_Reset();  // Reset PID parameters
                break;
            case 'b':  // Closed-loop backward (lowercase b)
                sprintf(buffer, "Closed-loop backward (Target:%.0f)\r\n", TARGET_SPEED);
                HAL_UART_Transmit(&huart1, (uint8_t *)buffer, strlen(buffer), HAL_MAX_DELAY);
                current_dir = BACKWARD;
                PID_Reset();
                break;
            case 'l':  // Closed-loop left turn (lowercase l)
                sprintf(buffer, "Closed-loop left (Target:%.0f)\r\n", TARGET_SPEED);
                HAL_UART_Transmit(&huart1, (uint8_t *)buffer, strlen(buffer), HAL_MAX_DELAY);
                current_dir = LEFT;
                PID_Reset();
                break;
            case 'r':  // Closed-loop right turn (lowercase r)
                sprintf(buffer, "Closed-loop right (Target:%.0f)\r\n", TARGET_SPEED);
                HAL_UART_Transmit(&huart1, (uint8_t *)buffer, strlen(buffer), HAL_MAX_DELAY);
                current_dir = RIGHT;
                PID_Reset();
                break;
            default:
                // Unknown command
                uint8_t msg[] = "status: unknown command\r\n";
                HAL_UART_Transmit(&huart1, msg, sizeof(msg)-1, 0xFFFF);
                break;
        }
        
        // Restart UART receive interrupt
        HAL_UART_Receive_IT(&huart1, rx_buffer, 1);
    }
    
  if (huart->Instance == USART3)
  {
    uint8_t Res = uart3_rx_buf[uart3_rx_idx];
    uint8_t temp;
    
    // Data parsing logic (maintain original protocol handling)
    if (uart3_rx_con < 3)
    {
      if(uart3_rx_con == 0)  // Frame header 1
      {
        if((Res >> 4) == LS_HEADER1)
        {
          uart3_rx_buf[uart3_rx_con] = Res;
          uart3_rx_con = 1;					
        }
      }
      else if(uart3_rx_con == 1)  // Frame header 2
      {
        if((Res >> 4) == LS_HEADER2)
        {
          uart3_rx_buf[uart3_rx_con] = Res;
          uart3_rx_con = 2;					
        }
        else
        {
          uart3_rx_con = 0;						
        }				
      }
      else  // Receive first data
      {
        uart3_rx_buf[uart3_rx_con] = Res;
        uart3_rx_con = 3;
        uart3_rx_chksum = Res;	
      }
    }			
    else  // Receive data body
    {
      if(uart3_rx_con < (LS_F_LEN - 1))
      {
        uart3_rx_buf[uart3_rx_con] = Res;
        uart3_rx_con++;
        uart3_rx_chksum ^= Res;
      }
      else
      {
        // Receive last data
        uart3_rx_buf[uart3_rx_con] = Res;
        uart3_rx_chksum ^= Res;
        
        // Checksum verification
        temp = ((uint8_t)(uart3_rx_buf[1] << 4)) + (uint8_t)(uart3_rx_buf[0] & 0x0F);
        if(uart3_rx_chksum == temp)
        {
          LS_DataHandle();  // Process valid data frame
        }
        
        uart3_rx_con = 0;  // Reset receive counter
      }
    }

    // Continue receiving next byte
    uart3_rx_idx = (uart3_rx_idx + 1) % 100;
    HAL_UART_Receive_IT(&huart3, &uart3_rx_buf[uart3_rx_idx], 1);
  }
}



/* TIM6 update interrupt callback function */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM6)
  {
    // Code executed every 1 second
    ms_count++;
    if(ms_count >= 1000)
    {
      ms_count = 0;

			sprintf(buffer, "Encoder L Count: %d, Encoder R Count: %d\r\n", encoder_R_count, encoder_L_count);
			HAL_UART_Transmit(&huart1, (uint8_t *)buffer, strlen(buffer), HAL_MAX_DELAY);
      // RTC time display can be added here
      PrintDateTime(); // Print current time and date
      // Read sensor data
      if (MPU6500_ReadData() == HAL_OK) {
        // Successfully read data, attitude angles can be used here
        // angle_yaw, angle_pitch, angle_roll global variables have been updated
        
        // Example: Print angle data via UART
        sprintf(buffer, "Yaw:%.2f, Pitch:%.2f, Roll:%.2f\r\n", 
              angle_yaw, angle_pitch, angle_roll);
        HAL_UART_Transmit(&huart1, (uint8_t *)buffer, strlen(buffer), HAL_MAX_DELAY);
      }
    }


    ms_200_count++;
    if(ms_200_count >= 200)
    {
      ms_200_count = 0;
		// Output information
    sprintf(buffer, "A:%d D:%05d B:%d D:%05d \r\n",a_angle, a_distance,b_angle,b_distance);
    HAL_UART_Transmit(&huart1, (uint8_t *)buffer, strlen(buffer), HAL_MAX_DELAY);
 
      if (current_dir != STOP)  // Execute only in closed-loop mode
      {
        Vehicle_ClosedLoop_Control();
      }

    }
    
  }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_RTC_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM2_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */
  /* Start TIM6 timer interrupt */
  if (HAL_TIM_Base_Start_IT(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
    /* Start UART receive interrupt, receive 1 byte each time */
    HAL_UART_Receive_IT(&huart1, rx_buffer, 1);
    /* Start DMA reception for UART3, using circular mode */
  /* Start interrupt reception for UART3, ready to receive data */
		HAL_UART_Receive_DMA(&huart3, (uint8_t *)uart3_rx_buf, 100);
		
		AX_LASER_Start();

    HAL_UART_Transmit(&huart1, (uint8_t *)"Hello World!\r\n", 15, HAL_MAX_DELAY);
		HAL_UART_Transmit(&huart2, (uint8_t *)"Hello World!\r\n", 15, HAL_MAX_DELAY);


    // Initialize encoders
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_1);
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_2);
    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_1);
    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_2);

    // Initialize PWM
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);

    // Initialize to stop state
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 0);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 0);

    // Initialize MPU6500
    if (MPU6500_Init() != HAL_OK) {
      // Initialization failure handling, can light up error indicator, etc.
      Error_Handler();
    }
    
    // Calibrate MPU6500 (keep the sensor stationary during calibration)
    if (MPU6500_Calibrate() != HAL_OK) {
      Error_Handler();
    }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    while (1)
    {
        // Encoder count reading and display can be added here
			encoder_L_count = __HAL_TIM_GET_COUNTER(&htim3);
			encoder_R_count = __HAL_TIM_GET_COUNTER(&htim4);

			
			
    ///////////////////////// Traverse all angles and distances ///////////////////////
  //  for (uint8_t i = 0; i < 250; i++)
  //     {
  //       if (ax_ls_point[i].distance > 0)  // Only send valid data
  //       {
  //         char buf[32];
  //         sprintf(buf, "Angle:%d, Distance:%d\r\n", 
  //                 ax_ls_point[i].angle/100, 
  //                 ax_ls_point[i].distance);
  //         HAL_UART_Transmit(&huart2, (uint8_t*)buf, strlen(buf), 100);
  //       }
  //     }			
			


    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
// Other custom functions can be placed here
/* USER CODE END 4 */

/**
  * @brief  This function is executed when an error occurs
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */
    __disable_irq();
    while (1)
    {
    }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line number,
       ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
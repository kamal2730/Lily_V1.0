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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdlib.h"
#include "math.h"
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct __attribute__((packed)) {
  char      header_start;         // Should be '<'
  char      header_end;           // Should be '>'
  float     position;
  uint16_t  sensor_values[9];     // Your GUI is configured for 9 sensors
  uint8_t   status_code;
  float     kp;
  float     ki;
  float     kd;
  uint16_t  threshold;
  uint8_t   base_speed;
  float 	pid_error;
  float 	pid_output;
} TelemetryPacket;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;

/* USER CODE BEGIN PV */

//SENSOR READING FUNCTION
volatile int SensorIndex=0;

GPIO_TypeDef* IR_LED_PORTS[9] = { GPIOB, GPIOB, GPIOB, GPIOB, GPIOA, GPIOA, GPIOB, GPIOB, GPIOB};
uint32_t IR_LED_PINS[9] = {GPIO_PIN_12, GPIO_PIN_13, GPIO_PIN_14, GPIO_PIN_15, GPIO_PIN_12, GPIO_PIN_15, GPIO_PIN_3, GPIO_PIN_4, GPIO_PIN_5};
uint32_t SensorAdcChannel[9] = {
    ADC_CHANNEL_8, // sensor 0 → PB12
    ADC_CHANNEL_7, // sensor 1 → PB13
    ADC_CHANNEL_6, // sensor 2 → PB14
    ADC_CHANNEL_5, // sensor 3 → PB15
    ADC_CHANNEL_4, // sensor 4 → PA12
    ADC_CHANNEL_3, // sensor 5 → PA15
    ADC_CHANNEL_2, // sensor 6 → PB3
    ADC_CHANNEL_1, // sensor 7 → PA4
    ADC_CHANNEL_0, // sensor 8 → PB5
};
volatile int ADC_DONE=0;
volatile uint32_t adc_buf=0;
volatile uint32_t IR_ON[9];
volatile uint32_t IR_OFF[9];
volatile uint32_t SensorValues[9];
volatile uint32_t sensors_time;
volatile uint32_t run_time;
volatile uint32_t LastPIDTime;
volatile uint32_t signal_runtime[9];
uint32_t max_adc[9] = {4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095,4095};

//PID Variables
int thresh=1300;
int weights[9]={-40,-30,-20,-10,0,10,20,30,40};
double Kp = 2.0f, Ki = 0.0f, Kd = 0.7f;
int position,error;
int setpoint=0;
uint8_t base_speed = 70;
uint8_t turn_speed = 70;
int turn=1;

double P, I, D;
double correction = 0, lastInput = 0;
double lastTime = 0;
double integralMin = -25.0, integralMax = 25.0;
int threshold[9]={1000,1000,1000,1000,1000,1000,1000,1000,1000};

const int CHANGE_THRESHOLD[9]={700,700,700,700,700,700,700,700,700};
uint32_t prevValues[9]; // last ADC readings
uint8_t lastColor[9];   // 0 = black, 1 = white
int firstTime=1;
const uint32_t INTERVAL_MS = 5;
uint32_t last_update_time = 0;

//WIFI
volatile uint8_t status_to_send = 0;
#define RX_BUFFER_SIZE 32
uint8_t rx_buffer[RX_BUFFER_SIZE];
uint32_t last_telemetry_time = 0; // Stores the last time we sent data
const uint32_t TELEMETRY_INTERVAL_MS = 20;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
void ReadSensors();
void delay_us(uint32_t us);
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc);
int line_data(void);
void computePID(int32_t input);
void setMotorSpeed(uint8_t motor, int32_t speed);
void storePrevValues(void);
void detectColor(void);
void handle_received_command(uint8_t* buffer, uint16_t len) ;
void send_telemetry_data(float current_position,float pid_out) ;
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size);
void handle_received_bluetooth_command(uint8_t* buffer, uint16_t len);
void sendok();
void sendno();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void delay_us(uint32_t us){
    uint32_t start = __HAL_TIM_GET_COUNTER(&htim2);
    while ((__HAL_TIM_GET_COUNTER(&htim2) - start) < us);
}
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc){
	if(hadc -> Instance == ADC1){
		ADC_DONE = 1;
	}

}
void ReadSensors(void)
{
	   int sensorIndex = 0;


	   ADC_ChannelConfTypeDef sConfig; // sconfig to change channels
	   //ADC_Config.SamplingTime=ADC_SAMPLETIME_480CYCLES;

	   for(sensorIndex =0; sensorIndex<(9) ; sensorIndex++)
	   {
		   sConfig.Channel = SensorAdcChannel[sensorIndex]; // to update channels
		   sConfig.Rank = 1; //ADC_REGULAR_RANK_1;
		   HAL_ADC_ConfigChannel(&hadc1, &sConfig);


	 		//ir led on
 		HAL_GPIO_WritePin(IR_LED_PORTS[sensorIndex], IR_LED_PINS[sensorIndex], SET);
		delay_us(200);
	 		 HAL_ADC_Stop_DMA(&hadc1);
	 		ADC_DONE = 0;

	 	   HAL_ADC_Start_DMA(&hadc1, (uint32_t *)&adc_buf, 1);

	 	   //callback will be triggered after conversion
	 	   // wait until conversion is complete
	 	   while (ADC_DONE == 0);

	 	  signal_runtime[sensorIndex]=adc_buf;
	 	 // SensorValues[sensorIndex]= (max_adc[sensorIndex] - signal_runtime[sensorIndex])/10;


		   if (__HAL_ADC_GET_FLAG(&hadc1, ADC_FLAG_OVR)) {
		       // Overflow happened
		       __HAL_ADC_CLEAR_FLAG(&hadc1, ADC_FLAG_OVR);  // clear it
		   }

		   HAL_GPIO_WritePin(IR_LED_PORTS[sensorIndex], IR_LED_PINS[sensorIndex], RESET);

	}

	//  print on oled - sensor data took

}
int line_data(void) {
    int sum = 0;
    double weighted_sum = 0;
    int onLine = 0;

    if (firstTime == 1) {
        firstTime++;

        for (int i = 0; i < 9; i++) {
            if (signal_runtime[i] > threshold[i]) {
                lastColor[i] = 0;
                weighted_sum += weights[i];
                sum++;
                onLine = 1;
            } else {
                lastColor[i] = 1;

            }
        }
        storePrevValues();
    }
    else {
        detectColor();

//        for (int i = 1; i < 8; i++) {
//            if (lastColor[i] == 0) { // Detected black
//                if (i == 1) {
//                    // Check edge and next neighbor
//                    if (lastColor[0] == 1 && lastColor[2] == 1) {
//                        lastColor[i] = 1; // Force to white
//                    }
//                }
//                else if (i == 7) {
//                    // Check previous neighbor and edge
//                    if (lastColor[6] == 1 && lastColor[8] == 1) {
//                        lastColor[i] = 1; // Force to white
//                    }
//                }
//                else {
//                    // General case for middle sensors
//                    if (lastColor[i - 1] == 1 && lastColor[i + 1] == 1) {
//                        lastColor[i] = 1; // Force to white
//                    }
//                }
//            }
//        }
        for (int
        		i = 0; i < 9; i++) {
            if (lastColor[i] == 0) {
                weighted_sum += weights[i];
                sum++;
                onLine = 1;
            }
        }
        storePrevValues();
    }

    if (!onLine) {
        return 255;  // Line lost condition
    }

    return (int)(weighted_sum / sum);
}
void computePID(int32_t input) {
	P = Kp * error;
	I += Ki * error * 5.0;
	if (I > integralMax) I = integralMax;
	if (I < integralMin) I = integralMin;
	D = Kd * (input - lastInput) / 5.0;

	correction = P + I + D;
	lastInput = input;
	LastPIDTime = HAL_GetTick();
	correction = floor(correction);
	setMotorSpeed(0, base_speed - correction);
	setMotorSpeed(1, base_speed + correction);

}
void setMotorSpeed(uint8_t motor, int32_t speed)
{
	// tim1 ch1- left front, ch2- left back, ch3-right front, ch4- right back
    uint16_t pwm = abs(speed);
    if (pwm > 200) pwm = 200;  // Limit max speed

    if (motor == 0) {  // Left motor
        if (speed < 0) {
            TIM1->CCR3 = pwm;
            TIM1->CCR4 = 0;
        } else {
            TIM1->CCR3 = 0;
            TIM1->CCR4 = pwm;
        }
    }
    else if (motor == 1) {  // Right motor
        if (speed < 0) {
            TIM1->CCR2 = pwm;
            TIM1->CCR1 = 0;
        } else {
            TIM1->CCR2 = 0;
            TIM1->CCR1 = pwm;
        }
    }
}
void storePrevValues(void)
{
    for (int i = 0; i < 9; i++) {
        prevValues[i] = signal_runtime[i];
    }
}
void detectColor(void)
{
    ReadSensors();
    for (int i = 0; i < 9; i++)
    {
        uint32_t current = signal_runtime[i];

        if (current < 200) {
            lastColor[i] = 1; // White
        }
        else if(current>1200){
        	lastColor[i]=0;
        }
        else if (current > prevValues[i] + CHANGE_THRESHOLD[i]) {
            lastColor[i] = 0; // Black
        }
        else if (current + CHANGE_THRESHOLD[i] < prevValues[i]) {
            lastColor[i] = 1; // White
        }
        // else → no change → keep lastColor[i] as is

        prevValues[i] = current; // update reading for next loop
    }
}
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance == USART1)
    {
        // Pass received bytes directly to your parser
//        handle_received_command(rx_buffer, Size);
        handle_received_bluetooth_command(rx_buffer, Size);

        // Restart reception for the next command
        HAL_UARTEx_ReceiveToIdle_IT(&huart1, rx_buffer, RX_BUFFER_SIZE);
    }
}
void send_telemetry_data(float current_position,float pid_out) {
    static TelemetryPacket packet;

    // 1. Set header
    packet.header_start = '<';
    packet.header_end = '>';

    // 2. Add the live position data
    packet.position = current_position;

    // 3. Fill sensor data
    for(int i = 0; i < 9; i++) {
        packet.sensor_values[i] = (uint16_t)signal_runtime[i];
    }

    // 4. Fill with LIVE robot state
    packet.status_code = status_to_send;
    packet.kp = Kp;
    packet.ki = Ki;
    packet.kd = Kd;
    packet.threshold = thresh;
    packet.base_speed = (uint8_t)base_speed;
    packet.pid_error = -current_position;
    packet.pid_output = pid_out;

    // 5. Transmit the packet
    HAL_UART_Transmit(&huart1, (uint8_t*)&packet, sizeof(TelemetryPacket), 100);

    // 6. Reset the status code after sending
    status_to_send = 0;
}
void handle_received_command(uint8_t* buffer, uint16_t len) {
  // Create a local, null-terminated copy to work with safely.
  char cmd_string[len + 1];
  memcpy(cmd_string, buffer, len);
  cmd_string[len] = '\0';

  // Find the separator character ':'
  char* colon_ptr = strchr(cmd_string, ':');

  // Check if the separator was found
  if (colon_ptr != NULL) {
    *colon_ptr = '\0';
    char* key = cmd_string;
    char* value_str = colon_ptr + 1;
    float value = atof(value_str);
    if (strcmp(key, "KP") == 0) {
      Kp = value;
      status_to_send = 1;
    } else if (strcmp(key, "KI") == 0) {
      Ki = value;
      status_to_send = 1;
    } else if (strcmp(key, "KD") == 0) {
      Kd = value;
      status_to_send = 1;
    } else if (strcmp(key, "TH") == 0) {
      thresh = (uint16_t)value;
      status_to_send = 1;
    } else if (strcmp(key, "BS") == 0) {
      base_speed = (uint8_t)value; // Cast to uint8_t for consistency
      status_to_send = 1;
    } else {
      status_to_send = 200;
    }

  } else {
    status_to_send = 200;
  }
}
void handle_received_bluetooth_command(uint8_t* buffer, uint16_t len) {
    // Create a local, null-terminated copy
    char cmd_string[len + 1];
    memcpy(cmd_string, buffer, len);
    cmd_string[len] = '\0';

    // If empty, invalid
    if (len < 2) {
    	sendno();
        return;
    }

    // First character is the key, rest is the value
    char key = cmd_string[0];
    char* value_str = &cmd_string[1];
    float value = atof(value_str);

    switch (key) {
        case 'p': case 'P':
            Kp = value;
            sendok();
            break;
        case 'i': case 'I':
            Ki = value;
            sendok();
            break;
        case 'd': case 'D':
            Kd = value;
            sendok();
            break;
        case 'b': case 'B':
            base_speed = (uint8_t)value;
            sendok();
            break;
        case 't': case 'T':
            thresh = (uint16_t)value;
            sendok();
            break;
        default:
        	sendno();
            break;
    }
}

void sendok(){
	HAL_UART_Transmit(&huart1, (uint8_t *)"Updated !", strlen("Updated !"), 100);
}
void sendno(){
	HAL_UART_Transmit(&huart1, (uint8_t *)"Failed Updating !", strlen("Failed Updating !"), 100);
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
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start(&htim2);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);

  // Start PWM for motors
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
  HAL_UARTEx_ReceiveToIdle_IT(&huart1, rx_buffer, RX_BUFFER_SIZE);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  uint32_t start_time = HAL_GetTick();
	  uint32_t current_time = HAL_GetTick();
	  if((current_time-last_update_time)>=INTERVAL_MS){
		  last_update_time=HAL_GetTick();
	  ReadSensors();
	  sensors_time = HAL_GetTick()-start_time;

	  position=line_data();
	  if(position>0 && position!=255){turn=1;
	  }else if(position<0){turn=-1;}

	  uint32_t last_speed_update = HAL_GetTick();
	  turn_speed = 70; // initial

	  uint32_t last_loop_time = HAL_GetTick(); // ms timer start

	  while (position == 255) {
	      // Run loop contents only every 5 ms
	      if (HAL_GetTick() - last_loop_time >= 5) {
	          last_loop_time = HAL_GetTick();

	          // Control turning
	          if (turn == 1) { // Right turn
	              setMotorSpeed(0, turn_speed);
	              setMotorSpeed(1, -turn_speed);
	          } else if (turn == -1) { // Left turn
	              setMotorSpeed(0, -turn_speed);
	              setMotorSpeed(1, turn_speed);
	          }

	          // Gradually increase turn speed every 1 second up to 100
	          if (HAL_GetTick() - last_speed_update >= 1000) {
	              last_speed_update = HAL_GetTick();
	              if (turn_speed < 100) {
	                  turn_speed++;
	                  if (turn_speed > 100) turn_speed = 100;
	              }
	          }

	          // Sensor reading and position check
	          ReadSensors();
	          position = line_data();

	          // Extra condition check
	          if (position != 255) {
	              int pair_found = 0;
	              for (int i = 0; i < 8; i++) { // 0..7, check i and i+1
	                  if (lastColor[i] == 0 && lastColor[i + 1] == 0) {
	                      pair_found = 1;
	                      break;
	                  }
	              }
	              if (!pair_found) {
	                  position = 255; // No two adjacent black sensors → line lost
	              }
	          }
	      }
//	      while(HAL_GetTick()-last_telemetry_time>20){last_telemetry_time=HAL_GetTick();send_telemetry_data(position,correction);}
	  }


//	  if(position>0 && position!=255){turn=1;
//	  }else if(position<	0){turn=-1;}

	  error = -(position);
	  computePID(position);
	  }
	  run_time=HAL_GetTick()-start_time;
//	  while(HAL_GetTick()-last_telemetry_time>20){last_telemetry_time=HAL_GetTick();send_telemetry_data(position,correction);}
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};
  ADC_InjectionConfTypeDef sConfigInjected = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configures for the selected ADC injected channel its corresponding rank in the sequencer and its sample time
  */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_0;
  sConfigInjected.InjectedRank = 1;
  sConfigInjected.InjectedNbrOfConversion = 1;
  sConfigInjected.InjectedSamplingTime = ADC_SAMPLETIME_3CYCLES;
  sConfigInjected.ExternalTrigInjecConvEdge = ADC_EXTERNALTRIGINJECCONVEDGE_NONE;
  sConfigInjected.ExternalTrigInjecConv = ADC_INJECTED_SOFTWARE_START;
  sConfigInjected.AutoInjectedConv = DISABLE;
  sConfigInjected.InjectedDiscontinuousConvMode = DISABLE;
  sConfigInjected.InjectedOffset = 0;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 960-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 200-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 96-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_10|GPIO_PIN_12
                          |GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC13 PC14 PC15 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PB1 PB2 PB10 PB12
                           PB13 PB14 PB15 PB3
                           PB4 PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_10|GPIO_PIN_12
                          |GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PA12 PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
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

/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "can.h"
#include "crc.h"
#include "dma.h"
#include "eth.h"
#include "i2c.h"
#include "rng.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "cmsis_os.h"
#include "tcpserv.h"
#include "DataExchUART.h"
#include "DataExchCAN.h"
#include "DataExchETH.h"
#include "Common.h"
#include "INA226.h"
#include "ads867x.h"
#include "dacx0501.h"
#include "CRCCalc.h"

QueueHandle_t queue1, queue2, queue3;
// Глобальный массив для хранения измеренных данных
float adc_voltages[4] = {0}; // Значения напряжений с ADC
osMutexId_t adc_data_mutex; // Мьютекс для синхронизации доступа

#define DEBUG_RELAY_TEST

extern TIM_HandleTypeDef* BeepTim;

/* Public function prototypes. */
void SysTask_Init(void);
void DataExchUART_Init(void);
void DataExchCAN_Init(void);
void EthernetTask_Init(void);

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();
  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  //MX_ETH_Init();
  MX_USART3_UART_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_SPI6_Init();
  MX_SPI3_Init();
  MX_UART7_Init();
  MX_TIM4_Init();
  MX_CRC_Init();
  MX_RNG_Init();
  MX_CAN1_Init();
  MX_SPI4_Init();

  /* Start up beep. */

  /* Relay debug test. */
#ifdef DEBUG_RELAY_TEST
  HAL_GPIO_WritePin(PUMP_OUT_CTRL_GPIO_Port, PUMP_OUT_CTRL_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(HV_PUMP_LOCK_GPIO_Port, HV_PUMP_LOCK_Pin, GPIO_PIN_SET);
#endif

  	for (uint8_t i = 3; i > 0; i--) {
  		HAL_TIM_PWM_Start(BeepTim, TIM_CHANNEL_1);
  		HAL_Delay(150);
  		HAL_TIM_PWM_Stop(BeepTim, TIM_CHANNEL_1);
  		HAL_Delay(150);
  	}

#ifdef DEBUG_RELAY_TEST
  HAL_GPIO_WritePin(PUMP_OUT_CTRL_GPIO_Port, PUMP_OUT_CTRL_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(HV_PUMP_LOCK_GPIO_Port, HV_PUMP_LOCK_Pin, GPIO_PIN_RESET);
#endif

  	/* Call init function for freertos objects (in freertos.c) */
  	SysTask_Init();
  	DataExchUART_Init();
  	DataExchCAN_Init();
  	EthernetTask_Init();
  	osThreadDef(ADCTask, StartADCTask, osPriorityLow, 0, 128);
  	osThreadCreate(osThread(ADCTask), NULL);
  	tcp_server_init();

  	/* Start scheduler */
  	osKernelStart();

  	/* We should never get here as control is now taken by the scheduler */
  	while (1) {
  	}
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
	RCC_OscInitStruct.PLL.PLLM = 15;
	RCC_OscInitStruct.PLL.PLLN = 216;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 8;
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
	HAL_RCC_MCOConfig(RCC_MCO1, RCC_MCO1SOURCE_HSE, RCC_MCODIV_1);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM7 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM7) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
// My function in FreeRTOS
void StartADCTask(void *argument) {
    // Инициализация ADC
    ADS867x_GInst_t adc_device = {
        .delay = HAL_Delay,
        .spi_tx = ADS867x_SPI_Tx,
        .spi_rx = ADS867x_SPI_Rx
    };
    ADS867x_Init();

    // Количество устройств в daisy-chain
    uint8_t device_count = 4;
    float local_voltages[4] = {0};

    for (;;) {
        // Опрос каждого устройства
        for (uint8_t i = 0; i < device_count; i++) {
            local_voltages[i] = ADS867x_GetVoltage_DaisyChain(&adc_device, device_count, i);
        }

        // Сохранение данных в глобальный массив с использованием мьютекса
        if (osMutexAcquire(adc_data_mutex, osWaitForever) == osOK) {
            for (uint8_t i = 0; i < device_count; i++) {
                adc_voltages[i] = local_voltages[i];
            }
            osMutexRelease(adc_data_mutex);
        }
        // Можно добавить отправку через Ethernet?
        // Задержка для выполнения задачи с низким приоритетом нужна ли?
    }
}

// Глобальная переменная для хранения PCB сервера
static struct tcp_pcb *tcp_server_pcb;

// Буфер для отправки данных
#define SEND_BUFFER_SIZE 256
char send_buffer[SEND_BUFFER_SIZE];

void tcp_server_init(void) {
    ip_addr_t local_ip;
    IP_ADDR4(&local_ip, 192, 168, 0, 100); // Установка IP-адреса

    tcp_server_pcb = tcp_new();
    if (tcp_server_pcb == NULL) {
        printf("Ошибка создания TCP PCB.\n");
        return;
    }

    if (tcp_bind(tcp_server_pcb, &local_ip, TCP_SERVER_PORT) != ERR_OK) {
        printf("Ошибка привязки TCP PCB.\n");
        tcp_close(tcp_server_pcb);
        return;
    }

    tcp_server_pcb = tcp_listen(tcp_server_pcb);
    tcp_accept(tcp_server_pcb, tcp_server_accept);
    printf("TCP-сервер запущен. IP: 192.168.0.100, Порт: %d\n", TCP_SERVER_PORT);
}
static err_t tcp_server_accept(void *arg, struct tcp_pcb *newpcb, err_t err) {
    printf("Новое соединение принято.\n");

    // Назначить callback'и для соединения
    tcp_recv(newpcb, tcp_server_recv);
    tcp_err(newpcb, tcp_server_error);
    tcp_poll(newpcb, tcp_server_poll, 1);

    return ERR_OK;
}

static err_t tcp_server_recv(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err) {
    if (p == NULL) {
        printf("Соединение закрыто клиентом.\n");
        tcp_close(tpcb);
        return ERR_OK;
    }

    // Эхо-ответ клиенту
    tcp_write(tpcb, p->payload, p->len, TCP_WRITE_FLAG_COPY);
    pbuf_free(p);

    return ERR_OK;
}

static void tcp_server_error(void *arg, err_t err) {
    printf("Ошибка TCP-соединения: %d\n", err);
}

static err_t tcp_server_poll(void *arg, struct tcp_pcb *tpcb) {
    return ERR_OK;
}

//отправка данных ADC по Ethernet
void send_adc_results(struct tcp_pcb *tpcb) {
    // Локальный буфер для передачи
    char buffer[SEND_BUFFER_SIZE];
    int len;

    // Заблокировать доступ к данным
    if (osMutexAcquire(adc_data_mutex, osWaitForever) == osOK) {
        len = snprintf(buffer, SEND_BUFFER_SIZE,
            "ADC Voltages:\r\n1: %.2f V\r\n2: %.2f V\r\n3: %.2f V\r\n4: %.2f V\r\n",
            adc_voltages[0], adc_voltages[1], adc_voltages[2], adc_voltages[3]);
        osMutexRelease(adc_data_mutex);

        // Отправить данные через TCP
        if (tcp_write(tpcb, buffer, len, TCP_WRITE_FLAG_COPY) == ERR_OK) {
            tcp_output(tpcb);
            printf("Данные успешно отправлены.\n");
        } else {
            printf("Ошибка отправки данных.\n");
        }
    }
}
static err_t tcp_server_recv(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err) {
    if (p == NULL) {
        printf("Соединение закрыто клиентом.\n");
        tcp_close(tpcb);
        return ERR_OK;
    }

    // Отправить результаты измерений
    send_adc_results(tpcb);

    // Очистить буфер
    pbuf_free(p);

    return ERR_OK;
}

// function in INA226
extern I2C_HandleTypeDef hi2c1;


TaskHandle_t ina226TaskHandle = NULL;

// Структура для хранения данных INA226
typedef struct {
    float bus_voltage;
    float current;
    float power;
} INA226_Data_t;

// Функция инициализации INA226
void INA226_TaskInit(void) {
    // Инициализация INA226
    INA226_Init(&hi2c1, INA226_I2C_ADDRESS);

    // Установка конфигурации INA226
    INA226_Config_t config = {
        .avg = 0x0000,                     // Без усреднения
        .bus_conv_time = 0x001C,           // 1.1 мс
        .shunt_conv_time = 0x001C,         // 1.1 мс
        .mode = INA226_MODE_SHUNT_BUS_CONTINUOUS
    };
    INA226_SetConfiguration(&hi2c1, INA226_I2C_ADDRESS, &config);

    // Калибровка для шунта 0.01 Ом и максимального тока 10 А
    INA226_Calibrate(&hi2c1, INA226_I2C_ADDRESS, 0.01f, 10.0f);
}

// Функция для задачи считывания и отправки данных
void INA226_Task(void *argument) {
    // Инициализация INA226
    INA226_TaskInit();

    INA226_Data_t ina226_data;

    // Инициализация TCP (LWIP)
    struct netconn *conn;
    struct netbuf *buf;
    ip_addr_t dest_ip;
    IP4_ADDR(&dest_ip, 192, 168, 0, 101); // IP-адрес получателя

    conn = netconn_new(NETCONN_TCP);
    netconn_bind(conn, NULL, 0);
    netconn_connect(conn, &dest_ip, 9001); // Порт получателя

    for (;;) {
        // Считывание данных с INA226
        INA226_ReadBusVoltage(&hi2c1, INA226_I2C_ADDRESS, &ina226_data.bus_voltage);
        INA226_ReadCurrent(&hi2c1, INA226_I2C_ADDRESS, &ina226_data.current);
        INA226_ReadPower(&hi2c1, INA226_I2C_ADDRESS, &ina226_data.power);

        // Формирование строки для отправки
        char message[128];
        snprintf(message, sizeof(message),
                 "Bus Voltage: %.2f V, Current: %.2f A, Power: %.2f W\r\n",
                 ina226_data.bus_voltage, ina226_data.current, ina226_data.power);

        // Отправка данных по Ethernet
        buf = netbuf_new();
        netbuf_ref(buf, message, strlen(message));
        netconn_send(conn, buf);
        netbuf_delete(buf);

        // Задержка 1 секунда
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    // Завершение соединения
    netconn_close(conn);
    netconn_delete(conn);
}

// Function prototypes
void VacuumResetTask(void *param);
void SendCommandViaEthernet(const char *command);
void SendInfoToOperator(const char *message);
void ControlPumpState(GPIO_TypeDef *gpio, uint16_t pin, GPIO_PinState state);

// Task for vacuum reset procedure
void VacuumResetTask(void *param) {
    for (;;) {
        // Step 1: Check power supply state via Ethernet
        bool powerOff = CheckPowerSupplyState(); // Custom function to check state
        if (powerOff) {
            SendCommandViaEthernet("TURN_OFF_POWER");
            SendInfoToOperator("Power supply turned off.");
        }

        // Step 2: Check if low vacuum pump is off
        bool lowVacuumPumpOff = CheckLowVacuumPumpState(); // Custom function
        if (!lowVacuumPumpOff) {
            ControlPumpState(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET); // Example GPIO control
            SendInfoToOperator("Low vacuum pump turned off.");
        }

        // Step 3: Process KYKY system
        if (IsKykySystemOn()) {
            SendCommandViaEthernet("TURN_OFF_KYKY");
            SendInfoToOperator("KYKY system turned off.");
        }

        // Step 4: Check high vacuum pump state
        if (IsHighVacuumPumpOn()) {
            SendCommandViaEthernet("TURN_OFF_HIGH_VACUUM");
            SendInfoToOperator("High vacuum pump turned off.");
        }

        // Step 5: Finalize vacuum reset procedure
        if (!IsAllSystemsOff()) {
            SendInfoToOperator("All systems are now off. Vacuum reset complete.");
        }

        // Step 6: Add a delay between iterations
        vTaskDelay(pdMS_TO_TICKS(1000)); // 1 second delay
    }
}

// Send a command to a device via Ethernet
void SendCommandViaEthernet(const char *command) {
    Ethernet_SendCommand(command); // Replace with actual implementation
}

// Send information to the operator via Ethernet
void SendInfoToOperator(const char *message) {
    Ethernet_SendInfo(message); // Replace with actual implementation
}

// Control the state of a pump via GPIO
void ControlPumpState(GPIO_TypeDef *gpio, uint16_t pin, GPIO_PinState state) {
    HAL_GPIO_WritePin(gpio, pin, state);
}
//void InitINA226() {
//    // Инициализация INA226
//    INA226_Init(&hi2c1, INA226_I2C_ADDRESS);
//
//    // Установка конфигурации по умолчанию
//    INA226_Config_t config = {
//        .avg = 0x0000,                     // Без усреднения
//        .bus_conv_time = 0x001C,           // 1.1 мс
//        .shunt_conv_time = 0x001C,         // 1.1 мс
//        .mode = INA226_MODE_SHUNT_BUS_CONTINUOUS
//    };
//    INA226_SetConfiguration(&hi2c1, INA226_I2C_ADDRESS, &config);
//
//    // Калибровка для шунта 0.01 Ом и тока до 10 А?
//    INA226_Calibrate(&hi2c1, INA226_I2C_ADDRESS, 0.01f, 10.0f);
//}

//void ReadINA226Data() {
//    float bus_voltage = 0.0f;
//    float shunt_voltage = 0.0f;
//    float current = 0.0f;
//    float power = 0.0f;
//
//    // Считывание данных
//    INA226_ReadBusVoltage(&hi2c1, INA226_I2C_ADDRESS, &bus_voltage);
//    INA226_ReadCurrent(&hi2c1, INA226_I2C_ADDRESS, &current);
//    INA226_ReadPower(&hi2c1, INA226_I2C_ADDRESS, &power);
//
//    // Вывод данных через UART (пример)
//    printf("Bus Voltage: %.3f V\n", bus_voltage);
//    printf("Current: %.3f A\n", current);
//    printf("Power: %.3f W\n", power);
//}

void INA226_EthernetTask(void *argument) {
    char message[128];
    float bus_voltage, current, power;

    // Инициализация INA226
    INA226_Init(&hi2c1, INA226_I2C_ADDRESS);
    INA226_Config_t config = {
        .avg = 0x0000,                     // Без усреднения
        .bus_conv_time = 0x001C,           // 1.1 мс
        .shunt_conv_time = 0x001C,         // 1.1 мс
        .mode = INA226_MODE_SHUNT_BUS_CONTINUOUS
    };
    INA226_SetConfiguration(&hi2c1, INA226_I2C_ADDRESS, &config);
    INA226_Calibrate(&hi2c1, INA226_I2C_ADDRESS, 0.01f, 10.0f); // Шунт 0.01 Ом, ток до 10А

    // Бесконечный цикл задачи
    for (;;) {
        // Считываем данные
        INA226_ReadBusVoltage(&hi2c1, INA226_I2C_ADDRESS, &bus_voltage);
        INA226_ReadCurrent(&hi2c1, INA226_I2C_ADDRESS, &current);
        INA226_ReadPower(&hi2c1, INA226_I2C_ADDRESS, &power);

        // Формируем строку с данными
        snprintf(message, sizeof(message), "Voltage: %.3f V, Current: %.3f A, Power: %.3f W\r\n",
                 bus_voltage, current, power);

        // Отправляем данные через TCP-сервер
        tcp_server_send_message(message);

        // Задержка между передачами (1 секунда)
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
// Handler for the first task
void TaskHandler1(void *param) {
    uint32_t receivedData;
    for (;;) {
        if (xQueueReceive(queue1, &receivedData, portMAX_DELAY) == pdPASS) {
            // Process received data
            HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_0);  // Example of processing
        }
        vTaskDelay(pdMS_TO_TICKS(100));  // Delay for stabilization
    }
}

// Handler for the second task
void TaskHandler2(void *param) {
    uint32_t data = 0;
    for (;;) {
        // Generate data
        data++;
        xQueueSend(queue2, &data, portMAX_DELAY);

        // Synchronization using semaphore
        xSemaphoreGive(semaphore2);

        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

// Handler for the third task
void TaskHandler3(void *param) {
    uint32_t receivedData;
    for (;;) {
        if (xSemaphoreTake(semaphore3, portMAX_DELAY) == pdPASS) {
            // Receive data from the queue
            if (xQueueReceive(queue3, &receivedData, portMAX_DELAY) == pdPASS) {
                // Process received data
                HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0);  // Example of processing
            }
        }
    }
}

// Initialization of tasks, queues
void InitTasks() {
    queue1 = xQueueCreate(10, sizeof(uint32_t));
    queue2 = xQueueCreate(10, sizeof(uint32_t));
    queue3 = xQueueCreate(10, sizeof(uint32_t));

    semaphore1 = xSemaphoreCreateBinary();
    semaphore2 = xSemaphoreCreateBinary();
    semaphore3 = xSemaphoreCreateBinary();

    xTaskCreate(TaskHandler1, "Task1", 128, NULL, 2, NULL);
    xTaskCreate(TaskHandler2, "Task2", 128, NULL, 2, NULL);
    xTaskCreate(TaskHandler3, "Task3", 128, NULL, 2, NULL);
}


void EthernetTask_Init(void) {
    // Инициализация TCP-сервера
    tcp_server_init();

    // Создаем задачу для отправки данных с INA226
    xTaskCreate(INA226_EthernetTask, "INA226_ETH_Task", 512, NULL, osPriorityNormal, NULL);
}
// Thresholds for pump current
#define PUMP_CURRENT_NOMINAL 5.0f  // Nominal current in Amperes
#define PUMP_CURRENT_MIN 2.0f     // Minimum allowable current in Amperes

// Function prototypes
void TrainingModeTask(void *param);
void SendCommandToPowerSupply(const char *command);
void SendInfoToOperator(const char *message);

// Task for pump training mode
void TrainingModeTask(void *param) {
    float pumpCurrent = 0.0f;

    for (;;) {
        // Step 1: Get pump current via SPI
        pumpCurrent = GetPumpCurrentViaSPI();  // User-defined function

        // Step 2: Check if the pump current is below the nominal value
        if (pumpCurrent < PUMP_CURRENT_NOMINAL) {
            // Step 3: Check if the pump current is below the minimum threshold
            if (pumpCurrent < PUMP_CURRENT_MIN) {
                // Send error message to the operator
                SendInfoToOperator("Error: Pump current below minimum threshold. Shutting down system.");
                // Send shutdown command to the electron source power system
                SendCommandToPowerSupply("SHUTDOWN");
                // Wait for 10 minutes (block system)
                vTaskDelay(pdMS_TO_TICKS(600000));  // 10 minutes delay
                continue;
            }

            // Step 4: Reduce power supply settings and wait for stabilization
            SendCommandToPowerSupply("REDUCE_SETPOINT");
            SendInfoToOperator("Reducing power supply setpoint. Waiting for stabilization.");
            vTaskDelay(pdMS_TO_TICKS(5000));  // 5 seconds delay for stabilization
        } else {
            // Step 5: Increase power supply settings if current is nominal
            SendCommandToPowerSupply("INCREASE_SETPOINT");
        }

        // Send status update to the operator
        SendInfoToOperator("Pump current stabilized. Training mode continues.");

        // Delay before next iteration
        vTaskDelay(pdMS_TO_TICKS(1000));  // 1 second delay
    }
}

// Function to send a command to the power supply via Ethernet
void SendCommandToPowerSupply(const char *command) {
    // Implement Ethernet communication to send the command
    Ethernet_SendCommand(command);  // User-defined function
}

// Function to send information to the operator via Ethernet
void SendInfoToOperator(const char *message) {
    // Implement Ethernet communication to send the message
    Ethernet_SendInfo(message);  // User-defined function
}
//int main(void) {
//    HAL_Init();
//    SystemClock_Config();
//    MX_GPIO_Init();
//    MX_DMA_Init();
//    MX_I2C1_Init();
//    MX_USART3_UART_Init();
//
//    // Инициализация INA226
//    InitINA226();
//
//    while (1) {
//        // Считывание данных INA226
//        ReadINA226Data();
//        HAL_Delay(1000); // Задержка 1 секунда
//    }
//}



#ifdef  USE_FULL_ASSERT
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

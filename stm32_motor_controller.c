/*
 * STM32 Motor Controller Firmware
 * Принимает команды Twist через UART и управляет моторами
 * 
 * Протокол: 8 байт
 * [steer_low][steer_high][speed_low][speed_high][crc32_low][crc32_med1][crc32_med2][crc32_high]
 * 
 * steer: -1000 до 1000 (поворот)
 * speed: -1000 до 1000 (скорость)
 * crc32: контрольная сумма
 */

#include "stm32f1xx_hal.h"

// UART handle
UART_HandleTypeDef huart1;

// Motor control pins (пример для STM32F103)
#define MOTOR1_PWM_PIN GPIO_PIN_8
#define MOTOR1_DIR_PIN GPIO_PIN_9
#define MOTOR2_PWM_PIN GPIO_PIN_10
#define MOTOR2_DIR_PIN GPIO_PIN_11
#define MOTOR_PORT GPIOA

// Встроенный светодиод (BluePill: PC13). Нужен для индикации приёма UART
#define LED_PORT GPIOC
#define LED_PIN  GPIO_PIN_13

// PWM timer
TIM_HandleTypeDef htim1;

// Структура команды
typedef struct {
    int16_t steer;  // -1000 до 1000
    int16_t speed;  // -1000 до 1000
    uint32_t crc32; // контрольная сумма
} motor_command_t;

// Буфер для приема данных
uint8_t rx_buffer[8];
motor_command_t current_command = {0, 0, 0};

// Флаг для мигания светодиодом после успешного приёма пакета
volatile uint8_t packet_received_blink_ticks = 0; // количество тиков по 10 мс

// Функция вычисления CRC32
uint32_t calculate_crc32(uint8_t* data, uint8_t length) {
    uint32_t crc = 0xFFFFFFFF;
    
    for (uint8_t i = 0; i < length; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 1) {
                crc = (crc >> 1) ^ 0xEDB88320;
            } else {
                crc >>= 1;
            }
        }
    }
    
    return crc ^ 0xFFFFFFFF;
}

// Функция управления мотором
void set_motor_speed(int16_t left_speed, int16_t right_speed) {
    // Ограничение значений
    if (left_speed > 1000) left_speed = 1000;
    if (left_speed < -1000) left_speed = -1000;
    if (right_speed > 1000) right_speed = 1000;
    if (right_speed < -1000) right_speed = -1000;
    
    // Установка направления
    if (left_speed >= 0) {
        HAL_GPIO_WritePin(MOTOR_PORT, MOTOR1_DIR_PIN, GPIO_PIN_SET);
    } else {
        HAL_GPIO_WritePin(MOTOR_PORT, MOTOR1_DIR_PIN, GPIO_PIN_RESET);
        left_speed = -left_speed;
    }
    
    if (right_speed >= 0) {
        HAL_GPIO_WritePin(MOTOR_PORT, MOTOR2_DIR_PIN, GPIO_PIN_SET);
    } else {
        HAL_GPIO_WritePin(MOTOR_PORT, MOTOR2_DIR_PIN, GPIO_PIN_RESET);
        right_speed = -right_speed;
    }
    
    // Установка PWM (масштабирование 0-1000 в 0-1000)
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, left_speed);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, right_speed);
}

// Обработчик UART
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART1) {
        // Проверяем CRC
        uint32_t received_crc = *(uint32_t*)(rx_buffer + 4);
        uint32_t calculated_crc = calculate_crc32(rx_buffer, 4);
        
        if (received_crc == calculated_crc) {
            // Извлекаем команду
            current_command.steer = *(int16_t*)(rx_buffer);
            current_command.speed = *(int16_t*)(rx_buffer + 2);
            current_command.crc32 = received_crc;
            
            // Преобразуем в команды для моторов
            // Простая модель: speed - линейная скорость, steer - разность скоростей
            int16_t left_speed = current_command.speed - current_command.steer/2;
            int16_t right_speed = current_command.speed + current_command.steer/2;
            
            set_motor_speed(left_speed, right_speed);

            // Отметить успешный приём: моргнуть светодиодом в основном цикле ~50 мс
            packet_received_blink_ticks = 5; // 5 * 10 мс = 50 мс
        }
        
        // Перезапускаем прием
        HAL_UART_Receive_IT(&huart1, rx_buffer, 8);
    }
}

// Инициализация UART
void MX_USART1_UART_Init(void) {
    huart1.Instance = USART1;
    huart1.Init.BaudRate = 115200;
    huart1.Init.WordLength = UART_WORDLENGTH_8B;
    huart1.Init.StopBits = UART_STOPBITS_1;
    huart1.Init.Parity = UART_PARITY_NONE;
    huart1.Init.Mode = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    
    if (HAL_UART_Init(&huart1) != HAL_OK) {
        // Ошибка инициализации
    }
}

// Инициализация GPIO
void MX_GPIO_Init(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    
    // Включаем тактирование
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    
    // Настройка пинов моторов
    GPIO_InitStruct.Pin = MOTOR1_PWM_PIN | MOTOR1_DIR_PIN | MOTOR2_PWM_PIN | MOTOR2_DIR_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(MOTOR_PORT, &GPIO_InitStruct);
    
    // Настройка UART пинов
    GPIO_InitStruct.Pin = GPIO_PIN_9;  // TX
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    
    GPIO_InitStruct.Pin = GPIO_PIN_10; // RX
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    // Настройка светодиода PC13 (активный уровень низкий на BluePill)
    GPIO_InitStruct.Pin = LED_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LED_PORT, &GPIO_InitStruct);
    HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_SET); // выкл (лог.1)
}

// Инициализация таймера для PWM
void MX_TIM1_Init(void) {
    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    TIM_OC_InitTypeDef sConfigOC = {0};
    TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};
    
    htim1.Instance = TIM1;
    htim1.Init.Prescaler = 72-1;  // 1MHz
    htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim1.Init.Period = 1000-1;   // 1kHz PWM
    htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim1.Init.RepetitionCounter = 0;
    htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    
    if (HAL_TIM_Base_Init(&htim1) != HAL_OK) {
        // Ошибка
    }
    
    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK) {
        // Ошибка
    }
    
    if (HAL_TIM_PWM_Init(&htim1) != HAL_OK) {
        // Ошибка
    }
    
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK) {
        // Ошибка
    }
    
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
    sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
    
    if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) {
        // Ошибка
    }
    
    if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK) {
        // Ошибка
    }
    
    sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
    sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
    sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
    sBreakDeadTimeConfig.DeadTime = 0;
    sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
    sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
    sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
    if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK) {
        // Ошибка
    }
    
    HAL_TIM_MspPostInit(&htim1);
}

// Основная функция
int main(void) {
    // Инициализация HAL
    HAL_Init();
    
    // Инициализация периферии
    MX_GPIO_Init();
    MX_USART1_UART_Init();
    MX_TIM1_Init();
    
    // Запуск PWM
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    
    // Запуск приема UART
    HAL_UART_Receive_IT(&huart1, rx_buffer, 8);
    
    // Основной цикл
    while (1) {
        // Обработка индикации приёма пакета: короткая вспышка светодиода
        if (packet_received_blink_ticks > 0) {
            HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_RESET); // вкл (лог.0)
            packet_received_blink_ticks--;
        } else {
            HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_SET); // выкл (лог.1)
        }

        HAL_Delay(10);
    }
}

// Обработчик SysTick
void SysTick_Handler(void) {
    HAL_IncTick();
}

#include "main.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

/* CẤU HÌNH VÀ BIẾN */
/* NOTE: Khu vực này khai báo các hằng số, chế độ hoạt động và biến dùng chung cho toàn hệ thống. */
#define MOVING_AVERAGE_SIZE 5        // NOTE: Lấy trung bình 5 mẫu đo để giảm nhiễu cảm biến
#define I2C_LCD_ADDR (0x27 << 1)    // NOTE: Địa chỉ LCD I2C, nếu LCD không hiện có thể cần đổi 0x27 thành 0x3F

typedef enum { BOOT_MODE = 0, AUTO_MODE, MANUAL_MODE } Mode_t;       // NOTE: 3 chế độ: chờ khởi động, tự động, điều khiển tay
typedef enum { STATE_SAFE = 0, STATE_WARNING, STATE_DANGER } Safety_t; // NOTE: 3 mức cảnh báo theo khoảng cách

I2C_HandleTypeDef hi2c1;
UART_HandleTypeDef huart1;

volatile Mode_t current_mode = BOOT_MODE;     // Bắt đầu ở Boot Mode
volatile Safety_t safety_state = STATE_SAFE;

volatile uint32_t rawDistance = 150;          // NOTE: Khoảng cách đo trực tiếp từ HC-SR04, đơn vị cm
volatile float filtered_distance = 150.0f;  // NOTE: Khoảng cách sau khi lọc trung bình
float previous_distance = 150.0f;

uint32_t last_sensor_time = 0;
uint32_t last_uart_time = 0;
uint32_t last_lcd_time = 0;
uint32_t last_velocity_time = 0;
uint32_t last_boot_tick = 0;
uint32_t brake_start_time = 0;

volatile uint8_t is_braking_active = 0;
float current_speed_m_s = 0.0f;
volatile float d_safe = 30.0f;                // NOTE: Khoảng cách an toàn động, dùng để quyết định phanh

volatile char rx_buffer = 0;
volatile uint8_t rx_flag = 0;
volatile int32_t boot_countdown = 10;     // Thời gian chờ Boot

uint8_t hal_rx_byte = 0;

/* ========================================================================== */
/* PROTOTYPES */
/* ========================================================================== */
void Delay_us(volatile uint32_t au32microseconds);
void GPIO_Init_RegisterLevel(void);
void Timers_Init(void);
void Local_MX_USART1_UART_Init(void);
void Local_MX_I2C1_Init(void);

void LCD_Init(void);
void LCD_Write(uint8_t data, uint8_t control_bits);
void LCD_Send_Cmd(char cmd);
void LCD_Send_Data(char data);
void LCD_SetCursor(uint8_t row, uint8_t col);
void LCD_Print_Fixed(const char *str);
void LCD_Print_Num(uint32_t num);

void UART_SendString(const char *str);
void Set_Motor(int8_t speed_A, int8_t speed_B);
void Set_Hardware_Alerts(Safety_t state);
void Update_Safety_And_Motors(void);
const char* Get_Safety_Text(Safety_t state);

/* ========================================================================== */
/* INIT FUNCTIONS */
/* ========================================================================== */
/* NOTE: Hàm delay ngắn theo micro giây, dùng để tạo xung TRIG 10us cho cảm biến siêu âm. */
void Delay_us(volatile uint32_t au32microseconds) {
    au32microseconds *= 3;
    while (au32microseconds--);
}

/* NOTE: Cấu hình chân GPIO ở mức thanh ghi: cảm biến, LED, buzzer, motor, UART và LCD I2C. */
void GPIO_Init_RegisterLevel(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN;

    // PA0 Còi, PA1 TRIG, PA5 LED, PA6 ECHO
    GPIOA->MODER &= ~(3 << GPIO_MODER_MODER0_Pos); GPIOA->MODER |= (1 << GPIO_MODER_MODER0_Pos);
    GPIOA->MODER &= ~(3 << GPIO_MODER_MODER1_Pos); GPIOA->MODER |= (1 << GPIO_MODER_MODER1_Pos);
    GPIOA->MODER &= ~(3 << GPIO_MODER_MODER5_Pos); GPIOA->MODER |= (1 << GPIO_MODER_MODER5_Pos);

    GPIOA->MODER &= ~(3 << GPIO_MODER_MODER6_Pos); GPIOA->MODER |= (2 << GPIO_MODER_MODER6_Pos);
    GPIOA->AFR[0] &= ~(0xF << (6*4)); GPIOA->AFR[0] |= (2 << (6*4));

    // PWM PA8, PA9
    GPIOA->MODER &= ~((3 << GPIO_MODER_MODER8_Pos) | (3 << GPIO_MODER_MODER9_Pos));
    GPIOA->MODER |= (2 << GPIO_MODER_MODER8_Pos) | (2 << GPIO_MODER_MODER9_Pos);
    GPIOA->AFR[1] &= ~((0xF << 0) | (0xF << 4));
    GPIOA->AFR[1] |= (1 << 0) | (1 << 4);

    // UART1: PB6 TX - PA10 RX
    GPIOA->MODER &= ~(3 << GPIO_MODER_MODER10_Pos); GPIOA->MODER |= (2 << GPIO_MODER_MODER10_Pos);
    GPIOA->AFR[1] &= ~(0xF << (10*4 - 32)); GPIOA->AFR[1] |= (7 << (10*4 - 32));

    GPIOB->MODER &= ~(3 << GPIO_MODER_MODER6_Pos); GPIOB->MODER |= (2 << GPIO_MODER_MODER6_Pos);
    GPIOB->AFR[0] &= ~(0xF << (6*4)); GPIOB->AFR[0] |= (7 << (6*4));

    GPIOA->OSPEEDR |= (3 << (10*2));
    GPIOB->OSPEEDR |= (3 << (6*2));

    // LED PC6,8,9
    GPIOC->MODER &= ~((3 << (6*2)) | (3 << (8*2)) | (3 << (9*2)));
    GPIOC->MODER |= ((1 << (6*2)) | (1 << (8*2)) | (1 << (9*2)));

    // TB6612
    GPIOC->MODER &= ~((3 << (0*2)) | (3 << (1*2)) | (3 << (2*2)) | (3 << (3*2)));
    GPIOC->MODER |= ((1 << (0*2)) | (1 << (1*2)) | (1 << (2*2)) | (1 << (3*2)));

    // I2C1
    GPIOB->MODER &= ~((3 << (7*2)) | (3 << (8*2)));
    GPIOB->MODER |= ((2 << (7*2)) | (2 << (8*2)));
    GPIOB->OTYPER |= (1 << 7) | (1 << 8);
    GPIOB->AFR[0] &= ~(0xF << 28); GPIOB->AFR[0] |= (4 << 28);
    GPIOB->AFR[1] &= ~(0xF << 0);  GPIOB->AFR[1] |= (4 << 0);
}

/* NOTE: Khởi tạo TIM1 để tạo PWM điều khiển motor và TIM3 để đo xung ECHO của cảm biến. */
void Timers_Init(void) {
    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
    TIM1->PSC = 159; TIM1->ARR = 99;
    TIM1->CCMR1 |= (6 << TIM_CCMR1_OC1M_Pos) | (6 << TIM_CCMR1_OC2M_Pos);
    TIM1->CCMR1 |= TIM_CCMR1_OC1PE | TIM_CCMR1_OC2PE;
    TIM1->CCER |= TIM_CCER_CC1E | TIM_CCER_CC2E;
    TIM1->BDTR |= TIM_BDTR_MOE;
    TIM1->CR1 |= TIM_CR1_CEN;

    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
    TIM3->PSC = 15; TIM3->ARR = 0xFFFF;
    TIM3->CCMR1 |= (1 << TIM_CCMR1_CC1S_Pos);
    TIM3->CCER |= TIM_CCER_CC1E | TIM_CCER_CC1P | TIM_CCER_CC1NP;
    TIM3->DIER |= TIM_DIER_CC1IE;
    NVIC_SetPriority(TIM3_IRQn, 0);
    NVIC_EnableIRQ(TIM3_IRQn);
    TIM3->CR1 |= TIM_CR1_CEN;
}

/* NOTE: Khởi tạo UART1 tốc độ 9600 để nhận lệnh từ Bluetooth/app điện thoại. */
void Local_MX_USART1_UART_Init(void) {
    __HAL_RCC_USART1_CLK_ENABLE();
    huart1.Instance = USART1;
    huart1.Init.BaudRate = 9600;
    huart1.Init.WordLength = UART_WORDLENGTH_8B;
    huart1.Init.StopBits = UART_STOPBITS_1;
    huart1.Init.Parity = UART_PARITY_NONE;
    huart1.Init.Mode = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    HAL_UART_Init(&huart1);

    HAL_NVIC_SetPriority(USART1_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
}

/* NOTE: Khởi tạo I2C1 để giao tiếp với màn hình LCD 16x2 qua module I2C. */
void Local_MX_I2C1_Init(void) {
    __HAL_RCC_I2C1_CLK_ENABLE();
    hi2c1.Instance = I2C1;
    hi2c1.Init.ClockSpeed = 100000;
    hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
    hi2c1.Init.OwnAddress1 = 0;
    hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2 = 0;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    HAL_I2C_Init(&hi2c1);
}

/* LCD */
/* NOTE: Nhóm hàm LCD dùng để gửi lệnh, gửi ký tự, đặt con trỏ và in dữ liệu lên LCD. */
void LCD_Write(uint8_t data, uint8_t control_bits) {
    uint8_t data_u = data & 0xF0;
    uint8_t data_l = (data << 4) & 0xF0;
    uint8_t pkt[4] = {
        data_u | control_bits | 0x0C,
        data_u | control_bits | 0x08,
        data_l | control_bits | 0x0C,
        data_l | control_bits | 0x08
    };
    HAL_I2C_Master_Transmit(&hi2c1, I2C_LCD_ADDR, pkt, 4, 10);
}

void LCD_Send_Cmd(char cmd)  { LCD_Write(cmd, 0x00); }
void LCD_Send_Data(char data){ LCD_Write(data, 0x01); }

void LCD_Init(void) {
    HAL_Delay(50);
    LCD_Send_Cmd(0x02); HAL_Delay(10);
    LCD_Send_Cmd(0x28); HAL_Delay(5);
    LCD_Send_Cmd(0x0C); HAL_Delay(5);
    LCD_Send_Cmd(0x01); HAL_Delay(20);
}

void LCD_SetCursor(uint8_t row, uint8_t col) {
    LCD_Send_Cmd((row == 0) ? (0x80 + col) : (0xC0 + col));
}

void LCD_Print_Fixed(const char *str) {
    uint8_t i = 0;
    while (str[i] != '\0' && i < 16) LCD_Send_Data(str[i++]);
    while (i < 16) { LCD_Send_Data(' '); i++; }
}

void LCD_Print_Num(uint32_t num) {
    char buf[8]; int i = 0;
    if (num == 0) { LCD_Send_Data('0'); return; }
    while (num && i < 8) {
        buf[i++] = (num % 10) + '0';
        num /= 10;
    }
    while (i--) LCD_Send_Data(buf[i]);
}

/* UART & Motor & Safety */
/* NOTE: Gửi chuỗi debug/trạng thái qua UART. */
void UART_SendString(const char *str) {
    HAL_UART_Transmit(&huart1, (uint8_t*)str, strlen(str), 100);
}

/* NOTE: Điều khiển 2 motor. Giá trị dương = tiến, âm = lùi, 0 = dừng. */
void Set_Motor(int8_t speed_A, int8_t speed_B) {
    if (speed_A > 0) { GPIOC->BSRR = (1 << 0) | (1 << (1 + 16)); TIM1->CCR1 = speed_A; }
    else if (speed_A < 0) { GPIOC->BSRR = (1 << 1) | (1 << (0 + 16)); TIM1->CCR1 = -speed_A; }
    else { GPIOC->BSRR = (1 << (0 + 16)) | (1 << (1 + 16)); TIM1->CCR1 = 0; }

    if (speed_B > 0) { GPIOC->BSRR = (1 << 2) | (1 << (3 + 16)); TIM1->CCR2 = speed_B; }
    else if (speed_B < 0) { GPIOC->BSRR = (1 << 3) | (1 << (2 + 16)); TIM1->CCR2 = -speed_B; }
    else { GPIOC->BSRR = (1 << (2 + 16)) | (1 << (3 + 16)); TIM1->CCR2 = 0; }
}

/* NOTE: Chuyển trạng thái an toàn thành chữ để hiển thị lên LCD. */
const char* Get_Safety_Text(Safety_t state) {
    if (state == STATE_SAFE) return "SAFE";
    else if (state == STATE_WARNING) return "WARN";
    else return "DANGER";
}

/* NOTE: Bật LED/buzzer theo trạng thái SAFE, WARN hoặc DANGER. */
void Set_Hardware_Alerts(Safety_t state) {
    GPIOC->BSRR = (1 << (6 + 16)) | (1 << (8 + 16)) | (1 << (9 + 16));
    GPIOA->BSRR = (1 << (0 + 16));
    if (state == STATE_SAFE) GPIOC->BSRR = (1 << 6);
    else if (state == STATE_WARNING) GPIOC->BSRR = (1 << 8);
    else if (state == STATE_DANGER) {
        GPIOC->BSRR = (1 << 9);
        GPIOA->BSRR = (1 << 0);
    }
}

/* NOTE: Cập nhật trạng thái an toàn và tự phanh nếu xe đang ở AUTO_MODE. */
void Update_Safety_And_Motors(void) {
    float cur_d = filtered_distance;
    if (cur_d >= 100.0f) safety_state = STATE_SAFE;
    else if (cur_d >= 50.0f) safety_state = STATE_WARNING;
    else safety_state = STATE_DANGER;

    Set_Hardware_Alerts(safety_state);

    if (current_mode == AUTO_MODE) {
        if (cur_d <= d_safe) {
            if (!is_braking_active) {
                is_braking_active = 1;
                brake_start_time = HAL_GetTick();
            }
            if (HAL_GetTick() - brake_start_time <= 120)
                Set_Motor(-70, -70);
            else
                Set_Motor(0, 0);
        } else {
            is_braking_active = 0;
            TIM1->CCR1 = 40; // Cố định tốc độ Auto tiến 50%
            TIM1->CCR2 = 40;
            GPIOC->BSRR = (1u<<0)|(1u<<(1+16))|(1u<<2)|(1u<<(3+16));
        }
    }
}

/* INTERRUPTS */
/* NOTE: Các hàm ngắt xử lý sự kiện xảy ra bất ngờ, ví dụ có xung ECHO hoặc có dữ liệu Bluetooth. */
/* NOTE: Ngắt TIM3 đo độ rộng xung ECHO để tính khoảng cách từ cảm biến siêu âm. */
void TIM3_IRQHandler(void) {
    static uint32_t t_start = 0;
    if (TIM3->SR & TIM_SR_CC1IF) {
        uint32_t t_capture = TIM3->CCR1;
        if (GPIOA->IDR & GPIO_IDR_IDR_6) {
            t_start = t_capture;
        } else {
            uint32_t t_echo = (t_capture >= t_start) ? (t_capture - t_start) : (0xFFFF - t_start + t_capture + 1);
            rawDistance = t_echo / 58;

            static float buf[MOVING_AVERAGE_SIZE] = {150,150,150,150,150};
            static uint8_t idx = 0;
            if (rawDistance > 0 && rawDistance < 400) {
                buf[idx] = (float)rawDistance;
                idx = (idx + 1) % MOVING_AVERAGE_SIZE;
            }
            float sum = 0;
            for(int i = 0; i < MOVING_AVERAGE_SIZE; i++) sum += buf[i];
            filtered_distance = sum / MOVING_AVERAGE_SIZE;

            Update_Safety_And_Motors();
        }
        TIM3->SR &= ~TIM_SR_CC1IF;
    }
}

/* NOTE: Callback UART được gọi khi STM32 nhận được 1 ký tự từ Bluetooth. */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART1) {
        char temp_char = (char)hal_rx_byte;
        if (temp_char >= 32 && temp_char <= 126) {
            rx_buffer = temp_char;
            rx_flag = 1;

            char debug[20];
            sprintf(debug, "RX:%c\r\n", temp_char);
            UART_SendString(debug);
        }
        HAL_UART_Receive_IT(&huart1, &hal_rx_byte, 1);
    }
}

/* ========================================================================== */
/* MAIN */
/* ========================================================================== */
/* NOTE: Hàm main khởi tạo toàn bộ hệ thống và chạy vòng lặp điều khiển chính. */
int main(void) {
    HAL_Init();
    GPIO_Init_RegisterLevel();
    Timers_Init();
    Local_MX_I2C1_Init();
    Local_MX_USART1_UART_Init();

    LCD_Init();
    HAL_UART_Receive_IT(&huart1, &hal_rx_byte, 1);

    UART_SendString("\r\n==================================\r\n");
    UART_SendString(" ROBOT ADAS CORE ONLINE!\r\n");
    UART_SendString("==================================\r\n");
    UART_SendString("BOOT MODE - Send 'A' or 'M' to start\r\n");

    while (1) {
        uint32_t current_time = HAL_GetTick();

        /* Sensor Trigger */
        /* NOTE: Mỗi 60ms tạo xung TRIG 10us để cảm biến HC-SR04 bắt đầu đo khoảng cách. */
        if (current_time - last_sensor_time >= 60) {
            GPIOA->BSRR = GPIO_BSRR_BS1;
            Delay_us(10);
            GPIOA->BSRR = GPIO_BSRR_BR1;
            last_sensor_time = current_time;
        }

        /* Tính d_safe */
        /* NOTE: Mỗi 1 giây ước lượng tốc độ tiến gần vật cản và tính khoảng cách an toàn. */
        if (current_time - last_velocity_time >= 1000) {
            float delta_d = previous_distance - filtered_distance;
            current_speed_m_s = (delta_d > 0.1f) ? (delta_d / 100.0f) : 0.0f;
            float calculated = (current_speed_m_s * 0.5f) + (current_speed_m_s * current_speed_m_s / 5.0f);
            d_safe = calculated * 100.0f;
            if (d_safe < 22.0f) d_safe = 22.0f;
            if (d_safe > 45.0f) d_safe = 45.0f;
            previous_distance = filtered_distance;
            last_velocity_time = current_time;
        }

        /* ==================== XỬ LÝ CHẾ ĐỘ MANUAL (LUÔN QUÉT AN TOÀN) ==================== */
        if (current_mode == MANUAL_MODE) {

            // 1. ĐỌC VÀ XỬ LÝ LỆNH BLUETOOTH TRƯỚC
            uint8_t has_new_cmd = 0;
            char cmd = 0;

            if (rx_flag) {
                rx_flag = 0;
                cmd = rx_buffer;
                rx_buffer = 0;
                has_new_cmd = 1;
            }

            // 2. KIỂM TRA ĐIỀU KIỆN KHOẢNG CÁCH NGUY HIỂM (PHANH KHẨN CẤP)
            if (filtered_distance <= d_safe) {

                // Nếu có lệnh mới truyền xuống
                if (has_new_cmd) {
                    if (cmd == 'C' || cmd == 'c') {
                        current_mode = AUTO_MODE;
                    }
                    else if (cmd == 'A' || cmd == 'a') {
                        // BỎ QUA LỆNH TIẾN: Không làm gì cả để hệ thống tiếp tục phanh phía dưới
                    }
                    else if (cmd == 'B' || cmd == 'b') {
                        Set_Motor(-50, -50);      // Lùi cố định 50% (Cho phép lùi ra xa vật cản)
                        is_braking_active = 0;    // Thoát trạng thái phanh tự động để ưu tiên lệnh lùi người dùng
                    }
                    else if (cmd == 'R' || cmd == 'r') {
                        Set_Motor(-80, 80);       // Quay trái
                        is_braking_active = 0;
                    }
                    else if (cmd == 'L' || cmd == 'l') {
                        Set_Motor(80, -80);       // Quay phải
                        is_braking_active = 0;
                    }
                    else if (cmd == 'S' || cmd == 's') {
                        Set_Motor(0, 0);          // Dừng xe chủ động
                        is_braking_active = 0;
                    }
                }

                // Nếu người dùng không bấm lùi/quay/dừng hoặc vừa bấm lệnh TIẾN bị chặn,
                // thì hệ thống tự động phanh khẩn cấp kích hoạt
                if (cmd == 0 || cmd == 'A' || cmd == 'a') {
                    if (!is_braking_active) {
                        is_braking_active = 1;
                        brake_start_time = HAL_GetTick();
                    }
                    if (HAL_GetTick() - brake_start_time <= 120) {
                        Set_Motor(-70, -70);     // Phanh lùi khẩn cấp
                    } else {
                        Set_Motor(0, 0);          // Ép đứng yên
                    }
                }
            }
            /* Trường hợp khoảng cách AN TOÀN (> d_safe): Xe chạy bình thường theo lệnh */
            else {
                is_braking_active = 0;

                if (has_new_cmd) {
                    if (cmd == 'C' || cmd == 'c') {
                        current_mode = AUTO_MODE;
                    }
                    else if (cmd == 'A' || cmd == 'a') {
                        Set_Motor(40, 40);        // Tiến cố định 50%
                    }
                    else if (cmd == 'B' || cmd == 'b') {
                        Set_Motor(-40, -40);      // Lùi cố định 50%
                    }
                    else if (cmd == 'R' || cmd == 'r') {
                        Set_Motor(-50, 50);       // Quay trái cố định
                    }
                    else if (cmd == 'L' || cmd == 'l') {
                        Set_Motor(50, -50);       // Quay phải cố định
                    }
                    else if (cmd == 'S' || cmd == 's') {
                        Set_Motor(0, 0);          // Dừng xe
                    }
                }
            }
        }

        /* ==================== XỬ LÝ LỆNH BLUETOOTH CHO CÁC CHẾ ĐỘ CÒN LẠI ==================== */
        if (rx_flag && current_mode != MANUAL_MODE) {
            rx_flag = 0;
            char cmd = rx_buffer;
            rx_buffer = 0;

            if (current_mode == BOOT_MODE) {
                if (cmd == 'M' || cmd == 'm') current_mode = MANUAL_MODE;
                else if (cmd == 'A' || cmd == 'a') current_mode = AUTO_MODE;
            }
            else if (current_mode == AUTO_MODE) {
                if (cmd == 'C' || cmd == 'c') current_mode = MANUAL_MODE;
            }
        }

        /* Boot Mode */
        /* NOTE: Khi mới bật nguồn, xe đứng yên và chờ người dùng chọn AUTO hoặc MANUAL. */
        if (current_mode == BOOT_MODE) {
            Set_Motor(0, 0);
            if (current_time - last_boot_tick >= 1000) {
                boot_countdown--;
                last_boot_tick = current_time;
                if (boot_countdown <= 0) current_mode = AUTO_MODE;
            }
        }

        /* LCD Display */
        /* NOTE: Cập nhật LCD định kỳ: dòng 1 hiển thị mode, dòng 2 hiển thị khoảng cách/trạng thái. */
        if (current_time - last_lcd_time >= 300) {
            LCD_SetCursor(0, 0);
            if (current_mode == BOOT_MODE)
                LCD_Print_Fixed("MODE: BOOT ");
            else if (current_mode == AUTO_MODE)
                LCD_Print_Fixed("MODE: AUTO ");
            else
                LCD_Print_Fixed("MODE: MANUAL ");

            LCD_SetCursor(1, 0);
            if (current_mode == BOOT_MODE) {
                LCD_Print_Fixed("WAIT: ");
                LCD_SetCursor(1, 6);
                LCD_Print_Num(boot_countdown);
                LCD_Send_Data('s');
            } else {
                char lcd_line[17];
                snprintf(lcd_line, sizeof(lcd_line), "DIST:%lucm %s",
                         (unsigned long)((uint32_t)filtered_distance),
                         Get_Safety_Text(safety_state));
                LCD_Print_Fixed(lcd_line);
            }
            last_lcd_time = current_time;
        }
    }
}

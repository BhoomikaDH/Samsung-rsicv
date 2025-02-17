#include "debug.h"
#include <ch32v00x.h>

// Ultrasonic sensor pins
#define TRIGGER_PIN GPIO_Pin_4
#define ECHO_PIN GPIO_Pin_3

// Servo motor PWM pin
#define SERVO_PWM_PIN GPIO_Pin_2

// Threshold distance for obstacle detection (in cm)
#define OBSTACLE_THRESHOLD 70

// Servo motor PWM parameters
#define SERVO_PWM_PERIOD 20000 // 20 ms period (50 Hz frequency)
#define SERVO_PWM_PULSE_MIN 1000 // 1 ms pulse (0 degrees)
#define SERVO_PWM_PULSE_MAX 2000 // 2 ms pulse (180 degrees)

// Motor A (Left Motor) Pins
#define MOTOR_A_FORWARD GPIO_Pin_5
#define MOTOR_A_REVERSE GPIO_Pin_6

// Motor B (Right Motor) Pins
#define MOTOR_B_FORWARD GPIO_Pin_7
#define MOTOR_B_REVERSE GPIO_Pin_0

// Function to configure GPIO pins for ultrasonic sensor
void GPIO_Config(void)
{
    GPIO_InitTypeDef GPIO_InitStructure = {0};

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);

    // Pin PD3: Input for Ultrasonic sensor echo
    GPIO_InitStructure.GPIO_Pin = ECHO_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; // Input with Pull-Up
    GPIO_Init(GPIOD, &GPIO_InitStructure);

    // Pin PD4: Output for Ultrasonic sensor trigger
    GPIO_InitStructure.GPIO_Pin = TRIGGER_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; // Output Push-Pull
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOD, &GPIO_InitStructure);
}

// Function to initialize PWM for the servo motor
void TIM1_PWMOut_Init(uint16_t pulseWidth)
{
    GPIO_InitTypeDef GPIO_InitStructure = {0};
    TIM_OCInitTypeDef TIM_OCInitStructure = {0};
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure = {0};

    // Enable GPIO clock for PD2 (servo PWM pin)
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);
    GPIO_InitStructure.GPIO_Pin = SERVO_PWM_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; // Alternate Function Push-Pull
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
    GPIO_Init(GPIOD, &GPIO_InitStructure);

    // Enable Timer 1 clock
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);

    // Configure Timer 1 for PWM
    TIM_TimeBaseInitStructure.TIM_Period = SERVO_PWM_PERIOD - 1; // 20 ms period
    TIM_TimeBaseInitStructure.TIM_Prescaler = (SystemCoreClock / 1000000) - 1; // 1 MHz clock
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM1, &TIM_TimeBaseInitStructure);

    // Configure PWM mode
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = pulseWidth; // Set pulse width
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OC1Init(TIM1, &TIM_OCInitStructure);

    // Enable PWM outputs
    TIM_CtrlPWMOutputs(TIM1, ENABLE);
    TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);
    TIM_ARRPreloadConfig(TIM1, ENABLE);
    TIM_Cmd(TIM1, ENABLE);
}

// Function to set servo motor angle
void Set_Servo_Angle(uint16_t angle)
{
    // Map angle (0-180 degrees) to pulse width (1000-2000 us)
    uint16_t pulseWidth = SERVO_PWM_PULSE_MIN + (angle * (SERVO_PWM_PULSE_MAX - SERVO_PWM_PULSE_MIN) / 180);
    TIM1_PWMOut_Init(pulseWidth);
    Delay_Ms(500); // Wait for servo to reach the desired position
}

// Function to trigger the ultrasonic sensor and read the echo duration
uint32_t Ultrasonic_Read(void)
{
    uint32_t echoTime = 0;

    GPIO_WriteBit(GPIOD, TRIGGER_PIN, SET); // Setting Trigger Pin to send pulses
    Delay_Us(10); // Pulse Width
    GPIO_WriteBit(GPIOD, TRIGGER_PIN, RESET); // Resetting Trigger Pin

    while (GPIO_ReadInputDataBit(GPIOD, ECHO_PIN) == Bit_RESET); // Wait for Echo to go high
    while (GPIO_ReadInputDataBit(GPIOD, ECHO_PIN) == Bit_SET) echoTime++; // Measure the time Echo is high

    return echoTime;
}

// Function to calculate distance from echo time
float Calculate_Distance(uint32_t echoTime)
{
    // Speed of sound in air is 340 m/s or 0.034 cm/us
    // Distance is (time / 2) * speed_of_sound
    return (echoTime / 2.0) * 0.034;
}

// Function to configure GPIO pins for motor control
void GPIO_Motor_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure = {0};

    // Enable GPIO clock for port D
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);

    // Motor A (Left Motor) Pins: IN1 (PD5), IN2 (PD6)
    GPIO_InitStructure.GPIO_Pin = MOTOR_A_FORWARD | MOTOR_A_REVERSE;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOD, &GPIO_InitStructure);

    // Motor B (Right Motor) Pins: IN3 (PD7), IN4 (PD0)
    GPIO_InitStructure.GPIO_Pin = MOTOR_B_FORWARD | MOTOR_B_REVERSE;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOD, &GPIO_InitStructure);
}

// Function to stop both motors
void Stop_Motors(void)
{
    GPIO_WriteBit(GPIOD, MOTOR_A_FORWARD, Bit_RESET);
    GPIO_WriteBit(GPIOD, MOTOR_A_REVERSE, Bit_RESET);
    GPIO_WriteBit(GPIOD, MOTOR_B_FORWARD, Bit_RESET);
    GPIO_WriteBit(GPIOD, MOTOR_B_REVERSE, Bit_RESET);
    printf("DC Motors Stopped\n");
}

// Function to move forward
void Move_Forward(void)
{
    GPIO_WriteBit(GPIOD, MOTOR_A_FORWARD, Bit_SET);
    GPIO_WriteBit(GPIOD, MOTOR_A_REVERSE, Bit_RESET);
    GPIO_WriteBit(GPIOD, MOTOR_B_FORWARD, Bit_SET);
    GPIO_WriteBit(GPIOD, MOTOR_B_REVERSE, Bit_RESET);
    printf("Moving Forward\n");
}

// Function to move in reverse
void Move_Reverse(void)
{
    GPIO_WriteBit(GPIOD, MOTOR_A_FORWARD, Bit_RESET);
    GPIO_WriteBit(GPIOD, MOTOR_A_REVERSE, Bit_SET);
    GPIO_WriteBit(GPIOD, MOTOR_B_FORWARD, Bit_RESET);
    GPIO_WriteBit(GPIOD, MOTOR_B_REVERSE, Bit_SET);
    printf("Moving Reverse\n");
}

// Function to turn left
// Function to turn left
void Turn_Left(void)
{
    GPIO_WriteBit(GPIOD, MOTOR_A_FORWARD, Bit_RESET);
    GPIO_WriteBit(GPIOD, MOTOR_A_REVERSE, Bit_SET);
    GPIO_WriteBit(GPIOD, MOTOR_B_FORWARD, Bit_SET);
    GPIO_WriteBit(GPIOD, MOTOR_B_REVERSE, Bit_RESET);
    printf("Turning Left\n");

    // Wait for 1 second to simulate turning
    Delay_Ms(1000); 
    
    // Reset the servo to 90 degrees (neutral position)
    Set_Servo_Angle(90); // Reset servo
    Delay_Ms(100); // Wait for the servo to stabilize at neutral position
    Move_Reverse(); // Stop motors after turning
}

// Function to turn right
void Turn_Right(void)
{
    GPIO_WriteBit(GPIOD, MOTOR_A_FORWARD, Bit_SET);
    GPIO_WriteBit(GPIOD, MOTOR_A_REVERSE, Bit_RESET);
    GPIO_WriteBit(GPIOD, MOTOR_B_FORWARD, Bit_RESET);
    GPIO_WriteBit(GPIOD, MOTOR_B_REVERSE, Bit_SET);
    printf("Turning Right\n");

    // Wait for 1 second to simulate turning
    Delay_Ms(1000); 

    // Reset the servo to 90 degrees (neutral position)
    Set_Servo_Angle(90); // Reset servo
    Delay_Ms(100); // Wait for the servo to stabilize at neutral position
    Move_Reverse(); // Stop motors after turning
}


// Main function for obstacle avoidance
int main(void)
{
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
    SystemCoreClockUpdate();
    Delay_Init();
    GPIO_Config(); // Initialize ultrasonic sensor pins
    GPIO_Motor_Init(); // Initialize motor control pins
    USART_Printf_Init(115200); // Initialize debug USART

    // Initialize servo motor to 90 degrees (neutral position)
    Set_Servo_Angle(90);
    Delay_Ms(1000); // Wait for servo to stabilize

    while (1)
    {
        // Check forward path distance
        uint32_t echoTime = Ultrasonic_Read();
        float distance = Calculate_Distance(echoTime);
        printf("Distance: %.2f cm\n", distance); // Print the distance

         // Check if distance is greater than threshold
         if (distance > OBSTACLE_THRESHOLD)
         {
             // Path is clear, move forward
             Move_Forward();
         }
         else
         {
             // Obstacle detected, move backward
             Move_Reverse();
        
            // Move servo to left to check left side
            Set_Servo_Angle(0); // Turn servo to 0 degrees (left)
            echoTime = Ultrasonic_Read();
            distance = Calculate_Distance(echoTime);
            printf("Left distance: %.2f cm\n", distance); // Print left side distance

            if (distance > OBSTACLE_THRESHOLD)
            {
                // Path is clear on the left, turn left
                Turn_Left();
                Delay_Ms(1000); // Simulate turning left for 1 second
                Stop_Motors(); // Stop motors after turning
                Delay_Ms(2000); // Wait a bit before next action
            }
            else
            {
                // Left side blocked, check right side
                Move_Reverse();
                Set_Servo_Angle(180); // Turn servo to 180 degrees (right)
                echoTime = Ultrasonic_Read();
                distance = Calculate_Distance(echoTime);
                printf("Right distance: %.2f cm\n", distance); // Print right side distance

                if (distance > OBSTACLE_THRESHOLD)
                {
                    // Path is clear on the right, turn right
                    Turn_Right();
                    Delay_Ms(1000); // Simulate turning right for 1 second
                    Stop_Motors(); // Stop motors after turning
                    Delay_Ms(2000); // Wait a bit before next action
                }
                else
                {
                    // Both sides are blocked, stop
                    Move_Reverse();
                    Stop_Motors();
                    Set_Servo_Angle(90); // Reset servo to neutral position (center)
                    Delay_Ms(1000); // Wait a bit before checking again
                }
            }
        }

        Delay_Ms(100); // Wait for a short period before checking again
    }
}
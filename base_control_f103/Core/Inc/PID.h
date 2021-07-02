/*
*   File name: PID.h
*   Author: Linh Nguyen
*   Last modified: 1/6/2021
*   Description: This header file is for public API function for further call from other modules.
*/
#ifndef __PID_H
#define __PID_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "gpio.h"


#define PI                     3.14159265359


// Define all PID parameters
typedef struct
{   
    double       Kp;
    double       Kd;
    double       Ki;
    volatile double       error;
    volatile double       previous_error;
    volatile double       last_p;
    volatile double       last_i;
    volatile double       last_d;
    double      OUTPUT_MAX;
    double      OUTPUT_MIN;
} PID_TypeDef;


// Struct contains infomation of motor
typedef struct 
{
    volatile double     speed;        // RPM
    volatile double     set_speed;    // Effected by UART
    volatile double     measure;      // Effected by TIM5 Interrupt
    uint32_t            last_tick;
    uint32_t            last_enc_cnt;
    double      pwm_output;
    uint8_t     dir;
    uint16_t    DIR1_PIN;
    uint16_t    DIR2_PIN;
    PID_TypeDef     *pPID_params;
    GPIO_TypeDef    *GPIO;
    volatile uint32_t *PWM_ADDR;
    volatile uint32_t *ENC_ADDR;
}MOTOR_TypeDef;


struct STR_ROBOT_PARAMS
{
    double wheel_diameter; // Wheels diameter in METTER
    double base_distance;    // Distance between 2 wheels
};



/* Public functions prototypes */

void PID_ComputeSpeedRPM(MOTOR_TypeDef *motor, uint32_t tick); // Convert motor speed from pulse/timeframe -> RPM
void PID_ComputeOutput(MOTOR_TypeDef *motor);
void PID_ComputeError(MOTOR_TypeDef *motor);
void PID_SetDuty(MOTOR_TypeDef *motor);
void PID_SetTunings(PID_TypeDef *pPID_Params, double setKp, double setKi, double setKd);
void PID_SetSpeedMPS(MOTOR_TypeDef *motor, int16_t _set_speed_mps);
void PID_SetSpeedRPM(MOTOR_TypeDef *motor, double _set_speed);
void PID_PreProcess(MOTOR_TypeDef *motor, double _set_speed);
double PID_FirstOrderFilter(double raw_sig, double last_filtered_sig, double a);


// Init GPIOs and PID params that relevant to motor.
void PID_MotorInit( MOTOR_TypeDef *motor,               // Motor str address
                    PID_TypeDef *pPID_Params,           // PID str address
                    GPIO_TypeDef* PORT,                 // GPIO PORT for direction control
                    uint16_t PIN1,                      // GPIO PINS for direction control
                    uint16_t PIN2,                      // GPIO PINS for direction control
                    double max_output,                  // Max Output of PWM
                    double min_output,                  // Min Input of PWM
                    volatile uint32_t * TIM_ENC_ADDR,   // TIMER interfacing encoder's counter register address: TIMx->CNT 
                    volatile uint32_t * TIM_PWM_ADDR,   // TIMER Captur and Compare Register address: TIMx->CCRx
                    double* pSet_Params);               // Set PID parameters array

#ifdef __cplusplus
}
#endif

#endif /* __PID_H */
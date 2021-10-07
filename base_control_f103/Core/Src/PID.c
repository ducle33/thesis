/*
*   File name: PID.c
*   Author: Linh Nguyen
*   Last modified: 1/6/2021
*   Description: This contains all interface functions for PID controlling motors
*/

#include "PID.h"
#include "string.h"
#include "math.h"
#include "config.h"
#include "gpio.h"
#include "stm32f1xx_hal_def.h"

// volatile double i_term;
// volatile double p_term;
// volatile double d_term;


#ifdef ENABLE_PID
/* Private function prototypes*/

double PID_Limit(double input, double max, double min);




// Convert encoder pulses count to speed in RPM, tick in milisecond
void PID_ComputeSpeedRPM(MOTOR_TypeDef *motor, uint32_t tick)
{ 
    motor->speed = ((motor->measure) / (MOTOR_PPR * tick))*60000;
}

void PID_SetSpeedRPM(MOTOR_TypeDef *motor, double _set_speed)
{
    if (_set_speed >= 0) 
    {
        motor->dir = 0;
        motor->set_speed = _set_speed;
    } 
    else 
    {
        motor->dir = 1;
        motor->set_speed = - (_set_speed);
    }    
}

// Convert speed in M/S to RPM and write to SET SPEED
void PID_SetSpeedMPS(MOTOR_TypeDef *motor, int16_t _set_speed_mps)
{
    double speed_rps = _set_speed_mps / (PI * ROBOT_WHEEL_DIAMETER);
    motor->set_speed = (double)(speed_rps) * 60;
}

/*
*  Note:
*/
void PID_ComputeError(MOTOR_TypeDef *motor)
{
    motor->pPID_params->error = (motor->set_speed) - (motor->speed);
}

/*
*  Note: 
*/
void PID_ComputeOutput(MOTOR_TypeDef *motor)
{   
    double i_term;
    double p_term;

    double output = 0.0f;
    double _OUTPUT_MAX = motor->pPID_params->OUTPUT_MAX;
    double _OUTPUT_MIN =motor->pPID_params->OUTPUT_MIN;

    PID_TypeDef *PID ;
    PID = motor->pPID_params;

    p_term = PID->Kp*(PID->error);

    // Compute upper limit of Integration Term 
    float i_term_max, i_term_min;
    if (PID->OUTPUT_MAX > p_term)
    {
        i_term_max = PID->OUTPUT_MAX - p_term;
    } 
    else 
    {
        i_term_max = 0.0f;
    }
    // Compute lower limit of Integration Term
    if (PID->OUTPUT_MIN < p_term)
    {
        i_term_min = PID->OUTPUT_MIN - p_term ;
    } 
    else 
    {
        i_term_min = 0.0f;
    }

    i_term = PID->last_i +  PID->Ki * ( (PID->error) + (PID->previous_error) );
    i_term = PID_Limit(i_term, i_term_max, i_term_min);
    
    output = p_term + i_term;
    output = PID_Limit(output, _OUTPUT_MAX, _OUTPUT_MIN);
    
    motor->pwm_output = (output < 0) ? -output : output;
    PID->previous_error = PID->error;
    PID->last_i = i_term;
}

// Limit the signal in range (max,min)
double PID_Limit(double input, double max, double min)
{
    if (input > max)
    {
        return max;
    }
    else if (input < min)
    {
        return min;
    }
    else
    {
        return input;
    }
}



// PID_SetDuty(&(htimx->CCRx->CNT), MOTOR_TypeDef *motor)
void PID_SetDuty(MOTOR_TypeDef *motor)
{
    if (motor->dir == 1)
    {
        HAL_GPIO_WritePin(motor->GPIO, motor->DIR1_PIN, 0);
        HAL_GPIO_WritePin(motor->GPIO, motor->DIR2_PIN, 1);
    }
    else 
    {
        HAL_GPIO_WritePin(motor->GPIO, motor->DIR1_PIN, 1);
        HAL_GPIO_WritePin(motor->GPIO, motor->DIR2_PIN, 0);  
    }
    if (motor->set_speed != 0)
    {
        *(motor->PWM_ADDR) = round(motor->pwm_output);      
    }
    else
    {
        // HAL_GPIO_WritePin(motor->GPIO, motor->DIR1_PIN, 0);
        // HAL_GPIO_WritePin(motor->GPIO, motor->DIR2_PIN, 0);
        *(motor->PWM_ADDR) = 0;
    }
}


void PID_SetTunings(PID_TypeDef *pPID_Params, double setKp, double setKi, double setKd)
{
    pPID_Params->Kp = setKp;
    pPID_Params->Ki = setKi;
    pPID_Params->Kd = setKd;
}


// Init GPIOs and PID params that relevant to motor.
void PID_MotorInit( MOTOR_TypeDef *motor,               // Motor str address
                    PID_TypeDef *pPID_Params,           // PID str address
                    GPIO_TypeDef* PORT,                 // GPIO PORT for direction control
                    uint16_t PIN1,                      // GPIO PINS for direction control
                    uint16_t PIN2,                      // GPIO PINS for direction control
                    double max_output,                  // Max Output of PWM
                    double min_output,                  // Min Input of PWM
                    volatile uint32_t * TIM_ENC_ADDR,   // TIMER interfacing encoder's counter register address: TIMx->CNT 
                    volatile uint32_t * TIM_PWM_ADDR,   // TIMER Capture and Compare Register address: TIMx->CCRx
                    double* pSet_Params)                // Set PID parameters array
{

    #ifndef __GPIO_H__
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    GPIO_InitStruct.Pin = PIN1 | PIN2;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(PORT, &GPIO_InitStruct);
    #endif

    memset(pPID_Params , 0, sizeof(PID_TypeDef) );

    motor->pPID_params = pPID_Params;
    motor->GPIO = PORT;
    motor->DIR1_PIN = PIN1;
    motor->DIR2_PIN = PIN2;

    motor->dir = 1;

    motor->ENC_ADDR = TIM_ENC_ADDR;
    motor->PWM_ADDR = TIM_PWM_ADDR;

    pPID_Params->OUTPUT_MAX = max_output;
    pPID_Params->OUTPUT_MIN = min_output;

    PID_SetTunings(pPID_Params, pSet_Params[0] , pSet_Params[1], pSet_Params[2]);
}


// Note: Nothing
double PID_FirstOrderFilter(double raw_sig, double last_filtered_sig, double a)
{
    return  (a * last_filtered_sig +  (1-a) * raw_sig);
}


// Do sampling and computing speed in RPM and PID Error
void PID_PreProcess(MOTOR_TypeDef *motor, double _set_speed)
{

    // Sampling: get ticks at the same time with getting encoder pos
    uint32_t enc_cnt = *(motor->ENC_ADDR); // Similar to enc_cnt = TIMx->CNT
    uint32_t tick = HAL_GetTick();
    // End of sampling

    uint32_t d_tick = tick - motor->last_tick; // Calc the derivative of time
    if (d_tick <= 0) d_tick = 10;

    uint32_t last_enc_cnt = motor->last_enc_cnt;

    double d_measure = (enc_cnt>last_enc_cnt) ? (enc_cnt-last_enc_cnt) : (last_enc_cnt-enc_cnt);
    // Handle overflow or underflow of encoder interfacing timer
    if (d_measure > 50) d_measure = motor->measure;
    // Filter the measurement
    d_measure  = PID_FirstOrderFilter(d_measure, motor->measure, _1ST_FILTER_ALPHA);

    motor->measure = d_measure;

    PID_ComputeSpeedRPM( motor, d_tick );
    PID_SetSpeedRPM( motor , _set_speed );
    PID_ComputeError( motor );
 
    motor->last_tick = tick;
    motor->last_enc_cnt = enc_cnt;

}


#endif
/*
*   File name: config.h
*   Author: Linh Nguyen
*   Email: dlinhvn@gmail.com
*   Last modified: 1/6/2021
*   Description: This header file contain configurable params of PID controller and orther functionality.
*/


#ifndef _CONFIG_H_
#define _CONFIG_H_


// #define ENABLE_PID

// Config UART
#define RX_BUFFER_SIZE   16
#define TX_BUFFER_SIZE   16


#ifdef ENABLE_PID
// Right motor PID config
#define ENABLE_RIGHT_MOTOR
#define RIGHT_MOTOR_KP  2.0F  
#define RIGHT_MOTOR_KI  0.15F  
#define RIGHT_MOTOR_KD  0.0F  


// Left motor PID config
#define ENABLE_LEFT_MOTOR
#define LEFT_MOTOR_KP  0.0F  
#define LEFT_MOTOR_KI  0.0F  
#define LEFT_MOTOR_KD  0.0F  



#define MOTOR_PPR              800.0f      // Motor Pulses per Rotation
#define ROBOT_WHEEL_DIAMETER   0.065f   
#define ROBOT_WHEEL_BASE       0.300f       // Distance between 2 wheels


#define _1ST_FILTER_ALPHA       0.5         // Parameter of 1ST ORDER FILTER


#ifdef USE_ADVANCED_PID
#define ALPHA                  0.0f         // Parameter for filter of Derivative term. DISABLED
#endif

#endif

#endif
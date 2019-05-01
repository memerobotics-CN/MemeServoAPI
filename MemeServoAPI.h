
/* Define to prevent recursive inclusion -------------------------------------*/

#ifndef __MEME_SERVO_API_H__
#define __MEME_SERVO_API_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/

#include <stdint.h>


/* Exported types ------------------------------------------------------------*/

#ifndef BOOL
#define BOOL uint8_t
#endif

#ifndef NULL
#define NULL ((void*)0)
#endif

#ifndef TRUE
#define TRUE 1
#endif

#ifndef FALSE
#define FALSE 0
#endif


typedef enum
{
    MMS_PROTOCOL_UART,
    MMS_PROTOCOL_I2C
}   MMS_PROTOCOL;


/* Control status */
enum
{
    MMS_CTRL_STATUS_NO_CONTROL,
    MMS_CTRL_STATUS_POSITION_CONTROL,
    MMS_CTRL_STATUS_TIMED_POSITION_CONTROL,
    MMS_CTRL_STATUS_VELOCITY_CONTROL,
    MMS_CTRL_STATUS_PROFILED_VELOCITY_CONTROL,
    MMS_CTRL_STATUS_PROFILED_POSITION_CONTROL,
    MMS_CTRL_STATUS_LEARNING
};


/* Node error callback function prototype */
typedef void (*MMS_NODE_ERROR_CALLBACK)(uint8_t node_addr, uint8_t err);


/* Exported constants --------------------------------------------------------*/

/* Only matched major version could work */
#define FW_VER_MAJOR 3
#define FW_VER_MINOR 0


/* Exported macro ------------------------------------------------------------*/

/*
 * Servo error codes
 */
#define MMS_ERR_MOTOR_STALLED                         (0x00 + 0x01)  /* error */
#define MMS_ERR_ENCODER_OVERFLOW                      (0x00 + 0x02)  /* error */
#define MMS_ERR_ENCODER_UNDERFLOW                     (0x00 + 0x03)  /* error */
#define MMS_ERR_ERROR_MOTOR_OVER_CURRENT              (0x00 + 0x04)  /* error */

/*
 * command related errors
 */
#define MMS_ERR_COMMAND_INVALID_COMMAND               (0x20 + 0x01)  /* error */
#define MMS_ERR_COMMAND_INVALID_SET_COMMAND_BYTECOUNT (0x20 + 0x02)  /* error */
#define MMS_ERR_COMMAND_INVALID_ARGUMENT              (0x20 + 0x03)  /* error */
#define MMS_ERR_COMMAND_INVALID_FOR_MOTOR_STATE       (0x20 + 0x04)  /* error */
#define MMS_ERR_COMMAND_INVALID_GET_BYTECOUNT         (0x20 + 0x05)  /* warning */

/*
 * i2c related errors
 */
#define MMS_ERR_I2C_ERROR_NODEV                       (0x40 + 0x01)  /* error */
#define MMS_ERR_I2C_ERROR_BUS_ERROR                   (0x40 + 0x02)  /* error */
#define MMS_ERR_I2C_ERROR_ARBITRATION_LOST            (0x40 + 0x03)  /* warning */
#define MMS_ERR_I2C_ERROR_INDETERMINATE               (0x40 + 0x04)  /* error */
#define MMS_ERR_I2C_ERROR_WAIT_ON_BUS_READY_TIMEOUT   (0x40 + 0x05)  /* error */
#define MMS_ERR_I2C_ERROR_WAIT_ON_TRANSMIT_TIMEOUT    (0x40 + 0x06)  /* error */
#define MMS_ERR_I2C_ERROR_WAIT_ON_RECEIVE_TIMEOUT     (0x40 + 0x07)  /* error */

#define MMS_ERR_I2C_ERROR_RX_PACKET_OVERWRITTEN       (0x40 + 0x11)  /* error */
#define MMS_ERR_I2C_ERROR_INVALID_RX_BYTECOUNT        (0x40 + 0x12)  /* error */

/*
 * uart related errors
 */
#define MMS_ERR_UART_ERROR_MEMORY_ALLOCATION_ERROR    (0x60 + 0x01)  /* error, during initialization memory was not enough */
#define MMS_ERR_UART_ERROR_RX_BUFFER_EMPTY            (0x60 + 0x02)  /* error, there is no character in buffer or fifo */
#define MMS_ERR_UART_ERROR_RX_FRAME_ERROR             (0x60 + 0x03)  /* error, frame receive error (check host settings) */
#define MMS_ERR_UART_ERROR_RX_PARITY_ERROR            (0x60 + 0x04)  /* error, parity receive error (check host settings) */
#define MMS_ERR_UART_ERROR_RX_BUFFER_OVERFLOW         (0x60 + 0x05)  /* error, buffer overflow, user code needs to call getChar() more frequently */
#define MMS_ERR_UART_ERROR_RX_DATA_OVERRUN            (0x60 + 0x06)  /* error, FIFO overflow, more ISR time is needed */
#define MMS_ERR_UART_ERROR_TX_TIMEOUT                 (0x60 + 0x07)  /* error, no space in Tx buffer for transmit_timeout_miliseconds */

#define MMS_ERR_UART_ERROR_HALF_PACKET                (0x60 + 0x11)  /* error */

/*
 * protocol related errors
 */
#define MMS_ERR_PROTOCOL_ERROR_RESPONSE_TIMEOUT       (0x80 + 0x01)  /* error */
#define MMS_ERR_PROTOCOL_WRONG_LRC                    (0x80 + 0x02)  /* error */



/*
 * error reactions
 */
#define REACTION_NONE 0
#define REACTION_STOP 1
#define REACTION_HALT 2


/*
 * API return values
 */
#define MMS_RESP_SUCCESS                               0x00
#define MMS_RESP_TIMER_FUCN_NOT_SET                    0x01           /* local error, timer function not set, call MMS_SetTimerFunction(...) first */
#define MMS_RESP_INTERFACE_NOT_SET                     0x02           /* local error, send data function not set, call MMS_SetProtocol(...) first */
#define MMS_RESP_TIMEOUT                               0x03           /* local error, time out when waiting node resopnse */
#define MMS_RESP_INVALID_OUT_BUFFER_SIZE               0x04           /* local error, out buffer size not matched to returned data */
#define MMS_RESP_UNMATCHED_ADDR                        0x05           /* local error, unmatched node id between returned and waiting */
#define MMS_RESP_UNMATCHED_CMD                         0x06           /* local error, unmatched command between returned and waiting */
#define MMS_RESP_WRONG_LRC                             0x07           /* local error, lrc error */
#define MMS_RESP_SERVO_ERROR                           0xFF           /* servo error, error call back will be called with servo error codes listed above */


/*
 * Modes for start command
 */
#define MMS_MODE_RESET     0     /* Start in place & reset position */
#define MMS_MODE_ZERO      1     /* Start & move to costumer set zero */
#define MMS_MODE_OFFSET    2     /* Start & make position = encoder - custom offset */
#define MMS_MODE_KEEP      3     /* Start in place & keep position */
#define MMS_MODE_LEARNING  4     /* Enter learning mode */



/* Exported functions ------------------------------------------------------- */

/*
 * Meme Servo API
 */

/**
  * @brief  Set protocol, master address, send data function and receive data function
  * @note   For none intrerupt receive function, pass receive function pointer as parameter
  *         RecvDataImpl and call MMS_OnData(...) in receive function. For interrupt receive
  *         mode, pass NULL as parameter RecvDataImpl and call MMS_OnData(...) in ISR. Call
  *         this before using other get & set funtions
  * @param  protocol: MMS_PROTOCOL_UART or MMS_PROTOCOL_I2C
  * @param  address_master: master node address
  * @param  SendDataImpl: send data function pointer
  * @param  RecvDataImpl: receive data function pointer, NULL for intrerupt receive mode
  * @retval None
  */
void MMS_SetProtocol(
  MMS_PROTOCOL protocol,
  uint8_t address_master,
  void (*SendDataImpl)(uint8_t addr, uint8_t *data, uint8_t size),
  void (*RecvDataImpl)()
);


/**
  * @brief  Set timer releated function
  * @note   Call this before using get & set funtions
  * @param  GetMilliSecondsImpl: pointer of function to get miliseconds
  * @param  DelayMilliSecondsImpl: pointer of function to delay miliseconds
  * @retval None
  */
void MMS_SetTimerFunction(uint32_t (*GetMilliSecondsImpl)(void), void (*DelayMilliSecondsImpl)(uint32_t ms));


/**
  * @brief  Set command timeout value
  * @note   Call this before using get & set funtions. Default value is 255ms
  * @param  timeout: timeout in miliseconds
  * @param  DelayMilliSecondsImpl: pointer of function to delay miliseconds
  * @retval None
  */
void MMS_SetCommandTimeOut(uint16_t timeout);


/**
  * @brief  Feed data to internal parser
  * @note   Call this in data receive function, either RecvDataImpl or ISR
  * @param  data: data byte received
  * @retval None
  */
void MMS_OnData(uint8_t data);



/*
 * Sets
 */

/**
  * @brief  Set PID gain P for servo
  * @note
  * @param  address: address of servo
  * @param  gain_p: gain P to set
  * @param  error_callback: callback function of type MMS_NODE_ERROR_CALLBACK, can be NULL
  * @retval MMS_RESP_SUCCESS: success
  *          Others: failed, see definition of API return values
  */
uint8_t MMS_SetGainP(uint8_t address, uint16_t gain_p, MMS_NODE_ERROR_CALLBACK error_callback);


/**
  * @brief  Set PID gain I for servo
  * @note   Will modify anti-windup to MIN(anti_wind_up, 0x000FFFFF / gainI)
  * @param  address: address of servo
  * @param  gain_i: gain I to set
  * @param  error_callback: callback function of type MMS_NODE_ERROR_CALLBACK, can be NULL
  * @retval MMS_RESP_SUCCESS: success
  *          Others: failed, see definition of API return values
  */
uint8_t MMS_SetGainI(uint8_t address, uint16_t gain_i, MMS_NODE_ERROR_CALLBACK error_callback);


/**
  * @brief  Set PID gain D for servo
  * @note
  * @param  address: address of servo
  * @param  gain_d: gain D to set
  * @param  error_callback: callback function of type MMS_NODE_ERROR_CALLBACK, can be NULL
  * @retval MMS_RESP_SUCCESS: success
  *          Others: failed, see definition of API return values
  */
uint8_t MMS_SetGainD(uint8_t address, uint16_t gain_d, MMS_NODE_ERROR_CALLBACK error_callback);


/**
  * @brief  Set PID intergration anti-windup for servo
  * @note   Actual value will be MIN(anti_wind_up, 0x000FFFFF / gainI)
  * @param  address: address of servo
  * @param  anti_wind_up: anti wind up to set
  * @param  error_callback: callback function of type MMS_NODE_ERROR_CALLBACK, can be NULL
  * @retval MMS_RESP_SUCCESS: success
  *          Others: failed, see definition of API return values
  */
uint8_t MMS_SetAntiWindUp(uint8_t address, uint32_t anti_wind_up, MMS_NODE_ERROR_CALLBACK error_callback);


/**
  * @brief  Set error tolerance for servo
  * @note   Value must not less than 2
  * @param  address: address of servo
  * @param  error_tolerance: error tolerance value to set
  * @param  error_callback: callback function of type MMS_NODE_ERROR_CALLBACK, can be NULL
  * @retval MMS_RESP_SUCCESS: success
  *          Others: failed, see definition of API return values
  */
uint8_t MMS_SetErrorTolerance(uint8_t address, uint16_t error_tolerance, MMS_NODE_ERROR_CALLBACK error_callback);


/**
  * @brief  Set acceleration for profiled move
  * @note
  * @param  address: address of servo
  * @param  acceleration: acceleration value to set
  * @param  error_callback: callback function of type MMS_NODE_ERROR_CALLBACK, can be NULL
  * @retval MMS_RESP_SUCCESS: success
  *          Others: failed, see definition of API return values
  */
uint8_t MMS_SetProfileAcceleration(uint8_t address, uint16_t acceleration, MMS_NODE_ERROR_CALLBACK error_callback);


/**
  * @brief  Set velocity for profiled move
  * @note
  * @param  address: address of servo
  * @param  velocity: velocity value to set, normally [1, 4096]
  * @param  error_callback: callback function of type MMS_NODE_ERROR_CALLBACK, can be NULL
  * @retval MMS_RESP_SUCCESS: success
  *          Others: failed, see definition of API return values
  */
uint8_t MMS_SetProfileVelocity(uint8_t address, uint16_t velocity, MMS_NODE_ERROR_CALLBACK error_callback);


/**
  * @brief  Set PWM dead zone for servo
  * @note   PWM output value that less than this value will be regarded as 0 to eliminate noise
  * @param  address: address of servo
  * @param  pwm_dead_zone: PWM dead zone value to set
  * @param  error_callback: callback function of type MMS_NODE_ERROR_CALLBACK, can be NULL
  * @retval MMS_RESP_SUCCESS: success
  *          Others: failed, see definition of API return values
  */
uint8_t MMS_SetPwmDeadZone(uint8_t address, uint16_t pwm_dead_zone, MMS_NODE_ERROR_CALLBACK error_callback);


/**
  * @brief  Set torque limit for servo
  * @note   Limit tourque by means of limit PWM output value. Servo may not move(will report stalled) if this value is too small.
  * @param  address: address of servo
  * @param  torque_limit: torque limit value to set
  * @param  error_callback: callback function of type MMS_NODE_ERROR_CALLBACK, can be NULL
  * @retval MMS_RESP_SUCCESS: success
  *          Others: failed, see definition of API return values
  */
uint8_t MMS_SetTorqueLimit(uint8_t address, uint16_t torque_limit, MMS_NODE_ERROR_CALLBACK error_callback);


/**
  * @brief  Set current limit for servo
  * @note   If current exceeds current limit value and last for current limit duration, servo will stop
  * @param  address: address of servo
  * @param  current_limit: current limit value(in MA) to set
  * @param  error_callback: callback function of type MMS_NODE_ERROR_CALLBACK, can be NULL
  * @retval MMS_RESP_SUCCESS: success
  *          Others: failed, see definition of API return values
  */
uint8_t MMS_SetCurrentLimit(uint8_t address, uint16_t current_limit, MMS_NODE_ERROR_CALLBACK error_callback);


/**
  * @brief  Set current limit duration for servo
  * @note   If current exceeds current limit value and last for current limit duration, servo will stop
  * @param  address: address of servo
  * @param  limit_duration: current limit duration value(in MS) to set
  * @param  error_callback: callback function of type MMS_NODE_ERROR_CALLBACK, can be NULL
  * @retval MMS_RESP_SUCCESS: success
  *          Others: failed, see definition of API return values
  */
uint8_t MMS_SetCurrentLimitDuration(uint8_t address, uint16_t limit_duration, MMS_NODE_ERROR_CALLBACK error_callback);


/**
  * @brief  Let servo move with specified velocity
  * @note   Unreachable velocity will cause servo stop
  * @param  address: address of servo
  * @param  velocity: moving velocity, in ticks per second, normally [1, 4096]
  * @param  error_callback: callback function of type MMS_NODE_ERROR_CALLBACK, can be NULL
  * @retval MMS_RESP_SUCCESS: success
  *          Others: failed, see definition of API return values
  */
uint8_t MMS_VelocityMove(uint8_t address, int16_t velocity, MMS_NODE_ERROR_CALLBACK error_callback);


/**
  * @brief  Let servo move to absolute position
  * @note   4096 ticks = 1 turn
  * @param  address: address of servo
  * @param  pos: desired absolute position
  * @param  error_callback: callback function of type MMS_NODE_ERROR_CALLBACK, can be NULL
  * @retval MMS_RESP_SUCCESS: success
  *          Others: failed, see definition of API return values
  */
uint8_t MMS_AbsolutePositionMove(uint8_t address, int32_t pos, MMS_NODE_ERROR_CALLBACK error_callback);


/**
  * @brief  Let servo move to relative position
  * @note   4096 ticks = 1 turn
  * @param  address: address of servo
  * @param  pos: desired relative position
  * @param  error_callback: callback function of type MMS_NODE_ERROR_CALLBACK, can be NULL
  * @retval MMS_RESP_SUCCESS: success
  *          Others: failed, see definition of API return values
  */
uint8_t MMS_RelativePositionMove(uint8_t address, int32_t pos, MMS_NODE_ERROR_CALLBACK error_callback);



/**
  * @brief  Let servo move to position with desired time
  * @note   4096 ticks = 1 turn
  * @param  address: address of servo
  * @param  time: desired time in ms
  * @param  pos: desired relative position
  * @param  error_callback: callback function of type MMS_NODE_ERROR_CALLBACK, can be NULL
  * @retval MMS_RESP_SUCCESS: success
  *          Others: failed, see definition of API return values
  */
uint8_t MMS_TimedAbsolutePositionMove(uint8_t address, uint16_t time, int32_t pos, MMS_NODE_ERROR_CALLBACK error_callback);


/**
  * @brief  Let servo speed up/down to specified velocity with set acceleration, then move with specified velocity
  * @note   Unreachable velocity will cause servo stop
  * @param  address: address of servo
  * @param  velocity: moving velocity, in ticks per second, normally [1, 4096]
  * @param  error_callback: callback function of type MMS_NODE_ERROR_CALLBACK, can be NULL
  * @retval MMS_RESP_SUCCESS: success
  *          Others: failed, see definition of API return values
  */
uint8_t MMS_ProfiledVelocityMove(uint8_t address, int16_t velocity, MMS_NODE_ERROR_CALLBACK error_callback);


/**
  * @brief  Let servo move to absolute position using trapezidal curve, with profiled acceleration and profiled velocity
  * @note
  * @param  address: address of servo
  * @param  pos: absolute position
  * @param  error_callback: callback function of type MMS_NODE_ERROR_CALLBACK, can be NULL
  * @retval MMS_RESP_SUCCESS: success
  *          Others: failed, see definition of API return values
  */
uint8_t MMS_ProfiledAbsolutePositionMove(uint8_t address, int32_t pos, MMS_NODE_ERROR_CALLBACK error_callback);


/**
  * @brief  Let servo move to relative position using trapezidal curve, with profiled acceleration and profiled velocity
  * @note
  * @param  address: address of servo
  * @param  pos: relative position
  * @param  error_callback: callback function of type MMS_NODE_ERROR_CALLBACK, can be NULL
  * @retval MMS_RESP_SUCCESS: success
  *          Others: failed, see definition of API return values
  */
uint8_t MMS_ProfiledRelativePositionMove(uint8_t address, int32_t pos, MMS_NODE_ERROR_CALLBACK error_callback);


/**
  * @brief  Set velocity move set point
  * @note   Servo starts to move after MMS_GlobalMove() called
  * @param  address: address of servo
  * @param  velocity: moving velocity, in ticks per second, normally [1, 4096]
  * @param  error_callback: callback function of type MMS_NODE_ERROR_CALLBACK, can be NULL
  * @retval MMS_RESP_SUCCESS: success
  *          Others: failed, see definition of API return values
  */
uint8_t MMS_SetVelocitySetpoint(uint8_t address, int16_t velocity, MMS_NODE_ERROR_CALLBACK error_callback);


/**
  * @brief  Set move to absolute position set point
  * @note   Servo starts to move after MMS_GlobalMove() called
  * @param  address: address of servo
  * @param  pos: absolute position
  * @param  error_callback: callback function of type MMS_NODE_ERROR_CALLBACK, can be NULL
  * @retval MMS_RESP_SUCCESS: success
  *          Others: failed, see definition of API return values
  */
uint8_t MMS_SetAbsolutePositionSetpoint(uint8_t address, int32_t pos, MMS_NODE_ERROR_CALLBACK error_callback);


/**
  * @brief  Set move to relative position set point
  * @note   Servo starts to move after MMS_GlobalMove() called
  * @param  address: address of servo
  * @param  pos: relative position
  * @param  error_callback: callback function of type MMS_NODE_ERROR_CALLBACK, can be NULL
  * @retval MMS_RESP_SUCCESS: success
  *          Others: failed, see definition of API return values
  */
uint8_t MMS_SetRelativePositionSetpoint(uint8_t address, int32_t pos, MMS_NODE_ERROR_CALLBACK error_callback);


/**
  * @brief  Set timed absolute position move set point
  * @note   Servo starts to move after MMS_GlobalMove() called
  * @param  address: address of servo
  * @param  time: desired time in ms
  * @param  pos: ausolute position
  * @param  error_callback: callback function of type MMS_NODE_ERROR_CALLBACK, can be NULL
  * @retval MMS_RESP_SUCCESS: success
  *          Others: failed, see definition of API return values
  */
uint8_t MMS_SetTimedAbsolutePositionSetpoint(uint8_t address, uint16_t time, int32_t pos, MMS_NODE_ERROR_CALLBACK error_callback);


/**
  * @brief  Set profiled velocity move set point
  * @note   Servo starts to move after MMS_GlobalMove() called
  * @param  address: address of servo
  * @param  velocity: moving velocity, in ticks per second, normally [1, 4096]
  * @param  error_callback: callback function of type MMS_NODE_ERROR_CALLBACK, can be NULL
  * @retval MMS_RESP_SUCCESS: success
  *          Others: failed, see definition of API return values
  */
uint8_t MMS_SetProfiledVelocitySetpoint(uint8_t address, int16_t velocity, MMS_NODE_ERROR_CALLBACK error_callback);


/**
  * @brief  Set profiled absolute position move set point
  * @note   Servo starts to move after MMS_GlobalMove() called
  * @param  address: address of servo
  * @param  pos: absolute position
  * @param  error_callback: callback function of type MMS_NODE_ERROR_CALLBACK, can be NULL
  * @retval MMS_RESP_SUCCESS: success
  *          Others: failed, see definition of API return values
  */
uint8_t MMS_SetProfiledAbsolutePositionSetpoint(uint8_t address, int32_t pos, MMS_NODE_ERROR_CALLBACK error_callback);


/**
  * @brief  Set profiled relative position move set point
  * @note   Servo starts to move after MMS_GlobalMove() called
  * @param  address: address of servo
  * @param  pos: relative position
  * @param  error_callback: callback function of type MMS_NODE_ERROR_CALLBACK, can be NULL
  * @retval MMS_RESP_SUCCESS: success
  *          Others: failed, see definition of API return values
  */
uint8_t MMS_SetProfiledRelativePositionSetpoint(uint8_t address, int32_t pos, MMS_NODE_ERROR_CALLBACK error_callback);


/**
  * @brief  Configure digital IO inpput/output
  * @note   Set input/output for digital IO 0 ~ digital IO 3
  * @param  address: address of servo
  * @param  io_dir: IO dir, bit 0 for digital IO 0, 0 as input, 1 as output, bit 1/2/3 for digital IO 1/2/3.
  * @param  error_callback: callback function of type MMS_NODE_ERROR_CALLBACK, can be NULL
  * @retval MMS_RESP_SUCCESS: success
  *          Others: failed, see definition of API return values
  */
uint8_t MMS_ConfigureDigitalIO(uint8_t address, uint8_t io_dir, MMS_NODE_ERROR_CALLBACK error_callback);


/**
  * @brief  Configure digital IO output level
  * @note   Set input pullup or output level for digital IO 0 ~ digital IO 3
  * @param  address: address of servo
  * @param  output: bit 0 for digital IO 0, 0 as output low(or no pullup for input mode), 1 as output high(or internal pullup for input mode), bit 1/2/3 for digital IO 1/2/3.
  * @param  error_callback: callback function of type MMS_NODE_ERROR_CALLBACK, can be NULL
  * @retval MMS_RESP_SUCCESS: success
  *          Others: failed, see definition of API return values
  */
uint8_t MMS_SetDigitalOut(uint8_t address, uint8_t output, MMS_NODE_ERROR_CALLBACK error_callback);


/**
  * @brief  Set node address
  * @note   Make real effect after next power up
  * @param  address: address of servo
  * @param  new_address: new address for servo
  * @param  error_callback: callback function of type MMS_NODE_ERROR_CALLBACK, can be NULL
  * @retval MMS_RESP_SUCCESS: success
  *          Others: failed, see definition of API return values
  */
uint8_t MMS_SetNodeID(uint8_t address, uint8_t new_address, MMS_NODE_ERROR_CALLBACK error_callback);


/**
  * @brief  Set local acceptance mask
  * @note   Servo respond to command if "actual node address & local acceptance mask" equals to node id in command, or node id in command is broadcast address(0x00)
  * @param  address: address of servo
  * @param  mask: local acceptance mask for servo
  * @param  error_callback: callback function of type MMS_NODE_ERROR_CALLBACK, can be NULL
  * @retval MMS_RESP_SUCCESS: success
  *          Others: failed, see definition of API return values
  */
uint8_t MMS_SetLocalAcceptanceMask(uint8_t address, uint8_t mask, MMS_NODE_ERROR_CALLBACK error_callback);


/**
  * @brief  Set UART baud rate
  * @note   Make real effect after next power up
  * @param  address: address of servo
  * @param  bund_rate: UART bund rate for servo
  * @param  error_callback: callback function of type MMS_NODE_ERROR_CALLBACK, can be NULL
  * @retval MMS_RESP_SUCCESS: success
  *          Others: failed, see definition of API return values
  */
uint8_t MMS_SetBaudUart(uint8_t address, uint32_t bund_rate, MMS_NODE_ERROR_CALLBACK error_callback);


/**
  * @brief  Set zero position for start mode "MMS_MODE_ZERO"
  * @note
  * @param  address: address of servo
  * @param  error_callback: callback function of type MMS_NODE_ERROR_CALLBACK, can be NULL
  * @retval MMS_RESP_SUCCESS: success
  *          Others: failed, see definition of API return values
  */
uint8_t MMS_SetZeroPosition(uint8_t address, MMS_NODE_ERROR_CALLBACK error_callback);


/**
  * @brief  Reset position for servo
  * @note   Current position will be zero. Can be called either in still status or in moving.
  * @param  address: address of servo
  * @param  error_callback: callback function of type MMS_NODE_ERROR_CALLBACK, can be NULL
  * @retval MMS_RESP_SUCCESS: success
  *          Others: failed, see definition of API return values
  */
uint8_t MMS_ResetPosition(uint8_t address, MMS_NODE_ERROR_CALLBACK error_callback);


/**
  * @brief  Start servo
  * @note
  * @param  address: address of servo
  * @param  mode: start mode, see "Modes for start command" macros
  * @param  error_callback: callback function of type MMS_NODE_ERROR_CALLBACK, can be NULL
  * @retval MMS_RESP_SUCCESS: success
  *          Others: failed, see definition of API return values
  */
uint8_t MMS_StartServo(uint8_t address, uint8_t mode, MMS_NODE_ERROR_CALLBACK error_callback);


/**
  * @brief  Halt servo
  * @note   Servo stops move and keeps current position
  * @param  address: address of servo
  * @param  error_callback: callback function of type MMS_NODE_ERROR_CALLBACK, can be NULL
  * @retval MMS_RESP_SUCCESS: success
  *          Others: failed, see definition of API return values
  */
uint8_t MMS_HaltServo(uint8_t address, MMS_NODE_ERROR_CALLBACK error_callback);


/**
  * @brief  Stop servo
  * @note   Servo stops, motor disabled
  * @param  address: address of servo
  * @param  error_callback: callback function of type MMS_NODE_ERROR_CALLBACK, can be NULL
  * @retval MMS_RESP_SUCCESS: success
  *          Others: failed, see definition of API return values
  */
uint8_t MMS_StopServo(uint8_t address, MMS_NODE_ERROR_CALLBACK error_callback);


/**
  * @brief  Set servo reactions for different errors
  * @note
  * @param  address: address of servo
  * @param  error: servo error, see servo error codes for reference
  * @param  reaction: reaction for error, REACTION_NONE/REACTION_STOP/REACTION_HALT
  * @param  error_callback: callback function of type MMS_NODE_ERROR_CALLBACK, can be NULL
  * @retval MMS_RESP_SUCCESS: success
  *          Others: failed, see definition of API return values
  */
uint8_t MMS_SetErrorReaction(uint8_t address, uint8_t error, uint8_t reaction, MMS_NODE_ERROR_CALLBACK error_callback);


/**
  * @brief  Clear any errors in error queue
  * @note
  * @param  address: address of servo
  * @param  error_callback: callback function of type MMS_NODE_ERROR_CALLBACK, can be NULL
  * @retval MMS_RESP_SUCCESS: success
  *          Others: failed, see definition of API return values
  */
uint8_t MMS_ResetError(uint8_t address, MMS_NODE_ERROR_CALLBACK error_callback);


/**
  * @brief  Set error reporting level
  * @note
  * @param  address: address of servo
  * @param  level: 0/1, if 1, errors marked as "Warning" will not be reported as error by servo. Default 0.
  * @param  error_callback: callback function of type MMS_NODE_ERROR_CALLBACK, can be NULL
  * @retval MMS_RESP_SUCCESS: success
  *          Others: failed, see definition of API return values
  */
uint8_t MMS_SetErrorReportingLevel(uint8_t address, uint8_t level, MMS_NODE_ERROR_CALLBACK error_callback);



/*
 * Gets
 */

/**
  * @brief  Get firmware version
  * @note
  * @param  address: address of servo
  * @param  version: pointer to uint16_t type var, when success, the major version will be stored in high byte,
  *                  while the minor ver in the low byte 
  * @param  error_callback: callback function of type MMS_NODE_ERROR_CALLBACK, can be NULL
  * @retval MMS_RESP_SUCCESS: success
  *          Others: failed, see definition of API return values
  */
uint8_t MMS_GetFirmwareVersion(uint8_t address, uint16_t *version, MMS_NODE_ERROR_CALLBACK error_callback);


/**
  * @brief  Get PID gain p
  * @note
  * @param  address: address of servo
  * @param  gain_p: pointer to uint16_t type var to store PID gain i
  * @param  error_callback: callback function of type MMS_NODE_ERROR_CALLBACK, can be NULL
  * @retval MMS_RESP_SUCCESS: success
  *          Others: failed, see definition of API return values
  */
uint8_t MMS_GetGainP(uint8_t address, uint16_t *gain_p, MMS_NODE_ERROR_CALLBACK error_callback);


/**
  * @brief  Get PID gain i
  * @note
  * @param  address: address of servo
  * @param  gain_i: pointer to uint16_t type var to store PID gain i
  * @param  error_callback: callback function of type MMS_NODE_ERROR_CALLBACK, can be NULL
  * @retval MMS_RESP_SUCCESS: success
  *          Others: failed, see definition of API return values
  */
uint8_t MMS_GetGainI(uint8_t address, uint16_t *gain_i, MMS_NODE_ERROR_CALLBACK error_callback);


/**
  * @brief  Get PID gain d
  * @note
  * @param  address: address of servo
  * @param  gain_d: pointer to uint16_t type var to store PID gain d
  * @param  error_callback: callback function of type MMS_NODE_ERROR_CALLBACK, can be NULL
  * @retval MMS_RESP_SUCCESS: success
  *          Others: failed, see definition of API return values
  */
uint8_t MMS_GetGainD(uint8_t address, uint16_t *gain_d, MMS_NODE_ERROR_CALLBACK error_callback);


/**
  * @brief  Get anti-windup value
  * @note
  * @param  address: address of servo
  * @param  anti_wind_up: pointer to uint32_t type var to store anti-windup
  * @param  error_callback: callback function of type MMS_NODE_ERROR_CALLBACK, can be NULL
  * @retval MMS_RESP_SUCCESS: success
  *          Others: failed, see definition of API return values
  */
uint8_t MMS_GetAntiWindUp(uint8_t address, uint32_t *anti_wind_up, MMS_NODE_ERROR_CALLBACK error_callback);


/**
  * @brief  Get error tolerance
  * @note
  * @param  address: address of servo
  * @param  error_tolerance: pointer to uint16_t type var to store error tolerance
  * @param  error_callback: callback function of type MMS_NODE_ERROR_CALLBACK, can be NULL
  * @retval MMS_RESP_SUCCESS: success
  *          Others: failed, see definition of API return values
  */
uint8_t MMS_GetErrorTolerance(uint8_t address, uint16_t *error_tolerance, MMS_NODE_ERROR_CALLBACK error_callback);


/**
  * @brief  Get profile acceleration
  * @note
  * @param  address: address of servo
  * @param  acceleration: pointer to uint16_t type var to store profile acceleration
  * @param  error_callback: callback function of type MMS_NODE_ERROR_CALLBACK, can be NULL
  * @retval MMS_RESP_SUCCESS: success
  *          Others: failed, see definition of API return values
  */
uint8_t MMS_GetProfileAcceleration(uint8_t address, uint16_t *acceleration, MMS_NODE_ERROR_CALLBACK error_callback);


/**
  * @brief  Get profile velocity
  * @note
  * @param  address: address of servo
  * @param  velocity: pointer to uint16_t type var to store profile velocity
  * @param  error_callback: callback function of type MMS_NODE_ERROR_CALLBACK, can be NULL
  * @retval MMS_RESP_SUCCESS: success
  *          Others: failed, see definition of API return values
  */
uint8_t MMS_GetProfileVelocity(uint8_t address, uint16_t *velocity, MMS_NODE_ERROR_CALLBACK error_callback);


/**
  * @brief  Get pwm dead zone
  * @note
  * @param  address: address of servo
  * @param  pwm_dead_zone: pointer to uint16_t type var to store pwm dead zone
  * @param  error_callback: callback function of type MMS_NODE_ERROR_CALLBACK, can be NULL
  * @retval MMS_RESP_SUCCESS: success
  *          Others: failed, see definition of API return values
  */
uint8_t MMS_GetPwmDeadZone(uint8_t address, uint16_t *pwm_dead_zone, MMS_NODE_ERROR_CALLBACK error_callback);


/**
  * @brief  Get torque limit
  * @note
  * @param  address: address of servo
  * @param  torque_limit: pointer to uint16_t type var to store torque limit
  * @param  error_callback: callback function of type MMS_NODE_ERROR_CALLBACK, can be NULL
  * @retval MMS_RESP_SUCCESS: success
  *          Others: failed, see definition of API return values
  */
uint8_t MMS_GetTorqueLimit(uint8_t address, uint16_t *torque_limit, MMS_NODE_ERROR_CALLBACK error_callback);


/**
  * @brief  Get current limit
  * @note
  * @param  address: address of servo
  * @param  current_limit: pointer to uint16_t type var to store current limit
  * @param  error_callback: callback function of type MMS_NODE_ERROR_CALLBACK, can be NULL
  * @retval MMS_RESP_SUCCESS: success
  *          Others: failed, see definition of API return values
  */
uint8_t MMS_GetCurrentLimit(uint8_t address, uint16_t *current_limit, MMS_NODE_ERROR_CALLBACK error_callback);


/**
  * @brief  Get current limit duration
  * @note
  * @param  address: address of servo
  * @param  limit_duration: pointer to uint16_t type var to store current limit duration
  * @param  error_callback: callback function of type MMS_NODE_ERROR_CALLBACK, can be NULL
  * @retval MMS_RESP_SUCCESS: success
  *          Others: failed, see definition of API return values
  */
uint8_t MMS_GetCurrentLimitDuration(uint8_t address, uint16_t *limit_duration, MMS_NODE_ERROR_CALLBACK error_callback);


/**
  * @brief  Get configured digitl IO direction
  * @note
  * @param  address: address of servo
  * @param  io_dir: pointer to uint8_t type var to store digital IO direction
  * @param  error_callback: callback function of type MMS_NODE_ERROR_CALLBACK, can be NULL
  * @retval MMS_RESP_SUCCESS: success
  *          Others: failed, see definition of API return values
  */
uint8_t MMS_GetDioConfiguration(uint8_t address, uint8_t *io_dir, MMS_NODE_ERROR_CALLBACK error_callback);


/**
  * @brief  Get input value of digital IOs
  * @note
  * @param  address: address of servo
  * @param  in: pointer to uint8_t type var to store digital input value, bit 0 for digital IO 0, etc.
  * @param  error_callback: callback function of type MMS_NODE_ERROR_CALLBACK, can be NULL
  * @retval MMS_RESP_SUCCESS: success
  *          Others: failed, see definition of API return values
  */
uint8_t MMS_GetDigitalIn(uint8_t address, uint8_t *in, MMS_NODE_ERROR_CALLBACK error_callback);


/**
  * @brief  Get analog in
  * @note
  * @param  address: address of servo
  * @param  in: address of array to store analog input values
  * @param  count: pointer to uint8_t type var to tell API the size of array in[] & get the output count
  * @param  error_callback: callback function of type MMS_NODE_ERROR_CALLBACK, can be NULL
  * @retval MMS_RESP_SUCCESS: success
  *          Others: failed, see definition of API return values
  */
uint8_t MMS_GetAnalogIn(uint8_t address, uint16_t in[], uint8_t *count, MMS_NODE_ERROR_CALLBACK error_callback);


/**
  * @brief  Get local acceptance mask
  * @note
  * @param  address: address of servo
  * @param  mask: pointer to uint8_t type var to store local acceptance mask
  * @param  error_callback: callback function of type MMS_NODE_ERROR_CALLBACK, can be NULL
  * @retval MMS_RESP_SUCCESS: success
  *          Others: failed, see definition of API return values
  */
uint8_t MMS_GetLocalAcceptanceMask(uint8_t address, uint8_t *mask, MMS_NODE_ERROR_CALLBACK error_callback);


/**
  * @brief  Get absolute position
  * @note
  * @param  address: address of servo
  * @param  pos: pointer to uint32_t type var to store absolute position
  * @param  error_callback: callback function of type MMS_NODE_ERROR_CALLBACK, can be NULL
  * @retval MMS_RESP_SUCCESS: success
  *          Others: failed, see definition of API return values
  */
uint8_t MMS_GetAbsolutePosition(uint8_t address, int32_t *pos, MMS_NODE_ERROR_CALLBACK error_callback);


/**
  * @brief  Get encoder value
  * @note
  * @param  address: address of servo
  * @param  pos: pointer to uint16_t type var to store encoder value
  * @param  error_callback: callback function of type MMS_NODE_ERROR_CALLBACK, can be NULL
  * @retval MMS_RESP_SUCCESS: success
  *          Others: failed, see definition of API return values
  */
uint8_t MMS_GetEncoderValue(uint8_t address, uint16_t *pos, MMS_NODE_ERROR_CALLBACK error_callback);


/**
  * @brief  Get velocity
  * @note
  * @param  address: address of servo
  * @param  velocity: pointer to uint16_t type var to store velocity
  * @param  error_callback: callback function of type MMS_NODE_ERROR_CALLBACK, can be NULL
  * @retval MMS_RESP_SUCCESS: success
  *          Others: failed, see definition of API return values
  */
uint8_t MMS_GetVelocity(uint8_t address, int16_t *velocity, MMS_NODE_ERROR_CALLBACK error_callback);


/**
  * @brief  Get current
  * @note
  * @param  address: address of servo
  * @param  current: pointer to uint16_t type var to store current
  * @param  error_callback: callback function of type MMS_NODE_ERROR_CALLBACK, can be NULL
  * @retval MMS_RESP_SUCCESS: success
  *          Others: failed, see definition of API return values
  */
uint8_t MMS_GetCurrent(uint8_t address, uint16_t *current, MMS_NODE_ERROR_CALLBACK error_callback);


/**
  * @brief  Get voltage
  * @note
  * @param  address: address of servo
  * @param  voltage: pointer to uint16_t type var to store voltage
  * @param  error_callback: callback function of type MMS_NODE_ERROR_CALLBACK, can be NULL
  * @retval MMS_RESP_SUCCESS: success
  *          Others: failed, see definition of API return values
  */
uint8_t MMS_GetVoltage(uint8_t address, uint16_t *voltage, MMS_NODE_ERROR_CALLBACK error_callback);


/**
  * @brief  Get temprature
  * @note
  * @param  address: address of servo
  * @param  temprature: pointer to uint16_t type var to store temprature
  * @param  error_callback: callback function of type MMS_NODE_ERROR_CALLBACK, can be NULL
  * @retval MMS_RESP_SUCCESS: success
  *          Others: failed, see definition of API return values
  */
uint8_t MMS_GetTemperature(uint8_t address, uint16_t *temprature, MMS_NODE_ERROR_CALLBACK error_callback);


/**
  * @brief  Get control status
  * @note
  * @param  address: address of servo
  * @param  status: pointer to uint8_t type var to store control status(see "Control status")
  * @param  in_position: pointer to uint8_t type var to store in_position, which indicates
  *                      whether the servo is in desired position(0 for no, 1 for yes)
  * @param  error_callback: callback function of type MMS_NODE_ERROR_CALLBACK, can be NULL
  * @retval MMS_RESP_SUCCESS: success
  *          Others: failed, see definition of API return values
  */
uint8_t MMS_GetControlStatus(uint8_t address, uint8_t *status, uint8_t *in_position, MMS_NODE_ERROR_CALLBACK error_callback);


/**
  * @brief  Get error reactions
  * @note
  * @param  address: address of servo
  * @param  error_reactions: address of array to store error reaction values in forms of [error_type_1, reaction_1, error_type_2, reaction_2, ...]
  * @param  count: pointer to uint8_t type var to tell API the size of array error_reactions[] & get the output count
  * @param  error_callback: callback function of type MMS_NODE_ERROR_CALLBACK, can be NULL
  * @retval MMS_RESP_SUCCESS: success
  *          Others: failed, see definition of API return values
  */
uint8_t MMS_GetErrorReaction(uint8_t address, uint8_t error_reactions[], uint8_t *count, MMS_NODE_ERROR_CALLBACK error_callback);


/**
  * @brief  Get errors
  * @note
  * @param  address: address of servo
  * @param  errors: address of array to store errors
  * @param  count: pointer to uint8_t type var to tell API the size of array errors[] & get the output count
  * @param  error_callback: callback function of type MMS_NODE_ERROR_CALLBACK, can be NULL
  * @retval MMS_RESP_SUCCESS: success
  *          Others: failed, see definition of API return values
  */
uint8_t MMS_GetError(uint8_t address, uint8_t errors[], uint8_t *count, MMS_NODE_ERROR_CALLBACK error_callback);


/**
  * @brief  Get error reporting level
  * @note
  * @param  address: address of servo
  * @param  level: pointer to uint16_t type var to store error reporting level
  * @param  error_callback: callback function of type MMS_NODE_ERROR_CALLBACK, can be NULL
  * @retval MMS_RESP_SUCCESS: success
  *          Others: failed, see definition of API return values
  */
uint8_t MMS_GetErrorReportingLevel(uint8_t address, uint8_t *level, MMS_NODE_ERROR_CALLBACK error_callback);


/**
  * @brief  Get warnings
  * @note
  * @param  address: address of servo
  * @param  warnings: address of array to store errors
  * @param  count: pointer to uint8_t type var to tell API the size of array warnings[] & get the output count
  * @param  error_callback: callback function of type MMS_NODE_ERROR_CALLBACK, can be NULL
  * @retval MMS_RESP_SUCCESS: success
  *          Others: failed, see definition of API return values
  */
uint8_t MMS_GetWarning(uint8_t address, uint8_t warnings[], uint8_t *count, MMS_NODE_ERROR_CALLBACK error_callback);



/*
 * Global commands
 */

/**
  * @brief  Global move
  * @note
  * @retval MMS_RESP_SUCCESS: success
  *          Others: failed, see definition of API return values
  */
uint8_t MMS_GlobalMove();


/**
  * @brief  Global start
  * @note
  * @param  mode: start mode, see "Modes for start command" for modes
  * @retval MMS_RESP_SUCCESS: success
  *          Others: failed, see definition of API return values
  */
uint8_t MMS_GlobalStart(uint8_t mode);


/**
  * @brief  Global halt
  * @note
  * @retval MMS_RESP_SUCCESS: success
  *          Others: failed, see definition of API return values
  */
uint8_t MMS_GlobalHalt();


/**
  * @brief  Global stop
  * @note
  * @retval MMS_RESP_SUCCESS: success
  *          Others: failed, see definition of API return values
  */
uint8_t MMS_GlobalStop();


#ifdef __cplusplus
}
#endif
#endif //__MEME_SERVO_API_H__

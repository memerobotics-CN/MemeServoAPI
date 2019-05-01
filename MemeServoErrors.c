
#include "MemeServoErrors.h"

#include "MemeServoAPI.h"


static ERROR_NAME error_names[] =
{
    {MMS_ERR_MOTOR_STALLED,                         "MOTOR_STALLED"},
    {MMS_ERR_ENCODER_OVERFLOW,                      "ENCODER_OVERFLOW"},
    {MMS_ERR_ENCODER_UNDERFLOW,                     "ENCODER_UNDERFLOW"},
    {MMS_ERR_ERROR_MOTOR_OVER_CURRENT,              "ERROR_MOTOR_OVER_CURRENT"},

    {MMS_ERR_COMMAND_INVALID_COMMAND,               "COMMAND_INVALID_COMMAND"},
    {MMS_ERR_COMMAND_INVALID_SET_COMMAND_BYTECOUNT, "COMMAND_INVALID_SET_COMMAND_BYTECOUNT"},
    {MMS_ERR_COMMAND_INVALID_ARGUMENT,              "COMMAND_INVALID_ARGUMENT"},
    {MMS_ERR_COMMAND_INVALID_FOR_MOTOR_STATE,       "COMMAND_INVALID_FOR_MOTOR_STATE"},
    {MMS_ERR_COMMAND_INVALID_GET_BYTECOUNT,         "COMMAND_INVALID_GET_BYTECOUNT"},

    {MMS_ERR_I2C_ERROR_NODEV,                       "I2C_ERROR_NODEV"},
    {MMS_ERR_I2C_ERROR_BUS_ERROR,                   "I2C_ERROR_BUS_ERROR"},
    {MMS_ERR_I2C_ERROR_ARBITRATION_LOST,            "I2C_ERROR_ARBITRATION_LOST"},
    {MMS_ERR_I2C_ERROR_INDETERMINATE,               "I2C_ERROR_INDETERMINATE"},
    {MMS_ERR_I2C_ERROR_WAIT_ON_BUS_READY_TIMEOUT,   "I2C_ERROR_WAIT_ON_BUS_READY_TIMEOUT"},
    {MMS_ERR_I2C_ERROR_WAIT_ON_TRANSMIT_TIMEOUT,    "I2C_ERROR_WAIT_ON_TRANSMIT_TIMEOUT"},
    {MMS_ERR_I2C_ERROR_WAIT_ON_RECEIVE_TIMEOUT,     "I2C_ERROR_WAIT_ON_RECEIVE_TIMEOUT"},

    {MMS_ERR_I2C_ERROR_RX_PACKET_OVERWRITTEN,       "I2C_ERROR_RX_PACKET_OVERWRITTEN"},
    {MMS_ERR_I2C_ERROR_INVALID_RX_BYTECOUNT,        "I2C_ERROR_INVALID_RX_BYTECOUNT"},

    {MMS_ERR_UART_ERROR_MEMORY_ALLOCATION_ERROR,    "UART_ERROR_MEMORY_ALLOCATION_ERROR"},
    {MMS_ERR_UART_ERROR_RX_BUFFER_EMPTY,            "UART_ERROR_RX_BUFFER_EMPTY"},
    {MMS_ERR_UART_ERROR_RX_FRAME_ERROR,             "UART_ERROR_RX_FRAME_ERROR"},
    {MMS_ERR_UART_ERROR_RX_PARITY_ERROR,            "UART_ERROR_RX_PARITY_ERROR"},
    {MMS_ERR_UART_ERROR_RX_BUFFER_OVERFLOW,         "UART_ERROR_RX_BUFFER_OVERFLOW"},
    {MMS_ERR_UART_ERROR_RX_DATA_OVERRUN,            "UART_ERROR_RX_DATA_OVERRUN"},
    {MMS_ERR_UART_ERROR_TX_TIMEOUT,                 "UART_ERROR_TX_TIMEOUT"},

    {MMS_ERR_UART_ERROR_HALF_PACKET,                "UART_ERROR_HALF_PACKET"},

    {MMS_ERR_PROTOCOL_ERROR_RESPONSE_TIMEOUT,       "PROTOCOL_ERROR_RESPONSE_TIMEOUT"},
    {MMS_ERR_PROTOCOL_WRONG_LRC,                    "PROTOCOL_WRONG_LRC"}
};


static REACTION_NAME reaction_names[] =
{
    {REACTION_NONE, "NONE"},
    {REACTION_STOP, "STOP"},
    {REACTION_HALT, "HALT"}
};


const char* MMS_GetErrorName(uint8_t error)
{
    uint8_t i;
    
    for (i=0; i<sizeof(error_names)/sizeof(error_names[0]); i++)
    {
        if (error_names[i].error == error)
        {
            return error_names[i].name;
        }
    }

    return NULL;
}


const char* MMS_GetReactionName(uint8_t reaction)
{
    uint8_t i;
    
    for (i=0; i<sizeof(reaction_names)/sizeof(reaction_names[0]); i++)
    {
        if (reaction_names[i].reaction == reaction)
        {
            return reaction_names[i].name;
        }
    }

    return NULL;
}

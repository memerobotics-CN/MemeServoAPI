
#include "MemeServoAPI.h"

#include <string.h>
#include <stdio.h>

#ifndef MIN
#define MIN(a,b) (((a)<(b))?(a):(b))
#endif

#ifndef MAX
#define MAX(a,b) (((a)>(b))?(a):(b))
#endif


#ifndef NULL
#define NULL ((void*)0)
#endif

//----------------------------------------
// Private data & funtions

typedef enum {
  WAIT_ON_HEADER_0,
  WAIT_ON_HEADER_1,
  WAIT_ON_ADDRESSED_NODE_ID,
  WAIT_ON_OWN_NODE_ID,
  WAIT_ON_COMMAND_ID,
  WAIT_ON_BYTECOUNT,
  WAIT_ON_DATA,
  WAIT_ON_LRC
} MMS_PROTOCOL_DECODER_STATE;

#define MAX_PROTOCOL_DATA_SIZE 64


enum {
  /*0x00*/  MMS_CMD_SET_GAIN_P = 0x00,
  /*0x01*/  MMS_CMD_SET_GAIN_I,
  /*0x02*/  MMS_CMD_SET_GAIN_D,
  /*0x03*/  MMS_CMD_SET_ANTI_WIND_UP,
  /*0x04*/  MMS_CMD_SET_ERROR_TOLERANCE,
  /*0x05*/  MMS_CMD_SET_PROFILE_ACCELERATION,
  /*0x06*/  MMS_CMD_SET_PROFILE_VELOCITY,
  /*0x07*/  MMS_CMD_SET_PWM_DEAD_ZONE,
  /*0x08*/  MMS_CMD_SET_TORQUE_LIMIT,
  /*0x09*/  MMS_CMD_SET_CURRENT_LIMIT,
  /*0x0A*/  MMS_CMD_SET_CURRENT_LIMIT_DURATION,
  /*0x0B*/  MMS_CMD_VELOCITY_MOVE,
  /*0x0C*/  MMS_CMD_ABSOLUTE_POSITION_MOVE,
  /*0x0D*/  MMS_CMD_RELATIVE_POSITION_MOVE,
  /*0x0E*/  MMS_CMD_TIMED_ABSOLUTE_POSITION_MOVE,
  /*0x0F*/  MMS_CMD_PROFILED_VELOCITY_MOVE,
  /*0x10*/  MMS_CMD_PROFILED_ABSOLUTE_POSITION_MOVE,
  /*0x11*/  MMS_CMD_PROFILED_RELATIVE_POSITION_MOVE,
  /*0x12*/  MMS_CMD_SET_VELOCITY_SETPOINT,
  /*0x13*/  MMS_CMD_SET_ABSOLUTE_POSITION_SETPOINT,
  /*0x14*/  MMS_CMD_SET_RELATIVE_POSITION_SETPOINT,
  /*0x15*/  MMS_CMD_SET_TIMED_ABSOLUTE_POSITION_SETPOINT,
  /*0x16*/  MMS_CMD_SET_PROFILED_VELOCITY_SETPOINT,
  /*0x17*/  MMS_CMD_SET_PROFILED_ABSOLUTE_POSITION_SETPOINT,
  /*0x18*/  MMS_CMD_SET_PROFILED_RELATIVE_POSITION_SETPOINT,
  /*0x19*/  MMS_CMD_CONFIGURE_DIGITAL_IO,
  /*0x1A*/  MMS_CMD_SET_DIGITAL_OUT,
  /*0x1B*/  MMS_CMD_SET_NODE_ID,
  /*0x1C*/  MMS_CMD_SET_LOCAL_ACCEPTANCE_MASK,
  /*0x1D*/  MMS_CMD_SET_BAUD_UART,
  /*0x1E*/	MMS_CMD_SET_ZERO_POSITION,
  /*0x1F*/	MMS_CMD_RESET_POSITION,
  /*0x20*/	MMS_CMD_START,
  /*0x21*/	MMS_CMD_HALT,
  /*0x22*/	MMS_CMD_STOP,
  /*0x23*/	MMS_CMD_SET_ERROR_REACTION,
  /*0x24*/	MMS_CMD_RESET_ERROR,
  /*0x25*/	MMS_CMD_SET_ERROR_REPORTING_LEVEL,
  /*0x26*/	MMS_CMD_SET_COMMANDS_END
};


enum {
  /*0x40*/  MMS_CMD_GET_FIRMWARE_VERSION = 0x40,
  /*0x41*/  MMS_CMD_GET_GAIN_P,
  /*0x42*/  MMS_CMD_GET_GAIN_I,
  /*0x43*/  MMS_CMD_GET_GAIN_D,
  /*0x44*/  MMS_CMD_GET_ANTI_WIND_UP,
  /*0x45*/  MMS_CMD_GET_ERROR_TOLERANCE,
  /*0x46*/  MMS_CMD_GET_PROFILE_ACCELERATION,
  /*0x47*/  MMS_CMD_GET_PROFILE_VELOCITY,
  /*0x48*/  MMS_CMD_GET_PWM_DEAD_ZONE,
  /*0x49*/  MMS_CMD_GET_TORQUE_LIMIT,
  /*0x4A*/  MMS_CMD_GET_CURRENT_LIMIT,
  /*0x4B*/  MMS_CMD_GET_CURRENT_LIMIT_DURATION,
  /*0x4C*/  MMS_CMD_GET_DIO_CONFIGURATION,
  /*0x4D*/  MMS_CMD_GET_LOCAL_ACCEPTANCE_MASK,
  /*0x4E*/  MMS_CMD_GET_DIGITAL_IN,
  /*0x4F*/  MMS_CMD_GET_ANALOG_IN,
  /*0x50*/  MMS_CMD_GET_ABSOLUTE_POSITION,
  /*0x51*/  MMS_CMD_GET_ENCODER_VALUE,
  /*0x52*/  MMS_CMD_GET_VELOCITY,
  /*0x53*/  MMS_CMD_GET_CURRENT,
  /*0x54*/  MMS_CMD_GET_VOLTAGE,
  /*0x55*/  MMS_CMD_GET_TEMPRATURE,
  /*0x56*/  MMS_CMD_GET_CTRL_STATUS,
  /*0x57*/  MMS_CMD_GET_ERROR_REACTION,
  /*0x58*/  MMS_CMD_GET_ERROR,
  /*0x59*/  MMS_CMD_GET_ERROR_REPORTING_LEVEL,
  /*0x5A*/  MMS_CMD_GET_WARNING,
  /*0x5B*/  MMS_CMD_GET_COMMANDS_END
};

enum {
  /*0x80*/  MMS_CMD_COMMAND_DO_MOVE = 0x80,
  /*0x81*/  MMS_CMD_GLOBAL_START,
  /*0x82*/  MMS_CMD_GLOBAL_HALT,
  /*0x83*/  MMS_CMD_GLOBAL_STOP,
  /*0x84*/  MMS_CMD_BRC_COMMANDS_END
};


enum {
  /*0xFA*/  MMS_CMD_SERVO_ERROR = 0xFA               /*special command id indicates error, check errors in buffer please*/
};


// Byte name for data in _packet_data
#define NODE_ID_BYTE_NUM 0
#define OWN_ID_BYTE_NUM 1
#define CMD_ID_BYTE_NUM 2
#define DATA_CNT_BYTE_NUM 3
#define DATA_START_BYTE_NUM 4


static uint16_t _timeout = 255; /* ms */
static volatile uint8_t _is_whole_packet = 0;
static volatile MMS_PROTOCOL_DECODER_STATE _decode_state;
static uint8_t _packet_data[MAX_PROTOCOL_DATA_SIZE + 5];  /* node_id, own_id, command_id, byte_cnt, data[], lrc */

static MMS_PROTOCOL _protocol = MMS_PROTOCOL_I2C;
static uint8_t _address_master = 0x01;
static MMS_PROTOCOL_DECODER_STATE _initial_state = WAIT_ON_ADDRESSED_NODE_ID;

static void (*_send_data_impl)(uint8_t addr, uint8_t *data, uint8_t size) = NULL;
static void (*_recv_data_impl)() = NULL;

static void (*_delay_milli_secnods_impl)(uint32_t ms) = NULL;
static uint32_t (*_get_milli_seconds_impl)(void) = NULL;



uint8_t MMS_ProtocolLRC(uint8_t *lrcBytes, uint8_t lrcByteCount)
{
  uint8_t lrc = 0;
  uint8_t i;

  for (i = 0; i < lrcByteCount; i++)
  {
    lrc ^= lrcBytes[i];
  }

  return lrc;
}


/*
 * I2C protocol ommits the header and adressed ID:
 *   <ownID> <commandID> <data byte count> <data1> <data2> ... <dataN> <lrc>
 */

uint8_t MMS_SendCmd(uint8_t addr, uint8_t cmd, uint8_t* data, uint8_t nb_data)
{
  if (_send_data_impl == NULL)
    return MMS_RESP_INTERFACE_NOT_SET;

  if (_delay_milli_secnods_impl == NULL || _get_milli_seconds_impl == NULL)
    return MMS_RESP_TIMER_FUCN_NOT_SET;

  uint8_t lrc = MMS_ProtocolLRC(data, nb_data);
  lrc ^= cmd;
  lrc ^= nb_data;

  uint8_t plop[nb_data + 7];
  plop[0] = 0x55;
  plop[1] = 0xAA;
  plop[2] = addr;
  plop[3] = _address_master;
  plop[4] = cmd;
  plop[5] = nb_data;

  uint8_t i;
  for (i = 0; i < nb_data; i++)
  {
    plop[6 + i] = data[i];
  }

  plop[nb_data + 6] = lrc;

  _is_whole_packet = 0;

  switch(_protocol)
  {
  case MMS_PROTOCOL_UART:
    _send_data_impl(addr, plop, nb_data + 7);
    break;

  case MMS_PROTOCOL_I2C:
    _send_data_impl(addr, plop + 2, nb_data + 5);
    break;
  }

  return MMS_RESP_SUCCESS;
}



// validate command resopnse with time out (in ms)
// return value:
//  0x00: command success, resp_bytes stores response data
//  other: error number
uint8_t MMS_GetResponse(uint8_t addr, uint8_t cmd, uint16_t time_out, void *resp_bytes, uint8_t *byte_count, MMS_NODE_ERROR_CALLBACK error_callback)
{
  // check if there is response data in buffer with time out

  uint32_t wait_start = _get_milli_seconds_impl();
  uint32_t wait_timeout = wait_start + time_out;

  while (_is_whole_packet == 0)
  {
    _delay_milli_secnods_impl(1);

    if (_recv_data_impl != NULL)
      _recv_data_impl();

    if (wait_timeout < _get_milli_seconds_impl())
    {
      _decode_state = _initial_state;

      return MMS_RESP_TIMEOUT;
    }
  }

  // validate lrc
  uint8_t lrc = MMS_ProtocolLRC(_packet_data + CMD_ID_BYTE_NUM, _packet_data[DATA_CNT_BYTE_NUM] + 2); // lrc for command_id, byte_cnt, data[]

  if (lrc != _packet_data[DATA_START_BYTE_NUM + _packet_data[DATA_CNT_BYTE_NUM]])
  {
    _is_whole_packet = 0;
    return MMS_RESP_WRONG_LRC;
  }

  // check if response is error
  if (_packet_data[CMD_ID_BYTE_NUM] == MMS_CMD_SERVO_ERROR)
  {
    if (error_callback)
    {
      uint8_t node_addr = _packet_data[OWN_ID_BYTE_NUM];
      uint8_t nb_err = _packet_data[DATA_CNT_BYTE_NUM]; // count

      uint8_t i;
      
      for (i=0; i<MIN(nb_err, sizeof(_packet_data) - 5/*node_id, own_id, command_id, byte_cnt, lrc*/); i++)
      {
        error_callback(node_addr, _packet_data[i + DATA_START_BYTE_NUM]);
      }
    }

    return MMS_RESP_SERVO_ERROR;
  }

  if (_packet_data[OWN_ID_BYTE_NUM] != addr)
  {
    _is_whole_packet = 0;
    return MMS_RESP_UNMATCHED_ADDR;
  }

  if (_packet_data[CMD_ID_BYTE_NUM] != cmd)
  {
    _is_whole_packet = 0;
    return MMS_RESP_UNMATCHED_CMD;
  }

  uint8_t ret = MMS_RESP_SUCCESS;

  // copy response data
  if (resp_bytes != NULL)
  {
    uint8_t packet_byte_count = _packet_data[DATA_CNT_BYTE_NUM];
    void *p = &(_packet_data[DATA_START_BYTE_NUM]);

    if (cmd == MMS_CMD_GET_WARNING || cmd == MMS_CMD_GET_ERROR || cmd == MMS_CMD_GET_ERROR_REACTION)
    {
      // For those have variable return size
      if (packet_byte_count > *byte_count)
        ret = MMS_RESP_INVALID_OUT_BUFFER_SIZE;
      else
      {
        memcpy(resp_bytes, p, packet_byte_count);
        *byte_count = packet_byte_count;
      }
    }
    else
    {
      if (packet_byte_count != *byte_count)
        ret = MMS_RESP_INVALID_OUT_BUFFER_SIZE;
      else
      {
        memcpy(resp_bytes, p, packet_byte_count);
        *byte_count = packet_byte_count;
      }
    }
  }

  _is_whole_packet = 0;

  return ret;
}



// -------------------------------------------------------
// Servo API


void MMS_SetProtocol(MMS_PROTOCOL protocol,  uint8_t address_master, void (*SendDataImpl)(uint8_t addr, uint8_t *data, uint8_t size), void (*RecvDataImpl)())
{
  _protocol = protocol;
  _address_master = address_master;

  switch (_protocol)
  {
  case MMS_PROTOCOL_UART:
    _initial_state = WAIT_ON_HEADER_0;
    break;

  case MMS_PROTOCOL_I2C:
    _initial_state = WAIT_ON_ADDRESSED_NODE_ID;
    break;
  }

  _decode_state = _initial_state;
  _send_data_impl = SendDataImpl;
  _recv_data_impl = RecvDataImpl;
}


void MMS_SetTimerFunction(uint32_t (*GetMilliSecondsImpl)(void), void (*DelayMilliSecnodsImpl)(uint32_t ms))
{
  _get_milli_seconds_impl = GetMilliSecondsImpl;
  _delay_milli_secnods_impl = DelayMilliSecnodsImpl;
}


void MMS_SetCommandTimeOut(uint16_t timeout)
{
  _timeout = timeout;
}


void MMS_OnData(uint8_t data)
{
//  static uint32_t last_time = 0;
  static uint8_t byte_count;

  if (_is_whole_packet == 1)
    return; /* Old packet is not processed, ignore */

//  uint32_t curr_time = _get_milli_seconds_impl();
//  uint32_t time_elapsed = (uint32_t)(curr_time - last_time);

//  if (time_elapsed > 50)
//    _decode_state = _initial_state; /* Timeout, reset decode status */

//  last_time = curr_time;

  switch(_decode_state)
  {
  case WAIT_ON_HEADER_0:
    if (data == 0x55)
      _decode_state = WAIT_ON_HEADER_1;

    break;

  case WAIT_ON_HEADER_1:
    _decode_state = (data == 0xAA)? WAIT_ON_ADDRESSED_NODE_ID : WAIT_ON_HEADER_0;
    break;

  case WAIT_ON_ADDRESSED_NODE_ID:
    //if (data == _address_master)
    //{
      _decode_state = WAIT_ON_OWN_NODE_ID;
      //_is_whole_packet = 0;
      _packet_data[NODE_ID_BYTE_NUM] = data;
    //}
    //else
    //{
    //  _decode_state = _initial_state;
    //}

    break;

  case WAIT_ON_OWN_NODE_ID:
    _packet_data[OWN_ID_BYTE_NUM] = data;
    _decode_state = WAIT_ON_COMMAND_ID;

    break;

  case WAIT_ON_COMMAND_ID:
    _packet_data[CMD_ID_BYTE_NUM] = data;
    _decode_state = WAIT_ON_BYTECOUNT;

    break;

  case WAIT_ON_BYTECOUNT:
    _packet_data[DATA_CNT_BYTE_NUM] = data;
    byte_count = data;

    if (byte_count > MAX_PROTOCOL_DATA_SIZE)
      _decode_state = _initial_state;
    else if (byte_count > 0)
      _decode_state = WAIT_ON_DATA;
    else
      _decode_state = WAIT_ON_LRC;

    break;

  case WAIT_ON_DATA:
    _packet_data[DATA_START_BYTE_NUM + _packet_data[DATA_CNT_BYTE_NUM] - byte_count--] = data;

    if (byte_count == 0)
      _decode_state =  WAIT_ON_LRC;

    break;

  case WAIT_ON_LRC:
    _packet_data[DATA_START_BYTE_NUM + _packet_data[DATA_CNT_BYTE_NUM]] = data;

    if (_packet_data[NODE_ID_BYTE_NUM] == _address_master)
        _is_whole_packet = 1;

    /*
    printf("Recv from device: 0x55 0xaa ");
    int i;
    for (i=0; i<=DATA_START_BYTE_NUM + _packet_data[DATA_CNT_BYTE_NUM]; i++)
        printf("0x%02x ", _packet_data[i]);
    printf("\n");
    */

    _decode_state = _initial_state;

    break;
  }
}


//
// Set commands

uint8_t MMS_SetGainP(uint8_t address, uint16_t gain_p, MMS_NODE_ERROR_CALLBACK error_callback)
{
  uint8_t err;

  const uint8_t nb_data = 2;
  uint8_t data[nb_data];

  data[0] = (uint8_t)gain_p;
  data[1] = (uint8_t)(gain_p >> 8);

  err = MMS_SendCmd(address, MMS_CMD_SET_GAIN_P, data, nb_data);

  if (err == MMS_RESP_SUCCESS)
    err = MMS_GetResponse(address, MMS_CMD_SET_GAIN_P, _timeout, NULL, NULL, error_callback);

  return err;
}


uint8_t MMS_SetGainI(uint8_t address, uint16_t gain_i, MMS_NODE_ERROR_CALLBACK error_callback)
{
  uint8_t err;

  const uint8_t nb_data = 2;
  uint8_t data[nb_data];

  data[0] = (uint8_t)gain_i;
  data[1] = (uint8_t)(gain_i >> 8);

  err = MMS_SendCmd(address, MMS_CMD_SET_GAIN_I, data, nb_data);

  if (err == MMS_RESP_SUCCESS)
    err = MMS_GetResponse(address, MMS_CMD_SET_GAIN_I, _timeout, NULL, NULL, error_callback);

  return err;
}


uint8_t MMS_SetGainD(uint8_t address, uint16_t gain_d, MMS_NODE_ERROR_CALLBACK error_callback)
{
  uint8_t err;

  const uint8_t nb_data = 2;
  uint8_t data[nb_data];

  data[0] = (uint8_t)gain_d;
  data[1] = (uint8_t)(gain_d >> 8);

  err = MMS_SendCmd(address, MMS_CMD_SET_GAIN_D, data, nb_data);

  if (err == MMS_RESP_SUCCESS)
    err = MMS_GetResponse(address, MMS_CMD_SET_GAIN_D, _timeout, NULL, NULL, error_callback);

  return err;
}


uint8_t MMS_SetAntiWindUp(uint8_t address, uint32_t anti_wind_up, MMS_NODE_ERROR_CALLBACK error_callback)
{
  uint8_t err;

  const uint8_t nb_data = 4;
  uint8_t data[nb_data];

  data[0] = (uint8_t)anti_wind_up;
  data[1] = (uint8_t)(anti_wind_up >> 8);
  data[2] = (uint8_t)(anti_wind_up >> 16);
  data[3] = (uint8_t)(anti_wind_up >> 24);

  err = MMS_SendCmd(address, MMS_CMD_SET_ANTI_WIND_UP, data, nb_data);

  if (err == MMS_RESP_SUCCESS)
    err = MMS_GetResponse(address, MMS_CMD_SET_ANTI_WIND_UP, _timeout, NULL, NULL, error_callback);

  return err;
}


uint8_t MMS_SetErrorTolerance(uint8_t address, uint16_t error_tolerance, MMS_NODE_ERROR_CALLBACK error_callback)
{
  uint8_t err;

  const uint8_t nb_data = 2;
  uint8_t data[nb_data];

  data[0] = (uint8_t)error_tolerance;
  data[1] = (uint8_t)(error_tolerance >> 8);

  err = MMS_SendCmd(address, MMS_CMD_SET_ERROR_TOLERANCE, data, nb_data);

  if (err == MMS_RESP_SUCCESS)
    err = MMS_GetResponse(address, MMS_CMD_SET_ERROR_TOLERANCE, _timeout, NULL, NULL, error_callback);

  return err;
}


uint8_t MMS_SetProfileAcceleration(uint8_t address, uint16_t acceleration, MMS_NODE_ERROR_CALLBACK error_callback)
{
  uint8_t err;

  const uint8_t nb_data = 2;
  uint8_t data[nb_data];

  data[0] = (uint8_t)acceleration;
  data[1] = (uint8_t)(acceleration >> 8);

  err = MMS_SendCmd(address, MMS_CMD_SET_PROFILE_ACCELERATION, data, nb_data);

  if (err == MMS_RESP_SUCCESS)
    err = MMS_GetResponse(address, MMS_CMD_SET_PROFILE_ACCELERATION, _timeout, NULL, NULL, error_callback);

  return err;
}


uint8_t MMS_SetProfileVelocity(uint8_t address, uint16_t velocity, MMS_NODE_ERROR_CALLBACK error_callback)
{
  uint8_t err;

  const int nb_data = 2;
  uint8_t data[nb_data];

  data[0] = (uint8_t)velocity;
  data[1] = (uint8_t)(velocity >> 8);

  err = MMS_SendCmd(address, MMS_CMD_SET_PROFILE_VELOCITY, data, nb_data);

  if (err == MMS_RESP_SUCCESS)
    err = MMS_GetResponse(address, MMS_CMD_SET_PROFILE_VELOCITY, _timeout, NULL, NULL, error_callback);

  return err;
}


uint8_t MMS_SetPwmDeadZone(uint8_t address, uint16_t pwm_dead_zone, MMS_NODE_ERROR_CALLBACK error_callback)
{
  uint8_t err;

  const uint8_t nb_data = 2;
  uint8_t data[nb_data];

  data[0] = (uint8_t)pwm_dead_zone;
  data[1] = (uint8_t)(pwm_dead_zone >> 8);

  err = MMS_SendCmd(address, MMS_CMD_SET_PWM_DEAD_ZONE, data, nb_data);

  if (err == MMS_RESP_SUCCESS)
    err = MMS_GetResponse(address, MMS_CMD_SET_PWM_DEAD_ZONE, _timeout, NULL, NULL, error_callback);

  return err;
}


uint8_t MMS_SetTorqueLimit(uint8_t address, uint16_t torque_limit, MMS_NODE_ERROR_CALLBACK error_callback)
{
  uint8_t err;

  const uint8_t nb_data = 2;
  uint8_t data[nb_data];

  data[0] = (uint8_t)torque_limit;
  data[1] = (uint8_t)(torque_limit >> 8);

  err = MMS_SendCmd(address, MMS_CMD_SET_TORQUE_LIMIT, data, nb_data);

  if (err == MMS_RESP_SUCCESS)
    err = MMS_GetResponse(address, MMS_CMD_SET_TORQUE_LIMIT, _timeout, NULL, NULL, error_callback);

  return err;
}


uint8_t MMS_SetCurrentLimit(uint8_t address, uint16_t current_limit, MMS_NODE_ERROR_CALLBACK error_callback)
{
  uint8_t err;

  const int nb_data = 2;
  uint8_t data[nb_data];

  data[0] = (uint8_t)current_limit;
  data[1] = (uint8_t)(current_limit >> 8);

  err = MMS_SendCmd(address, MMS_CMD_SET_CURRENT_LIMIT, data, nb_data);

  if (err == MMS_RESP_SUCCESS)
    err = MMS_GetResponse(address, MMS_CMD_SET_CURRENT_LIMIT, _timeout, NULL, NULL, error_callback);

  return err;
}


uint8_t MMS_SetCurrentLimitDuration(uint8_t address, uint16_t limit_duration, MMS_NODE_ERROR_CALLBACK error_callback)
{
  uint8_t err;

  const int nb_data = 2;
  uint8_t data[nb_data];

  data[0] = (uint8_t)limit_duration;
  data[1] = (uint8_t)(limit_duration >> 8);

  err = MMS_SendCmd(address, MMS_CMD_SET_CURRENT_LIMIT_DURATION, data, nb_data);

  if (err == MMS_RESP_SUCCESS)
    err = MMS_GetResponse(address, MMS_CMD_SET_CURRENT_LIMIT_DURATION, _timeout, NULL, NULL, error_callback);

  return err;
}


uint8_t MMS_VelocityMove(uint8_t address, int16_t velocity, MMS_NODE_ERROR_CALLBACK error_callback)
{
  uint8_t err;

  const uint8_t nb_data = 2;
  uint8_t data[nb_data];

  data[0] = (uint8_t)velocity;
  data[1] = (uint8_t)(velocity >> 8);

  err = MMS_SendCmd(address, MMS_CMD_VELOCITY_MOVE, data, nb_data);

  if (err == MMS_RESP_SUCCESS)
    err = MMS_GetResponse(address, MMS_CMD_VELOCITY_MOVE, _timeout, NULL, NULL, error_callback);

  return err;
}


uint8_t MMS_AbsolutePositionMove(uint8_t address, int32_t pos, MMS_NODE_ERROR_CALLBACK error_callback)
{
  uint8_t err;

  const uint8_t nb_data = 4;
  uint8_t data[nb_data];

  data[0] = (uint8_t)pos;
  data[1] = (uint8_t)(pos >> 8);
  data[2] = (uint8_t)(pos >> 16);
  data[3] = (uint8_t)(pos >> 24);

  err = MMS_SendCmd(address, MMS_CMD_ABSOLUTE_POSITION_MOVE, data, nb_data);

  if (err == MMS_RESP_SUCCESS)
    err = MMS_GetResponse(address, MMS_CMD_ABSOLUTE_POSITION_MOVE, _timeout, NULL, NULL, error_callback);

  return err;
}


uint8_t MMS_RelativePositionMove(uint8_t address, int32_t pos, MMS_NODE_ERROR_CALLBACK error_callback)
{
  uint8_t err;

  const uint8_t nb_data = 4;
  uint8_t data[nb_data];

  data[0] = (uint8_t)pos;
  data[1] = (uint8_t)(pos >> 8);
  data[2] = (uint8_t)(pos >> 16);
  data[3] = (uint8_t)(pos >> 24);

  err = MMS_SendCmd(address, MMS_CMD_RELATIVE_POSITION_MOVE, data, nb_data);

  if (err == MMS_RESP_SUCCESS)
    err = MMS_GetResponse(address, MMS_CMD_RELATIVE_POSITION_MOVE, _timeout, NULL, NULL, error_callback);

  return err;
}


uint8_t MMS_TimedAbsolutePositionMove(uint8_t address, uint16_t time, int32_t pos, MMS_NODE_ERROR_CALLBACK error_callback)
{
  uint8_t err;

  const uint8_t nb_data = 6;
  uint8_t data[nb_data];

  data[0] = (uint8_t)time;
  data[1] = (uint8_t)(time >> 8);

  data[2] = (uint8_t)pos;
  data[3] = (uint8_t)(pos >> 8);
  data[4] = (uint8_t)(pos >> 16);
  data[5] = (uint8_t)(pos >> 24);

  err = MMS_SendCmd(address, MMS_CMD_TIMED_ABSOLUTE_POSITION_MOVE, data, nb_data);

  if (err == MMS_RESP_SUCCESS)
    err = MMS_GetResponse(address, MMS_CMD_TIMED_ABSOLUTE_POSITION_MOVE, _timeout, NULL, NULL, error_callback);

  return err;
}


uint8_t MMS_ProfiledVelocityMove(uint8_t address, int16_t velocity, MMS_NODE_ERROR_CALLBACK error_callback)
{
  uint8_t err;

  const uint8_t nb_data = 2;
  uint8_t data[nb_data];

  data[0] = (uint8_t)velocity;
  data[1] = (uint8_t)(velocity >> 8);

  err = MMS_SendCmd(address, MMS_CMD_PROFILED_VELOCITY_MOVE, data, nb_data);

  if (err == MMS_RESP_SUCCESS)
    err = MMS_GetResponse(address, MMS_CMD_PROFILED_VELOCITY_MOVE, _timeout, NULL, NULL, error_callback);

  return err;
}


uint8_t MMS_ProfiledAbsolutePositionMove(uint8_t address, int32_t pos, MMS_NODE_ERROR_CALLBACK error_callback)
{
  uint8_t err;

  const uint8_t nb_data = 4;
  uint8_t data[nb_data];

  data[0] = (uint8_t)pos;
  data[1] = (uint8_t)(pos >> 8);
  data[2] = (uint8_t)(pos >> 16);
  data[3] = (uint8_t)(pos >> 24);

  err = MMS_SendCmd(address, MMS_CMD_PROFILED_ABSOLUTE_POSITION_MOVE, data, nb_data);

  if (err == MMS_RESP_SUCCESS)
    err = MMS_GetResponse(address, MMS_CMD_PROFILED_ABSOLUTE_POSITION_MOVE, _timeout, NULL, NULL, error_callback);

  return err;
}


uint8_t MMS_ProfiledRelativePositionMove(uint8_t address, int32_t pos, MMS_NODE_ERROR_CALLBACK error_callback)
{
  uint8_t err;

  const uint8_t nb_data = 4;
  uint8_t data[nb_data];

  data[0] = (uint8_t)pos;
  data[1] = (uint8_t)(pos >> 8);
  data[2] = (uint8_t)(pos >> 16);
  data[3] = (uint8_t)(pos >> 24);

  err = MMS_SendCmd(address, MMS_CMD_PROFILED_RELATIVE_POSITION_MOVE, data, nb_data);

  if (err == MMS_RESP_SUCCESS)
    err = MMS_GetResponse(address, MMS_CMD_PROFILED_RELATIVE_POSITION_MOVE, _timeout, NULL, NULL, error_callback);

  return err;
}


uint8_t MMS_SetVelocitySetpoint(uint8_t address, int16_t velocity, MMS_NODE_ERROR_CALLBACK error_callback)
{
  uint8_t err;

  const uint8_t nb_data = 2;
  uint8_t data[nb_data];

  data[0] = (uint8_t)velocity;
  data[1] = (uint8_t)(velocity >> 8);

  err = MMS_SendCmd(address, MMS_CMD_SET_VELOCITY_SETPOINT, data, nb_data);

  if (err == MMS_RESP_SUCCESS)
    err = MMS_GetResponse(address, MMS_CMD_SET_VELOCITY_SETPOINT, _timeout, NULL, NULL, error_callback);

  return err;
}


uint8_t MMS_SetAbsolutePositionSetpoint(uint8_t address, int32_t pos, MMS_NODE_ERROR_CALLBACK error_callback)
{
  uint8_t err;

  const uint8_t nb_data = 4;
  uint8_t data[nb_data];

  data[0] = (uint8_t)pos;
  data[1] = (uint8_t)(pos >> 8);
  data[2] = (uint8_t)(pos >> 16);
  data[3] = (uint8_t)(pos >> 24);

  err = MMS_SendCmd(address, MMS_CMD_SET_ABSOLUTE_POSITION_SETPOINT, data, nb_data);

  if (err == MMS_RESP_SUCCESS)
    err = MMS_GetResponse(address, MMS_CMD_SET_ABSOLUTE_POSITION_SETPOINT, _timeout, NULL, NULL, error_callback);

  return err;
}


uint8_t MMS_SetRelativePositionSetpoint(uint8_t address, int32_t pos, MMS_NODE_ERROR_CALLBACK error_callback)
{
  uint8_t err;

  const uint8_t nb_data = 4;
  uint8_t data[nb_data];

  data[0] = (uint8_t)pos;
  data[1] = (uint8_t)(pos >> 8);
  data[2] = (uint8_t)(pos >> 16);
  data[3] = (uint8_t)(pos >> 24);

  err = MMS_SendCmd(address, MMS_CMD_SET_RELATIVE_POSITION_SETPOINT, data, nb_data);

  if (err == MMS_RESP_SUCCESS)
    err = MMS_GetResponse(address, MMS_CMD_SET_RELATIVE_POSITION_SETPOINT, _timeout, NULL, NULL, error_callback);

  return err;
}


uint8_t MMS_SetTimedAbsolutePositionSetpoint(uint8_t address, uint16_t time, int32_t pos, MMS_NODE_ERROR_CALLBACK error_callback)
{
  uint8_t err;

  const uint8_t nb_data = 6;
  uint8_t data[nb_data];

  data[0] = (uint8_t)time;
  data[1] = (uint8_t)(time >> 8);

  data[2] = (uint8_t)pos;
  data[3] = (uint8_t)(pos >> 8);
  data[4] = (uint8_t)(pos >> 16);
  data[5] = (uint8_t)(pos >> 24);

  err = MMS_SendCmd(address, MMS_CMD_SET_TIMED_ABSOLUTE_POSITION_SETPOINT, data, nb_data);

  if (err == MMS_RESP_SUCCESS)
    err = MMS_GetResponse(address, MMS_CMD_SET_TIMED_ABSOLUTE_POSITION_SETPOINT, _timeout, NULL, NULL, error_callback);

  return err;
}


uint8_t MMS_SetProfiledVelocitySetpoint(uint8_t address, int16_t velocity, MMS_NODE_ERROR_CALLBACK error_callback)
{
  uint8_t err;

  const uint8_t nb_data = 2;
  uint8_t data[nb_data];

  data[0] = (uint8_t)velocity;
  data[1] = (uint8_t)(velocity >> 8);

  err = MMS_SendCmd(address, MMS_CMD_SET_PROFILED_VELOCITY_SETPOINT, data, nb_data);

  if (err == MMS_RESP_SUCCESS)
    err = MMS_GetResponse(address, MMS_CMD_SET_PROFILED_VELOCITY_SETPOINT, _timeout, NULL, NULL, error_callback);

  return err;
}


uint8_t MMS_SetProfiledAbsolutePositionSetpoint(uint8_t address, int32_t pos, MMS_NODE_ERROR_CALLBACK error_callback)
{
  uint8_t err;

  const uint8_t nb_data = 4;
  uint8_t data[nb_data];

  data[0] = (uint8_t)pos;
  data[1] = (uint8_t)(pos >> 8);
  data[2] = (uint8_t)(pos >> 16);
  data[3] = (uint8_t)(pos >> 24);

  err = MMS_SendCmd(address, MMS_CMD_SET_PROFILED_ABSOLUTE_POSITION_SETPOINT, data, nb_data);

  if (err == MMS_RESP_SUCCESS)
    err = MMS_GetResponse(address, MMS_CMD_SET_PROFILED_ABSOLUTE_POSITION_SETPOINT, _timeout, NULL, NULL, error_callback);

  return err;
}


uint8_t MMS_SetProfiledRelativePositionSetpoint(uint8_t address, int32_t pos, MMS_NODE_ERROR_CALLBACK error_callback)
{
  uint8_t err;

  const uint8_t nb_data = 4;
  uint8_t data[nb_data];

  data[0] = (uint8_t)pos;
  data[1] = (uint8_t)(pos >> 8);
  data[2] = (uint8_t)(pos >> 16);
  data[3] = (uint8_t)(pos >> 24);

  err = MMS_SendCmd(address, MMS_CMD_SET_PROFILED_RELATIVE_POSITION_SETPOINT, data, nb_data);

  if (err == MMS_RESP_SUCCESS)
    err = MMS_GetResponse(address, MMS_CMD_SET_PROFILED_RELATIVE_POSITION_SETPOINT, _timeout, NULL, NULL, error_callback);

  return err;
}


uint8_t MMS_ConfigureDigitalIO(uint8_t address, uint8_t io_dir, MMS_NODE_ERROR_CALLBACK error_callback)
{
  uint8_t err;

  const uint8_t nb_data = 1;
  uint8_t data[nb_data];

  data[0] = (uint8_t)io_dir;

  err = MMS_SendCmd(address, MMS_CMD_CONFIGURE_DIGITAL_IO, data, nb_data);

  if (err == MMS_RESP_SUCCESS)
    err = MMS_GetResponse(address, MMS_CMD_CONFIGURE_DIGITAL_IO, _timeout, NULL, NULL, error_callback);

  return err;
}


/* uint8_t MMS_SetDigitalOut(uint8_t address, uint8_t ch, uint8_t output, ERROR_CALLBACK error_callback); */

/* uint8_t MMS_SetAnalogOut(uint8_t address, uint8_t ch, uint8_t output, ERROR_CALLBACK error_callback); */


uint8_t MMS_SetDigitalOut(uint8_t address, uint8_t output, MMS_NODE_ERROR_CALLBACK error_callback)
{
  uint8_t err;

  const uint8_t nb_data = 1;
  uint8_t data[nb_data];

  data[0] = (uint8_t)output;

  err = MMS_SendCmd(address, MMS_CMD_SET_DIGITAL_OUT, data, nb_data);

  if (err == MMS_RESP_SUCCESS)
    err = MMS_GetResponse(address, MMS_CMD_SET_DIGITAL_OUT, _timeout, NULL, NULL, error_callback);

  return err;
}


uint8_t MMS_SetNodeID(uint8_t address, uint8_t new_address, MMS_NODE_ERROR_CALLBACK error_callback)
{
  uint8_t err;

  const uint8_t nb_data = 1;
  uint8_t data[nb_data];

  data[0] = new_address;

  err = MMS_SendCmd(address, MMS_CMD_SET_NODE_ID, data, nb_data);

  if (err == MMS_RESP_SUCCESS)
    err = MMS_GetResponse(address, MMS_CMD_SET_NODE_ID, _timeout, NULL, NULL, error_callback);

  return err;
}


uint8_t MMS_SetLocalAcceptanceMask(uint8_t address, uint8_t mask, MMS_NODE_ERROR_CALLBACK error_callback)
{
  uint8_t err;

  const uint8_t nb_data = 1;
  uint8_t data[nb_data];

  data[0] = mask;

  err = MMS_SendCmd(address, MMS_CMD_SET_LOCAL_ACCEPTANCE_MASK, data, nb_data);

  if (err == MMS_RESP_SUCCESS)
    err = MMS_GetResponse(address, MMS_CMD_SET_LOCAL_ACCEPTANCE_MASK, _timeout, NULL, NULL, error_callback);

  return err;
}


uint8_t MMS_SetBaudUart(uint8_t address, uint32_t bund_rate, MMS_NODE_ERROR_CALLBACK error_callback)
{
  uint8_t err;

  const uint8_t nb_data = 4;
  uint8_t data[nb_data];

  data[0] = (uint8_t)bund_rate;
  data[1] = (uint8_t)(bund_rate >> 8);
  data[2] = (uint8_t)(bund_rate >> 16);
  data[3] = (uint8_t)(bund_rate >> 24);

  err = MMS_SendCmd(address, MMS_CMD_SET_BAUD_UART, data, nb_data);

  if (err == MMS_RESP_SUCCESS)
    err = MMS_GetResponse(address, MMS_CMD_SET_BAUD_UART, _timeout, NULL, NULL, error_callback);

  return err;
}


uint8_t MMS_SetZeroPosition(uint8_t address, MMS_NODE_ERROR_CALLBACK error_callback)
{
  uint8_t err;

  err = MMS_SendCmd(address, MMS_CMD_SET_ZERO_POSITION, NULL, 0);

  if (err == MMS_RESP_SUCCESS)
    err = MMS_GetResponse(address, MMS_CMD_SET_ZERO_POSITION, _timeout, NULL, NULL, error_callback);

  return err;
}


uint8_t MMS_ResetPosition(uint8_t address, MMS_NODE_ERROR_CALLBACK error_callback)
{
  uint8_t err;

  err = MMS_SendCmd(address, MMS_CMD_RESET_POSITION, NULL, 0);

  if (err == MMS_RESP_SUCCESS)
    err = MMS_GetResponse(address, MMS_CMD_RESET_POSITION, _timeout, NULL, NULL, error_callback);

  return err;
}


uint8_t MMS_StartServo(uint8_t address, uint8_t mode, MMS_NODE_ERROR_CALLBACK error_callback)
{
  uint8_t err;

  const uint8_t nb_data = 1;
  uint8_t data[nb_data];

  data[0] = mode;

  err = MMS_SendCmd(address, MMS_CMD_START, data, nb_data);

  if (err == MMS_RESP_SUCCESS)
    err = MMS_GetResponse(address, MMS_CMD_START, _timeout, NULL, NULL, error_callback);

  return err;
}


uint8_t MMS_HaltServo(uint8_t address, MMS_NODE_ERROR_CALLBACK error_callback)
{
  uint8_t err;

  err = MMS_SendCmd(address, MMS_CMD_HALT, NULL, 0);

  if (err == MMS_RESP_SUCCESS)
    err = MMS_GetResponse(address, MMS_CMD_HALT, _timeout, NULL, NULL, error_callback);

  return err;
}


uint8_t MMS_StopServo(uint8_t address, MMS_NODE_ERROR_CALLBACK error_callback)
{
  uint8_t err;

  err = MMS_SendCmd(address, MMS_CMD_STOP, NULL, 0);

  if (err == MMS_RESP_SUCCESS)
    err = MMS_GetResponse(address, MMS_CMD_STOP, _timeout, NULL, NULL, error_callback);

  return err;
}


uint8_t MMS_SetErrorReaction(uint8_t address, uint8_t error, uint8_t reaction, MMS_NODE_ERROR_CALLBACK error_callback)
{
  uint8_t err;

  const uint8_t nb_data = 2;
  uint8_t data[nb_data];

  data[0] = error;
  data[1] = reaction;

  err = MMS_SendCmd(address, MMS_CMD_SET_ERROR_REACTION, data, nb_data);

  if (err == MMS_RESP_SUCCESS)
    err = MMS_GetResponse(address, MMS_CMD_SET_ERROR_REACTION, _timeout, NULL, NULL, error_callback);

  return err;
}


uint8_t MMS_ResetError(uint8_t address, MMS_NODE_ERROR_CALLBACK error_callback)
{
  uint8_t err;

  err = MMS_SendCmd(address, MMS_CMD_RESET_ERROR, NULL, 0);

  if (err == MMS_RESP_SUCCESS)
    err = MMS_GetResponse(address, MMS_CMD_RESET_ERROR, _timeout, NULL, NULL, error_callback);

  return err;
}


uint8_t MMS_SetErrorReportingLevelServo(uint8_t address, uint8_t level, MMS_NODE_ERROR_CALLBACK error_callback)
{
  uint8_t err;

  const uint8_t nb_data = 1;
  uint8_t data[nb_data];

  data[0] = level;

  err = MMS_SendCmd(address, MMS_CMD_SET_ERROR_REPORTING_LEVEL, data, nb_data);

  if (err == MMS_RESP_SUCCESS)
    err = MMS_GetResponse(address, MMS_CMD_SET_ERROR_REPORTING_LEVEL, _timeout, NULL, NULL, error_callback);

  return err;
}



//
// Get commands

uint8_t MMS_GetFirmwareVersion(uint8_t address, uint16_t *version, MMS_NODE_ERROR_CALLBACK error_callback)
{
  uint8_t err;
  uint8_t byte_count = sizeof(*version);

  err = MMS_SendCmd(address, MMS_CMD_GET_FIRMWARE_VERSION, NULL, 0);

  if (err == MMS_RESP_SUCCESS)
    err = MMS_GetResponse(address, MMS_CMD_GET_FIRMWARE_VERSION, _timeout, version, &byte_count, error_callback);

  return err;
}


uint8_t MMS_GetGainP(uint8_t address, uint16_t *gain_p, MMS_NODE_ERROR_CALLBACK error_callback)
{
  uint8_t err;
  uint8_t byte_count = sizeof(*gain_p);

  err = MMS_SendCmd(address, MMS_CMD_GET_GAIN_P, NULL, 0);

  if (err == MMS_RESP_SUCCESS)
    err = MMS_GetResponse(address, MMS_CMD_GET_GAIN_P, _timeout, gain_p, &byte_count, error_callback);

  return err;
}


uint8_t MMS_GetGainI(uint8_t address, uint16_t *gain_i, MMS_NODE_ERROR_CALLBACK error_callback)
{
  uint8_t err;
  uint8_t byte_count = sizeof(*gain_i);

  err = MMS_SendCmd(address, MMS_CMD_GET_GAIN_I, NULL, 0);

  if (err == MMS_RESP_SUCCESS)
    err = MMS_GetResponse(address, MMS_CMD_GET_GAIN_I, _timeout, gain_i, &byte_count, error_callback);

  return err;
}


uint8_t MMS_GetGainD(uint8_t address, uint16_t *gain_d, MMS_NODE_ERROR_CALLBACK error_callback)
{
  uint8_t err;
  uint8_t byte_count = sizeof(*gain_d);

  err = MMS_SendCmd(address, MMS_CMD_GET_GAIN_D, NULL, 0);

  if (err == MMS_RESP_SUCCESS)
    err = MMS_GetResponse(address, MMS_CMD_GET_GAIN_D, _timeout, gain_d, &byte_count, error_callback);

  return err;
}


uint8_t MMS_GetAntiWindUp(uint8_t address, uint32_t *anti_wind_up, MMS_NODE_ERROR_CALLBACK error_callback)
{
  uint8_t err;
  uint8_t byte_count = sizeof(*anti_wind_up);

  err = MMS_SendCmd(address, MMS_CMD_GET_ANTI_WIND_UP, NULL, 0);

  if (err == MMS_RESP_SUCCESS)
    err = MMS_GetResponse(address, MMS_CMD_GET_ANTI_WIND_UP, _timeout, anti_wind_up, &byte_count, error_callback);

  return err;
}


uint8_t MMS_GetErrorTolerance(uint8_t address, uint16_t *error_tolerance, MMS_NODE_ERROR_CALLBACK error_callback)
{
  uint8_t err;
  uint8_t byte_count = sizeof(*error_tolerance);

  err = MMS_SendCmd(address, MMS_CMD_GET_ERROR_TOLERANCE, NULL, 0);

  if (err == MMS_RESP_SUCCESS)
    err = MMS_GetResponse(address, MMS_CMD_GET_ERROR_TOLERANCE, _timeout, error_tolerance, &byte_count, error_callback);

  return err;
}


uint8_t MMS_GetProfileAcceleration(uint8_t address, uint16_t *acceleration, MMS_NODE_ERROR_CALLBACK error_callback)
{
  uint8_t err;
  uint8_t byte_count = sizeof(*acceleration);

  err = MMS_SendCmd(address, MMS_CMD_GET_PROFILE_ACCELERATION, NULL, 0);

  if (err == MMS_RESP_SUCCESS)
    err = MMS_GetResponse(address, MMS_CMD_GET_PROFILE_ACCELERATION, _timeout, acceleration, &byte_count, error_callback);

  return err;
}


uint8_t MMS_GetProfileVelocity(uint8_t address, uint16_t *velocity, MMS_NODE_ERROR_CALLBACK error_callback)
{
  uint8_t err;
  uint8_t byte_count = sizeof(*velocity);

  err = MMS_SendCmd(address, MMS_CMD_GET_PROFILE_VELOCITY, NULL, 0);

  if (err == MMS_RESP_SUCCESS)
    err = MMS_GetResponse(address, MMS_CMD_GET_PROFILE_VELOCITY, _timeout, velocity, &byte_count, error_callback);

  return err;
}


uint8_t MMS_GetPwmDeadZone(uint8_t address, uint16_t *pwm_dead_zone, MMS_NODE_ERROR_CALLBACK error_callback)
{
  uint8_t err;
  uint8_t byte_count = sizeof(*pwm_dead_zone);

  err = MMS_SendCmd(address, MMS_CMD_GET_PWM_DEAD_ZONE, NULL, 0);

  if (err == MMS_RESP_SUCCESS)
    err = MMS_GetResponse(address, MMS_CMD_GET_PWM_DEAD_ZONE, _timeout, pwm_dead_zone, &byte_count, error_callback);

  return err;
}


uint8_t MMS_GetTorqueLimit(uint8_t address, uint16_t *torque_limit, MMS_NODE_ERROR_CALLBACK error_callback)
{
  uint8_t err;
  uint8_t byte_count = sizeof(*torque_limit);

  err = MMS_SendCmd(address, MMS_CMD_GET_TORQUE_LIMIT, NULL, 0);

  if (err == MMS_RESP_SUCCESS)
    err = MMS_GetResponse(address, MMS_CMD_GET_TORQUE_LIMIT, _timeout, torque_limit, &byte_count, error_callback);

  return err;
}


uint8_t MMS_GetCurrentLimit(uint8_t address, uint16_t *current_limit, MMS_NODE_ERROR_CALLBACK error_callback)
{
  uint8_t err;
  uint8_t byte_count = sizeof(*current_limit);

  err = MMS_SendCmd(address, MMS_CMD_GET_CURRENT_LIMIT, NULL, 0);

  if (err == MMS_RESP_SUCCESS)
    err = MMS_GetResponse(address, MMS_CMD_GET_CURRENT_LIMIT, _timeout, current_limit, &byte_count, error_callback);

  return err;
}


uint8_t MMS_GetCurrentLimitDuration(uint8_t address, uint16_t *limit_duration, MMS_NODE_ERROR_CALLBACK error_callback)
{
  uint8_t err;
  uint8_t byte_count = sizeof(*limit_duration);

  err = MMS_SendCmd(address, MMS_CMD_GET_CURRENT_LIMIT_DURATION, NULL, 0);

  if (err == MMS_RESP_SUCCESS)
    err = MMS_GetResponse(address, MMS_CMD_GET_CURRENT_LIMIT_DURATION, _timeout, limit_duration, &byte_count, error_callback);

  return err;
}


uint8_t MMS_GetDioConfiguration(uint8_t address, uint8_t *io_dir, MMS_NODE_ERROR_CALLBACK error_callback)
{
  uint8_t err;
  uint8_t byte_count = sizeof(*io_dir);

  err = MMS_SendCmd(address, MMS_CMD_GET_DIO_CONFIGURATION, NULL, 0);

  if (err == MMS_RESP_SUCCESS)
    err = MMS_GetResponse(address, MMS_CMD_GET_DIO_CONFIGURATION, _timeout, io_dir, &byte_count, error_callback);

  return err;
}


uint8_t MMS_GetDigitalIn(uint8_t address, uint8_t *in, MMS_NODE_ERROR_CALLBACK error_callback)
{
  uint8_t err;
  uint8_t byte_count = sizeof(*in);

  err = MMS_SendCmd(address, MMS_CMD_GET_DIGITAL_IN, NULL, 0);

  if (err == MMS_RESP_SUCCESS)
    err = MMS_GetResponse(address, MMS_CMD_GET_DIGITAL_IN, _timeout, in, &byte_count, error_callback);

  return err;
}


//uint8_t MMS_GetAnalogIn(uint8_t address, uint8_t ch, uint16_t *in, ERROR_CALLBACK error_callback);
uint8_t MMS_GetAnalogIn(uint8_t address, uint16_t in[], uint8_t *count, MMS_NODE_ERROR_CALLBACK error_callback)
{
  uint8_t err;
  uint8_t byte_count = sizeof(*in) * (*count);

  err = MMS_SendCmd(address, MMS_CMD_GET_ANALOG_IN, NULL, 0);

  if (err == MMS_RESP_SUCCESS)
  {
    err = MMS_GetResponse(address, MMS_CMD_GET_ANALOG_IN, _timeout, in, &byte_count, error_callback);
    *count = byte_count / sizeof(*in);
  }

  return err;
}


uint8_t MMS_GetLocalAcceptanceMask(uint8_t address, uint8_t *mask, MMS_NODE_ERROR_CALLBACK error_callback)
{
  uint8_t err;
  uint8_t byte_count = sizeof(*mask);

  err = MMS_SendCmd(address, MMS_CMD_GET_LOCAL_ACCEPTANCE_MASK, NULL, 0);

  if (err == MMS_RESP_SUCCESS)
    err = MMS_GetResponse(address, MMS_CMD_GET_LOCAL_ACCEPTANCE_MASK, _timeout, mask, &byte_count, error_callback);

  return err;
}


uint8_t MMS_GetAbsolutePosition(uint8_t address, int32_t *pos, MMS_NODE_ERROR_CALLBACK error_callback)
{
  uint8_t err;
  uint8_t byte_count = sizeof(*pos);

  err = MMS_SendCmd(address, MMS_CMD_GET_ABSOLUTE_POSITION, NULL, 0);

  if (err == MMS_RESP_SUCCESS)
    err = MMS_GetResponse(address, MMS_CMD_GET_ABSOLUTE_POSITION, _timeout, pos, &byte_count, error_callback);

  return err;
}


uint8_t MMS_GetEncoderValue(uint8_t address, uint16_t *pos, MMS_NODE_ERROR_CALLBACK error_callback)
{
  uint8_t err;
  uint8_t byte_count = sizeof(*pos);

  err = MMS_SendCmd(address, MMS_CMD_GET_ENCODER_VALUE, NULL, 0);

  if (err == MMS_RESP_SUCCESS)
    err = MMS_GetResponse(address, MMS_CMD_GET_ENCODER_VALUE, _timeout, pos, &byte_count, error_callback);

  return err;
}


uint8_t MMS_GetVelocity(uint8_t address, int16_t *velocity, MMS_NODE_ERROR_CALLBACK error_callback)
{
  uint8_t err;
  uint8_t byte_count = sizeof(*velocity);

  err = MMS_SendCmd(address, MMS_CMD_GET_VELOCITY, NULL, 0);

  if (err == MMS_RESP_SUCCESS)
    err = MMS_GetResponse(address, MMS_CMD_GET_VELOCITY, _timeout, velocity, &byte_count, error_callback);

  return err;
}


uint8_t MMS_GetCurrent(uint8_t address, uint16_t *current, MMS_NODE_ERROR_CALLBACK error_callback)
{
  uint8_t err;
  uint8_t byte_count = sizeof(*current);

  err = MMS_SendCmd(address, MMS_CMD_GET_CURRENT, NULL, 0);

  if (err == MMS_RESP_SUCCESS)
    err = MMS_GetResponse(address, MMS_CMD_GET_CURRENT, _timeout, current, &byte_count, error_callback);

  return err;
}


uint8_t MMS_GetVoltage(uint8_t address, uint16_t *voltage, MMS_NODE_ERROR_CALLBACK error_callback)
{
  uint8_t err;
  uint8_t byte_count = sizeof(*voltage);

  err = MMS_SendCmd(address, MMS_CMD_GET_VOLTAGE, NULL, 0);

  if (err == MMS_RESP_SUCCESS)
    err = MMS_GetResponse(address, MMS_CMD_GET_VOLTAGE, _timeout, voltage, &byte_count, error_callback);

  return err;
}


uint8_t MMS_GetTemperature(uint8_t address, uint16_t *temprature, MMS_NODE_ERROR_CALLBACK error_callback)
{
  uint8_t err;
  uint8_t byte_count = sizeof(*temprature);

  err = MMS_SendCmd(address, MMS_CMD_GET_TEMPRATURE, NULL, 0);

  if (err == MMS_RESP_SUCCESS)
    err = MMS_GetResponse(address, MMS_CMD_GET_TEMPRATURE, _timeout, temprature, &byte_count, error_callback);

  return err;
}


uint8_t MMS_GetControlStatus(uint8_t address, uint8_t *status, uint8_t *in_position, MMS_NODE_ERROR_CALLBACK error_callback)
{
  uint8_t err;
  uint8_t byte_count = sizeof(*status) + sizeof(*in_position);
  uint8_t out[byte_count];

  err = MMS_SendCmd(address, MMS_CMD_GET_CTRL_STATUS, NULL, 0);

  if (err == MMS_RESP_SUCCESS)
  {
    err = MMS_GetResponse(address, MMS_CMD_GET_CTRL_STATUS, _timeout, out, &byte_count, error_callback);
    *status = out[0];
    *in_position = out[1];
  }

  return err;
}


uint8_t MMS_GetErrorReaction(uint8_t address, uint8_t error_reactions[], uint8_t *count, MMS_NODE_ERROR_CALLBACK error_callback)
{
  uint8_t err;
  uint8_t byte_count = sizeof(error_reactions[0]) * (*count);

  err = MMS_SendCmd(address, MMS_CMD_GET_ERROR_REACTION, NULL, 0);

  if (err == MMS_RESP_SUCCESS)
  {
    err = MMS_GetResponse(address, MMS_CMD_GET_ERROR_REACTION, _timeout, error_reactions, &byte_count, error_callback);
    *count = byte_count / sizeof(error_reactions[0]);
  }

  return err;
}


uint8_t MMS_GetError(uint8_t address, uint8_t errors[], uint8_t *count, MMS_NODE_ERROR_CALLBACK error_callback)
{
  uint8_t err;
  uint8_t byte_count = sizeof(errors[0]) * (*count);

  err = MMS_SendCmd(address, MMS_CMD_GET_ERROR, NULL, 0);

  if (err == MMS_RESP_SUCCESS)
  {
    err = MMS_GetResponse(address, MMS_CMD_GET_ERROR, _timeout, errors, &byte_count, error_callback);
    *count = byte_count / sizeof(errors[0]);
  }

  return err;
}


uint8_t MMS_GetErrorReportingLevel(uint8_t address, uint8_t *level, MMS_NODE_ERROR_CALLBACK error_callback)
{
  uint8_t err;
  uint8_t byte_count = sizeof(*level);

  err = MMS_SendCmd(address, MMS_CMD_GET_ERROR_REPORTING_LEVEL, NULL, 0);

  if (err == MMS_RESP_SUCCESS)
    err = MMS_GetResponse(address, MMS_CMD_GET_ERROR_REPORTING_LEVEL, _timeout, level, &byte_count, error_callback);

  return err;
}


uint8_t MMS_GetWarning(uint8_t address, uint8_t warnings[], uint8_t *count, MMS_NODE_ERROR_CALLBACK error_callback)
{
  uint8_t err;
  uint8_t byte_count = sizeof(warnings[0]) * (*count);

  err = MMS_SendCmd(address, MMS_CMD_GET_WARNING, NULL, 0);

  if (err == MMS_RESP_SUCCESS)
  {
    err = MMS_GetResponse(address, MMS_CMD_GET_WARNING, _timeout, warnings, &byte_count, error_callback);
    *count = byte_count / sizeof(warnings[0]);
  }

  return err;
}


//
// Global commands

uint8_t MMS_GlobalMove()
{
  uint8_t err;

  err = MMS_SendCmd(0x00, MMS_CMD_COMMAND_DO_MOVE, NULL, 0);

  // NO response

  return err;
}


uint8_t MMS_GlobalStart(uint8_t mode)
{
  uint8_t err;

  const uint8_t nb_data = 1;
  uint8_t data[nb_data];

  data[0] = mode;
  
  err = MMS_SendCmd(0x00, MMS_CMD_GLOBAL_START, data, nb_data);

  // NO response

  return err;
}


uint8_t MMS_GlobalHalt()
{
  uint8_t err;

  err = MMS_SendCmd(0x00, MMS_CMD_GLOBAL_HALT, NULL, 0);

  // NO response

  return err;
}


uint8_t MMS_GlobalStop()
{
  uint8_t err;

  err = MMS_SendCmd(0x00, MMS_CMD_GLOBAL_STOP, NULL, 0);

  // NO response

  return err;
}

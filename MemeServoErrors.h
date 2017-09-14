
#ifndef __MEME_SERVO_ERRORS_H__
#define __MEME_SERVO_ERRORS_H__

#ifdef __cplusplus
extern "C" {
#endif


#include <stdint.h>

typedef struct
{
    uint8_t reaction;
    const char *name;
}	REACTION_NAME;

typedef struct
{
    uint8_t error;
    const char *name;
}	ERROR_NAME;


const char* MMS_GetErrorName(uint8_t error);
const char* MMS_GetReactionName(uint8_t reaction);

#ifdef __cplusplus
}
#endif

#endif //__MEME_SERVO_ERRORS_H__

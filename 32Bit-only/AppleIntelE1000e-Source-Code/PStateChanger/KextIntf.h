/*
 *  KextIntf.h
 *  PStateChanger
 *
 *
 */
#include <stdint.h>
typedef struct {
	uint32_t frequency;	// MHz
	uint32_t voltage;	// mV
	uint8_t fid;
	uint8_t vid;
	uint8_t did;
	uint8_t pstate;
} PSTATE_CTL_INFO;

typedef struct {
	uint32_t frequency;	// MHz
	uint32_t voltage;	// mV
	uint16_t temperature;	// degree ?
	uint8_t pstate;
	uint8_t pad;
} PSTATE_STATUS_INFO;

int getPstateTable(PSTATE_CTL_INFO*);

int getPstateCurrent(PSTATE_STATUS_INFO*);

int getPstateInterval();

void setPstate(unsigned int newState);
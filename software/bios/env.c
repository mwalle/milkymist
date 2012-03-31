#include <string.h>
#include <net/microudp.h>
#include <hw/flash.h>
#include "env.h"

#define LOCALIP1 192
#define LOCALIP2 168
#define LOCALIP3 0
#define LOCALIP4 42

#define REMOTEIP1 192
#define REMOTEIP2 168
#define REMOTEIP3 0
#define REMOTEIP4 14

unsigned char env_macaddr[6];
unsigned int env_myip;
unsigned int env_serverip;

void env_init()
{
	memcpy(env_macaddr, (void*)FLASH_OFFSET_MAC_ADDRESS, 6);
	env_myip = IPTOINT(LOCALIP1, LOCALIP2, LOCALIP3, LOCALIP4);
	env_serverip = IPTOINT(REMOTEIP1, REMOTEIP2, REMOTEIP3, REMOTEIP4);
}

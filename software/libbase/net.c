#include <stdio.h>
#include <base/net.h>

char *inet_ntoa(unsigned int ip)
{
	static char ipstr[16];

	sprintf(ipstr, "%d.%d.%d.%d",
			(ip >> 24) & 0xff,
			(ip >> 16) & 0xff,
			(ip >> 8) & 0xff,
			ip & 0xff);

	return ipstr;
}

char *enet_ntoa(unsigned char *mac)
{
	static char macstr[18];

	sprintf(macstr, "%02x:%02x:%02x:%02x:%02x:%02x",
			mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

	return macstr;
}

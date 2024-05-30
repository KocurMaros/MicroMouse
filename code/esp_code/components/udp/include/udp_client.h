#ifndef UDP_CLIENT_H
#define UDP_CLIENT_H

#include <stdint.h>

void init_udp(void);
void send_empty_mess();
void send_message(char *message, double *P, double *I, double *D, uint8_t *flag);

#endif
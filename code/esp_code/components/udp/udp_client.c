/* BSD Socket API Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include "include/udp_client.h"
#include <string.h>
#include <sys/param.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "protocol_examples_common.h"

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>
#include "sdkconfig.h"

#define HOST_IP_ADDR "10.42.0.1"        //set here you ip address
#define PORT 3333

static const char *TAG = "udp_client";
char rx_buffer[128];
int addr_family = 0;
int ip_protocol = 0;    
int sock;
struct sockaddr_in dest_addr;
void send_empty_mess(){
    int err = sendto(sock, "micromouse", strlen("micromouse"), 0, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
    if (err < 0) {
        //ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
    }
}
void send_message(char *message, double *P, double *I, double *D, uint8_t *flag){
   // printf("Sending message\n   %s\n",message);
    int err = sendto(sock, message, strlen(message), 0, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
    if (err < 0) {
        //ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
    }

    //RECEIVE MESSAGE
    // socklen_t len = sizeof(dest_addr);
    // err = recvfrom(sock, rx_buffer, sizeof(rx_buffer) - 1, 0, (struct sockaddr *)&dest_addr, &len);
    // printf("Received message\n   %s\n",rx_buffer);
    // if (err < 0) {
    //     //ESP_LOGE(TAG, "recvfrom failed: errno %d", errno);
    //     *flag = 0;
    // } else {
    //     *flag = 0;
    //     rx_buffer[err] = '\0'; // Null-terminate whatever we received and treat like a string
    //     if(rx_buffer[0] != '0'){
    //         *flag = rx_buffer[0] - '0';
    //         char *token = strtok(rx_buffer, ",");
    //         while (token != NULL) {
    //             if(token[0] == 'P'){
    //                 token = strtok(NULL, ",");
    //                 *P = atof(token);
    //             } else if(token[0] == 'I'){
    //                 token = strtok(NULL, ",");
    //                 *I = atof(token);
    //             } else if(token[0] == 'D'){
    //                 token = strtok(NULL, ",");
    //                 *D = atof(token);
    //             }
    //         }
    //     }
    // }
    //TODO receive message with paramaters 
    //server side send 4 parameters flag P I D if flag 0 then no change
}

void init_udp(void){
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    ESP_ERROR_CHECK(example_connect());

    
    dest_addr.sin_addr.s_addr = inet_addr(HOST_IP_ADDR);
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(PORT);
    addr_family = AF_INET;
    ip_protocol = IPPROTO_IP;
    sock = socket(addr_family, SOCK_DGRAM, ip_protocol);
    if (sock < 0) {
        ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
    }
    ESP_LOGI(TAG, "Socket created, sending to %s:%d", HOST_IP_ADDR, PORT);
    // if (sock != -1) {
    //     ESP_LOGE(TAG, "Shutting down socket and restarting...");
    //     shutdown(sock, 0);
    //     close(sock);
    // }
}
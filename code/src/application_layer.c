#include "link_layer.h"
#include <stdio.h>
#include <string.h>

void applicationLayer(const char *serialPort, const char *role, int baudRate, int nTries, int timeout, const char *filename)
{
    LinkLayer connectionParams;

    strncpy(connectionParams.serialPort, serialPort, sizeof(connectionParams.serialPort) - 1);
    connectionParams.serialPort[sizeof(connectionParams.serialPort) - 1] = '\0';  
    

    connectionParams.baudRate = baudRate;
    connectionParams.nRetransmissions = nTries;
    connectionParams.timeout = timeout;

    if (strcmp(role, "tx") == 0) {
        connectionParams.role = LlTx; 
        printf("[INFO] Starting as Transmitter on %s\n", connectionParams.serialPort);
    } else if (strcmp(role, "rx") == 0) {
        connectionParams.role = LlRx;  
        printf("[INFO] Starting as Receiver on %s\n", connectionParams.serialPort);
    } else {
        printf("[ERROR] Invalid role specified. Must be 'tx' or 'rx'.\n");
        return;
    }

    if (llopen(connectionParams) < 0) {
        printf("[ERROR] Failed to establish link layer connection.\n");
        return;
    }

    if (connectionParams.role == LlTx) {
        printf("[INFO] Transmitter waiting for 1 second before sending data...\n");
        sleep(1);  
        
        printf("[INFO] Transmitting data...\n");
        const char testMessage[] = "O EDU E GAY 123456!";
        int bytesWritten = llwrite((unsigned char *)testMessage, strlen(testMessage)-1);

        if (bytesWritten == strlen(testMessage)) {
            printf("[INFO] Successfully transmitted %d bytes.\n", bytesWritten);
        } else {
            printf("[ERROR] Failed to transmit the full message. Sent %d bytes.\n", bytesWritten);
        }
    }

    if (connectionParams.role == LlRx) {
        printf("[INFO] Receiving data...\n");

        unsigned char buffer[256];
        int bytesReceived = 0;
        int retries = 0;
        const int maxRetries = 10;  

        while (retries < maxRetries) {
            bytesReceived = llread(buffer);
            
            if (bytesReceived > 0) {
                printf("[INFO] Received %d bytes: %.*s\n", bytesReceived, bytesReceived, buffer);
                break;  
            } else if (bytesReceived == 0) {
                retries++;
                printf("[INFO] No data received, retrying (%d/%d)...\n", retries, maxRetries);
                sleep(1); 
            } else {
                printf("[ERROR] Error while receiving data. Retrying...\n");
                retries++;
                sleep(1);
            }
        }

        if (retries == maxRetries) {
            printf("[ERROR] No data received after %d retries. Giving up.\n", retries);
        } else {
            printf("[INFO] Reception complete.\n");
        }
    }

    if (llclose(1) < 0) {
        printf("[ERROR] Failed to close the link layer connection.\n");
    }
}

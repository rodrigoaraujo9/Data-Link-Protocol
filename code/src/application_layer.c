#include "application_layer.h"
#include "link_layer.h"
#include <stdio.h>
#include <string.h>

// Application layer function
void applicationLayer(const char *serialPort, const char *role, int baudRate,
                      int nTries, int timeout, const char *filename)
{
    // Setup link layer connection parameters
    LinkLayer connectionParams;
    
    // Copy the serial port string safely
    strncpy(connectionParams.serialPort, serialPort, sizeof(connectionParams.serialPort) - 1);
    connectionParams.serialPort[sizeof(connectionParams.serialPort) - 1] = '\0';  // Ensure null termination
    
    connectionParams.baudRate = baudRate;
    connectionParams.nRetransmissions = nTries;
    connectionParams.timeout = timeout;

    // Open the file to transmit or receive
    FILE *file = NULL;

    if (strcmp(role, "tx") == 0) {
        connectionParams.role = LlTx;  // Set as transmitter
        printf("[INFO] Starting as Transmitter\n");

        // Open file to read data for transmission
        file = fopen(filename, "rb");
        if (file == NULL) {
            printf("[ERROR] Could not open file %s for reading\n", filename);
            return;
        }
    } else if (strcmp(role, "rx") == 0) {
        connectionParams.role = LlRx;  // Set as receiver
        printf("[INFO] Starting as Receiver\n");

        // Open file to write received data
        file = fopen(filename, "wb");
        if (file == NULL) {
            printf("[ERROR] Could not open file %s for writing\n", filename);
            return;
        }
    } else {
        printf("[ERROR] Invalid role specified. Must be 'tx' or 'rx'\n");
        return;
    }

    // Establish connection using llopen
    if (llopen(connectionParams) < 0) {
        printf("[ERROR] Failed to establish link layer connection\n");
        return;
    }

    // Transmit or receive data
    if (connectionParams.role == LlTx) {
        printf("[INFO] Transmitting data...\n");
        unsigned char buffer[256];
        int bytesRead;

        // Read from file and send over link layer
        while ((bytesRead = fread(buffer, 1, sizeof(buffer), file)) > 0) {
            if (llwrite(buffer, bytesRead) < 0) {
                printf("[ERROR] Failed to transmit data\n");
                break;
            }
        }

        printf("[INFO] Transmission complete\n");
    } else if (connectionParams.role == LlRx) {
        printf("[INFO] Receiving data...\n");
        unsigned char buffer[256];
        int bytesReceived;

        // Receive data from the link layer and write to file
        while ((bytesReceived = llread(buffer)) > 0) {
            if (fwrite(buffer, 1, bytesReceived, file) < bytesReceived) {
                printf("[ERROR] Failed to write received data to file\n");
                break;
            }
        }

        printf("[INFO] Reception complete\n");
    }

    // Close the link layer
    if (llclose(1) < 0) {
        printf("[ERROR] Failed to close the link layer connection\n");
    }

    // Close the file
    fclose(file);
}

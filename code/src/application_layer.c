#include "link_layer.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>

// Constants for packet types
#define PACKET_START 0x02
#define PACKET_END 0x03
#define PACKET_DATA 0x01
#define MAX_DATA_SIZE 256

// Function to read the file into memory
unsigned char* readFile(const char* filename, int* fileSize) {
    FILE* file = fopen(filename, "rb");
    if (!file) {
        printf("[ERROR] Failed to open file: %s\n", filename);
        return NULL;
    }

    fseek(file, 0, SEEK_END);
    *fileSize = ftell(file);  // Get the file size
    fseek(file, 0, SEEK_SET);

    unsigned char* fileData = (unsigned char*)malloc(*fileSize);
    if (!fileData) {
        printf("[ERROR] Memory allocation failed for file data\n");
        fclose(file);
        return NULL;
    }

    fread(fileData, 1, *fileSize, file);
    fclose(file);
    return fileData;
}

int writeFile(const char* filename, unsigned char* fileData, int fileSize) {
    // Open the file in binary write mode (wb) to overwrite any existing file
    FILE* file = fopen(filename, "wb");
    if (!file) {
        printf("[ERROR] Failed to open file for writing: %s\n", filename);
        return -1;
    }

    // Write the data to the file
    size_t written = fwrite(fileData, 1, fileSize, file);
    if (written != fileSize) {
        printf("[ERROR] Failed to write the full file to disk. Written: %zu, Expected: %d\n", written, fileSize);
        fclose(file);
        return -1;
    }

    // Close the file to ensure data is flushed
    fclose(file);
    printf("[INFO] File successfully written to: %s\n", filename);
    return 0;
}

// Function to create a control packet (start/end)
unsigned char* createControlPacket(int type, int fileSize, const char* filename, int* packetSize) {
    int filenameLength = strlen(filename);
    *packetSize = 2 + 4 + 2 + filenameLength;

    unsigned char* packet = (unsigned char*)malloc(*packetSize);
    if (!packet) {
        printf("[ERROR] Memory allocation failed for control packet\n");
        return NULL;
    }

    packet[0] = type;  // Packet type (start/end)
    packet[1] = 0x00;  // T1 (file size identifier)
    packet[2] = 0x04;  // L1 (length of file size field)
    packet[3] = (fileSize >> 24) & 0xFF;
    packet[4] = (fileSize >> 16) & 0xFF;
    packet[5] = (fileSize >> 8) & 0xFF;
    packet[6] = fileSize & 0xFF;
    packet[7] = 0x01;  // T2 (filename identifier)
    packet[8] = filenameLength;  // L2 (length of filename)
    memcpy(&packet[9], filename, filenameLength);  // Filename

    return packet;
}


void sendFile(const char* filename) {
    int fileSize = 0;
    unsigned char* fileData = readFile(filename, &fileSize);
    if (!fileData) {
        return; 
    }

    int packetSize;
    unsigned char* startPacket = createControlPacket(PACKET_START, fileSize, filename, &packetSize);
    llwrite(startPacket, packetSize);
    free(startPacket);

    int bytesSent = 0;
    unsigned char dataPacket[MAX_DATA_SIZE + 4];
    int seq = 0;
    while (bytesSent < fileSize) {
        int dataSize = (fileSize - bytesSent > MAX_DATA_SIZE) ? MAX_DATA_SIZE : fileSize - bytesSent;
        dataPacket[0] = PACKET_DATA;
        dataPacket[1] = seq % 256;
        dataPacket[2] = (dataSize >> 8) & 0xFF; 
        dataPacket[3] = dataSize & 0xFF; 
        memcpy(&dataPacket[4], &fileData[bytesSent], dataSize);

        llwrite(dataPacket, dataSize + 4);
        bytesSent += dataSize;
        seq++;
    }


    unsigned char* endPacket = createControlPacket(PACKET_END, fileSize, filename, &packetSize);
    llwrite(endPacket, packetSize);
    free(endPacket);
    free(fileData);

    printf("[INFO] File transmission completed: %s\n", filename);
}

void receiveFile(const char* filename) {
    unsigned char buffer[512];
    int fileSize = 0;
    unsigned char* fileData = NULL;
    int bytesReceived = 0;
    int expectedSeq = 0;  // Variable to track expected sequence number

    // Set the full file path inside the container. You can customize this.
    const char* fullFilePath = filename;

    int receiving = 1;
    while (receiving) {
        printf("[DEBUG] Starting llread\n");
        int packetSize = llread(buffer);
        if (packetSize < 0) {
            printf("[ERROR] Failed to read packet\n");
            continue;
        }

        unsigned char packetType = buffer[0];
        if (packetType == PACKET_START) {
            // Extract the file size from the start packet
            fileSize = (buffer[3] << 24) | (buffer[4] << 16) | (buffer[5] << 8) | buffer[6];
            fileData = (unsigned char*)malloc(fileSize);
            if (!fileData) {
                printf("[ERROR] Memory allocation failed for file reassembly\n");
                return;
            }
            printf("[INFO] Start packet received. Expected file size: %d\n", fileSize);
        } else if (packetType == PACKET_DATA) {
            // Extract sequence number and data size
            int seq = buffer[1];
            int dataSize = (buffer[2] << 8) | buffer[3];

            // Validate packet sequence number
            if (seq != expectedSeq) {
                printf("[ERROR] Unexpected sequence number. Expected: %d, Received: %d\n", expectedSeq, seq);
                free(fileData);
                return;
            }

            // Ensure received data does not exceed allocated buffer size
            if (bytesReceived + dataSize > fileSize) {
                printf("[ERROR] Data packet size exceeds expected file size. Aborting.\n");
                free(fileData);
                return;
            }

            // Copy the data into the correct position in fileData
            memcpy(&fileData[bytesReceived], &buffer[4], dataSize);
            bytesReceived += dataSize;
            printf("[INFO] Data packet received. Sequence: %d, Size: %d, Total bytes received: %d\n", seq, dataSize, bytesReceived);

            // Update expected sequence number
            expectedSeq = (expectedSeq + 1) % 256;  // Assuming sequence number wraps at 256
        } else if (packetType == PACKET_END) {
            printf("[INFO] End packet received. Total bytes received: %d\n", bytesReceived);

            // Check if the number of bytes received matches the expected file size
            if (bytesReceived != fileSize) {
                printf("[ERROR] Bytes received (%d) do not match expected file size (%d)\n", bytesReceived, fileSize);
                free(fileData);
                return;
            }

            // Create directory for output if not exists
            struct stat st = {0};
            if (stat("/output", &st) == -1) {
                mkdir("/output", 0700);  // Create directory inside Docker
            }

            // Write the reassembled file to disk
            char finalPath[256];
            snprintf(finalPath, sizeof(finalPath), "/output/%s", filename);  // Writing file in /output directory
            if (writeFile(finalPath, fileData, bytesReceived) == 0) {
                printf("[INFO] File reassembled and saved successfully: %s\n", finalPath);
            } else {
                printf("[ERROR] Failed to write file: %s\n", finalPath);
            }

            receiving = 0;  // Stop receiving
        }
    }

    free(fileData);
}




void applicationLayer(const char *serialPort, const char *role, int baudRate, int nTries, int timeout, const char *filename) {
    LinkLayer connectionParams;

    strncpy(connectionParams.serialPort, serialPort, sizeof(connectionParams.serialPort) - 1);
    connectionParams.serialPort[sizeof(connectionParams.serialPort) - 1] = '\0';

    connectionParams.baudRate = baudRate;
    connectionParams.nRetransmissions = nTries;
    connectionParams.timeout = timeout;

    if (strcmp(role, "tx") == 0) {
        connectionParams.role = LlTx;
        printf("[INFO] Starting as Transmitter on %s\n", connectionParams.serialPort);
        if (llopen(connectionParams) < 0) {
            printf("[ERROR] Failed to establish link layer connection.\n");
            return;
        }

        sendFile(filename);
    } else if (strcmp(role, "rx") == 0) {
        connectionParams.role = LlRx;
        printf("[INFO] Starting as Receiver on %s\n", connectionParams.serialPort);
        if (llopen(connectionParams) < 0) {
            printf("[ERROR] Failed to establish link layer connection.\n");
            return;
        }

        receiveFile("received.gif");  // You can use another name for the received file
    } else {
        printf("[ERROR] Invalid role specified. Must be 'tx' or 'rx'.\n");
        return;
    }

    if (llclose(1) < 0) {
        printf("[ERROR] Failed to close the link layer connection.\n");
    }
}

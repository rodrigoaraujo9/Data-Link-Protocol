#include "link_layer.h"
#include "serial_port.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <time.h>
#include <sys/time.h>

#define PACKET_START 0x02
#define PACKET_END 0x03
#define PACKET_DATA 0x01
#define MAX_DATA_SIZE 512 // You can modify this value to change the chunk size globally.

extern double transmissionStartTime=0.0;
extern double transmissionEndTime=0.0;

extern int totalBytesSent;
extern int totalBytesReceived;

double calculateReceivedBitrate(int totalBytes, double transmissionTime) {
    int totalBits = totalBytes * 8;
    return totalBits / transmissionTime;
}

#include "link_layer.h"
#include "serial_port.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <time.h>
#include <sys/time.h>

#define PACKET_START 0x02
#define PACKET_END 0x03
#define PACKET_DATA 0x01
#define MAX_DATA_SIZE 512

int totalBytesSent = 0;
int totalBytesReceived = 0;

long getFileSize(const char* filename) {
    struct stat fileStat;
    if (stat(filename, &fileStat) != 0) {
        printf("[ERROR] Could not retrieve file information for: %s\n", filename);
        return -1;
    }
    return fileStat.st_size;
}

double getCurrentTime() {
    struct timespec spec;
    clock_gettime(CLOCK_REALTIME, &spec);
    return spec.tv_sec + spec.tv_nsec / 1e9;
}

double calculateEfficiency(int totalBytes, int baudRate, double transmissionTime) {
    int totalBits = totalBytes * 8;
    return (double)totalBits / (baudRate * transmissionTime);
}

unsigned char* readFile(const char* filename, int* fileSize) {
    FILE* file = fopen(filename, "rb");
    if (!file) return NULL;

    *fileSize = getFileSize(filename);
    if (*fileSize < 0) {
        fclose(file);
        return NULL;
    }

    unsigned char* fileData = (unsigned char*)malloc(*fileSize);
    if (!fileData) {
        fclose(file);
        return NULL;
    }

    fread(fileData, 1, *fileSize, file);
    fclose(file);
    return fileData;
}

unsigned char* createControlPacket(int type, int fileSize, const char* filename, int* packetSize) {
    int filenameLength = strlen(filename);
    *packetSize = 9 + filenameLength;
    unsigned char* packet = (unsigned char*)malloc(*packetSize);

    if (!packet) return NULL;

    packet[0] = type;
    packet[1] = 0x00;
    packet[2] = 0x04;
    packet[3] = (fileSize >> 24) & 0xFF;
    packet[4] = (fileSize >> 16) & 0xFF;
    packet[5] = (fileSize >> 8) & 0xFF;
    packet[6] = fileSize & 0xFF;
    packet[7] = 0x01;
    packet[8] = filenameLength;
    memcpy(&packet[9], filename, filenameLength);

    return packet;
}

void sendFile(const char* filename) {
    int fileSize = 0;
    unsigned char* fileData = readFile(filename, &fileSize);
    if (!fileData) return;

    int packetSize;
    unsigned char* startPacket = createControlPacket(PACKET_START, fileSize, filename, &packetSize);

    llwrite(startPacket, packetSize);
    free(startPacket);

    int bytesSent = 0;
    unsigned char* dataPacket = (unsigned char*)malloc(MAX_DATA_SIZE + 4);
    int seq = 0;

    while (bytesSent < fileSize) {
        int dataSize = (fileSize - bytesSent > MAX_DATA_SIZE) ? MAX_DATA_SIZE : fileSize - bytesSent;
        dataPacket[0] = PACKET_DATA;
        dataPacket[1] = seq;
        dataPacket[2] = (dataSize >> 8) & 0xFF;
        dataPacket[3] = dataSize & 0xFF;
        memcpy(&dataPacket[4], &fileData[bytesSent], dataSize);

        if (llwrite(dataPacket, dataSize + 4) >= 0) {
            seq = (seq + 1) % 256;
            bytesSent += dataSize;
        } else {
            printf("[ERROR] Transmission failed, resending frame with sequence %d\n", seq);
        }
    }

    unsigned char* endPacket = createControlPacket(PACKET_END, fileSize, filename, &packetSize);
    llwrite(endPacket, packetSize);
    free(endPacket);
    free(dataPacket);
    free(fileData);
}

void receiveFile(const char* filename) {
    unsigned char* buffer = (unsigned char*)malloc(MAX_DATA_SIZE + 4);
    int totalBytesReceived = 0;
    int expectedSeq = 0;

    FILE* file = fopen(filename, "wb");
    if (!file) {
        printf("[ERROR] Could not open file for writing: %s\n", filename);
        free(buffer);
        return;
    }

    while (1) {
        int packetSize = llread(buffer);
        if (packetSize < 0) {
            printf("[ERROR] Packet read failed.\n");
            continue;
        }

        unsigned char packetType = buffer[0];
        if (packetType == PACKET_START) {
            int fileSize = (buffer[3] << 24) | (buffer[4] << 16) | (buffer[5] << 8) | buffer[6];
            printf("[INFO] Receiving file. Expected size: %d bytes.\n", fileSize);
        } else if (packetType == PACKET_DATA) {
            int seq = buffer[1];
            int dataSize = (buffer[2] << 8) | buffer[3];

            if (seq == expectedSeq) {
                fwrite(&buffer[4], 1, dataSize, file);
                totalBytesReceived += dataSize;
                expectedSeq = (expectedSeq + 1) % 256;
                sendRR();
            } else {
                printf("[WARN] Sequence mismatch. Expected %d, received %d. Sending REJ.\n", expectedSeq, seq);
                sendREJ();
            }
        } else if (packetType == PACKET_END) {
            printf("[INFO] File transfer complete. Total bytes received: %d\n", totalBytesReceived);
            break;
        }
    }

    fclose(file);
    free(buffer);
}



void applicationLayer(const char* serialPort, const char* role, int baudRate, int nTries, int timeout, const char* filename) {
    LinkLayer connectionParams;
    strncpy(connectionParams.serialPort, serialPort, sizeof(connectionParams.serialPort) - 1);
    connectionParams.serialPort[sizeof(connectionParams.serialPort) - 1] = '\0';
    connectionParams.baudRate = baudRate;
    connectionParams.nRetransmissions = nTries;
    connectionParams.timeout = timeout;

    printf("\n[INFO] Running transmission test with chunk size: %d bytes...\n", MAX_DATA_SIZE);

    totalBytesSent = 0;
    totalBytesReceived = 0;

    if (strcmp(role, "tx") == 0) {
        connectionParams.role = LlTx;
        if (llopen(connectionParams) < 0) {
            printf("[ERROR] Failed to open link layer connection.\n");
            return;
        }

        double startTime = getCurrentTime();
        sendFile(filename);
        double endTime = getCurrentTime();

        double transmissionTime = endTime - startTime;
        double efficiency = calculateReceivedBitrate(totalBytesSent, transmissionTime) / baudRate;

        printf("[INFO] Transmission Efficiency: %f\n", efficiency);
        printf("[INFO] Transmission Time: %f seconds\n", transmissionTime);

        if (llclose(1) < 0) {
            printf("[ERROR] Failed to close the link layer connection.\n");
        }

    } else if (strcmp(role, "rx") == 0) {
        connectionParams.role = LlRx;
        if (llopen(connectionParams) < 0) {
            printf("[ERROR] Failed to open link layer connection.\n");
            return;
        }

        double startTime = getCurrentTime();
        receiveFile(filename);
        double endTime = getCurrentTime();

        double transmissionTime = endTime - startTime;
        double efficiency = calculateReceivedBitrate(totalBytesReceived, transmissionTime) / baudRate;

        printf("[INFO] Reception Efficiency: %f\n", efficiency);
        printf("[INFO] Reception Time: %f seconds\n", transmissionTime);

        if (llclose(1) < 0) {
            printf("[ERROR] Failed to close the link layer connection.\n");
        }
    }
}
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
#define MAX_DATA_SIZE 1000 // You can modify this value to change the chunk size globally.

double transmissionStartTime = 0.0;
double transmissionEndTime = 0.0;

int totalBytesSent = 0;
int totalBytesReceived = 0;

double calculateReceivedBitrate(int totalBytes, double transmissionTime) {
    int totalBits = totalBytes * 8;
    return totalBits / transmissionTime;
}


long getFileSize(const char* filename) {
    struct stat fileStat;
    if (stat(filename, &fileStat) != 0) {
        printf("[ERROR] Could not retrieve file information for: %s\n", filename);
        return -1; // Return -1 to indicate an error
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
    double sentBitrate = (double)totalBits / transmissionTime;
    return sentBitrate / baudRate;
}

unsigned char* readFile(const char* filename, int* fileSize) {
    FILE* file = fopen(filename, "rb");
    if (!file) return NULL;

    fseek(file, 0, SEEK_END);
    *fileSize = getFileSize(filename);
    fseek(file, 0, SEEK_SET);

    unsigned char* fileData = (unsigned char*)malloc(*fileSize);
    if (!fileData) {
        fclose(file);
        return NULL;
    }

    fread(fileData, 1, *fileSize, file);
    fclose(file);
    return fileData;
}

int writeFile(const char* filename, unsigned char* fileData, int fileSize) {
    FILE* file = fopen(filename, "wb");
    if (!file) return -1;

    size_t written = fwrite(fileData, 1, fileSize, file);
    fclose(file);
    return (written == fileSize) ? 0 : -1;
}

unsigned char* createControlPacket(int type, int fileSize, const char* filename, int* packetSize) {
    int filenameLength = strlen(filename);
    *packetSize = 2 + 4 + 2 + filenameLength;

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

    int chunkSize = MAX_DATA_SIZE; // Set the chunk size based on the global MAX_DATA_SIZE.
    int packetSize;
    unsigned char* startPacket = createControlPacket(PACKET_START, fileSize, filename, &packetSize);

    struct timespec startTime, endTime;
    clock_gettime(CLOCK_MONOTONIC, &startTime);

    llwrite(startPacket, packetSize);
    free(startPacket);

    int bytesSent = 0;
    unsigned char* dataPacket = (unsigned char*)malloc(chunkSize + 4);
    int seq = 0;

    while (bytesSent < fileSize) {
        int dataSize = (fileSize - bytesSent > chunkSize) ? chunkSize : fileSize - bytesSent;
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
    free(dataPacket);
    free(fileData);

    clock_gettime(CLOCK_MONOTONIC, &endTime);
    double transmissionTime = (endTime.tv_sec - startTime.tv_sec) + (endTime.tv_nsec - startTime.tv_nsec) / 1e9;

    double efficiency = calculateEfficiency(bytesSent, 9600, transmissionTime);
    printf("[INFO] Transmission Efficiency: %.2f, Time: %.2f seconds\n", efficiency, transmissionTime);
}

void receiveFile(const char* filename) {
    int chunkSize = MAX_DATA_SIZE; // Set the chunk size based on the global MAX_DATA_SIZE.
    unsigned char* buffer = (unsigned char*)malloc(chunkSize + 4);
    int fileSize = 0;
    unsigned char* fileData = NULL;
    int totalBytesReceived = 0;
    int bytesReceived = 0;
    int expectedSeq = 0;

    int receiving = 1;
    double startTime = getCurrentTime();

    while (receiving) {
        int packetSize = llread(buffer);
        if (packetSize < 0) continue;

        unsigned char packetType = buffer[0];
        if (packetType == PACKET_START) {
            fileSize = (buffer[3] << 24) | (buffer[4] << 16) | (buffer[5] << 8) | buffer[6];
            fileData = (unsigned char*)malloc(fileSize);
            if (!fileData) {
                free(buffer);
                return;
            }
        } else if (packetType == PACKET_DATA) {
            int seq = buffer[1];
            int dataSize = (buffer[2] << 8) | buffer[3];
            if (seq != expectedSeq) {
                free(buffer);
                return;
            }

            memcpy(&fileData[bytesReceived], &buffer[4], dataSize);
            bytesReceived += dataSize;
            totalBytesReceived += dataSize;
            expectedSeq = (expectedSeq + 1) % 256;
        } else if (packetType == PACKET_END) {
            if (bytesReceived != fileSize) {
                free(buffer);
                return;
            }

            struct stat st = {0};
            if (stat("/output", &st) == -1) mkdir("/output", 0700);
            char finalPath[256];
            snprintf(finalPath, sizeof(finalPath), "/output/%s", filename);
            writeFile(finalPath, fileData, bytesReceived);
            receiving = 0;
        }
    }

    free(fileData);
    free(buffer);
    double endTime = getCurrentTime();
    double transmissionTime = endTime - startTime;
    double efficiency = calculateEfficiency(totalBytesReceived, 9600, transmissionTime);
    printf("[INFO] Reception Efficiency: %.2f, Time: %.2f seconds\n", efficiency, transmissionTime);
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
        receiveFile("received.gif");
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

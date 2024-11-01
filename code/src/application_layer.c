#include "link_layer.h"
#include "serial_port.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <time.h>
#include <math.h>
#include <sys/time.h>

#define PACKET_START 0x02
#define PACKET_END 0x03
#define PACKET_DATA 0x01
#define MAX_DATA_SIZE 256
#define NUM_RUNS 1

extern double HEADER_ERR_PROB;
extern double DATA_ERR_PROB;
extern int PROP_DELAY_MS;

int totalBytesSent = 0;
int totalBytesReceived = 0;

double calculateReceivedBitrate(int totalBytes, double transmissionTime) {
    int totalBits = totalBytes * 8;
    return totalBits / transmissionTime;
}

double calculateAverage(double* data, int num_elements) {
    double sum = 0.0;
    for (int i = 0; i < num_elements; i++) {
        sum += data[i];
    }
    return sum / num_elements;
}

double calculateStdDev(double* data, int num_elements, double mean) {
    double sum = 0.0;
    for (int i = 0; i < num_elements; i++) {
        sum += pow(data[i] - mean, 2);
    }
    return sqrt(sum / num_elements);
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
    *fileSize = ftell(file);
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

    int packetSize;
    unsigned char* startPacket = createControlPacket(PACKET_START, fileSize, filename, &packetSize);
    
    struct timespec startTime, endTime;
    clock_gettime(CLOCK_MONOTONIC, &startTime);

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

    clock_gettime(CLOCK_MONOTONIC, &endTime);
    double transmissionTime = (endTime.tv_sec - startTime.tv_sec) + (endTime.tv_nsec - startTime.tv_nsec) / 1e9;

    double efficiency = calculateEfficiency(bytesSent, 9600, transmissionTime);
    printf("[INFO] Transmission Efficiency: %.2f, Time: %.2f seconds\n", efficiency, transmissionTime);
}

void receiveFile(const char* filename) {
    unsigned char buffer[512];
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
            if (!fileData) return;
        } else if (packetType == PACKET_DATA) {
            int seq = buffer[1];
            int dataSize = (buffer[2] << 8) | buffer[3];
            if (seq != expectedSeq) return;

            memcpy(&fileData[bytesReceived], &buffer[4], dataSize);
            bytesReceived += dataSize;
            totalBytesReceived += dataSize;
            expectedSeq = (expectedSeq + 1) % 256;
        } else if (packetType == PACKET_END) {
            if (bytesReceived != fileSize) return;

            struct stat st = {0};
            if (stat("/output", &st) == -1) mkdir("/output", 0700);
            char finalPath[256];
            snprintf(finalPath, sizeof(finalPath), "/output/%s", filename);
            writeFile(finalPath, fileData, bytesReceived);
            receiving = 0;
        }
    }

    free(fileData);
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

    double efficiencies[NUM_RUNS];
    double transmissionTimes[NUM_RUNS];
    double FERs[] = {0.0, 0.1};
    int T_props[] = {100, 300};
    int numFERs = sizeof(FERs) / sizeof(FERs[0]);
    int numDelays = sizeof(T_props) / sizeof(T_props[0]);

    for (int f = 0; f < numFERs; f++) {
        for (int d = 0; d < numDelays; d++) {
            HEADER_ERR_PROB = FERs[f];
            DATA_ERR_PROB = FERs[f];
            PROP_DELAY_MS = T_props[d];

            printf("\n[INFO] Testing FER=%.2f, T_prop=%d ms\n", FERs[f], T_props[d]);
            for (int i = 0; i < NUM_RUNS; i++) {
                totalBytesSent = 0;
                totalBytesReceived = 0;

                if (strcmp(role, "tx") == 0) {
                    connectionParams.role = LlTx;
                    if (llopen(connectionParams) < 0) return;

                    double startTime = getCurrentTime();
                    sendFile(filename);
                    double endTime = getCurrentTime();

                    double transmissionTime = endTime - startTime;
                    transmissionTimes[i] = transmissionTime;
                    efficiencies[i] = calculateReceivedBitrate(totalBytesSent, transmissionTime) / baudRate;

                    if (llclose(1) < 0) printf("[ERROR] Failed to close the link layer connection.\n");
                } else if (strcmp(role, "rx") == 0) {
                    connectionParams.role = LlRx;
                    if (llopen(connectionParams) < 0) return;

                    double startTime = getCurrentTime();
                    receiveFile("received.gif");
                    double endTime = getCurrentTime();

                    double transmissionTime = endTime - startTime;
                    transmissionTimes[i] = transmissionTime;
                    efficiencies[i] = calculateReceivedBitrate(totalBytesReceived, transmissionTime) / baudRate;

                    if (llclose(1) < 0) printf("[ERROR] Failed to close the link layer connection.\n");
                }
            }

            double avgEfficiency = calculateAverage(efficiencies, NUM_RUNS);
            double avgTransmissionTime = calculateAverage(transmissionTimes, NUM_RUNS);
            double stdDevEfficiency = calculateStdDev(efficiencies, NUM_RUNS, avgEfficiency);
            double stdDevTransmissionTime = calculateStdDev(transmissionTimes, NUM_RUNS, avgTransmissionTime);

            printf("[INFO] Summary for FER=%.2f, T_prop=%d ms:\n", FERs[f], T_props[d]);
            printf("Average Efficiency = %f\n", avgEfficiency);
            printf("Standard Deviation of Efficiency = %f\n", stdDevEfficiency);
            printf("Average Transmission Time = %f seconds\n", avgTransmissionTime);
            printf("Standard Deviation of Transmission Time = %f seconds\n", stdDevTransmissionTime);
        }
    }
}

#include "link_layer.h"
#include "serial_port.h"
#include <stdio.h>
#include <unistd.h>
#include <sys/signal.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <time.h>
#define _POSIX_SOURCE 1
#define FLAG 0x7E
#define ADDR_TX_COMMAND 0x03
#define ADDR_RX_COMMAND 0x01
#define CTRL_SET 0x03
#define CTRL_UA 0x07
#define CTRL_RR0 0xAA
#define CTRL_RR1 0xAB
#define CTRL_REJ0 0x54
#define CTRL_REJ1 0x55
#define CTRL_DISC 0x0B
#define BCC1(addr, ctrl) ((addr) ^ (ctrl))
#define MAX_RETRIES 10
#define TIMEOUT_SECONDS 2
#define READ_RETRIES 5
#define ERR_MAX_RETRIES_EXCEEDED -2
#define ERR_WRITE_FAILED -3
#define ERR_READ_TIMEOUT -4
#define ERR_INVALID_BCC -5
#define ERR_FRAME_REJECTED -6
#define ERR_WRITE_TIMEOUT -7

enum StateSND {SEND_SET, WAIT_UA, SND_STOP};
enum StateRCV {START, FLAG_RCV, A_RCV, C_RCV, BCC_OK, RCV_STOP};

extern int totalBytesSent;
extern int totalBytesReceived;
extern double transmissionStartTime;
extern double transmissionEndTime;

double HEADER_ERR_PROB =0.7;
double DATA_ERR_PROB  =0.6;
int PROP_DELAY_MS =300;

volatile int alarmTriggered = FALSE;
int alarmCount = 0;
int timeout = TIMEOUT_SECONDS;
LinkLayerRole role;

void handle_alarm(int sig) {
    alarmTriggered = 1;
    alarmCount++;
    printf("[ALARM] Timeout occurred. Alarm count: %d seconds.\n", alarmCount);
    alarm(1);
}

void introduceErrors(unsigned char *frame, int frameSize, double headerErrorProb, double dataErrorProb) {
    srand(time(NULL)); // Seed random generator, if needed

    // Introduce header error with probability `headerErrorProb`
    if ((double)rand() / RAND_MAX < headerErrorProb) {
        frame[1] ^= 0xFF; // Flip all bits in the address field as an example
        printf("[INFO] Header error introduced\n");
    }

    // Introduce data errors with probability `dataErrorProb` for each byte in the data field
    for (int i = 4; i < frameSize - 2; i++) { // Assuming data starts after the header
        if ((double)rand() / RAND_MAX < dataErrorProb) {
            frame[i] ^= 0xFF; // Flip all bits in the byte
            printf("[INFO] Data error introduced at byte %d\n", i);
        }
    }
}

void applyPropagationDelay() {
    if (PROP_DELAY_MS > 0) {
        usleep(PROP_DELAY_MS * 1000); // Convert ms to microseconds
        printf("[INFO] Applied propagation delay of %d ms\n", PROP_DELAY_MS);
    }
}


int llopen(LinkLayer connectionParameters) {
    connectionParameters.timeout = timeout;
    role = connectionParameters.role;
    signal(SIGALRM, handle_alarm);  
    if (openSerialPort(connectionParameters.serialPort, connectionParameters.baudRate) < 0) {
        printf("[ERROR] Failed to open serial port\n");
        return -1;
    }

    if (role == LlTx) {
        unsigned char setFrame[5] = {FLAG, ADDR_TX_COMMAND, CTRL_SET, 0x00, FLAG};
        setFrame[3] = BCC1(setFrame[1], setFrame[2]);

        enum StateSND state = SEND_SET;
        unsigned char uaFrame[5] = {0};
        int retry = 0;

        while (state != SND_STOP && retry < MAX_RETRIES) {
            switch (state) {
                case SEND_SET: {
                    if (writeBytesSerialPort(setFrame, sizeof(setFrame)) < 0) {
                        printf("[ERROR] Failed to send SET frame\n");
                        return ERR_WRITE_FAILED;
                    }

                    printf("[INFO] Sent SET frame, setting alarm for 1 second retry\n");
                    alarmTriggered = 0;
                    alarm(1);
                    state = WAIT_UA;
                    break;
                }

                case WAIT_UA: {
                    enum StateRCV stateR = START;
                    unsigned char byte;
                    int bytesRead = 0;

                    while (stateR != RCV_STOP && bytesRead < 5 && alarmTriggered == 0) {
                        int res = readByteSerialPort(&byte);
                        if (res <= 0) {
                            if (alarmTriggered) {
                                retry++;
                                if (retry >= MAX_RETRIES) {
                                    printf("[ERROR] Maximum retries exceeded while trying to establish connection\n");
                                    alarm(0);
                                    return ERR_MAX_RETRIES_EXCEEDED;
                                }
                                printf("[WARN] Timeout while waiting for UA frame. Retrying... (%d/%d)\n", retry, MAX_RETRIES);
                                alarmTriggered = 0;
                                state = SEND_SET;
                                break;
                            }
                            continue;
                        }

                        uaFrame[bytesRead++] = byte;

                        switch (stateR) {
                            case START:
                                if (byte == FLAG) stateR = FLAG_RCV;
                                break;

                            case FLAG_RCV:
                                if (byte == ADDR_RX_COMMAND) stateR = A_RCV;
                                else if (byte != FLAG) stateR = START;
                                break;

                            case A_RCV:
                                if (byte == CTRL_UA) stateR = C_RCV;
                                else if (byte == FLAG) stateR = FLAG_RCV;
                                else stateR = START;
                                break;

                            case C_RCV:
                                if (byte == (uaFrame[1] ^ uaFrame[2])) stateR = BCC_OK;
                                else if (byte == FLAG) stateR = FLAG_RCV;
                                else stateR = START;
                                break;

                            case BCC_OK:
                                if (byte == FLAG) stateR = RCV_STOP;
                                else stateR = START;
                                break;
                            default:
                                break;
                        }
                    }

                    if (stateR == RCV_STOP) {
                        printf("[INFO] UA frame successfully received, connection established\n");
                        alarm(0);
                        return 1;
                    }
                    break;
                }
                default:
                    break;
            }
        }

        printf("[ERROR] Maximum retries exceeded while trying to establish connection\n");
        alarm(0);
        return ERR_MAX_RETRIES_EXCEEDED;
    } else if (role == LlRx) {
        signal(SIGALRM, handle_alarm);

        enum StateRCV state = START;
        unsigned char byte;
        unsigned char setFrame[5];
        int retry = 0;

        alarm(1);
        alarmTriggered = 0;

        while (state != RCV_STOP && retry < MAX_RETRIES) {
            int res = readByteSerialPort(&byte);
            if (res <= 0) {
                if (alarmTriggered) {
                    retry++;
                    if (retry >= MAX_RETRIES) {
                        printf("[ERROR] Maximum retries exceeded while waiting for SET frame\n");
                        return ERR_MAX_RETRIES_EXCEEDED;
                    }
                    printf("[ERROR] Timeout while waiting for SET frame, retrying... (%d/%d)\n", retry, MAX_RETRIES);
                    alarmTriggered = 0;
                    state = START;
                    alarm(1);
                    continue;
                }
                continue;
            }

            switch (state) {
                case START:
                    if (byte == FLAG) state = FLAG_RCV;
                    break;

                case FLAG_RCV:
                    if (byte == ADDR_TX_COMMAND) {
                        state = A_RCV;
                        setFrame[1] = byte;
                    } else if (byte != FLAG) state = START;
                    break;

                case A_RCV:
                    if (byte == CTRL_SET) {
                        state = C_RCV;
                        setFrame[2] = byte;
                    } else if (byte == FLAG) state = FLAG_RCV;
                    else state = START;
                    break;

                case C_RCV:
                    if (byte == (setFrame[1] ^ setFrame[2])) state = BCC_OK;
                    else if (byte == FLAG) state = FLAG_RCV;
                    else state = START;
                    break;

                case BCC_OK:
                    if (byte == FLAG) state = RCV_STOP;
                    else state = START;
                    break;
                default:
                                break;
            }

            if (alarmTriggered && retry >= MAX_RETRIES) {
                printf("[ERROR] Maximum retries exceeded while waiting for SET frame\n");
                return ERR_MAX_RETRIES_EXCEEDED;
            }

            if (alarmTriggered) {
                retry++;
                printf("[ERROR] Timeout reached while waiting for SET frame, retrying... (%d/%d)\n", retry, MAX_RETRIES);
                alarmTriggered = 0;
                alarm(1);
            }
        }

        alarm(0);

        unsigned char uaFrame[5] = {FLAG, ADDR_RX_COMMAND, CTRL_UA, 0x00, FLAG};
        uaFrame[3] = BCC1(uaFrame[1], uaFrame[2]);

        if (writeBytesSerialPort(uaFrame, sizeof(uaFrame)) < 0) {
            printf("[ERROR] Failed to send UA frame\n");
            return ERR_WRITE_FAILED;
        }

        printf("[INFO] UA frame sent successfully\n");
        printf("[INFO] SUCCESS!\n");
    }

    return 1;
}

int llwrite(const unsigned char *buf, int bufSize) {
    int overhead = 6;
    int maxFrameSize = bufSize * 2 + overhead;
    unsigned char *frame = (unsigned char*)malloc(maxFrameSize);
    
    if (frame == NULL) {
        printf("[ERROR] Memory allocation failed\n");
        return -1;
    }

    int index = 0;
    static int Ns = 0; // Sequence number
    unsigned char bcc2 = 0;
    int retries = 0;
    int result = 0;
    enum StateRCV ackState = START;

    // Construct the frame with the buffer data and BCC checks
    frame[index++] = FLAG;
    frame[index++] = ADDR_TX_COMMAND;
    frame[index++] = (Ns == 0) ? 0x00 : 0x80;
    frame[index++] = BCC1(frame[1], frame[2]);

    for (int i = 0; i < bufSize; i++) {
        bcc2 ^= buf[i];
        if (buf[i] == FLAG) {
            frame[index++] = 0x7D;
            frame[index++] = 0x5E;
        } else if (buf[i] == 0x7D) {
            frame[index++] = 0x7D;
            frame[index++] = 0x5D;
        } else {
            frame[index++] = buf[i];
        }
    }

    if (bcc2 == FLAG) {
        frame[index++] = 0x7D;
        frame[index++] = 0x5E;
    } else if (bcc2 == 0x7D) {
        frame[index++] = 0x7D;
        frame[index++] = 0x5D;
    } else {
        frame[index++] = bcc2;
    }

    frame[index++] = FLAG;
    
    // Retry loop for sending the frame
    while (retries < MAX_RETRIES) {
        printf("[INFO] Attempting to send frame (Attempt %d of %d)\n", retries + 1, MAX_RETRIES);
        
        result = writeBytesSerialPort(frame, index);
        if (result < 0 || result != index) {
            printf("[ERROR] Failed to write full frame to serial port. Attempt %d\n", retries + 1);
            retries++;
            continue;
        }

        totalBytesSent += index;  // Track total bytes sent
        printf("[DEBUG] Frame sent successfully, total bytes sent so far: %d\n", totalBytesSent);

        printf("[DEBUG] Frame sent successfully, awaiting acknowledgment...\n");
        alarm(1);
        alarmTriggered = 0;

        // Waiting for RR (acknowledgment)
        while (ackState != RCV_STOP && alarmTriggered == 0) {
            unsigned char ackByte;
            int readResult = readByteSerialPort(&ackByte);

            if (readResult <= 0) {
                if (alarmTriggered) {
                    retries++;
                    if (retries >= MAX_RETRIES) {
                        printf("[ERROR] Maximum retries exceeded while waiting for RR\n");
                        free(frame);
                        return ERR_MAX_RETRIES_EXCEEDED;
                    }
                    printf("[WARN] Timeout waiting for RR, retrying frame transmission\n");
                    break;
                }
                continue;
            }

            // Processing the acknowledgment frame (RR/REJ)
            switch (ackState) {
                case START:
                    if (ackByte == FLAG) ackState = FLAG_RCV;
                    break;
                case FLAG_RCV:
                    if (ackByte == ADDR_RX_COMMAND) ackState = A_RCV;
                    else if (ackByte != FLAG) ackState = START;
                    break;
                case A_RCV:
                    if (ackByte == CTRL_RR0 || ackByte == CTRL_RR1) {
                        int Nr = (ackByte == CTRL_RR0) ? 0 : 1;
                        if (Nr == Ns) { // Valid acknowledgment
                            ackState = RCV_STOP;
                            printf("[INFO] Acknowledgment (RR) received, transmission confirmed\n");
                            free(frame);
                            Ns = (Ns + 1) % 2; // Toggle Ns only after RR
                            return bufSize;
                        }
                    } else if (ackByte == CTRL_REJ0 || ackByte == CTRL_REJ1) {
                        printf("[WARN] REJ received, resending frame without toggling sequence\n");
                        ackState = START;  // Reset state machine
                        break;
                    } else if (ackByte == FLAG) {
                        ackState = FLAG_RCV;
                    } else {
                        ackState = START;
                    }
                    break;
                default:
                    break;
            }
        }

        if (ackState == RCV_STOP) break; // Break the retry loop if acknowledgment was received

        if (retries >= MAX_RETRIES) {
            printf("[ERROR] Failed to get acknowledgment after maximum retries\n");
            free(frame);
            return ERR_MAX_RETRIES_EXCEEDED;
        }
    }

    printf("[ERROR] Transmission failed after retries\n");
    free(frame);
    return ERR_WRITE_TIMEOUT;
}



void sendRR() {
    unsigned char rrFrame[5] = {FLAG, ADDR_RX_COMMAND, CTRL_RR0, 0x00, FLAG};
    rrFrame[3] = BCC1(rrFrame[1], rrFrame[2]);
    writeBytesSerialPort(rrFrame, sizeof(rrFrame));
}

void sendREJ() {
    unsigned char rejFrame[5] = {FLAG, ADDR_RX_COMMAND, CTRL_REJ0, 0x00, FLAG};
    rejFrame[3] = BCC1(rejFrame[1], rejFrame[2]);
    writeBytesSerialPort(rejFrame, sizeof(rejFrame));
}

int llread(unsigned char *packet) {
    enum StateRCV state = START;
    unsigned char byte;
    unsigned char frame[256];
    int frameIndex = 0;
    unsigned char bcc2 = 0;
    int bytesRead = 0;
    int retries = 0;
    const int maxRetries = MAX_RETRIES;
    const int readTimeout = 1;
    static int expectedSequence = 0;

    printf("[DEBUG] Starting llread\n");

    while (state != RCV_STOP && retries < maxRetries) {
        int res = readByteSerialPort(&byte);

        if (res <= 0) {
            printf("[ERROR] Read timeout (retry %d of %d)\n", retries + 1, maxRetries);
            retries++;
            sleep(readTimeout);
            continue;
        }

        retries = 0;

        switch (state) {
            case START:
                if (byte == FLAG) {
                    state = FLAG_RCV;
                    frameIndex = 0;
                    bytesRead = 0;
                    bcc2 = 0;
                }
                break;

            case FLAG_RCV:
                if (byte == ADDR_TX_COMMAND) {
                    frame[frameIndex++] = byte;
                    state = A_RCV;
                } else if (byte != FLAG) {
                    state = START;
                }
                break;

            case A_RCV:
                if (byte == 0x00 || byte == 0x80) {
                    frame[frameIndex++] = byte;
                    state = C_RCV;
                } else if (byte == FLAG) {
                    state = FLAG_RCV;
                } else {
                    state = START;
                }
                break;

            case C_RCV:
                if (byte == BCC1(frame[frameIndex - 2], frame[frameIndex - 1])) {
                    frame[frameIndex++] = byte;
                    state = BCC_OK;
                } else if (byte == FLAG) {
                    state = FLAG_RCV;
                } else {
                    state = START;
                }
                break;

            case BCC_OK:
                if (byte == FLAG) {
                    if (bcc2 == 0) {
                        applyPropagationDelay();
                        introduceErrors(frame, frameIndex, HEADER_ERR_PROB, DATA_ERR_PROB);
                        if (expectedSequence == ((frame[2] & 0x80) ? 1 : 0)) {
                            state = RCV_STOP;
                            sendRR(); // Send acknowledgment
                            printf("[INFO] Frame successfully received\n");
                            
                            expectedSequence = (expectedSequence + 1) % 2;
                            totalBytesReceived += bytesRead;
                        } else {
                            sendREJ(); // Send rejection for unexpected sequence
                            printf("[ERROR] Unexpected sequence number. Expected: %d, Received: %d\n", expectedSequence, frame[2] & 0x80 ? 1 : 0);
                            state = START; // Restart for next frame
                        }
                    } else {
                        sendREJ();
                        printf("[ERROR] BCC2 mismatch, frame rejected\n");
                    }
                } else {
                    if (byte == 0x7D) {
                        int res = readByteSerialPort(&byte);
                        if (res > 0) {
                            if (byte == 0x5E) byte = 0x7E;
                            if (byte == 0x5D) byte = 0x7D;
                        }
                    }
                    packet[bytesRead++] = byte;
                    bcc2 ^= byte;
                }
                break;
            default:
                break;
        }
    }

    if (retries >= maxRetries) {
        printf("[ERROR] Maximum retries exceeded. Failed to receive the packet.\n");
        return ERR_MAX_RETRIES_EXCEEDED;
    }

    printf("[INFO] Received %d bytes in the frame\n", bytesRead);
    return bytesRead;
}




void sendDISC() {
    unsigned char discFrame[5];

    discFrame[0] = FLAG; 
    discFrame[1] = ADDR_RX_COMMAND; 
    discFrame[2] = CTRL_DISC; 
    discFrame[3] = BCC1(discFrame[1], discFrame[2]);
    discFrame[4] = FLAG; 

    writeBytesSerialPort(discFrame, sizeof(discFrame));
}



int llclose(int showStatistics) {
    
    if (role == LlTx) {
        int retries = 0;

        while (retries < MAX_RETRIES) {
            sendDISC(); 
            printf("[INFO] DISC packet from transmitter sent.\n");
            printf("[INFO] Waiting for DISC from receiver...\n");

            unsigned char byte;
            enum StateRCV state = START;
            alarm(1); 

            while (state != RCV_STOP) {
                int res = readByteSerialPort(&byte);
            
                if (res <= 0) {
                    sleep(1);
                    printf("[ERROR] Timeout while waiting for DISC frame\n");
                    retries++;
                    if (retries >= MAX_RETRIES) {
                        printf("[ERROR] Maximum retries exceeded while waiting for DISC frame\n");
                        return ERR_MAX_RETRIES_EXCEEDED;
                    }
                    printf("[WARN] Retrying sending DISC...\n");
                    break; 
                }


                switch (state) {
                    case START:
                        if (byte == FLAG) {
                            state = FLAG_RCV;
                        }
                        break;
                    case FLAG_RCV:
                        if (byte == ADDR_RX_COMMAND) {
                            state = A_RCV;
                        } else if (byte != FLAG) {
                            state = START;
                        }
                        break;
                    case A_RCV:
                        if (byte == CTRL_DISC) {
                            state = C_RCV;
                        } else if (byte == FLAG) {
                            state = FLAG_RCV;
                        } else {
                            state = START;
                        }
                        break;
                    case C_RCV:
                        if (byte == BCC1(ADDR_RX_COMMAND, CTRL_DISC)) {
                            state = BCC_OK;
                        } else if (byte == FLAG) {
                            state = FLAG_RCV;
                        } else {
                            state = START;
                        }
                        break;
                    case BCC_OK:
                        if (byte == FLAG) {
                            state = RCV_STOP;
                            printf("[INFO] Received DISC frame from receiver\n");
                        } else {
                            state = START;
                        }
                        break;
                    default:
                                break;
                }
            }

            if (state == RCV_STOP) {
                break; 
            }
        }

        unsigned char uaFrame[5] = {FLAG, ADDR_TX_COMMAND, CTRL_UA, 0x00, FLAG}; 
        uaFrame[3] = BCC1(uaFrame[1], uaFrame[2]);

        if (writeBytesSerialPort(uaFrame, sizeof(uaFrame)) < 0) {
            printf("[ERROR] Failed to send UA frame\n");
            return ERR_WRITE_FAILED;
        }
        printf("[INFO] Sent UA frame to confirm connection closure\n");
    } 
    else if (role == LlRx) {
        unsigned char byte;
        enum StateRCV state = START;

        while (state != RCV_STOP) {
            int res = readByteSerialPort(&byte);
            if (res <= 0) {
                printf("[ERROR] Timeout while waiting for DISC frame\n");
                return ERR_READ_TIMEOUT;
            }

            switch (state) {
                case START:
                    if (byte == FLAG) {
                        state = FLAG_RCV;
                    }
                    break;

                case FLAG_RCV:
                    if (byte == ADDR_RX_COMMAND) {
                        state = A_RCV; 
                    } else if (byte != FLAG) {
                        state = START; 
                    }
                    break;

                case A_RCV:
                    if (byte == CTRL_DISC) {
                        state = C_RCV; 
                    } else if (byte == FLAG) {
                        state = FLAG_RCV; 
                        break;
                    } else {
                        state = START; 
                    }
                    break;

                case C_RCV:
                    if (byte == BCC1(ADDR_RX_COMMAND, CTRL_DISC)) {
                        state = BCC_OK;
                    } else if (byte == FLAG) {
                        state = FLAG_RCV; 
                    } else {
                        state = START; 
                    }
                    break;

                case BCC_OK:
                    if (byte == FLAG) {
                        state = RCV_STOP; 
                        printf("[INFO] Received DISC frame from transmitter\n");
                        sendDISC(); 
                        printf("[INFO] DISC packet from receiver sent.\n");
                    } else {
                        state = START;
                    }
                    break;
                default:
                                break;
            }
        }
    }

    // Close serial port
    int clstat = closeSerialPort();
    if (clstat != -1) {
        printf("[INFO] connection closed.\n");
    }


    if (showStatistics) {
        printf("[INFO] Statistics:\n");
        // Pôr aqui as estatísticas
    }
    return clstat;
}



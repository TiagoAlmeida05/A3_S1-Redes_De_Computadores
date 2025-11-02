// Link layer protocol implementation

#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <stdbool.h>
#include <errno.h>
#include "link_layer.h"
#include "serial_port.h"

// MISC
#define _POSIX_SOURCE 1 // POSIX compliant source
#define FLAG 0x7E
#define ADRESS_SENDER 0x03
#define ADRESS_RECEIVER 0x01
#define SET 0x03
#define UA 0x07
#define DISC 0x0B
#define BUF_SIZE 256
#define FALSE 0
#define TRUE 1

#define C_Information(Ns) (Ns << 7)
#define C_Ready(Nr) (0xAA | (Nr & 0x01))
#define C_Reject(Nr) (0x54 | (Nr & 0x01))

// Global Variables
volatile int alarmEnabled = FALSE;
volatile int alarmCount = 0;
unsigned char infoFrameSender = 0;
int retransmitions = 0;
int timeout = 0;
int Nr = 0;

// States
enum my_states{ 
    START, 
    FLAG_RCV, 
    A_RCV, 
    C_RCV, 
    BCC_OK, 
    STOPS 
};

// Function Declarations
unsigned char readControlFrame();
void alarmHandler(int signal);
int baseControlFrame(unsigned char controlByte, int timeout, LinkLayerRole role);
void byteDestuffing(const unsigned char *input, int inputSize, unsigned char *output, int *outputSize);
void byteStuffing(const unsigned char *input, int inputSize, unsigned char *output, int *outputSize);

////////////////////////////////////////////////
// LLOPEN
////////////////////////////////////////////////
int llopen(LinkLayer connectionParameters)
{
    const char *serialPort = connectionParameters.serialPort;
    int baudRate = connectionParameters.baudRate;
    LinkLayerRole role = connectionParameters.role;
    enum my_states state = START;
    timeout = connectionParameters.timeout;

    if (openSerialPort(serialPort, baudRate) < 0) {
        perror("openSerialPort");
        exit(-1);
    }

    retransmitions = connectionParameters.nRetransmissions;

    switch (role) {
        case LlTx: {
            struct sigaction sa;
            sa.sa_handler = alarmHandler;
            sigemptyset(&sa.sa_mask);
            sa.sa_flags = 0; 
            sigaction(SIGALRM, &sa, NULL);
            
            int tries = retransmitions;
            
            while (tries > 0 && state != STOPS) {   

                unsigned char buf[5] = {FLAG, ADRESS_SENDER, SET, ADRESS_SENDER ^ SET, FLAG};

                printf("[llopen TX] Sending SET frame: ");
                for(int k = 0; k < 5; k++) printf("%02X ", buf[k]);
                printf("\n");

                writeBytesSerialPort(buf, 5);
                
                alarmCount = 0;
                alarmEnabled = TRUE;
                alarm(timeout);
                
                int res = baseControlFrame(UA, timeout, LlTx);

                alarm(0); 
                alarmEnabled = FALSE;

                if(res == 0){
                    printf("[llopen TX] UA received successfully!\n");
                    state = STOPS;
                    break;
                }

                printf("[llopen TX] Failed to receive UA (alarmCount=%d, res=%d)\n", alarmCount, res);
                tries--;
                
                printf("[llopen TX] Tries remaining: %d\n", tries);
                
                if (tries > 0) {
                    printf("[llopen TX] Retrying...\n\n");
                }
            }
                        
            if(state != STOPS){
                printf("[llopen TX] ERROR: Exceeded maximum retransmissions (%d attempts)\n", retransmitions);
                closeSerialPort();
                return -1;               
            }
            break;
        }
        case LlRx: {
            printf("[llopen RX] Waiting for SET frame...\n");
            baseControlFrame(SET, timeout, LlRx);
            unsigned char buf[5] = {FLAG, ADRESS_RECEIVER, UA, ADRESS_RECEIVER ^ UA, FLAG};

            printf("[llopen RX] Sending UA frame: ");
            for(int k = 0; k < 5; k++) printf("%02X ", buf[k]);
            printf("\n");

            writeBytesSerialPort(buf, 5);
            break;
        }
        default:
            break;
    }

    return 0;
}

////////////////////////////////////////////////
// LLWRITE
////////////////////////////////////////////////
int llwrite(const unsigned char *buf, int bufSize)
{
    int frameSize = 6 + bufSize;
    unsigned char *frame = (unsigned char *)malloc(frameSize);

    frame[0] = FLAG;
    frame[1] = ADRESS_SENDER;
    frame[2] = C_Information(infoFrameSender);
    frame[3] = frame[1] ^ frame[2];

    memcpy(frame + 4, buf, bufSize);

    unsigned char BCC2 = buf[0];
    for (unsigned int i = 1; i < (unsigned int)bufSize; i++) {
        BCC2 ^= buf[i];
    }

    int j = 4;
    for (unsigned int i = 0; i < (unsigned int)bufSize; i++) {
        if (buf[i] == FLAG || buf[i] == 0x7D) {
            frame = realloc(frame, ++frameSize);
            frame[j++] = 0x7D;
            frame[j++] = buf[i] ^ 0x20;
        }
        else{
            frame[j++] = buf[i];
        }
    }

    if(BCC2 == FLAG || BCC2 == 0x7D){
        frame = realloc(frame, ++frameSize);
        frame[j++] = 0x7D;
        frame[j++] = BCC2 ^ 0x20;
    }
    else{
        frame[j++] = BCC2;
    }

    frame[j++] = FLAG;

    int currentTransmission = 0;
    int rej = 0;
    int acp = 0;

    struct sigaction sa;
    sa.sa_handler = alarmHandler;
    sigemptyset(&sa.sa_mask);
    sa.sa_flags = 0;
    sigaction(SIGALRM, &sa, NULL);

    while (currentTransmission < retransmitions) {
        rej = 0;
        acp = 0;
        alarmCount = 0;
        alarmEnabled = TRUE;

        printf("[llwrite] Sending frame (attempt %d/%d)\n", currentTransmission + 1, retransmitions);
        writeBytesSerialPort(frame, j);
        
        alarm(timeout);

        while (alarmCount == 0 && !rej && !acp) {
            unsigned char res = readControlFrame();
            if (!res) {
                continue;
            } else if (res == C_Ready(0) || res == C_Ready(1)) {
                acp = 1;
                infoFrameSender = (infoFrameSender + 1) % 2;
            } else if (res == C_Reject(0) || res == C_Reject(1)) {
                rej = 1;
            }
        }
        
        alarm(0);
        alarmEnabled = FALSE;
        
        if (acp) {
            printf("[llwrite] Frame acknowledged successfully\n");
            break;
        }
        
        if (alarmCount > 0) {
            printf("[llwrite] Timeout on attempt %d (alarmCount=%d)\n", currentTransmission + 1, alarmCount);
        }
        
        currentTransmission++;
    }
    
    int final_len = j;
    free(frame);
    
    if (!acp) {
        printf("[llwrite] ERROR: Failed after %d attempts\n", retransmitions);
        return -1;
    }
    
    return final_len;
}

////////////////////////////////////////////////
// LLREAD
////////////////////////////////////////////////
int llread(unsigned char *packet)
{
    enum my_states state = START;
    unsigned char receivedFrame[MAX_PAYLOAD_SIZE * 2 + 5];
    int receivedFrameIdx = 0;
    unsigned char A_val = ADRESS_SENDER;

    (void) signal(SIGALRM, alarmHandler);
    alarmEnabled = FALSE;
    alarmCount = 0;

    while (state != STOPS) {
        unsigned char byte;
        if (readByteSerialPort(&byte)) {
            printf("[llread] Received byte: 0x%02X\n", byte);
            switch (state) {
                case START:
                    if (byte == FLAG) {
                        state = FLAG_RCV;
                        receivedFrameIdx = 0;
                        receivedFrame[receivedFrameIdx++] = byte;
                    }
                    break;
                case FLAG_RCV:
                    if (byte == A_val) {
                        state = A_RCV;
                        receivedFrame[receivedFrameIdx++] = byte;
                    } else if (byte != FLAG) {
                        state = START;
                        receivedFrameIdx = 0;
                    }
                    break;
                case A_RCV:
                    if (byte == C_Information(0) || byte == C_Information(1)) {
                        state = C_RCV;
                        receivedFrame[receivedFrameIdx++] = byte;
                    } else state = START;
                    break;
                case C_RCV:
                    if (byte == (receivedFrame[1] ^ receivedFrame[2])) {
                        state = BCC_OK;
                        receivedFrame[receivedFrameIdx++] = byte;
                    } else state = START;
                    break;
                case BCC_OK:
                    receivedFrame[receivedFrameIdx++] = byte;
                    if (byte == FLAG) {
                        state = STOPS;
                    }
                    break;
                default:
                    break;
            }
            if (receivedFrameIdx >= (int)sizeof(receivedFrame)) {
                fprintf(stderr, "[llread ERROR] receivedFrame overflow\n");
                return -1;
            }
        }
    }

    alarm(0);

    int stuffedDataSize = receivedFrameIdx - 5;
    if (stuffedDataSize <= 0) {
        fprintf(stderr, "[llread ERROR] stuffedDataSize <= 0 (%d)\n", stuffedDataSize);
        return -1;
    }

    unsigned char *stuffedData = (unsigned char *)malloc(stuffedDataSize);
    if (!stuffedData) { perror("malloc"); return -1; }
    memcpy(stuffedData, receivedFrame + 4, stuffedDataSize);

    printf("[llread DEBUG] stuffedDataSize = %d\n", stuffedDataSize);
    printf("[llread DEBUG] stuffed raw bytes: ");
    for(int _i = 0; _i < stuffedDataSize; _i++) printf("%02X ", stuffedData[_i]);
    printf("\n");

    unsigned char *destuffedData = (unsigned char *)malloc(stuffedDataSize);
    if (!destuffedData) { free(stuffedData); perror("malloc"); return -1; }
    int destuffedSize = 0;
    byteDestuffing(stuffedData, stuffedDataSize, destuffedData, &destuffedSize);
    free(stuffedData);

    printf("[llread DEBUG] destuffedDataSize = %d\n", destuffedSize);
    printf("[llread DEBUG] destuffed bytes (including BCC2 at end): ");
    for(int _i = 0; _i < destuffedSize; _i++) printf("%02X ", destuffedData[_i]);
    printf("\n");

    if (destuffedSize <= 0) {
        fprintf(stderr, "[llread ERROR] destuffedSize <= 0\n");
        free(destuffedData);
        return -1;
    }

    unsigned char receivedBCC2 = destuffedData[destuffedSize - 1];
    int payloadSize = destuffedSize - 1;
    if (payloadSize < 0) { free(destuffedData); return -1; }

    if (payloadSize > MAX_PAYLOAD_SIZE) {
        fprintf(stderr, "[llread ERROR] payloadSize %d > MAX_PAYLOAD_SIZE %d\n", payloadSize, MAX_PAYLOAD_SIZE);
        free(destuffedData);
        return -1;
    }

    unsigned char calculatedBCC2 = 0x00;
    for (int i = 0; i < payloadSize; i++) {
        calculatedBCC2 ^= destuffedData[i];
    }
    printf("[llread DEBUG] calculatedBCC2=0x%02X, receivedBCC2=0x%02X\n", calculatedBCC2, receivedBCC2);

    if (calculatedBCC2 == receivedBCC2) {
        memcpy(packet, destuffedData, payloadSize);
        Nr = 1 - Nr;
        unsigned char rrFrame[5] = {FLAG, ADRESS_RECEIVER, C_Ready(Nr), ADRESS_RECEIVER ^ C_Ready(Nr), FLAG};
        writeBytesSerialPort(rrFrame, 5);
        free(destuffedData);
        return payloadSize;
    } else {
        printf("[llread ERROR] BCC2 mismatch! calculated=0x%02X, received=0x%02X\n", calculatedBCC2, receivedBCC2);
        unsigned char rejFrame[5] = {FLAG, ADRESS_RECEIVER, C_Reject(Nr), ADRESS_RECEIVER ^ C_Reject(Nr), FLAG};
        writeBytesSerialPort(rejFrame, 5);
        free(destuffedData);
        return -1;
    }
}

////////////////////////////////////////////////
// LLCLOSE
////////////////////////////////////////////////
int llclose(LinkLayer connectionParameters)
{
    struct sigaction sa;
    sa.sa_handler = alarmHandler;
    sigemptyset(&sa.sa_mask);
    sa.sa_flags = 0;
    sigaction(SIGALRM, &sa, NULL);
    
    if (connectionParameters.role == LlTx) {
        int retriesLeft = connectionParameters.nRetransmissions;
        
        while (retriesLeft > 0) {
            
            unsigned char discFrame[5] = {FLAG, ADRESS_SENDER, DISC, ADRESS_SENDER ^ DISC, FLAG};
            printf("[llclose TX] Sending DISC frame\n");
            writeBytesSerialPort(discFrame, 5);
            
            alarmCount = 0; 
            alarmEnabled = TRUE;
            alarm(timeout);
            
            if (baseControlFrame(DISC, connectionParameters.timeout, LlTx) == 0) {
                alarm(0);
                alarmEnabled = FALSE;
                printf("[llclose TX] DISC received, sending UA\n");
                unsigned char uaFrame[5] = {FLAG, ADRESS_SENDER, UA, ADRESS_SENDER ^ UA, FLAG};
                writeBytesSerialPort(uaFrame, 5);
                break;
            }
            
            alarm(0); 
            alarmEnabled = FALSE;
            
            printf("[llclose TX] Timeout waiting for DISC (alarmCount=%d)\n", alarmCount);
            retriesLeft--;
            
            if (retriesLeft > 0) {
                printf("[llclose TX] Retrying...\n");
            }
        }
        
        if (retriesLeft == 0) {
            printf("[llclose TX] ERROR: Failed to close connection properly\n");
        }
        
    } else if (connectionParameters.role == LlRx) {
        printf("[llclose RX] Waiting for DISC from transmitter\n");
        
        if (baseControlFrame(DISC, connectionParameters.timeout, LlRx) == 0) {
            printf("[llclose RX] DISC received, sending DISC response\n");
            unsigned char discFrame[5] = {FLAG, ADRESS_RECEIVER, DISC, ADRESS_RECEIVER ^ DISC, FLAG};
            writeBytesSerialPort(discFrame, 5);
            
            printf("[llclose RX] Waiting for final UA\n");
            baseControlFrame(UA, connectionParameters.timeout, LlRx);
        }
    }
    
    printf("[llclose] Closing serial port\n");
    closeSerialPort();
    return 0;
}

////////////////////////////////////////////////
// AUXILIARY FUNCTIONS
////////////////////////////////////////////////
void alarmHandler(int signal)
{
    printf("[ALARM] Alarm #%d triggered!\n", alarmCount + 1);
    alarmEnabled = FALSE;
    alarmCount++;
}

unsigned char readControlFrame()
{
    unsigned char byte = 0;
    unsigned char c = 0;
    enum my_states state = START;
    
    while (state != STOPS) {
        if (alarmCount > 0) {
            printf("[readControlFrame] Timeout - alarm triggered (alarmCount=%d)\n", alarmCount);
            return 0;
        }
        
        if (readByteSerialPort(&byte)) {
            printf("[readControlFrame] Received byte: 0x%02X\n", byte);
            switch (state) {
                case START:
                    if (byte == FLAG) state = FLAG_RCV;
                    break;
                case FLAG_RCV:
                    if (byte == ADRESS_RECEIVER) state = A_RCV;
                    break;
                case A_RCV:
                    if (byte == C_Ready(0) || byte == C_Ready(1) ||
                        byte == C_Reject(0) || byte == C_Reject(1) || byte == DISC) {
                        state = C_RCV;
                        c = byte;
                    }
                    break;
                case C_RCV:
                    if (byte == (ADRESS_RECEIVER ^ c)) state = BCC_OK;
                    break;
                case BCC_OK:
                    if (byte == FLAG) state = STOPS;
                    break;
                default:
                    break;
            }
        }
    }
    
    printf("[readControlFrame] Control frame extracted: 0x%02X\n", c);
    return c;
}

void byteStuffing(const unsigned char *input, int inputSize, unsigned char *output, int *outputSize)
{
    *outputSize = 0;
    for (int i = 0; i < inputSize; i++) {
        if (input[i] == FLAG || input[i] == 0x7D) {
            output[(*outputSize)++] = 0x7D;
            output[(*outputSize)++] = input[i] ^ 0x20;
        } else {
            output[(*outputSize)++] = input[i];
        }
    }
}

void byteDestuffing(const unsigned char *input, int inputSize, unsigned char *output, int *outputSize)
{
    *outputSize = 0;
    for (int i = 0; i < inputSize; i++) {
        if (input[i] == 0x7D) {
            if (i + 1 < inputSize) {
                output[(*outputSize)++] = input[++i] ^ 0x20;
            }
        } else {
            output[(*outputSize)++] = input[i];
        }
    }
}

int baseControlFrame(unsigned char controlByte, int timeout, LinkLayerRole role)
{
    enum my_states state = START;
    unsigned char A_val = (role == LlTx) ? ADRESS_RECEIVER : ADRESS_SENDER;

    int bytesRead = 0;
    while (state != STOPS) {
        if (alarmCount > 0) {
            printf("[baseControlFrame] Alarm detected, breaking loop (alarmCount=%d, bytesRead=%d)\n", alarmCount, bytesRead);
            break;
        }
        
        unsigned char byte;
        int readResult = readByteSerialPort(&byte);
        
        if (readResult) {
            bytesRead++;
            printf("[baseControlFrame] Received byte: 0x%02X (state=%d, total bytes=%d)\n", byte, state, bytesRead);
            switch (state) {
                case START:
                    if (byte == FLAG) state = FLAG_RCV;
                    break;
                case FLAG_RCV:
                    if (byte == A_val) state = A_RCV;
                    else if (byte != FLAG) state = START;
                    break;
                case A_RCV:
                    if (byte == controlByte) state = C_RCV;
                    else if (byte == FLAG) state = FLAG_RCV;
                    else state = START;
                    break;
                case C_RCV:
                    if (byte == (A_val ^ controlByte)) state = BCC_OK;
                    else if (byte == FLAG) state = FLAG_RCV;
                    else state = START;
                    break;
                case BCC_OK:
                    if (byte == FLAG) state = STOPS;
                    else state = START;
                    break;
                default:
                    break;
            }
        } else {
            if (alarmCount > 0) {
                printf("[baseControlFrame] Alarm detected after empty read (alarmCount=%d)\n", alarmCount);
                break;
            }
        }
    }
    
    if (state == STOPS) {
        printf("[baseControlFrame] Successfully received control frame 0x%02X\n", controlByte);
        return 0;
    } else {
        printf("[baseControlFrame] Failed to receive control frame (state=%d, alarmCount=%d)\n", state, alarmCount);
        return -1;
    }
}
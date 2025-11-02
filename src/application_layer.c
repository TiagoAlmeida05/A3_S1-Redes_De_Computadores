// application_layer.c
// Application layer protocol implementation

#include "application_layer.h"
#include "link_layer.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

extern int fd;

unsigned char *getControlPacket(const unsigned int c, const char *filename, long int length, unsigned int *size);
unsigned char *getData(FILE *file, long int fileSize);
unsigned char *getDataPacket(unsigned char list, unsigned char *data, int dataSize, int *packetSize);
unsigned char *getSizeAndName(unsigned char *packet, int size, unsigned long int *fileSize);

void applicationLayer(const char *serialPort, const char *role, int baudRate, int nTries, int timeout, const char *filename)
{
    LinkLayer connectionParameters;
    strcpy(connectionParameters.serialPort, serialPort);

    if (strcmp("tx", role) == 0)
        connectionParameters.role = LlTx;
    else if (strcmp("rx", role) == 0)
        connectionParameters.role = LlRx;
    else {
        perror("Invalid role\n");
        exit(-1);
    }

    connectionParameters.baudRate = baudRate;
    connectionParameters.nRetransmissions = nTries;
    connectionParameters.timeout = timeout;

    if (llopen(connectionParameters) != 0) {
        perror("Connection Error\n");
        exit(-1);
    }

        switch (connectionParameters.role) {
        case LlTx: {
            FILE *file = fopen(filename, "rb");
            if (file == NULL) {
                perror("File not found\n");
                exit(-1);
            }

            int prev = ftell(file);
            fseek(file, 0L, SEEK_END);
            long int fileSize = ftell(file) - prev;
            fseek(file, prev, SEEK_SET);
            
            unsigned int cpSize;
            unsigned char *controlPacketStart = getControlPacket(1, filename, fileSize, &cpSize);
            
            if (llwrite(controlPacketStart, cpSize) == -1) {
                exit(-1);
            }
            free(controlPacketStart);

            unsigned char list = 0;
            unsigned char *cont = getData(file, fileSize);
            unsigned char *contPtr = cont;
            long int remain = fileSize;
            

            int packetCount = 0;
            while (remain > 0) {
                int dataSize = remain > (long int)(MAX_PAYLOAD_SIZE - 4) ? (MAX_PAYLOAD_SIZE - 4) : remain;
                unsigned char *data = (unsigned char *)malloc(dataSize);
                memcpy(data, contPtr, dataSize);

                int packetSize;
                unsigned char *packet = getDataPacket(list, data, dataSize, &packetSize);

                if (llwrite(packet, packetSize) == -1) {
                    exit(-1);
                }
                

                remain -= (long int)dataSize;
                contPtr += dataSize;
                list = (list + 1) % 255;
                packetCount++;

                free(packet);
                free(data);
            }

            free(cont);
            fclose(file);
            
            unsigned char *controlPacketEnd = getControlPacket(3, filename, fileSize, &cpSize);
            
            if (llwrite(controlPacketEnd, cpSize) == -1) {
                exit(-1);
            }

            free(controlPacketEnd);
            llclose(connectionParameters);
            break;
        }

        case LlRx: {
            unsigned char *packet = (unsigned char *)malloc(MAX_PAYLOAD_SIZE);
            if (!packet) { perror("malloc"); exit(-1); }
            int packetSize = -1;

            // receive START
            while((packetSize = llread(packet)) < 0);
            unsigned long int fileSize = 0;
            unsigned char *originalFileName = getSizeAndName(packet, packetSize, &fileSize);
            
            if (!originalFileName) {
                free(packet);
                llclose(connectionParameters);
                exit(-1);
            }
            free(originalFileName);

            FILE *newFile = fopen(filename, "wb");
            if (newFile == NULL) {
                perror("Error creating output file");
                free(packet);
                exit(-1);
            }

            unsigned long int receivedBytes = 0;
            unsigned char list = 0;

            while (1) {
                while ((packetSize = llread(packet)) < 0);
                if (packetSize <= 0) continue;

                if (packet[0] == 3) { // END
                    unsigned long int endFileSize = 0;
                    unsigned char *endFileName = getSizeAndName(packet, packetSize, &endFileSize);
                    if (endFileName) {
                        free(endFileName);
                    }
                    break;
                } else if (packet[0] == 2) {
                    if (packetSize < 4) {
                        continue;
                    }
                    unsigned int dataSize = ((unsigned int)packet[2] << 8) | packet[3];
                    if ((int)(4 + dataSize) > packetSize) {
                        continue;
                    }
                    fwrite(packet + 4, sizeof(unsigned char), dataSize, newFile);
                    receivedBytes += dataSize;
                    list = (list + 1) % 2; 
                }
            }

            fclose(newFile);
            free(packet);
            llclose(connectionParameters);
            break;
        }

        default:
            exit(-1);
            break;
    }

    printf("Done!\n");
}

unsigned char *getControlPacket(const unsigned int c, const char *filename, long int length, unsigned int *size)
{
    int L1 = 0;
    long int temp = length;
    while (temp > 0) {
        L1++;
        temp >>= 8;
    }
    if (L1 == 0) L1 = 1;

    const int L2 = (int)strlen(filename);

    *size = 1 + 1 + 1 + L1 + 1 + 1 + L2;

    unsigned char *packet = (unsigned char *)malloc(*size);
    if (!packet) { perror("malloc"); exit(-1); }

    unsigned int pos = 0;
    packet[pos++] = (unsigned char)c;    
    packet[pos++] = 0;                  
    packet[pos++] = (unsigned char)L1;  

    for (int i = L1 - 1; i >= 0; --i) {
        packet[pos++] = (unsigned char)((length >> (8 * i)) & 0xFF);
    }

    packet[pos++] = 1;                   
    packet[pos++] = (unsigned char)L2;   
    memcpy(packet + pos, filename, L2);
    pos += L2;

    if (pos != *size) {
        free(packet);
        exit(-1);
    }

    return packet;
}



unsigned char *getData(FILE *file, long int fileSize)
{
    unsigned char *cont = (unsigned char *)malloc(sizeof(unsigned char) * fileSize);
    fread(cont, sizeof(unsigned char), fileSize, file);
    return cont;
}

unsigned char *getDataPacket(unsigned char list, unsigned char *data, int dataSize, int *packetSize)
{
    *packetSize = 1 + 1 + 2 + dataSize;
    unsigned char *packet = (unsigned char *)malloc(*packetSize);

    packet[0] = 2;
    packet[1] = list;
    packet[2] = (dataSize >> 8) & 0xFF;
    packet[3] = dataSize & 0xFF;
    memcpy(packet + 4, data, dataSize);

    return packet;
}

unsigned char *getSizeAndName(unsigned char *packet, int size, unsigned long int *fileSize)
{
    if (!packet || size <= 0) {
        fprintf(stderr, "[getSizeAndName ERROR] null/zero packet\n");
        return NULL;
    }

    if (size < 5) {
        fprintf(stderr, "[getSizeAndName ERROR] packet too small (size=%d)\n", size);
        return NULL;
    }

    *fileSize = 0;
    unsigned int fileSizeBytes = (unsigned int)packet[2];

    if (3 + (int)fileSizeBytes >= size) {
        fprintf(stderr, "[getSizeAndName ERROR] fileSizeBytes too large: %u (packet size=%d)\n", fileSizeBytes, size);
        return NULL;
    }

    for (unsigned int i = 0; i < fileSizeBytes; i++) {
        *fileSize = (*fileSize << 8) | (unsigned long int)packet[3 + i];
    }

    unsigned int idx_after_size = 3 + fileSizeBytes;
    if (idx_after_size + 1 >= (unsigned int)size) {
        fprintf(stderr, "[getSizeAndName ERROR] missing filename length (packet size=%d)\n", size);
        return NULL;
    }

    unsigned int fileNameBytes = (unsigned int)packet[idx_after_size + 1];
    unsigned int nameOffset = idx_after_size + 2;

    if (nameOffset + fileNameBytes > (unsigned int)size) {
        fprintf(stderr, "[getSizeAndName ERROR] filename length too large: %u (packet size=%d)\n", fileNameBytes, size);
        return NULL;
    }

    unsigned char *name = (unsigned char *)malloc(fileNameBytes + 1);
    if (!name) { perror("malloc"); exit(-1); }

    memcpy(name, packet + nameOffset, fileNameBytes);
    name[fileNameBytes] = '\0';

    printf("[getSizeAndName DEBUG] fileSize=%lu, filename=\"%s\"\n", *fileSize, name);

    return name;
}

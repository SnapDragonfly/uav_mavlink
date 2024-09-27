#include <stdio.h>
#include <sys/time.h>
#include <netinet/in.h>

#include "rtp_head.h"

void parse_rtp_header(const unsigned char *buffer, struct rtp_header *rtp) {
    rtp->vpxcc = buffer[0];
    rtp->mpayload = buffer[1];
    rtp->sequence = ntohs(*(unsigned short *)(buffer + 2));
    rtp->timestamp = ntohl(*(unsigned int *)(buffer + 4));
    rtp->ssrc = ntohl(*(unsigned int *)(buffer + 8));
}

int validate_rtp_header(struct rtp_header *rtp, ssize_t packet_len) {
    // Check RTP version (Version should be 2)
    unsigned char version = (rtp->vpxcc >> 6) & 0x03;  // The highest 2 bits are the version number
    if (version != 2) {
        printf("Invalid RTP version: %u\n", version);
        return 0;
    }

    // Check sequence number
    if (rtp->sequence == 0) {
        printf("Invalid RTP sequence number: %u\n", rtp->sequence);
        return 0;
    }

    // Check Payload Type (Payload Type should be in the range 0-127)
    unsigned char payload_type = rtp->mpayload & 0x7F;
    if (payload_type > 127) {
        printf("Invalid payload type: %u\n", payload_type);
        return 0;
    }

    // Check timestamp
    if (rtp->timestamp == 0) {
        printf("Invalid RTP timestamp: %u\n", rtp->timestamp);
        return 0;
    }

    // Check if packet length is at least larger than the RTP header length
    if (packet_len < RTP_HEADER_SIZE) {
        printf("Invalid RTP packet length: %ld\n", packet_len);
        return 0;
    }

    return 1; // Valid
}

int validate_rtp_first(struct rtp_header *rtp, ssize_t packet_len) {
    // Check RTP version (Version should be 2)
    unsigned char version = (rtp->vpxcc >> 6) & 0x03;  // The highest 2 bits are the version number
    if (version != 2) {
        printf("Invalid RTP version: %u\n", version);
        return 0;
    }

    // Check sequence number
    if (rtp->sequence == 0) {
        printf("Invalid RTP sequence number: %u\n", rtp->sequence);
        return 0;
    }

    // Check Payload Type (Payload Type should be in the range 0-127)
    unsigned char payload_type = rtp->mpayload & 0x7F;
    if (payload_type > 127) {
        printf("Invalid payload type: %u\n", payload_type);
        return 0;
    }

    // Check timestamp
    if (rtp->timestamp == 0) {
        printf("Invalid RTP timestamp: %u\n", rtp->timestamp);
        return 0;
    }

    // Check if packet length is at least larger than the RTP header length
    if (packet_len < RTP_HEADER_SIZE) {
        printf("Invalid RTP packet length: %ld\n", packet_len);
        return 0;
    }

    // Check if this packet is the first packet of a new image frame (using Marker bit)
    unsigned char marker = (rtp->mpayload >> 7) & 0x01;  // Extract the Marker bit (the highest bit of the second byte)
    if (marker == 1) {
        return 1;
    }

    return 0; // Not first valid rtp packet for a frame
}


void print_rtp_header(struct rtp_header *rtp) {
    unsigned char version = (rtp->vpxcc >> 6) & 0x03;  // Highest 2 bits are the version number
    unsigned char padding = (rtp->vpxcc >> 5) & 0x01;  // Padding flag
    unsigned char extension = (rtp->vpxcc >> 4) & 0x01;  // Extension flag
    unsigned char csrc_count = rtp->vpxcc & 0x0F;  // CSRC count
    unsigned char marker = (rtp->mpayload >> 7) & 0x01;  // Marker flag
    unsigned char payload_type = rtp->mpayload & 0x7F;  // Payload type

    printf("RTP Header Information:\n");
    printf("  Version: %u\n", version);
    printf("  Padding: %u\n", padding);
    printf("  Extension: %u\n", extension);
    printf("  CSRC Count: %u\n", csrc_count);
    printf("  Marker: %u\n", marker);
    printf("  Payload Type: %u\n", payload_type);
    printf("  Sequence Number: %u\n", rtp->sequence);
    printf("  Timestamp: %u\n", rtp->timestamp);
    printf("  SSRC: %u\n", rtp->ssrc);
}


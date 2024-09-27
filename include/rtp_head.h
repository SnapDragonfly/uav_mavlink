#ifndef RTP_HEAD_PARSE_FUNCTION_H
#define RTP_HEAD_PARSE_FUNCTION_H

#define RTP_HEADER_SIZE 12

// RTP header structure
struct rtp_header {
    unsigned char vpxcc;       // Version, Padding, Extension, CSRC
    unsigned char mpayload;    // Marker and Payload Type
    unsigned short sequence;   // Sequence number
    unsigned int timestamp;    // Timestamp
    unsigned int ssrc;         // Synchronization Source identifier (SSRC)
};

/*
  0                   1                   2                   3
  0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1
 +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 |V=2|P|X|  CC   |M|     PT      |       sequence number         |
 +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 |                           timestamp                           |
 +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 |           synchronization source (SSRC) identifier            |
 +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 |            contributing source (CSRC) identifiers (optional)  |
 |                             ....                              |
 +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 |                         Payload Data                          |
 |                             ....                              |
 +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 Total CSRC bytes=CC×4
*/

void parse_rtp_header(const unsigned char *buffer, struct rtp_header *rtp);
int validate_rtp_header(struct rtp_header *rtp, ssize_t packet_len);
int validate_rtp_first(struct rtp_header *rtp, ssize_t packet_len);
void print_rtp_header(struct rtp_header *rtp);

#endif /* RTP_HEAD_PARSE_FUNCTION_H */
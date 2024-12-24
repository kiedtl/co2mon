#ifndef RF24WRAPPER_H
#define RF24WRAPPER_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define NETWORK_DEFAULT_ADDRESS 04444
#define MESH_DEFAULT_ADDRESS NETWORK_DEFAULT_ADDRESS

#define RF24_CRC_DISABLED  0
#define RF24_CRC_8         1
#define RF24_CRC_16        2

#define RF24_1MBPS         0
#define RF24_2MBPS         1
#define RF24_250KBPS       2

typedef struct RF24 RF24;
typedef struct RF24MeshWrapper RF24MeshWrapper;

/* RF24 */
RF24* RF24_create(uint8_t cePin, uint8_t csnPin);
void RF24_destroy(RF24* rf24);
int RF24_begin(RF24* rf24);
_Bool RF24_isChipConnected(RF24* rf24);
_Bool RF24_failureDetected(RF24* rf24);
void RF24_setChannel(RF24* rf24, uint8_t channel);
void RF24_setPayloadSize(RF24* rf24, uint8_t size);
void RF24_setPALevel(RF24* rf24, int level);
void RF24_setCRCLength(RF24* rf24, int length);
void RF24_setDataRate(RF24* rf24, int rate);
void RF24_openWritingPipe(RF24* rf24, unsigned char *address);
void RF24_openReadingPipe(RF24* rf24, uint8_t number, unsigned char *address);
void RF24_startListening(RF24* rf24);
void RF24_stopListening(RF24* rf24);
int RF24_write(RF24* rf24, const void* buffer, uint8_t length);
_Bool RF24_available(RF24* rf24);
void RF24_powerDown(RF24* rf24);

/* RF24Mesh */
RF24MeshWrapper* RF24Mesh_create(uint8_t cePin, uint8_t csnPin);
void RF24Mesh_destroy(RF24MeshWrapper* mesh);
int RF24Mesh_begin(RF24MeshWrapper* mesh, uint8_t channel, int dataRate, uint16_t timeout);
void RF24Mesh_setNodeID(RF24MeshWrapper* mesh, uint8_t nodeId);
uint8_t RF24Mesh_getNodeID(RF24MeshWrapper* mesh);
int RF24Mesh_renewAddress(RF24MeshWrapper* mesh);
_Bool RF24Mesh_write(RF24MeshWrapper* mesh, uint16_t to_node, void* data, uint8_t size, uint8_t msgType);
uint8_t RF24Mesh_read(RF24MeshWrapper* mesh, void* data, uint8_t size);
int RF24Mesh_checkConnection(RF24MeshWrapper* mesh);
void RF24Mesh_update(RF24MeshWrapper* mesh);
void* RF24Mesh_getNetwork(RF24MeshWrapper* mesh);
void* RF24Mesh_getRadio(RF24MeshWrapper* mesh);

#ifdef __cplusplus
}
#endif

#endif // RF24WRAPPER_H

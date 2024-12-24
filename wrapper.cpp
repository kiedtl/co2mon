#include <RF24.h>
#include <RF24Network.h>
#include <RF24Mesh.h>
#include <cstdint>

extern "C" {

static SPI spi;

typedef struct RF24MeshWrapper {
    RF24* radio;
    RF24Network* network;
    RF24Mesh* mesh;
} RF24MeshWrapper;


RF24* RF24_create(uint8_t cePin, uint8_t csnPin) {
    return new RF24(cePin, csnPin);
}

void RF24_destroy(RF24* rf24) {
    delete rf24;
}

bool RF24_begin(RF24* rf24) {
    spi.begin(spi0, 18, 19, 16);
    return rf24->begin(&spi);
}

void RF24_setChannel(RF24* rf24, uint8_t channel) {
    rf24->setChannel(channel);
}

void RF24_setDataRate(RF24* rf24, rf24_datarate_e rate) {
    rf24->setDataRate(rate);
}

void RF24_setCRCLength(RF24* rf24, rf24_crclength_e length) {
    rf24->setCRCLength(length);
}

void RF24_setPALevel(RF24* rf24, rf24_pa_dbm_e level) {
    rf24->setPALevel(level);
}

bool RF24_write(RF24* rf24, const void* buffer, uint8_t length) {
    return rf24->write(buffer, length);
}

bool RF24_available(RF24* rf24) {
    return rf24->available();
}

void RF24_read(RF24* rf24, void* buffer, uint8_t length) {
    rf24->read(buffer, length);
}

void RF24_setPayloadSize(RF24* rf24, uint8_t size) {
    rf24->setPayloadSize(size);
}

void RF24_openWritingPipe(RF24* rf24, const unsigned char *address) {
    rf24->openWritingPipe(address);
}

void RF24_openReadingPipe(RF24* rf24, uint8_t number, const unsigned char *address) {
    rf24->openReadingPipe(number, address);
}

void RF24_startListening(RF24* rf24) {
    rf24->startListening();
}

void RF24_stopListening(RF24* rf24) {
    rf24->stopListening();
}

void RF24_powerDown(RF24* rf24) {
    rf24->powerDown();
}

bool RF24_failureDetected(RF24* rf24) {
    return rf24->failureDetected;
}

bool RF24_isChipConnected(RF24* rf24) {
    return rf24->isChipConnected();
}

RF24MeshWrapper* RF24Mesh_create(uint8_t cePin, uint8_t csnPin) {
    RF24MeshWrapper* mesh = new RF24MeshWrapper();
    mesh->radio = new RF24(cePin, csnPin);
    mesh->network = new RF24Network(*mesh->radio);
    mesh->mesh = new RF24Mesh(*mesh->radio, *mesh->network);
    return mesh;
}

void RF24Mesh_destroy(RF24MeshWrapper* mesh) {
    delete mesh->mesh;
    delete mesh->network;
    delete mesh->radio;
    delete mesh;
}

bool RF24Mesh_begin(RF24MeshWrapper* mesh, uint8_t channel, rf24_datarate_e dataRate, uint16_t timeout) {
    mesh->network->returnSysMsgs = true;

    if (mesh->mesh->getNodeID() > 0) {
        if (mesh->mesh->renewAddress(timeout) == MESH_DEFAULT_ADDRESS) {
            return false;
        }
    } else {
#if !defined(MESH_NOMASTER)
#error "I deleted this part of the code, sorry"
#endif
    }

    return true;
}

void RF24Mesh_setNodeID(RF24MeshWrapper* mesh, uint8_t nodeId) {
    mesh->mesh->setNodeID(nodeId);
}

uint8_t RF24Mesh_getNodeID(RF24MeshWrapper* mesh) {
    return mesh->mesh->getNodeID();
}

bool RF24Mesh_renewAddress(RF24MeshWrapper* mesh) {
    return mesh->mesh->renewAddress();
}

bool RF24Mesh_write(RF24MeshWrapper* mesh, uint16_t to_node, void* data, uint8_t size, uint8_t msgType) {
    return mesh->mesh->write(to_node, data, msgType, size);
}

bool RF24Mesh_checkConnection(RF24MeshWrapper* mesh) {
    return mesh->mesh->checkConnection();
}

void RF24Mesh_update(RF24MeshWrapper* mesh) {
    mesh->mesh->update();
}

RF24Network* RF24Mesh_getNetwork(RF24MeshWrapper* mesh) {
    return mesh->network;
}

RF24* RF24Mesh_getRadio(RF24MeshWrapper* mesh) {
    return mesh->radio;
}

}

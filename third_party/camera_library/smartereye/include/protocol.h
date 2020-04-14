#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <cstdint>

namespace SATP {

class BlockHandler;

class Protocol
{
public:
    enum TramsmitPriority {
        DropWhenBusy = 0,
        EnqueueForcedly,
        WaitToSend
    };

    virtual bool isConnected() = 0;
    virtual bool isAppendable() = 0;
    virtual void sendBlock(uint32_t dataType, const char *block, int size, TramsmitPriority priority = DropWhenBusy) = 0;
    virtual void registerBlockHandler(BlockHandler *blockHandler) = 0;
    virtual void unregisterBlockHandler(BlockHandler *blockHandler) = 0;
};

}
#endif // PROTOCOL_H

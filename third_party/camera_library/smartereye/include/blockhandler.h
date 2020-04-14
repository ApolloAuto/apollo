#ifndef BLOCKHANDLER_H
#define BLOCKHANDLER_H

#include <cstdint>
#include <memory>

using namespace std;
namespace SATP {

class BlockHandler
{
public:
    virtual bool handleReceiveBlock(uint32_t dataType, const char *block, int size) = 0;
    virtual void handleReset(){}
    virtual void handleReady(){}
};

class EnhancedHandler : public BlockHandler
{
public:
    EnhancedHandler() {}
    virtual bool handleReceiveBlockEnhanced(uint32_t dataType, shared_ptr<const char> &block, int size){
        handleReceiveBlock(dataType, block.get(), size);
    }
};

}
#endif // BLOCKHANDLER_H

//
//  navi_share_memory.h
//
//  Created by zhengyu02@baidu.com on 2017/04/11.
//  Copyright © 2016+ baidu. All rights reserved.
//

#ifndef _NAVI_SHARE_MEMORY_H__
#define _NAVI_SHARE_MEMORY_H__

#include <sys/ipc.h>
#include <sys/shm.h>
#include <sys/mman.h>
#include <sys/types.h>

#include "BNATools/navi_car_adapter_def.h"

NAMESPACE_BAIDU // baidu
NAMESPACE_BAIDU_ADAPTER // navi_adapter


// POSIX ShareMemory Implementation
// @class NaviNamedShareMemory
class NaviNamedShareMemory {
public:
    NaviNamedShareMemory() :
        m_shm_id(-1),
        m_shm_size(0),
        m_attach_address(NULL) {}

    enum POSIX_OPEN_FLAGS {
        READ            = PROT_READ,       
        WRITE           = PROT_WRITE,       
        READWRITE       = PROT_READ | PROT_WRITE,       
        EXEC            = PROT_EXEC,
        READ_EXEC       = PROT_READ | PROT_EXEC,
        READWRITEEXEC   = PROT_READ | PROT_WRITE | PROT_EXEC,
    };

public:
    // interfaces
    bool Create(
        const std::string& name,
        size_t size, void *virtual_addr = NULL,
        int flags = READWRITE, mode_t mode = 0666);

    bool ForceOpen(
        const std::string& name,
        size_t size, void *virtual_addr = NULL,
        int flags = READWRITE, mode_t mode = 0666);

    bool Remove(const std::string& name);

    bool Attach(void *virtual_addr, int flags);

    bool Detach();


    void *GetShmAddress() const {
        return m_attach_address;
    }

    size_t GetSize() const {
        return m_shm_size;
    }

    int GetId() const {
        return m_shm_id;
    }

private:
    // prevent to generate copy-constructor
    NaviNamedShareMemory(const NaviNamedShareMemory& src);
    NaviNamedShareMemory& operator=(const NaviNamedShareMemory& src);

private:
    int         m_shm_id;
    size_t      m_shm_size;
    void*       m_attach_address;
};


// SYSTEM V ShareMemory Implementation
// @class NaviSysVShareMemory
class NaviSysVShareMemory {
public:
    NaviSysVShareMemory() :
        m_shm_id(-1),
        m_shm_size(0),
        m_attach_address(NULL) {
    }

public:
    // interfaces
    /**
     * @function: Create
     * @brief: basic interface, create shm and attach
     */
    bool Create(key_t key, size_t size,
            const void *virtual_addr = NULL, int flags = 0);

    bool Open(key_t key, size_t size,
            const void *virtual_addr = NULL, int flags = 0);

    /**
     * @function: ForceOpen
     * @brief: not exist create or size-not-match recreate
     */
    bool ForceOpen(key_t key, size_t size,
            const void *virtual_addr, int flags, bool *new_open);

    bool Remove();

    bool Attach(const void *virtual_addr, int flags);

    bool Detach();

    bool Control(int cmd, void *buf);

    void *GetShmAddress() const {
        return m_attach_address;
    }

    size_t GetSize() const {
        return m_shm_size;
    }

    int GetId() const {
        return m_shm_id;
    }

private:
    bool _open(key_t key, size_t size, int create, int perms);

private:
    // prevent to generate copy-constructor
    NaviSysVShareMemory(const NaviSysVShareMemory& src);
    NaviSysVShareMemory& operator=(const NaviSysVShareMemory& src);

private:
    int         m_shm_id;
    size_t      m_shm_size;
    void*       m_attach_address;
};

#ifdef __cplusplus
extern "C" {
#endif

#ifdef __cplusplus
}
#endif

NAMESPACE_END // navi_adapter
NAMESPACE_END // baidu

#endif // _NAVI_SHARE_MEMORY_H__

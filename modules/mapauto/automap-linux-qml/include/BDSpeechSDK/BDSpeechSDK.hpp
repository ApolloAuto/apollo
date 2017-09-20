/***************************************************************************
 *
 * Copyright (c) 2016 Baidu.com, Inc. All Rights Reserved
 *
 **************************************************************************/

/**
 * @file BDSpeechSDK.hpp
 * @author Vaino(lappivaeinoe@baidu.com)
 * @date 2016/10/26 17:52:07
 * @brief Main interface for speech SDK for Linux.
 *
 **/

#ifndef _BDS_SPEECH_SDK_HPP_
#define _BDS_SPEECH_SDK_HPP_

#include <string>
#include "BDSSDKMessage.hpp"
namespace bds {
    class BDSpeechSDK{
    public:
        static BDSpeechSDK* get_instance(const std::string &SDKType, std::string &outErrorDescription);
        virtual void set_event_listener(void(*listener)(BDSSDKMessage&,void*), void* userParam) = 0;
        static void release_instance(BDSpeechSDK* instance);

        virtual bool post(BDSSDKMessage &message, std::string &outErrorDescription) = 0;
        /*
         * Cleanup for closing the program.
         * This function will clean up some globals and makes sure that
         * everything has been stopped before returning.
         * Call is needed only when the whole program is about to shut down
         * Not calling this function before returning from main() function of the program may lead to a crash due to async nature of release_instance() functions.
         */
        static void do_cleanup();
    protected:
        BDSpeechSDK();
        virtual ~BDSpeechSDK();
    };
}

#endif

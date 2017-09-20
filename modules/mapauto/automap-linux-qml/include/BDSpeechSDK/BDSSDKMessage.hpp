/***************************************************************************
 *
 * Copyright (c) 2016 Baidu.com, Inc. All Rights Reserved
 *
 **************************************************************************/

/**
 * @file BDSSDKMessage.hpp
 * @author Vaino(lappivaeinoe@baidu.com)
 * @date 2016/10/26 17:51:32
 * @brief Message container to pass commands to SDK and receive events/callbacks from the SDK
 *
 **/

#ifndef _BDS_SDK_MESSAGE_HPP_
#define _BDS_SDK_MESSAGE_HPP_
#include <map>
#include <vector>
#include <string>

namespace bds {

    extern const std::string DATA_CHUNK;

    class BDSSDKMessageImpl;
    class BDSSDKMessage{
    public:
        BDSSDKMessage(const std::string &p_name);
        BDSSDKMessage();
        BDSSDKMessage(const BDSSDKMessage &m);
        virtual BDSSDKMessage& operator=(const BDSSDKMessage &m);

        std::string name;
        /*
        char* data;
        unsigned int length;
        */
        virtual ~BDSSDKMessage();

        /* Set parameters to message */
        void set_parameter(const std::string &key, const std::string &value);
        void set_parameter(const std::string &key, int value);
        void set_parameter(const std::string &key, const char* value, int valueLen);

        /* Get keys for parameters set to message */
        std::vector<std::string> string_param_keys();
        std::vector<std::string> int_param_keys();
        std::vector<std::string> char_param_keys();

        /* Get parameters from message */
        bool get_parameter(const std::string &key, std::string &outValue);
        bool get_parameter(const std::string &key, int &outValue);
        bool get_parameter(const std::string &key, const char* &outValue, int &outValueLen);
    private:
        BDSSDKMessageImpl* _impl;
        friend class BDSSDKMessageImpl;
    };
}

#endif

// Copyright 2016 Proyectos y Sistemas de Mantenimiento SL (eProsima).
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/**
 * @file doxygen_modules.h
 */

#ifndef RTPS_DOXYGEN_MODULES_H_
#define RTPS_DOXYGEN_MODULES_H_

//Description of doxygen modules, not used in actual code.

 /*!
 * @defgroup FASTRTPS_GENERAL_API eProsima Fast RTPS API Reference
 * @brief eProsima Fast RTPS API grouped in modules.
 */



 /*!
 * @defgroup RTPS_MODULE RTPS
 * @ingroup FASTRTPS_GENERAL_API
 * @brief RTPS API
 * This is an implementation of the RTPS communication standard defined by the OMG.
 */


/*!
 * @defgroup FASTRTPS_MODULE Publisher Subscriber Public API
 * @ingroup FASTRTPS_GENERAL_API
 * @brief Publisher Subscriber Public API
 * This Module contains the Publisher Subscriber Layer created to facilitate the use of the RTPS protocol.
 */

/** @defgroup FASTRTPS_ATTRIBUTES_MODULE Fast RTPS Attributes Module.
 * @ingroup FASTRTPS_MODULE
 * @brief Attributes class used to define the public entities that the user should use to control this library.
 */

/** @defgroup RTPS_ATTRIBUTES_MODULE RTPS Attributes Module.
 * @ingroup RTPS_MODULE
 * @brief Attributes class used to define the public entities that the user should use to control this library.
 */


/** @defgroup COMMON_MODULE Common Module.
 * @ingroup RTPS_MODULE
 * Common structures used by multiple elements.
 */

/** @defgroup NETWORK_MODULE Network Module
 * @ingroup RTPS_MODULE
 * Includes the elements necessary to interface between the 
 * transport layer and the FastRTPS library.
 */

/** @defgroup TRANSPORT_MODULE Transport Module.
 * @ingroup COMMON_MODULE
 * Built in and user defined transport layer implementations.
 */

/** @defgroup WRITER_MODULE Writer Module
 * @ingroup RTPS_MODULE
 * This module contains all classes and methods associated with RTPSWriter and its specifications, as well as other necessary classes.
 */

/** @defgroup READER_MODULE Reader Module
 * @ingroup RTPS_MODULE
 * This module contains all classes and methods associated with RTPSReader and its specifications, as well as other necessary classes.
 */

#ifndef DOXYGEN_SHOULD_SKIP_THIS_PUBLIC

/** @defgroup PARAMETER_MODULE Qos Module
 * @ingroup COMMON_MODULE
 * All QoS policies and parameters are included here.
 */

/** @defgroup MANAGEMENT_MODULE Management Module
 * @ingroup FASTRTPS_GENERAL_API
 * This module contains classes and methods associated with the management of all other objects. The most important ones
 * are the communication (ResourceSend and ResourceListen) and event (ResourceEvent) resources.
 */

/** @defgroup BUILTIN_MODULE Builtin Protocols Module
 * @ingroup MANAGEMENT_MODULE
 * This module contains the general Builtin Protocols.
 */

/** @defgroup DISCOVERY_MODULE Discovery Module
 * @ingroup MANAGEMENT_MODULE
 * This module contains the classes associated with the Discovery Protocols.
 */

/** @defgroup LIVELINESS_MODULE Liveliness Module
 * @ingroup MANAGEMENT_MODULE
 * This module contains the classes associated with the Writer Liveliness Protocols.
 */

#endif


/** @defgroup UTILITIES_MODULE Shared Utilities
 * @ingroup FASTRTPS_GENERAL_API
 * Shared utilities that can be used by one or more classes in different modules. They are not strictly part of the RTPS implementation
 * but very useful to implement different functionalities.
 */

/**
 * @namespace eprosima eProsima namespace.
 * @ingroup FASTRTPS_GENERAL_API
 */

/**
 * @namespace eprosima::fastrtps Contains the publisher subscriber layer.
 * @ingroup FASTRTPS_MODULE
 */

/**
 * @namespace eprosima::fastrtps::rtps Contains the RTPS protocol implementation
 * @ingroup RTPS_MODULE
 */

/**
 * @namespace eprosima::fastrtps::rtps::TimeConv Auxiliary methods to convert to Time_t to more manageable types.
 *  @ingroup UTILITIES_MODULE
  */

#endif /* RTPS_DOXYGEN_MODULES_H_ */

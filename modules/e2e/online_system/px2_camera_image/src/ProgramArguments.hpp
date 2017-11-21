/////////////////////////////////////////////////////////////////////////////////////////
// This code contains NVIDIA Confidential Information and is disclosed
// under the Mutual Non-Disclosure Agreement.
//
// Notice
// ALL NVIDIA DESIGN SPECIFICATIONS AND CODE ("MATERIALS") ARE PROVIDED "AS IS" NVIDIA MAKES
// NO REPRESENTATIONS, WARRANTIES, EXPRESSED, IMPLIED, STATUTORY, OR OTHERWISE WITH RESPECT TO
// THE MATERIALS, AND EXPRESSLY DISCLAIMS ANY IMPLIED WARRANTIES OF NONINFRINGEMENT,
// MERCHANTABILITY, OR FITNESS FOR A PARTICULAR PURPOSE.
//
// NVIDIA Corporation assumes no responsibility for the consequences of use of such
// information or for any infringement of patents or other rights of third parties that may
// result from its use. No license is granted by implication or otherwise under any patent
// or patent rights of NVIDIA Corporation. No third party distribution is allowed unless
// expressly authorized by NVIDIA.  Details are subject to change without notice.
// This code supersedes and replaces all information previously supplied.
// NVIDIA Corporation products are not authorized for use as critical
// components in life support devices or systems without express written approval of
// NVIDIA Corporation.
//
// Copyright (c) 2014-2016 NVIDIA Corporation. All rights reserved.
//
// NVIDIA Corporation and its licensors retain all intellectual property and proprietary
// rights in and to this software and related documentation and any modifications thereto.
// Any use, reproduction, disclosure or distribution of this software and related
// documentation without an express license agreement from NVIDIA Corporation is
// strictly prohibited.
//
/////////////////////////////////////////////////////////////////////////////////////////

#ifndef SAMPLES_COMMON_PROGRAMARGUMENTS_HPP__
#define SAMPLES_COMMON_PROGRAMARGUMENTS_HPP__

#include <map>
#include <string>
#include <vector>

// EXAMPLE USECASE:
//
//    ProgramArguments arguments(
//    { ProgramArguments::Option_t("optional_option", "with default value"),
//    ProgramArguments::Option_t("required_option")
//    }
//    );
//
//    if (!arguments.parse(argc, argv)) exit(0); // Exit if not all require arguments are provided
//    printf("Program Arguments:\n%s\n", arguments.printList());
//
class ProgramArguments
{
  public:
    struct Option_t {
        Option_t(const char *option_, const char *default_value_)
        {
            option        = option_;
            default_value = default_value_;
            required      = false;
        }

        Option_t(const char *option_)
        {
            option        = option_;
            default_value = "";
            required      = true;
        }

        std::string option;
        std::string default_value;
        bool required;
    };

  public:
    ProgramArguments() {}
    ProgramArguments(std::initializer_list<Option_t> options);
    ProgramArguments(const std::vector<Option_t>& options);
    ~ProgramArguments();

    bool parse(const int argc, const char **argv);

    bool has(const char *name) const;
    void addOption(const Option_t &newOption);

    const std::string &get(const char *name) const;
    void set(const char* option, const char* vaule);

    std::string printList() const;

    std::string parameterString() const;

  private:
    static std::string m_empty;

    std::map<std::string, std::string> arguments;
    std::vector<std::string> required_arguments;
};

#endif // SAMPLES_COMMON_PROGRAMARGUMENTS_HPP__

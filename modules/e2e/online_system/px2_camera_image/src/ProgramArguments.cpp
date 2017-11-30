/////////////////////////////////////////////////////////////////////////////////////////
// This code contains NVIDIA Confidential Information and is disclosed
// under the Mutual Non-Disclosure Agreement.

// Notice
// ALL NVIDIA DESIGN SPECIFICATIONS AND CODE ("MATERIALS") ARE PROVIDED "AS IS" NVIDIA MAKES
// NO REPRESENTATIONS, WARRANTIES, EXPRESSED, IMPLIED, STATUTORY, OR OTHERWISE WITH RESPECT TO
// THE MATERIALS, AND EXPRESSLY DISCLAIMS ANY IMPLIED WARRANTIES OF NONINFRINGEMENT,
// MERCHANTABILITY, OR FITNESS FOR A PARTICULAR PURPOSE.

// NVIDIA Corporation assumes no responsibility for the consequences of use of such
// information or for any infringement of patents or other rights of third parties that may
// result from its use. No license is granted by implication or otherwise under any patent
// or patent rights of NVIDIA Corporation. No third party distribution is allowed unless
// expressly authorized by NVIDIA.  Details are subject to change without notice.
// This code supersedes and replaces all information previously supplied.
// NVIDIA Corporation products are not authorized for use as critical
// components in life support devices or systems without express written approval of
// NVIDIA Corporation.

// Copyright (c) 2014-2016 NVIDIA Corporation. All rights reserved.

// NVIDIA Corporation and its licensors retain all intellectual property and proprietary
// rights in and to this software and related documentation and any modifications thereto.
// Any use, reproduction, disclosure or distribution of this software and related
// documentation without an express license agreement from NVIDIA Corporation is
// strictly prohibited.

/////////////////////////////////////////////////////////////////////////////////////////

#include "ProgramArguments.hpp"

#include <string>
#include <string.h>

std::string ProgramArguments::m_empty("");

ProgramArguments::ProgramArguments(std::initializer_list<Option_t> options)
{
    for (auto &&o : options) {
        arguments[o.option] = o.default_value;
        if (o.required)
            required_arguments.push_back(o.option);
    }
}

ProgramArguments::ProgramArguments(const std::vector<Option_t>& options)
{
    for (auto &&o : options) {
        if (o.required)
            required_arguments.push_back(o.option);
        else
            arguments[o.option] = o.default_value;
    }
}

ProgramArguments::~ProgramArguments()
{
}

bool ProgramArguments::parse(const int argc, const char **argv)
{
    bool show_help = false;

    //std::cmatch match_results;
    for (int i = 1; i < argc; ++i) {
        std::string arg   = argv[i];
        std::size_t mnPos = arg.find("--");

        // has --
        if (mnPos != std::string::npos) {
            arg = arg.substr(mnPos + 2);

            std::string name;
            std::string value;

            std::size_t eqPos = arg.find_first_of("=");
            if (eqPos == std::string::npos || eqPos == 0 || eqPos >= arg.length() - 1) {
                show_help = true;
                continue;
            }

            name  = arg.substr(0, eqPos);
            value = arg.substr(eqPos + 1);

            auto option_it = arguments.find(name);
            if (option_it != arguments.end()) {
                option_it->second = value;
            } else {
                arguments[name] = value;
            }
        }
    }

    // Show Help?
    if (show_help) {
        if (arguments.size() > 0) {
            printf("Options:\n%s\n", printList().c_str());
        }
        else {
            printf("Run application without command line arguments.\n");
        }
        return false;
    }

    // Check Required Arguments
    bool hasAllRequiredArguments = true;
    std::vector<std::string> missing_required;
    for (std::string required_argument : required_arguments) {
        if (!has(required_argument.c_str())) {
            missing_required.push_back(required_argument);
            hasAllRequiredArguments = false;
        }
    }
    if (!hasAllRequiredArguments) {
        std::string missing_required_message;
        std::string example_usage;
        for (std::string required_argument : missing_required) {
            missing_required_message.append("\"");
            missing_required_message.append(required_argument);
            missing_required_message.append("\", ");

            example_usage.append(" --");
            example_usage.append(required_argument);
            example_usage.append("=<value>");
        }
        std::string executable = argv[0];

        printf("ProgramArguments: Missing required arguments: %s e.g.\n\t%s %s\n",
               missing_required_message.c_str(), executable.c_str(), example_usage.c_str());
    }

    return hasAllRequiredArguments;
}

const std::string &ProgramArguments::get(const char *name) const
{
    auto it = arguments.find(name);
    if (it == arguments.end()) {
        printf("ProgramArguments: Missing argument '%s' requested\n", name);
        return ProgramArguments::m_empty;
    } else
        return it->second;
}

bool ProgramArguments::has(const char *name) const
{
    auto it = arguments.find(name);
    if( it == arguments.end() )
        return false;

    return it->second.length() > 0;
}

void ProgramArguments::addOption(const Option_t &newOption)
{
    if (has(newOption.option.c_str()))
        throw std::runtime_error(std::string("ProgramArguments already contains the new option: ") + newOption.option);

    arguments[newOption.option] = newOption.default_value;
    if (newOption.required)
        required_arguments.push_back(newOption.option);
}

void ProgramArguments::set(const char *option, const char *value)
{
    arguments[option] = value;
}


std::string ProgramArguments::printList() const
{
    std::string list;

    for (auto arg : arguments) {
        list.append("--");
        list.append(arg.first);
        list.append("=");
        list.append(arg.second);
        list.append("\n");
    }

    return list;
}

std::string ProgramArguments::parameterString() const
{
    std::string list;

    bool first = true;
    for (auto arg : arguments) {
        if (!first)
            list.append(",");
        list.append(arg.first);
        list.append("=");
        list.append(arg.second);
        first = false;
    }

    return list;
}

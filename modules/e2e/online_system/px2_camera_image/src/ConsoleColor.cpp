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
// Copyright (c) 2015-2016 NVIDIA Corporation. All rights reserved.
//
// NVIDIA Corporation and its licensors retain all intellectual property and proprietary
// rights in and to this software and related documentation and any modifications thereto.
// Any use, reproduction, disclosure or distribution of this software and related
// documentation without an express license agreement from NVIDIA Corporation is
// strictly prohibited.
//
/////////////////////////////////////////////////////////////////////////////////////////

#define _CRT_SECURE_NO_WARNINGS

#include "ConsoleColor.hpp"

#ifdef WINDOWS
    #include <windows.h>
    #include <wincon.h>
    #include <io.h>
    #define isatty(x) _isatty(x)
    #define fileno(x) _fileno(x)
#else
    #include <unistd.h>
#endif
#include <cstdlib>
#include <string>

//------------------------------------------------------------------------------
bool shouldUseColor(bool streamIsTTY) {
#ifdef WINDOWS
    // On Windows the TERM variable is usually not set, but the
    // console there does support colors.
    return streamIsTTY;
#else
    // On non-Windows platforms, we rely on the TERM variable.
    const char* cterm = getenv("TERM");
    if (cterm == nullptr) return false;
    std::string term(cterm);
    const bool termOk =
        (term == "xterm") ||
        (term == "xterm-color") ||
        (term == "xterm-256color") ||
        (term == "screen") ||
        (term == "screen-256color") ||
        (term == "tmux") ||
        (term == "tmux-256color") ||
        (term == "rxvt-unicode") ||
        (term == "rxvt-unicode-256color") ||
        (term == "linux") ||
        (term == "cygwin");
    return streamIsTTY && termOk;
#endif
}

//------------------------------------------------------------------------------
// Returns the ANSI color code for the given color.  COLOR_DEFAULT is
// an invalid input.
const char* getAnsiColorCode(EConsoleColor color) {
  switch (color) {
    case COLOR_RED:     return "1";
    case COLOR_GREEN:   return "2";
    case COLOR_YELLOW:  return "3";
    default:            return NULL;
  };
}

//------------------------------------------------------------------------------
#ifdef WINDOWS
// Returns the character attribute for the given color.
WORD getColorAttribute(EConsoleColor color) {
  switch (color) {
    case COLOR_RED:    return FOREGROUND_RED;
    case COLOR_GREEN:  return FOREGROUND_GREEN;
    case COLOR_YELLOW: return FOREGROUND_RED | FOREGROUND_GREEN;
    default:           return 0;
  }
}
#endif

//------------------------------------------------------------------------------
void printColored(FILE *fd, EConsoleColor color, const char* msg)
{
    bool useColor = (color != COLOR_DEFAULT);

    if(useColor) {
        useColor = shouldUseColor(isatty(fileno(fd)) != 0);
    }
    
#ifdef WINDOWS
    HANDLE fdHandle = 0;
    if(useColor) {
        if (fd == stdout)
            fdHandle = GetStdHandle(STD_OUTPUT_HANDLE);
        else if (fd == stderr)
            fdHandle = GetStdHandle(STD_ERROR_HANDLE);
        else
            useColor = false;
    }    
#endif

    if (!useColor) {
        fprintf(fd, "%s", msg);
        return;
    }

#ifdef WINDOWS
  // Gets the current text color.
  CONSOLE_SCREEN_BUFFER_INFO bufferInfo;
  GetConsoleScreenBufferInfo(fdHandle, &bufferInfo);
  const WORD oldAttrs = bufferInfo.wAttributes;

  // We need to flush the stream buffers into the console before each
  // SetConsoleTextAttribute call lest it affect the text that is already
  // printed but has not yet reached the console.
  fflush(fd);
  SetConsoleTextAttribute(fdHandle,
                          getColorAttribute(color) | FOREGROUND_INTENSITY);
  fprintf(fd, msg);

  fflush(stdout);
  // Restores the text color.
  SetConsoleTextAttribute(fdHandle, oldAttrs);
#else
  fprintf(fd, "\033[0;3%sm", getAnsiColorCode(color));
  fprintf(fd, "%s", msg);
  fprintf(fd, "\033[m");  // Resets the terminal to default.
#endif
}

//------------------------------------------------------------------------------
dwLogCallback getConsoleLoggerCallback(bool useColors, bool disableBuffering)
{
    if(disableBuffering)
        setbuf(stdout, NULL);

    if (useColors)
        return [](dwContextHandle_t, dwLoggerVerbosity level, const char *msg)
    {
        EConsoleColor color = COLOR_DEFAULT;
        FILE *fd = stdout;
        switch (level)
        {
        case DW_LOG_VERBOSE:
        case DW_LOG_DEBUG:
            break;
        case DW_LOG_WARN:
            color = COLOR_YELLOW;
            break;
        case DW_LOG_ERROR:
            color = COLOR_RED;
            fd = stderr;
            break;
        }
        printColored(fd, color, msg);
    };
    else
        return [](dwContextHandle_t, dwLoggerVerbosity, const char *msg) { printf("%s", msg); };
}

#ifndef DATAPROCESSOR_H
#define DATAPROCESSOR_H

#ifdef _WIN64
#   define DATAPROCESSOR_EXPORT     __declspec(dllexport)
#   define DATAPROCESSOR_IMPORT     __declspec(dllimport)
#else
#   define DATAPROCESSOR_EXPORT     __attribute__((visibility("default")))
#   define DATAPROCESSOR_IMPORT     __attribute__((visibility("default")))
#   define DATAPROCESSOR_HIDDEN     __attribute__((visibility("hidden")))
#endif

#if defined(DATAPROCESSOR_LIBRARY)
#  define DATAPROCESSOR_SHARED_EXPORT DATAPROCESSOR_EXPORT
#else
#  define DATAPROCESSOR_SHARED_EXPORT DATAPROCESSOR_IMPORT
#endif

#endif // DATAPROCESSOR_H

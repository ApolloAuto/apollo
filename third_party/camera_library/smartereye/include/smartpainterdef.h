#ifndef SMARTPAINTERDEF_H
#define SMARTPAINTERDEF_H

#ifdef _WIN64
#   define SMARTPAINTER_EXPORT     __declspec(dllexport)
#   define SMARTPAINTER_IMPORT     __declspec(dllimport)
#else
#   define SMARTPAINTER_EXPORT     __attribute__((visibility("default")))
#   define SMARTPAINTER_IMPORT     __attribute__((visibility("default")))
#   define SMARTPAINTER_HIDDEN     __attribute__((visibility("hidden")))
#endif

#if defined(SMARTPAINTER_LIBRARY)
#  define SMARTPAINTER_SHARED_EXPORT SMARTPAINTER_EXPORT
#else
#  define SMARTPAINTER_SHARED_EXPORT SMARTPAINTER_IMPORT
#endif

#endif // SMARTPAINTERDEF_H

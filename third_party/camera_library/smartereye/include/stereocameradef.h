#ifndef STEREOCAMERADEF_H
#define STEREOCAMERADEF_H

#ifdef _WIN64
#   define STEREO_EXPORT     __declspec(dllexport)
#   define STEREO_IMPORT     __declspec(dllimport)
#else
#   define STEREO_EXPORT     __attribute__((visibility("default")))
#   define STEREO_IMPORT     __attribute__((visibility("default")))
#   define STEREO_HIDDEN     __attribute__((visibility("hidden")))
#endif

#if defined(STEREOCAMERA_LIBRARY)
#  define STEREO_SHARED_EXPORT STEREO_EXPORT
#else
#  define STEREO_SHARED_EXPORT STEREO_IMPORT
#endif

#endif // STEREOCAMERADEF_H

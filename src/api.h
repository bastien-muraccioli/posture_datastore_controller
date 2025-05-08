#pragma once

#if defined _WIN32 || defined __CYGWIN__
#  define PostureDatastoreController_DLLIMPORT __declspec(dllimport)
#  define PostureDatastoreController_DLLEXPORT __declspec(dllexport)
#  define PostureDatastoreController_DLLLOCAL
#else
// On Linux, for GCC >= 4, tag symbols using GCC extension.
#  if __GNUC__ >= 4
#    define PostureDatastoreController_DLLIMPORT __attribute__((visibility("default")))
#    define PostureDatastoreController_DLLEXPORT __attribute__((visibility("default")))
#    define PostureDatastoreController_DLLLOCAL __attribute__((visibility("hidden")))
#  else
// Otherwise (GCC < 4 or another compiler is used), export everything.
#    define PostureDatastoreController_DLLIMPORT
#    define PostureDatastoreController_DLLEXPORT
#    define PostureDatastoreController_DLLLOCAL
#  endif // __GNUC__ >= 4
#endif // defined _WIN32 || defined __CYGWIN__

#ifdef PostureDatastoreController_STATIC
// If one is using the library statically, get rid of
// extra information.
#  define PostureDatastoreController_DLLAPI
#  define PostureDatastoreController_LOCAL
#else
// Depending on whether one is building or using the
// library define DLLAPI to import or export.
#  ifdef PostureDatastoreController_EXPORTS
#    define PostureDatastoreController_DLLAPI PostureDatastoreController_DLLEXPORT
#  else
#    define PostureDatastoreController_DLLAPI PostureDatastoreController_DLLIMPORT
#  endif // PostureDatastoreController_EXPORTS
#  define PostureDatastoreController_LOCAL PostureDatastoreController_DLLLOCAL
#endif // PostureDatastoreController_STATIC

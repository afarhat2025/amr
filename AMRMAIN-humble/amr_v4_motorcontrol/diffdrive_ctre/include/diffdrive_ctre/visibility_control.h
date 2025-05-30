

#ifndef DIFFDRIVE_CTRE__VISIBILITY_CONTROL_H_
#define DIFFDRIVE_CTRE__VISIBILITY_CONTROL_H_



#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define DIFFDRIVE_CTRE_EXPORT __attribute__((dllexport))
#define DIFFDRIVE_CTRE_IMPORT __attribute__((dllimport))
#else
#define DIFFDRIVE_CTRE_EXPORT __declspec(dllexport)
#define DIFFDRIVE_CTRE_IMPORT __declspec(dllimport)
#endif
#ifdef DIFFDRIVE_CTRE_BUILDING_DLL
#define DIFFDRIVE_CTRE_PUBLIC DIFFDRIVE_CTRE_EXPORT
#else
#define DIFFDRIVE_CTRE_PUBLIC DIFFDRIVE_CTRE_IMPORT
#endif
#define DIFFDRIVE_CTRE_PUBLIC_TYPE DIFFDRIVE_CTRE_PUBLIC
#define DIFFDRIVE_CTRE_LOCAL
#else
#define DIFFDRIVE_CTRE_EXPORT __attribute__((visibility("default")))
#define DIFFDRIVE_CTRE_IMPORT
#if __GNUC__ >= 4
#define DIFFDRIVE_CTRE_PUBLIC __attribute__((visibility("default")))
#define DIFFDRIVE_CTRE_LOCAL __attribute__((visibility("hidden")))
#else
#define DIFFDRIVE_CTRE_PUBLIC
#define DIFFDRIVE_CTRE_LOCAL
#endif
#define DIFFDRIVE_CTRE_PUBLIC_TYPE
#endif

#endif  // DIFFDRIVE_CTRE__VISIBILITY_CONTROL_H_

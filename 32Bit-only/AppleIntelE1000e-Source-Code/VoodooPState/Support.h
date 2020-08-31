/*
 --- Support ---
 (C) 2008 Superhai
 
 Contact	http://www.superhai.com/
 
 */

// Information helpers

#include <IOKit/IOLib.h>

extern const char * KextVersion;
extern const char * KextProductName;
extern const char * KextOSX;
extern const char * KextConfig;
extern const char * KextBuildDate;
extern const char * KextBuildTime;

const unsigned long Kilo = 1000;
const unsigned long Mega = Kilo * 1000;
const unsigned long Giga = Mega * 1000;


// Debug output support

#define INFO 1
#define ERROR 1

#if DEBUG
#define DebugLog(string, args...) IOLog("%s: [Debug] [%05u] " string "\n", KextProductName, __LINE__, ## args); IOSleep(100)
#define DebugMarker IOLog("%s: [Debug] [%s][%u] \n", KextProductName, __PRETTY_FUNCTION__, __LINE__); IOSleep(100)
#else
#define DebugLog(string, args...)
#define DebugMarker
#endif

#if INFO
#define InfoLog(string, args...) IOLog("%s: " string "\n", KextProductName, ## args)
#else
#define InfoLog(string, args...)
#endif

#if ERROR
#define ErrorLog(string, args...) IOLog("%s: [Error] " string "\n", KextProductName, ## args)
#define WarningLog(string, args...) IOLog("%s: [Warning] " string "\n", KextProductName, ## args)
#else
#define ErrorLog(string, args...)
#define WarningLog(string, args...)
#endif


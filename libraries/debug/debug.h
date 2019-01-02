#ifndef DEBUG_h
#define DEBUG_h

//  #ifdef DEBUG
    #define DBAUDRATE (115200)
    #define DSETUP() do{Serial.begin(DBAUDRATE);}while(0)
    #define DPRINT(_m) do{Serial.print(_m);}while(0)
    #define DPRINTLN(_m) do{Serial.println(_m);}while(0)
//  #else
//    #define DSETUP()
//    #define DPRINT(...)
//    #define DPRINTLN(...)
//  #endif

#endif

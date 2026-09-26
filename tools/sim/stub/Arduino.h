#pragma once
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <cctype>
#include <string>
#define IRAM_ATTR
#define INPUT_PULLUP 2
#define FALLING 2
#define MALLOC_CAP_SPIRAM 1
#define MALLOC_CAP_8BIT 2
#define SERIAL_8N1 0
uint32_t millis();
inline void delay(uint32_t){}
inline bool psramFound(){return true;}
inline void* heap_caps_malloc(size_t n,int){return malloc(n);}
inline void pinMode(int,int){}
inline int digitalPinToInterrupt(int p){return p;}
inline void attachInterrupt(int,void(*)(),int){}
inline void noInterrupts(){} inline void interrupts(){}
inline bool isHexadecimalDigit(int c){return isxdigit(c);}
template<class T,class L,class H> T constrain(T x,L l,H h){return x<l?l:(x>h?h:x);}
inline char* dtostrf(double v,signed char w,unsigned char p,char*b){sprintf(b,"%*.*f",w,p,v);return b;}
class String {
 public: std::string s;
  String(){} String(const char*c):s(c){} String(const std::string&x):s(x){}
  unsigned length()const{return s.size();}
  const char* c_str()const{return s.c_str();}
  int indexOf(const String&t)const{auto p=s.find(t.s);return p==std::string::npos?-1:(int)p;}
  String substring(unsigned a)const{return a>=s.size()?String():String(s.substr(a));}
  String substring(unsigned a,unsigned b)const{return a>=s.size()?String():String(s.substr(a,b-a));}
  void trim(){size_t a=s.find_first_not_of(" \t\r\n");size_t b=s.find_last_not_of(" \t\r\n");s=a==std::string::npos?"":s.substr(a,b-a+1);}
  void replace(const String&f,const String&r){size_t p=0;while((p=s.find(f.s,p))!=std::string::npos){s.replace(p,f.s.size(),r.s);p+=r.s.size();}}
  float toFloat()const{return atof(s.c_str());}
  char operator[](unsigned i)const{return s[i];}
  String& operator+=(char c){s+=c;return *this;}
  bool operator==(const char*c)const{return s==c;} bool operator!=(const char*c)const{return s!=c;}
  friend String operator+(const String&a,const String&b){return String(a.s+b.s);}
  friend String operator+(const String&a,const char*b){return String(a.s+b);}
};
struct SerialC{ template<class T> void print(T){} template<class T> void println(T){} void println(){} template<class...A> void printf(const char*,A...){} void flush(){} void begin(int){} explicit operator bool(){return true;} };
extern SerialC Serial;
class HardwareSerial{public:HardwareSerial(int){} void begin(int,int,int,int){} int available(){return 0;} int read(){return -1;} template<class T> void print(T){}};
// FreeRTOS
typedef void* SemaphoreHandle_t;
#define portMAX_DELAY 0xffffffff
inline SemaphoreHandle_t xSemaphoreCreateMutex(){return (void*)1;}
inline int xSemaphoreTake(SemaphoreHandle_t,uint32_t){return 1;}
inline int xSemaphoreGive(SemaphoreHandle_t){return 1;}
inline void vTaskDelay(int){}
inline int xTaskCreatePinnedToCore(void(*)(void*),const char*,int,void*,int,void*,int){return 1;}

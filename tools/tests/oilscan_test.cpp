// Test der Oel-Such-Logik (OilScan.h) auf dem PC:
//   g++ -Wall -I. -o /tmp/oiltest tools/tests/oilscan_test.cpp && /tmp/oiltest
#include <cstdio>
#include "OilScan.h"
static int fails=0;
#define CHECK(c) do{ if(!(c)){printf("FAIL %s:%d %s\n",__FILE__,__LINE__,#c);fails++;} }while(0)
int main(){
  uint8_t d[8]; uint8_t nrc=0; int n;
  n=uds_parse("62 10 01 7A",0x1001,d,8,&nrc); CHECK(n==1&&d[0]==0x7A);
  n=uds_parse("6210017A",0x1001,d,8,&nrc); CHECK(n==1&&d[0]==0x7A);
  n=uds_parse("62 20 2F 0E 9C",0x202F,d,8,&nrc); CHECK(n==2&&d[0]==0x0E&&d[1]==0x9C);
  n=uds_parse("7F 22 31",0x1001,d,8,&nrc); CHECK(n==UDS_NEG&&nrc==0x31);
  n=uds_parse("7F2233",0x1001,d,8,&nrc); CHECK(n==UDS_NEG&&nrc==0x33);
  n=uds_parse("NO DATA",0x1001,d,8,&nrc); CHECK(n==UDS_NONE);
  n=uds_parse("",0x1001,d,8,&nrc); CHECK(n==UDS_NONE);
  n=uds_parse("?",0x1001,d,8,&nrc); CHECK(n==UDS_NONE);
  n=uds_parse("SEARCHING...\n62 10 01 55",0x1001,d,8,&nrc); CHECK(n==1&&d[0]==0x55);
  n=uds_parse("7F 22 78\n62 10 01 55",0x1001,d,8,&nrc); CHECK(n==1&&d[0]==0x55);  // responsePending, dann Antwort
  n=uds_parse("00B\n0: 62 F1 90 01 02 03\n1: 04 05 06 07 08 09 0A",0xF190,d,8,&nrc); CHECK(n==8&&d[0]==0x01&&d[7]==0x08);
  n=uds_parse("62 10 02 7A",0x1001,d,8,&nrc); CHECK(n==UDS_NONE);  // falsche DID
  float v;
  uint8_t a1[]={0x7A}; CHECK(oil_fml_eval(0,a1,1,&v)&&v==82);
  uint8_t a2[]={0x0E,0x9C}; CHECK(oil_fml_eval(2,a2,2,&v)&&v>100.8f&&v<101.0f); // 3740/10 - 273.1 = 100.9
  printf("K/10: %.2f\n",v);
  CHECK(!oil_fml_eval(2,a1,1,&v));
  // Kandidaten: Kuehlwasser kalt 15 warm 90
  OilHit h[5]={};
  h[0]={0x1001,1,{55,0},{130,0},true};   // A-40: 15 -> 90 = wie Kuehlwasser
  h[1]={0x1002,1,{56,0},{140,0},true};   // A-40: 16 -> 100 -> Oel-Kandidat
  h[2]={0x1003,1,{100,0},{101,0},false}; // nicht warm gelesen
  h[3]={0x1004,2,{0x0B,0x6C},{0x0E,0xD8},true}; // K/10: 292.4-273.1=19.3 -> 380.0-273.1=106.9
  h[4]={0x1005,1,{10,0},{12,0},true};    // zu wenig Aenderung
  OilCand c[8]; int k=oil_scan_eval(h,5,15,90,c,8);
  for(int i=0;i<k;i++)printf("cand %04X %s %.1f->%.1f %s\n",c[i].did,OIL_FML_NAME[c[i].fml],c[i].cold,c[i].warm,c[i].likeCoolant?"(KW)":"");
  CHECK(k==3); CHECK(c[0].did==0x1002&&!c[0].likeCoolant); CHECK(c[1].did==0x1004&&c[1].fml==2); CHECK(c[2].did==0x1001&&c[2].likeCoolant);
  printf(fails?"%d FEHLER\n":"alle Tests ok\n",fails); return fails;
}

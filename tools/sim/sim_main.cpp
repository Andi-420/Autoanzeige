/*
 * Host-Simulator fuer die Oberflaeche von OBD_LVGL_v7.ino
 * Baut den echten Sketch gegen LVGL (Arduino/ESP32/FreeRTOS werden durch
 * Stubs in stub/ ersetzt), rendert alle Seiten als Bild und prueft, ob
 * Beschriftungen/Buttons ausserhalb des runden 480x480-Displays liegen.
 * Aufruf: tools/sim/build.sh  (siehe README.md daneben)
 */
#include "sketch.cpp"
#include <vector>
SerialC Serial; TwoWire Wire;
uint8_t Touch_interrupts=0; CST820_Touch touch_data={0};
uint8_t Touch_Read_Data(void){return 1;} void Touch_CST820_ISR(void){}
void LCD_Init(void){} void Set_Backlight(uint8_t){}
static uint32_t fakeMs=0; uint32_t millis(){return fakeMs;}
static uint16_t fb[480*480];
void LCD_DrawBitmap(uint16_t x,uint16_t y,uint16_t w,uint16_t h,uint8_t*c){
  uint16_t*p=(uint16_t*)c; for(int j=0;j<h;j++)for(int i=0;i<w;i++)fb[(y+j)*480+x+i]=p[j*w+i];}
static void save(const char*fn){FILE*f=fopen(fn,"wb");fprintf(f,"P6 480 480 255\n");
  for(int i=0;i<480*480;i++){uint16_t v=fb[i];unsigned char rgb[3]={(unsigned char)((v>>11)<<3),(unsigned char)(((v>>5)&63)<<2),(unsigned char)((v&31)<<3)};fwrite(rgb,1,3,f);}fclose(f);}
static const float R=230; // sichtbarer Radius abzueglich Rand-Ring (Ring bei 233..237 px)
static int check(lv_obj_t*o,const char*page){
  int bad=0; uint32_t n=lv_obj_get_child_count(o);
  for(uint32_t i=0;i<n;i++){lv_obj_t*c=lv_obj_get_child(o,i);
    bad+=check(c,page);
    if(!lv_obj_check_type(c,&lv_label_class) && !lv_obj_check_type(c,&lv_button_class)) continue;
    lv_area_t a; lv_obj_get_coords(c,&a);
    float worst=0; int xs[2]={a.x1,a.x2+1}, ys[2]={a.y1,a.y2+1};
    for(int xi=0;xi<2;xi++)for(int yi=0;yi<2;yi++){float dx=xs[xi]-240.f,dy=ys[yi]-240.f;float d=sqrtf(dx*dx+dy*dy);if(d>worst)worst=d;}
    if(worst>R){bad++;printf("  [%s] AUSSERHALB r=%.0f  (%d,%d)-(%d,%d)  '%s'\n",page,worst,a.x1,a.y1,a.x2,a.y2,
        lv_obj_check_type(c,&lv_label_class)?lv_label_get_text(c):"<button>");}
  }
  return bad;
}
int main(){
  dataMutex=xSemaphoreCreateMutex();
  lv_init(); lv_tick_set_cb(millis);
  static uint8_t buf[480*480*2];
  display=lv_display_create(480,480);
  lv_display_set_flush_cb(display,lvgl_flush_cb);
  lv_display_set_buffers(display,buf,NULL,sizeof(buf),LV_DISPLAY_RENDER_MODE_FULL);
  gMpuOK=true;
  build_page_dtc(); build_page_main(); build_page_brightness(); build_page_boost(); build_page_accel();
  // Beispieldaten (Worst Case fuer Textbreiten)
  gBatt=14.4f; gOilTemp=105; gOilOK=true; gCoolant=90; gRPM=6500; gBoostBar=-0.35f; gElmOK=gEcuOK=true;
  gAz=-0.45f; gAy=0.62f; gAxMax=-0.9f; gAyMax=0.8f;
  dtcCount=3; dtcList[0]="P0123"; dtcList[1]="U0100"; dtcList[2]="P0420"; gDtcState=DTC_DONE;
  const char* names[]={"dtc","main","bright","boost","accel"};
  int total=0;
  for(int p=0;p<PAGE_COUNT;p++){
    curPage=p;
    if(p==PAGE_MAIN)  refresh_main();
    if(p==PAGE_BOOST) refresh_boost();
    if(p==PAGE_DTC)   refresh_dtc_ui();
    if(p==PAGE_ACCEL) refresh_accel();
    lv_screen_load(screens[p]); fakeMs+=1000; lv_refr_now(display);
    char fn[64]; snprintf(fn,64,"out_%s.ppm",names[p]); save(fn);
    printf("Seite %s:\n",names[p]); total+=check(screens[p],names[p]);
  }
  printf("Gesamt ausserhalb: %d\n",total);
  return total ? 1 : 0;
}

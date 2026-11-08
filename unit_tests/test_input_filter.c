#include "tunuino/device.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
static tunuino_device d;
int main(void){
 tunuino_device_init(&d,NULL,NULL,NULL);uint8_t legacy[TUNUINO_IO_SETTINGS_SIZE],p[TUNUINO_IO_SETTINGS_SIZE];memcpy(legacy,d.active,sizeof(p));memcpy(p,legacy,sizeof(p));
 assert(tunuino_device_validate(p,sizeof(p)));p[70]=1;assert(tunuino_device_validate(p,sizeof(p)));
 for(unsigned i=71;i<82;++i){p[i]=7;assert(!tunuino_device_validate(p,sizeof(p)));p[i]=0;}
 for(unsigned i=82;i<96;++i){p[i]=1;assert(!tunuino_device_validate(p,sizeof(p)));p[i]=0;}
 p[70]=2;assert(!tunuino_device_validate(p,sizeof(p)));p[70]=1;
 for(unsigned code=1;code<=6;++code){p[71]=(uint8_t)code;assert(tunuino_device_restore(&d,p,sizeof(p)));uint16_t raw[16]={0};tunuino_device_sample(&d,0,raw,true,0,false,0,0);
  for(unsigned i=0;i<16;++i)raw[i]=1000;
  tunuino_device_sample(&d,2,raw,true,0,false,0,0);assert(d.raw[0]>0&&d.raw[0]<1000);assert(d.raw[1]==1000&&d.raw[11]==1000&&d.raw[15]==1000);
  const unsigned first=d.raw[0];tunuino_device_sample(&d,2,raw,true,0,false,0,0);assert(d.raw[0]==first);
  for(unsigned ms=4;ms<=1000;ms+=2)tunuino_device_sample(&d,ms,raw,true,0,false,0,0);
  assert(d.raw[0]>=998);
  tunuino_device_sample(&d,1002,raw,false,0,false,0,0);assert(!d.adc_valid&&d.raw[0]==0);raw[0]=2000;tunuino_device_sample(&d,1004,raw,true,0,false,0,0);assert(d.raw[0]==2000);
  raw[0]=3000;tunuino_device_sample(&d,1200,raw,true,0,false,0,0);assert(d.raw[0]==3000);raw[0]=4000;tunuino_device_sample(&d,10,raw,true,0,false,0,0);assert(d.raw[0]==4000);
  assert(tunuino_device_restore(&d,p,sizeof(p)));raw[0]=500;tunuino_device_sample(&d,12,raw,true,0,false,0,0);assert(d.raw[0]==500);
 }
 p[71]=6;p[6]=1;p[16]=0x27;p[17]=0x10;assert(tunuino_device_restore(&d,p,sizeof(p)));uint16_t r[16]={0};tunuino_device_sample(&d,0,r,true,0,false,0,0);r[0]=4095;tunuino_device_sample(&d,2,r,true,0,false,0,0);assert(d.raw[0]>=50&&d.raw[0]<=52);assert(d.live.map_milli_kpa>1200&&d.live.map_milli_kpa<1300);
 assert(tunuino_device_restore(&d,legacy,sizeof(legacy)));tunuino_device_sample(&d,4,r,true,0,false,0,0);assert(d.raw[0]==4095);
 puts("AV filters: six cutoffs, bypass, legacy, reserved fields, convergence, timing/fault/generation resets, unfiltered internal supply/AT and MAP integration passed (host simulation)");
}

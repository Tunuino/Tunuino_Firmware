#include "tunuino/device.h"
#include "tunuino/primary_input.h"
#include <assert.h>
#include <string.h>
#include <stdio.h>
static tunuino_device d;
int main(void){
 tunuino_device_init(&d,NULL,NULL,NULL);uint8_t p[TUNUINO_IO_SETTINGS_SIZE],original[TUNUINO_IO_SETTINGS_SIZE];memcpy(p,d.active,sizeof(p));memcpy(original,p,sizeof(p));
 assert(tunuino_device_validate(p,sizeof(p)));for(unsigned version=0;version<4;++version)for(unsigned level=0;level<6;++level){p[5]=(uint8_t)version;p[9]=(uint8_t)level;assert(tunuino_device_validate(p,sizeof(p))==((version==0&&level==0)||(version==1&&level<=3)));}
 const unsigned interval[]={0,5,10,20};for(unsigned level=0;level<=3;++level){tunuino_primary_input f;tunuino_primary_input_reset(&f,(uint8_t)level);assert(tunuino_primary_input_accept(&f,100,true));
  if(level){assert(!tunuino_primary_input_accept(&f,101,true));assert(tunuino_primary_input_accept(&f,102,false));assert(!tunuino_primary_input_accept(&f,100+interval[level]-1,true));assert(tunuino_primary_input_accept(&f,100+interval[level],true));}
  else assert(tunuino_primary_input_accept(&f,100,true));
  tunuino_primary_input_reset(&f,(uint8_t)level);assert(tunuino_primary_input_accept(&f,UINT32_MAX-2,true));if(level){assert(!tunuino_primary_input_accept(&f,0,true));assert(tunuino_primary_input_accept(&f,interval[level]-3,true));}
  tunuino_primary_input_reset(&f,(uint8_t)level);for(unsigned i=0;i<500;++i){assert(tunuino_primary_input_accept(&f,i*25,true));if(level)assert(!tunuino_primary_input_accept(&f,i*25+1,true));assert(tunuino_primary_input_accept(&f,i*25+2,false));}
  p[5]=1;p[9]=(uint8_t)level;assert(tunuino_device_restore(&d,p,sizeof(p)));for(unsigned i=0;i<sizeof(p);++i)if(i!=5&&i!=9)assert(d.active[i]==original[i]);
 }
 puts("Primary input: legacy/version/levels, first edge, exact limits, noise burst, independent CAM, counter wrap, reset and 25-us tooth train passed (host simulation)");
}

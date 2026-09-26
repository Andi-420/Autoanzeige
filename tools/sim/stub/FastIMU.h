#pragma once
struct calData{int x;}; struct AccelData{float accelX,accelY,accelZ;};
struct QMI8658{int init(calData,int){return 0;} void update(){} void getAccel(AccelData*a){a->accelX=1;a->accelY=0;a->accelZ=0;}};

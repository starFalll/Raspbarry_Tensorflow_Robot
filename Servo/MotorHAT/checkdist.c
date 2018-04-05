#include<stdio.h>
//#include<wiringPi.h>
#include <bcm2835.h>
#include<termio.h>
#include<sys/time.h>
#include<stdlib.h>


 

void checkdist(int GPIO_R,int *var,int *signal,int GPIO_S,void(*t_stop)(int)){
	struct timeval tv1;  
    	struct timeval tv2; 
	long start, stop; 
	double dis;
	if(!bcm2835_init()){ //when initialize wiring failed,print messageto screen  
        printf("setup bcm2835 failed !");  
        return;   
    } 
	bcm2835_gpio_fsel(GPIO_S, BCM2835_GPIO_FSEL_OUTP);
	bcm2835_gpio_fsel(GPIO_R, BCM2835_GPIO_FSEL_INPT);
	while(1){
		if(*signal==1)
			continue;
		else
			*signal=1;
		printf("GPIO:%d\n",GPIO_R);
		bcm2835_gpio_write(GPIO_S,HIGH);
		bcm2835_delayMicroseconds(15);//延时15us
		bcm2835_gpio_write(GPIO_S,LOW);
		while(!bcm2835_gpio_lev(GPIO_R));
			//printf("while1\n");
		gettimeofday(&tv1, NULL);
		while(bcm2835_gpio_lev(GPIO_R));
			//printf("while2\n");
		gettimeofday(&tv2, NULL);
		start = tv1.tv_sec * 1000000 + tv1.tv_usec;   //微秒级的时间  
    		stop  = tv2.tv_sec * 1000000 + tv2.tv_usec;
		dis = ((double)(stop - start) * 34000 / 2)/1000000;  //求出距离 
		if(dis<5){
			*var=1;
			t_stop(1);
		}
		else
			*var=0;
		printf("dist:%lfcm\n",dis);
		*signal=0;
		bcm2835_delay(100);//延时15ms
		
	}
	
}	


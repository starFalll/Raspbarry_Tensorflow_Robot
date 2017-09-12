#include<stdio.h>
#include<wiringPi.h>
#include<termio.h>
#include<time.h>
#include<stdlib.h>

float getdis(){
  int start,end;
  digitalWrite(21,HIGH);
  delayMicroseconds(10);
  digitalWrite(21,LOW);
  start=clock();
  while(!digitalRead(22));
  while(digitalRead(22));
  end=clock();
  return (end-start)*340/2000000.0;
}
int getch(void)
{
  struct termios tm,tm_old;
  int fd=0,ch;
  if (tcgetattr(fd,&tm)<0){
	return -1;
  }

  tm_old=tm;
  cfmakeraw(&tm);
  if(tcsetattr(fd,TCSANOW,&tm)<0){
	return -1;
  }
  ch=getchar();
  if (tcsetattr(fd,TCSANOW,&tm_old)<0){
	return -1;
  }
  return ch;
}

void qian(){
  digitalWrite(0,HIGH);
  digitalWrite(1,LOW);
  digitalWrite(2,HIGH);
  digitalWrite(3,LOW);
  digitalWrite(4,HIGH);
  digitalWrite(5,LOW);
  digitalWrite(6,HIGH);
  digitalWrite(27,LOW);
}

void hou(){
  digitalWrite(0,LOW);
  digitalWrite(1,HIGH);
  digitalWrite(2,LOW);
  digitalWrite(3,HIGH);
  digitalWrite(4,LOW);
  digitalWrite(5,HIGH);
  digitalWrite(6,LOW);
  digitalWrite(27,HIGH);
}
void zuo(){
  digitalWrite(0,HIGH);
  digitalWrite(1,LOW);
  digitalWrite(2,LOW);
  digitalWrite(3,HIGH);
  digitalWrite(4,HIGH);
  digitalWrite(5,LOW);
  digitalWrite(6,LOW);
  digitalWrite(27,HIGH);
}

void you(){
  digitalWrite(0,LOW);
  digitalWrite(1,HIGH);
  digitalWrite(2,HIGH);
  digitalWrite(3,LOW);
  digitalWrite(4,LOW);
  digitalWrite(5,HIGH);
  digitalWrite(6,HIGH);
  digitalWrite(27,LOW);
}

void stop(){
  digitalWrite(0,LOW);
  digitalWrite(1,LOW);
  digitalWrite(2,LOW);
  digitalWrite(3,LOW);
  digitalWrite(4,LOW);
  digitalWrite(5,LOW);
  digitalWrite(6,LOW);
  digitalWrite(27,LOW);
}

int main(){
  int A[4][4]={{LOW,LOW,LOW,HIGH},{LOW,LOW,HIGH,LOW},{LOW,HIGH,LOW,LOW},{HIGH,LOW,LOW,LOW}};
  system("./run-tensorflow-service.sh &");
  wiringPiSetup();
  pinMode (0,OUTPUT);
  pinMode (1,OUTPUT);
  pinMode (2,OUTPUT);
  pinMode (3,OUTPUT);
  pinMode (4,OUTPUT);
  pinMode (5,OUTPUT);
  pinMode (6,OUTPUT);
  pinMode (21,OUTPUT);
  pinMode (22,INPUT);
  pinMode (27,OUTPUT);
  pinMode (24,OUTPUT);
  pinMode (25,OUTPUT);
  pinMode (28,OUTPUT);
  pinMode (29,OUTPUT);
  int i=0;char ch;
  while(1){
	ch=getch();
	switch(ch){
		case 'w':qian();delay(30);stop();break;
		case 's':hou();delay(30);stop();break;
		case 'a':zuo();delay(30);stop();break;
		case 'd':you();delay(30);stop();break;
		case 'q':digitalWrite(24,A[i%3][0]);
			   digitalWrite(25,A[i%3][1]);
			   digitalWrite(28,A[i%3][2]);
			   digitalWrite(29,A[i%3][3]);
			   delay(20);i++;break;
		case 'e':digitalWrite(24,A[i%3][3]);
			   digitalWrite(25,A[i%3][2]);
			   digitalWrite(28,A[i%3][1]);
			   digitalWrite(29,A[i%3][0]);
			    delay(20);i++;break;
		case 'p':system("./client.sh");system("./Text_to_Speech.sh");break;
		default:stop();break;
	}
printf("%c\n",ch);
}
return 0;
}
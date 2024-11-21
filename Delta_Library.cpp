#include "Arduino.h"
#include "Delta_Library.h"
#include "math.h"

void DELTA::READ_Cur(int curv[3],int pin[3])
{
  analogReadResolution(12);
  curv[0] = analogRead(pin[0]);
  curv[1] = analogRead(pin[1]);
  curv[2] = analogRead(pin[2]);
}
void DELTA::KalmanCur(int CURV[3], float P[3],float K[3], float CUR_0[3],float Q, float R)
{
    int H = 1;        //measuremnet map equivalent with C matrix
    float CUR1[3];
  for (byte i = 0; i < 3; i = i + 1) 
  {
    K[i] = P[i]*H/(H*P[i]*H+R);//So the CUR_0 come in, start the KF process
  //update error covariance and PROJECT IT AHEAD
  P[i] = (1-K[i]*H)*P[i]+Q;
  
  CUR1[i] = CUR_0[i] + K[i]*(CURV[i]-H*CUR_0[i]);
  P[i] = P[i]+Q;
  CUR_0[i] = CUR1[i];
  }
}
void DELTA::Enc_update(int up_enc[2], int enc, int last_enc, int pinA, int pinB)
{
  int tmp[2];
  int MSB = digitalRead(pinA); //MSB = most significant bit
  int LSB = digitalRead(pinB); //LSB = least significant bit
  int encoded = (MSB << 1) |LSB; //converting the two pin value to single number, by left shifting 1 bit value from MSB and combine with LSB
  int sum  = (last_enc << 2) | encoded; //adding it to the previous encoded value

  if(sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) up_enc[1] = enc + 1;
  if(sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) up_enc[1] = enc - 1;;

  up_enc[0] = encoded; //store this value for next time
}
void DELTA::Pwm_out(int pin_Dir, int pin_Pwm,int ctrl, int res_Pwm)
{
  analogWriteResolution(res_Pwm);
  int tmp_res = res_Pwm;
  switch(res_Pwm)
  {
    case 8: tmp_res = 255;
    case 10: tmp_res = 1023;
    case 12: tmp_res = 4095;
  }
  if(ctrl>=0)
  {
    
    if(ctrl>=tmp_res) ctrl = tmp_res;
    analogWrite(pin_Pwm, ctrl);
    digitalWrite(pin_Dir,HIGH);
  }
  else
  {
    if(ctrl<=-tmp_res)  ctrl = -tmp_res;
    analogWrite(pin_Pwm,abs(ctrl));
    digitalWrite(pin_Dir,LOW);
  }
}
void DELTA::Send_to_PC(int send_enc[3], int send_ref[3], float send_cur[3])
{
  byte BTA[21];
      DELTA::Slipt_Sign_Number(send_enc[0],BTA[0],BTA[1],BTA[2]);
      DELTA::Slipt_Sign_Number(send_enc[1],BTA[3],BTA[4],BTA[5]);
      DELTA::Slipt_Sign_Number(send_enc[2],BTA[6],BTA[7],BTA[8]);
      DELTA::Slipt_Number(send_ref[0],BTA[9],BTA[10]);
      DELTA::Slipt_Number(send_ref[1],BTA[11],BTA[12]);
      DELTA::Slipt_Number(send_ref[2],BTA[13],BTA[14]);
      DELTA::Slipt_Number(send_cur[0],BTA[15],BTA[16]);
      DELTA::Slipt_Number(send_cur[1],BTA[17],BTA[18]);
      DELTA::Slipt_Number(send_cur[2],BTA[19],BTA[20]);
      for (byte i = 0; i<21; i = i+1)
      {
        SerialUSB.write(BTA[i]);
      }
      /*byte BTA[18];
      DELTA::Slipt_Number(send_enc[0],BTA[0],BTA[1]);
      DELTA::Slipt_Number(send_enc[1],BTA[2],BTA[3]);
      DELTA::Slipt_Number(send_enc[2],BTA[4],BTA[5]);
      DELTA::Slipt_Number(send_ref[0],BTA[6],BTA[7]);
      DELTA::Slipt_Number(send_ref[1],BTA[8],BTA[9]);
      DELTA::Slipt_Number(send_ref[2],BTA[10],BTA[11]);
      DELTA::Slipt_Number(send_cur[0],BTA[12],BTA[13]);
      DELTA::Slipt_Number(send_cur[1],BTA[14],BTA[15]);
      DELTA::Slipt_Number(send_cur[2],BTA[16],BTA[17]);
      for (byte i = 0; i<18; i = i+1)
      {
        SerialUSB.write(BTA[i]);
      }*/
}

void DELTA::Send_To_PCStr(int ENC[3], int REF[3], float Send_CUR[3])
{
  int tmp[3];
  tmp[0] = Send_CUR[0];tmp[1] = Send_CUR[1];tmp[2] = Send_CUR[2];
    DELTA::Slipt_To_DEC(ENC);
    DELTA::Slipt_To_DEC(REF);
    DELTA::Slipt_To_DEC(tmp);
    SerialUSB.println();
}
void DELTA::Slipt_Number(int divide, byte & MSB, byte & LSB)
      {
          if (divide>=255)
        {
          MSB = divide/255;
          LSB = divide%255;
        }
        else if (divide<0)
        {
          MSB = 0; LSB = 0;
          }
        else
        {
          MSB = 0; LSB = divide;
        }
      }
      void DELTA::Slipt_Sign_Number(int divide, byte &sign, byte & MSB, byte & LSB)
      {
          if (divide>=0)
        {
          sign = 0;
          MSB = divide/255;
          LSB = divide%255;
        }
        else if (divide<0)
        {
          sign = 1;
          MSB = abs(divide)/255; 
          LSB = abs(divide)%255;
          }
      }
void DELTA::Slipt_To_DEC(int input[3])
      {
        for(byte k = 0; k<3;k++)
        {
          if(input[k]<0)
          {
            byte tmp[3];
            
            tmp[0] = Abs(input[k])/100;
            tmp[1] = Abs(input[k])%100;
            tmp[1] = tmp[1]/10;
            tmp[2] = Abs(input[k])%10;
            SerialUSB.print('-');
            for (byte i = 0; i<3; i++)
            {
              SerialUSB.print(tmp[i]);
            }
            //SerialUSB.print(tmp[0]+tmp[1] + tmp[2]);
          }
          else
          {
            int tmp_cur[4];
            tmp_cur[0] = input[k]/1000;
            tmp_cur[1] = (input[k]%1000)/100;
            //tmp_cur[1] = tmp_cur[1]/100;
            tmp_cur[2] = (input[k]%100)/10;
            //tmp_cur[2] = tmp_cur[2]/10;
            tmp_cur[3] = input[k]%10;
            for (byte i = 0; i<4; i++)
            {
              SerialUSB.print(tmp_cur[i]);
            }
          }
        
        }
      }

void DELTA::recWithStartEndMarker(char receivedChars[], byte numChar, boolean newData)
{
  static boolean recvInProgress = false;
     static byte ndx = 0;
     unsigned char startMarker = '<';
 
     unsigned char  rc;
    
     if (SerialUSB.available() > 0) 
     {
          rc = SerialUSB.read();
 
          if (recvInProgress == true) 
          {
               if ((receivedChars[ndx-1] + rc*256) != 65535)                                                     //Dieu kien nhan dien ket thuc chuoi
               {
                    receivedChars[ndx] = rc;
                    ndx++;
                    if (ndx >= numChar) { ndx = numChar - 1; }                        //Dieu kien so sanh voi kich thuoc chuoi
               }
               else 
               {
                     receivedChars[ndx] = '\0'; // terminate the string
                     recvInProgress = false;
                     ndx = 0;
                     newData = true;
               }
          }
 
          else if (rc == startMarker) { recvInProgress = true; }
     }
 
}
void PROCESS_data::recWithBytesBuffer(unsigned char receivedChars[], byte numChar, boolean newData)
{
  static boolean recvInprogress = false;
  static byte ndx = 0;
  unsigned char startMarker = '<';


  unsigned char rc;

  if(SerialUSB.available()>0)
  {
    rc = SerialUSB.read();
      if(recvInprogress == true)
      {
          if((receivedChars[ndx-1] + rc*256) != 65535)
          {
              receivedChars[ndx] = rc;
              ndx++;
              if(ndx >= numChar) ndx = numChar - 1;
          }
          else
          {
            receivedChars[ndx] = '\0';
            recvInprogress = false;
            ndx = 0;
            newData = true;
          }
      }
      else if(rc == startMarker) recvInprogress = true;
  }
}
int PROCESS_data::convertString2Number(char receivedChars[], byte startPos)
{
    unsigned int tmp =0;
    tmp = (receivedChars[startPos]-48)*100;
    tmp = tmp + (receivedChars[startPos+1]-48) *10;
    tmp = tmp + receivedChars[startPos+2] - 48;
    return tmp;
}
int PROCESS_data::convertString2MinusNumber(char receivedChars[],byte startPos)
{
  unsigned int tmp = 0;
  if(receivedChars[startPos]=='-')
    tmp = -1*(receivedChars[startPos+1]-48)*10 + (receivedChars[startPos+2]-48);
  else tmp = 1*(receivedChars[startPos+1]-48)*10 + (receivedChars[startPos+2]-48);
  return tmp;
}
int PROCESS_data::convertByte2Number(unsigned char receivedChars[],byte startPos)
{
  int tmp = 0;
  tmp = (receivedChars[startPos]) + receivedChars[startPos+1] *256;
  return tmp;
}
int PROCESS_data::convertByte2Encoder(unsigned char receivedChars[],byte startPos)
{
  int tmp = 0;
  tmp = (receivedChars[startPos]) + receivedChars[startPos+1] *256;
  if (tmp>340) tmp =340;
  else if (tmp<0) tmp=0;
  return tmp;
}
double PROCESS_data::convertByte2MinusNumber(unsigned char receivedChars[], byte startPos)
{
  double tmp = 0;
  if(receivedChars[startPos] == 45)
  tmp = -1*( receivedChars[startPos+1] + receivedChars[startPos+2] * 256)/5;//20
  else if(receivedChars[startPos] == 43)
  //else 
  tmp = 1*(receivedChars[startPos+1] + receivedChars[startPos+2]*256)/5;//20//Cu chia 50
  return tmp;
}

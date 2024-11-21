#include <Arduino.h>
#ifndef DELTA_H
#define DELTA_H

class DELTA {
    public:
    
    void READ_Cur(int curv[3],int pin[3]);
    void KalmanCur(int CURV[3], float P[3],float K[3], float CUR_0[3],float Q, float R);
    void Enc_update(int up_enc[2], int enc, int last_enc, int pinA, int pinB);
    void Pwm_out(int pin_Dir, int pin_Pwm, int ctrl, int res_Pwm);
    void Send_to_PC(int send_enc[3], int send_ref[3], float send_cur[3]);
    void Send_To_PCStr(int send_enc[3], int send_ref[3], float send_cur[3]);
    void Slipt_Number(int divide, byte & MSB, byte & LSB);
    void Slipt_Sign_Number(int divide,byte &sign, byte & MSB, byte & LSB);
    void Slipt_To_DEC(int input[3]);
    void recWithStartEndMarker(char received_Chars[], byte numChar, boolean newData);
    


    private:
};
class PROCESS_data{
    public:
    int convertString2Number(char received_Chars[], byte startPos);
    int convertString2MinusNumber(char received_Chars[],byte startPos);
    int convertByte2Number(unsigned char received_Chars[],byte startPos);
    int convertByte2Encoder(unsigned char received_Chars[],byte startPos);
    int aByte2aNumber(unsigned char received_Chars[],byte startPos);
    double convertByte2MinusNumber(unsigned char recerived_Chars[], byte startPos);

    void recWithBytesBuffer(unsigned char receiChars[], byte startPos, boolean newdata);

    private:
};

#endif

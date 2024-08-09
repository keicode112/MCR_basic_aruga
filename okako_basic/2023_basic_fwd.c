/****************************************************************************/
/* �Ώۃ}�C�R�� 	R8C/38A													*/
/* ̧�ٓ��e     2023_BASIC_OKAKO�v���O����									*/
/* Date         2019.10.01                           �@                     */
/* Copyright    �W���p���}�C�R���J�[�����[���s�ψ���						*/
//�O�֋쓮�p
//�G���R�[�_�F72�p���X�i�A�b�v�_�E���J�E���g�j
/****************************************************************************/

/****************************************************************************/
/* ���̃v���O�����́A���L��ɑΉ����Ă��܂��B								*/
/*�ERY_R8C38�{�[�h															*/
/*�E���[�^�h���C�u���Ver.5													*/
/*�E�Z���T���Ver.5															*/
/****************************************************************************/

/*======================================*/
/* �C���N���[�h 						*/
/*======================================*/
#include <stdio.h>
#include <stdlib.h>

#include "sfr_r838a.h"                  /* R8C/38A SFR�̒�`�t�@�C��    */
#include "printf_lib.h"                 /* printf�g�p���C�u����         */

/*======================================*/
/* �V���{����`							*/
/*======================================*/
/* �萔�ݒ� */

#define PWM_CYCLE      29999 //27500 // 12ms:     10ms:25000 1mm:2500  (16ms)
// 10000                                        /* ���[�^PWM�̎���     2ms */

/* JR 539*/
#define SERVO_CENTER_W   3377 //�T�[�{�Z���^�[�@���F�� ��F�E
		  
#define HANDLE_STEP     22    /* 1�K���̒l   */

/* �}�X�N�l�ݒ� �~�F�}�X�N����(����)  ���F�}�X�N����(�L��)       	*/
#define MASK1_0         0X80        		/* ���~�~�~�~�~�~�~ 	*/
#define MASK0_1         0x01        		/* �~�~�~�~�~�~�~�� 	*/
#define MASK1_1         0x18        		/* �~�~�~�����~�~�~ 	*/
#define MASK1         	0x81        		/* ���~�~�~�~�~�~�� 	*/
#define MASK_right      0x20        		/* �~�~���~�~�~�~�~ 	*/
#define MASK_left      	0x04        		/* �~�~�~�~�~���~�~ 	*/
#define MASK2_0         0x30        		/* �~�~�����~�~�~�~ 	*/
#define MASK0_2         0x0c        		/* �~�~�~�~�����~�~ 	*/
#define MASK2_4         0x3f        		/* �~�~������������ 	*/
#define MASK4_2         0xfc        		/* �������������~�~ 	*/
#define MASK3_3         0xe7        		/* �������~�~������ 	*/
#define MASK0_3         0x07        		/* �~�~�~�~�~������ 	*/
#define MASK3_0         0xe0        		/* �������~�~�~�~�~ 	*/
#define MASK4_0         0xf0        		/* ���������~�~�~�~ 	*/
#define MASK0_4         0x0f        		/* �~�~�~�~�������� 	*/
#define MASK4_4         0xff        		/* ���������������� 	*/

#define LEFT 1	//�R�[�i�[�u���[�L���@�n���h�����O�����L��
#define RIGHT 2 //�R�[�i�[�u���[�L���@�n���h�����O�����L��

#define TargetSpeed 90

/*======================================*/
/* �v���g�^�C�v�錾                     */
/*======================================*/
void init( void );
void timer( unsigned long timer_set );
int check_crossline( void );
int check_rightline( void );
int check_leftline( void );
unsigned char sensor_inp( unsigned char mask );
unsigned char dipsw_get( void );
unsigned char pushsw_get( void );
unsigned char startbar_get( void );
void led_out( unsigned char led );
void motor( int accele_l, int accele_r );
void handle( int angle );
void handle_free( void );
void handle_on(void);
int diff( int pwm );
void motor_mode( int mode_l, int mode_r );
/*======================================*/
/* �O���[�o���ϐ��̐錾                 */
/*======================================*/
unsigned long   cnt0;                   /* timer�֐��p                  */
unsigned long   cnt1;                   /* main���Ŏg�p                 */
unsigned long   cnt_kasokudo;           /* �����x�v�Z���Ɏg�p           */
unsigned long   cnt_syu;                /* �I���^�C�}�[                 */
unsigned long   check_sen_cnt;          /* �^�C�}�p                     */
unsigned int    pattern;                /* �p�^�[���ԍ�                 */
int             servo_center;           /* �T�[�{�Z���^�l               */
int             syu_flag = 0;           /* �I���t���O                   */
int 			corner_flag=0;			/* �؂�ւ����p�t���O      LEFT or RIGHT  */
int             brake_time;	      	    //�������s��u���[�L����(���荞�ݓ��ŃC���N�������g)
int				brake_power_flag = 0;   /* �≺�̃R�[�i�[�u���[�L�΍�   */

/* �����x�v�Z�֘A */
unsigned long   cnt_kasokudo;           /* �����x�v�Z���Ɏg�p           */
unsigned long   kasokudo=0;        		/* �����x                       */
unsigned long   old_kyori=0;            /* �O�̑��s��                   */

/* LCD�֘A */
int             lcd_pattern = 4;        /* ������ʐݒ�                 */
int             base_set;

/* ���݂̏�ԕۑ��p */
int             handleBuff;             /* ���݂̃n���h���p�x�L�^       */
int             leftMotorBuff;          /* ���݂̍����[�^PWM�l�L�^      */
int             rightMotorBuff;         /* ���݂̉E���[�^PWM�l�L�^      */

/* �G���R�[�_�֘A */
unsigned long   iEncoderTotal=0;        /*ENC  �ώZ�l                   */
signed int      iEncoder=0;             /*ENC  ���ݒl                   */
unsigned int    uEncoderBuff=0;         /*ENC  �O��l�ۑ�               */
signed long     streat_ct=0;     //(���荞�݂Ȃ��ŃC���N�������g)
signed int      break_count=0;

/* �p�x�֘A */
int             angle_buff;             /* ���݃n���h���p�x�ێ��p       */

//���s���O�֌W
int             flag_st=0;       //���s���m�F�t���O 0:��~ 1:���s
int             log_ct=0;
int             angle_log,accele_r_log,accele_l_log;
int             log_pattern[500];
int             log_se[500];
int             log_a[500];
int             log_b[500];
int             log_c[500];
int             log_d[500];

//���x����֌W
const int crankTargetSpeed = 45;//GoPro:38 
const int harfTargetSpeed = 80;

//���g�p
const int revolution_difference[] = {   /* �p�x������ցA�O�։�]���v�Z */
   100,98,97,95,93,
   92,90,89,87,86,
   84,83,81,80,79,
   77,76,74,73,71,
   70,69,67,66,65,
   63,62,60,59,58,
   56,55,54,52,51,
   49,48,47,45,44,
   42,41,39,38,36,35
   };

int     streat_flag=0;

/* ���x�ړ����ώZ�o�p�ϐ�*/
signed int      sp;						/* ���x�@�ړ����ϒl           */
signed int      sp_sum=0;				/* ���x�@�ړ����ϒl���Z�p�ϐ� */
signed int      sp_buf[4];				/* ���x�@�ړ����ϒl���Z�p�ϐ� */
signed int      sp_count=0;				/* ���x�@�ړ����ϒl���Z�p�ϐ� */

//�֐��v���g�^�C�v�錾
void check_line(void);

/************************************************************************/
/* ���C���v���O����                                                     */
/************************************************************************/
void main( void ){
    int             i;                  /*���O�o�͎��̉��l            */
    /* �}�C�R���@�\�̏����� */
    init();                             /* ������                       */
    init_uart0_printf( SPEED_9600 );    /* UART0��printf�֘A�̏�����    */
    asm(" fset I ");                    /* �S�̂̊��荞�݋���           */
 	while( pushsw_get()) {
		base_set = 1;
        lcd_pattern = 1;
    }
   handle_on();/*�T�[�{���[�^on*/
    /* �}�C�R���J�[�̏�ԏ����� */
    handle( 0 );
    motor( 0, 0 );

	while( 1 ) {
        switch( pattern ) {
			/*****************************************************************
			    �p�^�[���ɂ���
			    0�F�X�C�b�`���͑҂�
			    1�F�X�^�[�g�o�[���J�������`�F�b�N
			    11�F�ʏ�g���[�X
			    12�F�E�֑�Ȃ��̏I���̃`�F�b�N
			    13�F���֑�Ȃ��̏I���̃`�F�b�N
			    20�F�N���X���C�����o���̏���
			    25�F�N���X���C����̃g���[�X�A�N�����N���o
			   150�F���N�����N���� 
			   250�F�E�N�����N����
			   300�F���n�[�t���C�����o���̏���
			   400�F�E�n�[�t���C�����o���̏���
			*****************************************************************/
            case 0:         /* �X�C�b�`���͑҂� */
                if( pushsw_get()) {
                    led_out( 0x0 );
                    pattern = 5;
                    cnt1 = 0;
                    cnt0 = 0;
                    break;
                }
                if( cnt1 < 100 ) {              /* LED�_�ŏ���*/
                    led_out( 0x1 );
                }
                else if( cnt1 < 200 ) {
                    led_out( 0x2 );
                }
                else {
                    cnt1 = 0;
                }
            break;

            case 5:
                if( pushsw_get() ){
                    if( cnt0 >= 1000 ){
                        pattern = 6;
                        cnt0 = 0;
                        break;
                    }
                    else{
                        led_out( 0x0 );
                    }
                }
                else{
                    pattern = 7;
                }
            break;

            case 6:
                if( (cnt0 % 1000) <= 150 ){
                    led_out( 0x3 );
                }
                else{
                    led_out( 0x2 );
                }
                if( cnt0 == 5000 ){
                    led_out( 0x0 );
                    syu_flag = 1;
                    pattern = 11;
                    flag_st=1;                     //���O�擾�J�n
                    cnt1 = 0;
                }
            break;
			
			case 7:                         //�I�[�g�A�A�Q�[�g�܂ōs����I
				if(startbar_get()){
					cnt1 = 0;
					pattern = 8;
					break;
				}
				else{
					motor(20,20);
				}
				
		        switch( sensor_inp(MASK4_4) ) {
			        case 0x00:
				        handle( 0 );
				    break ;

                    case 0x18://�������� ��������/* �Z���^���܂�����*/
                        handle( 0 );
                    break;

                    case 0x1c://�������� �������� /* �����ɍ���聨�E�֔��Ȃ� */
                        handle( 2 );
                    break;

         	        case 0x0c://�������� ��������
                        handle( 3 );
                    break;

           	        case 0x0e://�������� ��������
                        handle( 4 ); 
                    break;

                    case 0x38: //�������� ��������/* �����ɉE��聨���֔��Ȃ� */
                        handle( -2 );
                    break;

			        case 0x30: //�������� ��������
                        handle( -3 ); 
                    break;

           	        case 0x70://�������� ��������
                        handle( -4 ); 
                    break;

                    default:

                    break;
                }
				break;
			case 8:
				if(cnt1 < 50){
					motor(-80,-80);
				}
				else{
					motor(0,0);
					cnt1 = 0;
					pattern = 1;
					break;
				}
				break;
					

            case 1:        /* �X�^�[�g�o�[���J�������`�F�b�N */
                if( !startbar_get() ) { /* �X�^�[�g�I�I */
                    led_out( 0x0 );
                    syu_flag = 1;//�X�^�[�g�m�F()
                    flag_st=1;   //���O�擾�J�n
                    pattern = 11;
                    cnt1 = 0;
                    break;
                }
                if( cnt1 < 50 ) {               /* LED�_�ŏ���      */
                    led_out( 0x1 );
                }
                else if( cnt1 < 100 ) {
                    led_out( 0x2 );
                }
                else {
                    cnt1 = 0;
                }
            break;

            case 11:	//�ʏ�g���[�X
				handle_on();//�T�[�{�d��on
	            led_out( 0x0 );
                if( check_crossline() ) {       /* ��Ȃ������N���X���C���`�F�b�N */
                    pattern = 20;
					cnt1=0;
                    break;
                }
                if( check_rightline() ) {       /* �E�n�[�t���C���`�F�b�N       */
                    pattern = 400;
					cnt1=0;
                    break;
                }
                if( check_leftline() ) {        /* ���n�[�t���C���`�F�b�N       */
                    pattern = 300;
					cnt1=0;
                    break;
                }
				
		        switch( sensor_inp(MASK4_4) ) {
			        case 0x00:
				        handle( 0 );
				        motor( 0,0 );
				    break ;

                    case 0x18://�������� ��������/* �Z���^���܂�����*/
                        handle( 0 );
                        motor( 100,100 );
                    break;

                    //�����@�E�Ȃ�����
                    case 0x1c://�������� �������� /* �����ɍ���聨�E�֔��Ȃ� */
                        handle( 1 );
                        motor( 100 ,100 );
                    break;

         	        case 0x0c://�������� ��������
                        handle( 3 );
                        motor( 100 ,100 );
                    break;

           	        case 0x0e://�������� ��������
                        handle( 6 ); 
                        motor( 100 ,100 );
                    break;

			        case 0x06://�������� ��������/* ��������聨�E�֏��Ȃ� */
                        handle( 10 );
                        motor( 100 ,100 );
			            pattern = 12;
				        //�X�g���[�g��̃u���[�L��
				        if(sp >= 45 && break_count <= 2){	//�O�񍶃u���[�L:LEFT�@�O��E�u���[�L:RIGHT
					        pattern = 112;
		           	        cnt1 = 0;//�u���[�L�J�E���g���ԃN���A
				        }
                    break;

				    //�傫���E�Ȃ����[�h��
                    case 0x07://�������� ��������/* �����炢����聨�E�֒��Ȃ� */
			            pattern = 12;
				        //�X�g���[�g��̃u���[�L��
				        if(sp >= 45 && break_count <= 2){	//�O�񍶃u���[�L:LEFT�@�O��E�u���[�L:RIGHT
					        pattern = 112;
                	        cnt1 = 0;//�u���[�L�J�E���g���ԃN���A
				        }
                    break;

				    //�傫���E�Ȃ����[�h��
                    case 0x03://�������� ��������/* �傫������聨�E�֑�Ȃ� */
			            pattern = 12;
                        //�X�g���[�g��̃u���[�L��
				        if(sp >= 45 && break_count <= 2){	//�O�񍶃u���[�L:LEFT�@�O��E�u���[�L:RIGHT
					        pattern = 112;
                	        cnt1 = 0;//�u���[�L�J�E���g���ԃN���A
				        }
                    break;

                    //�E���@���Ȃ�����
                    case 0x38: //�������� ��������/* �����ɉE��聨���֔��Ȃ� */
                        handle( -1 );
                        motor( 100 ,100 );
                    break;

			        case 0x30: //�������� ��������
                        handle( -3 ); 
                        motor( 100 ,100 );//92 ,100
                    break;

           	        case 0x70://�������� ��������
                        handle( -6 ); 
                        motor( 100 ,100 );
                    break;

                    case 0x60://�������� ��������/* �����E��聨���֏��Ȃ� */
                        handle( -10 );
                        motor( 100 ,100 );
			            pattern = 13;
				        //�X�g���[�g��̃u���[�L��
				        if(sp >= 45 && break_count <= 2){	//�O�񍶃u���[�L:LEFT�@�O��E�u���[�L:RIGHT
					        pattern = 113;
                	        cnt1 = 0;//�u���[�L�J�E���g���ԃN���A
				        }
                    break;

                    //�傫�����Ȃ����[�h��
                    case 0xc0://�������� ��������/* �傫���E��聨���֑�Ȃ� */
                        pattern = 13;
                        //�X�g���[�g��̃u���[�L��
                        if(sp >= 45 && break_count <= 2){	//�O�񍶃u���[�L:LEFT�@�O��E�u���[�L:RIGHT
                            pattern = 113;
                            cnt1 = 0;//�u���[�L�J�E���g���ԃN���A
                        }
                    break;

                    case 0xe0://�������� ��������/* �����炢�E��聨���֒��Ȃ� */
                        pattern = 13;
                        //�X�g���[�g��̃u���[�L��
                        if(sp >= 45 && break_count <= 2){	//�O�񍶃u���[�L:LEFT�@�O��E�u���[�L:RIGHT
                            pattern = 113;
                            cnt1 = 0;//�u���[�L�J�E���g���ԃN���A
                        }
                    break;

                    default:

                    break;
                }
		  
				if(sp > 90){  //���x���� GoPro:65
					motor( 1 ,1 );
		    	}
		  
            break;

            //������̃u���[�L����(�����@�E�Ȃ�����)
            case 112:
                if( check_crossline() ) {       /* ��Ȃ������N���X���C���`�F�b�N */
                    pattern = 20;
					cnt1=0;
                    break;
                }
                if( check_rightline() ) {       /* �E�n�[�t���C���`�F�b�N       */
                    pattern = 400;
					cnt1=0;
                    break;
                }
                if( check_leftline() ) {        /* ���n�[�t���C���`�F�b�N       */
                    pattern = 300;
					cnt1=0;
                    break;
                }

                /* �E�Ȃ����̒����u���[�L����*///////////////////////////////////////////////////////////////////////////////
                if(break_count < 1){//�u���[�L1���
					if(sp >= 70){
						handle( 30 );
						motor( -100 ,-90 );
						brake_time=40;
					}
					else if(sp >= 60){
						handle( 30 );
						motor( -50 ,-100 );
						brake_time=35;
					}
				    else if(sp >= 50){
						handle( 30 );
						motor( -20 ,-80 );
						brake_time=30;
					}
                }                               // �u���[�L2���
                else if(sp >= 50){              // ������
                    handle_on();//�T�[�{�d��on
					handle( 45 );
                    motor( -100 ,-100 );
                    brake_time=40;
					brake_power_flag = 0;
                }
				else{                           // �����ł��Ȃ���
                    handle_on();//�T�[�{�d��on
					handle( 30 );
                    motor( 0 ,-50 );
                    brake_time=30;
					brake_power_flag = 0;
                }
				//////////////////////////////////////////////////////////////////////////////////////////////////
				
				
                //�������� ��������               �������� ��������              �������� ��������              �������� ��������             �������� ��������
                if(sensor_inp(MASK4_4)==0x30 || sensor_inp(MASK4_4)==0x38 || sensor_inp(MASK4_4)==0x18 || sensor_inp(MASK4_4)==0x1c || sensor_inp(MASK4_4)==0x0c){
                    pattern = 11;
                    streat_flag=0;//�X�g���[�g�t���O�N���A
                    corner_flag=RIGHT;//�R�[�i�[�t���O���E�Ȃ�
                    break_count++;
                }

                if(cnt1>brake_time){//�u���[�L����ms
                    pattern=12;
                    streat_flag=0;//�X�g���[�g�t���O�N���A
                    corner_flag=RIGHT;//�R�[�i�[�t���O���E�Ȃ�
                    break_count++;
                }
            break;

            //������̃u���[�L����(�E���@���Ȃ�����)
            case 113:
                if( check_crossline() ) {       /* ��Ȃ������N���X���C���`�F�b�N */
                    pattern = 20;
					cnt1=0;
                    break;
                }
                if( check_rightline() ) {       /* �E�n�[�t���C���`�F�b�N       */
                    pattern = 400;
					cnt1=0;
                    break;
                }
                if( check_leftline() ) {        /* ���n�[�t���C���`�F�b�N       *///��Ŗ���
				
                    pattern = 300;
					cnt1=0;
                    break;
                }

                /* ���Ȃ����̒����u���[�L����*/
                if(break_count < 1){//�u���[�L1���///////////////////////////////////////////////////////////////////////
					if(sp > 70){
						handle( -30 );
						motor( -100 ,-70 );
						brake_time=40;
					}
					else if(sp >= 60){
						handle( -30 );
						motor( -100 ,-50 );
						brake_time=40;
					}
                    else if(sp >= 50){
						handle( -30 );
						motor( -80 ,-20 );
						brake_time=30;
					}
                }                               // �u���[�L2���
                else if(sp >= 48){              // ������
                    handle_on();//�T�[�{�d��on
					handle( -45 );
                    motor( -100 ,50 );
                    brake_time=40;
					brake_power_flag = 0;
                }
                else{                           // �����ł��Ȃ���
                    handle_on();//�T�[�{�d��on
					handle( -30 );
                    motor( -50 ,0 );
                    brake_time=30;
					brake_power_flag = 0;
                }
				///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
				
                //�������� ��������               �������� ��������                  ������ ��������              �������� ��������       �������� ��������
                if(sensor_inp(MASK4_4)==0x30 || sensor_inp(MASK4_4)==0x38 || sensor_inp(MASK4_4)==0x18 || sensor_inp(MASK4_4)==0x0c || sensor_inp(MASK4_4)==0x1c) {
                    pattern = 11;
                    streat_flag=0;//�X�g���[�g�t���O�N���A
                    corner_flag=LEFT;//�R�[�i�[�t���O�����Ȃ�
                    break_count++;
                }
                if(cnt1>brake_time){//�u���[�L����ms
                    pattern=13;
                    streat_flag=0;//�X�g���[�g�t���O�N���A
                    corner_flag=LEFT;//�R�[�i�[�t���O�����Ȃ�
                    break_count++;
                }
            break;

            //�傫���E�Ȃ����[�h��	�������� ���������@��̏���
            case 12:
				handle_on();//�T�[�{�d��on
                if( check_crossline() ) {       /* ��Ȃ������N���X���C���`�F�b�N */
                    pattern = 20;
					cnt1=0;
                    break;
                }
                switch( sensor_inp(MASK3_3) ) {
                    case 0x04://�������� ��������	//pattern=11�@���A
                    case 0x06://�������� ��������	//pattern=11�@���A
                        pattern = 11;
                    break;

                    case 0x07:  //�������� ��������
                        handle( 10 );
                        motor( 100, 50 );
                    break;

                    case 0x03:  //�������� ��������
                        handle( 15 );
                        motor( 100, 45 );
                    break;

                    case 0x83:// �������� ��������
                        handle( 21 );
                        motor( 100, 40 );
                    break;

                    case 0x81://�������� ��������
                        handle( 22 );
                        motor( 100, 35 );
                    break;

                    case 0xc1://�������� �������� �ǉ�
                        handle( 22 );
                        motor( 100, 30 );
                    break;

                    case 0x80://�������� ��������  /* 80�ǉ� */
                        pattern=14;
                        handle( 23 );
                        motor( 100, 30 );
                    break;

                    case 0xc0://�������� ��������  /* c0�ǉ� */
                        pattern=14;
                        handle( 23 );
                        motor( 100, 30 );
                    break;

					case 0xe0://�������� ��������  /* c0�ǉ� */
                        pattern=14;
                        handle( 24 );
                        motor( 100, 30 );
                    break;

                    //�������� ��������   //e0 ���J�b�g
                    default :
                    
                    break;
                }
            break;

            case 14://���������i��O�ꎞ�̌��������j�p�^�[��12���
				handle_on();//�T�[�{�d��on
                switch( sensor_inp(MASK3_3) ) {
                    case 0x83:// �������� ��������
                        pattern=12;
                        handle( 24 );
                        motor( 100, 30 );
                    break;

                    case 0x81://�������� ��������
                        pattern=12;
                        handle( 27 );
                        motor( 100, 30 );
                    break;

                    case 0xc1://�������� ���������ǉ�
                        pattern=12;
                        handle( 29 );
                        motor( 100, 20 );
                    break;

                    case 0xc0://�������� ��������  /* c0�ǉ� */
                        handle( 31 );
                        motor( 100, 10 );
                    break;

					case 0xe0://�������� ��������  /* c0�ǉ� */
                        handle( 33 );
                        motor( 100, 0 );
                    break;
					
					case 0x60://�������� ��������  /* c0�ǉ� */
                        handle( 35 );
                        motor( 100, -100 );
                    break;
					
					case 0x70://�������� ��������  /* c0�ǉ� */
                        handle( 45 );
                        motor( 100, -100 );
                    break;
					
                    default :
                    
                    break;
                }
            break;

    //�傫�����Ȃ����[�h��	�������� ���������@��̏���
            case 13:
				handle_on();//�T�[�{�d��on
                if( check_crossline() ) {       /* ��Ȃ������N���X���C���`�F�b�N */
                    pattern = 20;
					cnt1=0;
                    break;
                }

                switch( sensor_inp(MASK3_3) ) {
                    case 0x20:	//�������� ��������//pattern=11�@���A
                    case 0x60:	//�������� ��������//pattern=11�@���A
                        pattern = 11;
                    break;

                    case 0xe0:	//�������� ��������
                        handle( -10 );
                        motor( 50, 100 );
                    break;

                    case 0xc0:  //�������� ��������
                        handle( -15 );
                        motor( 45, 100 );
                    break;

                    case 0xc1:  //�������� ��������
                        handle( -21 );
                        motor( 40, 100 );
                    break;

                    case 0x81 :  //�������� ��������
                        handle( -22 );
                        motor( 35, 100 );
                    break;

                    case 0x83:    //�������� ��������
                        handle( -22 );
                        motor( 30, 100 );
                    break;

                    case 0x01:	//�������� ��������
                        pattern=15;
                        handle( -23 );
                        motor( 30, 100 );
                    break;
                
                    case 0x03:	//�������� ��������
                        pattern=15;
                        handle( -23 );
                        motor( 30, 100 );
                    break;

					case 0x07:	//�������� ��������
                        pattern=15;
                        handle( -25 );
                        motor( 30, 100 );
                    break;

                    default :
                    
                    break;
                }
            break;

            case 15://�i��O�ꎞ�̌��������j�p�^�[��13���
				handle_on();//�T�[�{�d��on
                switch( sensor_inp(MASK3_3) ) {
                    case 0xc1:	//�������� ��������
                        pattern = 13;
                        handle( -25 );
                        motor( 30, 100 );
                    break;

                    case 0x81:  //�������� ��������
                        pattern = 13;
                        handle( -27 );
                        motor( 30, 100 );
                    break;

                    case 0x83:	//�������� ��������
                        pattern = 13;
                        handle( -29 );
                        motor( 20, 100 );
                    break;

                    case 0x03:	//�������� ��������
                        handle( -31 );
                        motor( 10, 100 );
                    break;
					
					case 0x07:	//�������� ��������
                        handle( -33 );
                        motor( 0, 100 );
                    break;
					
					case 0x06:	//�������� ��������
                        handle( -35 );
                        motor( -100, 100 );
                    break;
					
					case 0x0e:	//�������� ��������
                        handle( -45 );
                        motor( -100, 100 );
                    break;
					
                    default :

                    break;
                }
            break;


////////////�N�����N/////////////
            case 20:/* �N���X���C������ */
				handle_on();//�T�[�{�d��on
                cnt1 = 0;
				led_out(0x0);
				pattern=25;
            break;

            case 25:
				handle_on();//�T�[�{�d��on
				if(cnt1>35){/////////////////////�S���~
					led_out(0x0);
					pattern=120;
					cnt1 = 0;
				}
				switch( sensor_inp(MASK4_4) ) {
					case 0x18:// �������� ��������/* �Z���^���܂�����*/
	                    handle( 0 );

	                break;

	                case 0x1c://�������� �������� /* �����ɍ���聨�E�֔��Ȃ� */
	                    handle( 2 );
	                    motor( 100 ,100 );

					break;

	                case 0x0c://�������� ��������
	                    handle( 4 );
	               		motor( 100 ,100 );

	                break;

	                case 0x0e://�������� ��������
	                    handle( 7 );
	       				motor( 100,100 );

	                break;

	                case 0x06://�������� ��������/* ��������聨�E�֏��Ȃ� */
	                    handle( 10 );
	                    motor( 100 ,100 );

	                break;

	                case 0x38://�������� ��������/* �����ɉE��聨���֔��Ȃ� */
	                    handle( -2 );
	                    motor( 100 ,100 );

	                break;

	                case 0x30://�������� ��������
	                    handle( -4 );
	                    motor( 100 ,100 );

	                break;

	                case 0x70://�������� ��������
	                    handle( -7 );
	                    motor( 100 ,100 );
						
	                break;

	                case 0x60://�������� ��������/* �����E��聨���֏��Ȃ� */
	                    handle( -10 );
	                    motor( 100 ,100 );

	                break;
				}
			break;
			
				
			case 120:
		        /* ���N�����N�Ɣ��f�����N�����N�N���A������ */
				if( sensor_inp(MASK4_0)==0xf0 || sensor_inp(MASK4_0)==0xe0) {   /* ���������@��������  ���������@��������*/
		            handle_on();//�T�[�{�d��on
					handle( -50 );//�傫�߂ɂӂ�  
		            motor( -50, 100);
		            pattern = 150;//���N�����N����
		            cnt1 = 0; 
		            break;
				}
		        /* �E�N�����N�Ɣ��f���E�N�����N�N���A������ */
				if( sensor_inp(MASK0_4)==0x0f || sensor_inp(MASK0_4)==0x07){	/* ���������@��������  ���������@��������*/
					handle_on();//�T�[�{�d��on
					handle( 50 );//�傫�߂ɂӂ�  
		            motor( 100, -50);
		            pattern = 250;//�E�N�����N����
		            cnt1 = 0;					
		            break;
				}


				if (sp >= crankTargetSpeed+10) {            // �N�����N�O�̑��x���䁫
					motor( -60, -60);
				}
				else if(sp >= crankTargetSpeed+5){
					motor( -40, -40 );
				}
				else if(sp >= crankTargetSpeed+2){
					motor( -10, -10 );
				}
				
				else if(sp <= crankTargetSpeed-2){         // ���������������΍�
					motor( 30, 30 );
				}
				else if(sp <= crankTargetSpeed-6){
					motor( 50, 50 );
				}
				else{                                      // �i�����̑��x
					motor( 20, 20);
				}
				
		 		switch( sensor_inp(MASK4_4) ) {		//�N���X���C������n�[�t���C�����o�܂ł̃g���[�X
					case 0x18://�������� ��������
						handle( 0 );
					break;

					case 0x1c://�������� ��������
						handle( 2 );
					break;

					case 0x0c://�������� ��������
						handle( 4 );
					break;

					case 0x0e://�������� ��������
						handle( 7 );
					break;

					case 0x06://�������� ��������
						handle( 9 );
					break;

					case 0x07://�������� ��������
						handle( 11 );
					break;

					case 0x38://�������� ��������
						handle( -2 );
					break;

					case 0x30://�������� ��������
						handle( -4 );
					break;

					case 0x70://�������� ��������
						handle( -7 );
					break;

					case 0x60://�������� ��������
						handle( -9 );
					break;

					case 0xe0://�������� ��������
						handle( -11 );
					break;
				}
			break;

            case 150:	//���N�����N
				handle( -50 );//�傫�߂ɂӂ�  
		        motor( -50, 100);
                if(sensor_inp(MASK4_4)==0x00 ){//�������� ��������
					pattern = 155;//�ʏ�N�����N����
					cnt1=0;		//�^�C�}�[�N���A
                }
            break;
			
			
			case 155:
				handle_on();//�T�[�{�d��on
				handle( -50 );//�傫�߂ɂӂ�
				motor( -50, 100);

					
				if( sensor_inp(MASK0_1) == 0x01 ){//�������� ��������  (�O���C�����o)
					motor( -90 , 100 );
					pattern = 160;
					cnt1 = 0;
					break;
				}   		
				if( sensor_inp(MASK1_0) == 0x80 ){//�������� ��������  (�����C�����o)
					motor( -90 , 100 ); 
					handle_free();
					pattern = 170; 
					cnt1 = 0;
					break;
				}
			break;
			
			
			case 160:
				if( cnt1 > 90 ){	//70ms�� 152�ƍ��킹��150ms���x
					// �������� ��������  �������� �������� �������� ��������
					if( sensor_inp(MASK4_4) == 0x81  || sensor_inp(MASK4_4) == 0xc1 ||  sensor_inp(MASK4_4) == 0x83 || cnt1 > 250 ){
						handle_on();//�T�[�{�d��on
						handle( -50 );
						motor( -30 , 100 );
						pattern = 170;  
						cnt1 = 0;
						break;
					}
				}
			break;

					
			case 170:
				if( sensor_inp(MASK4_4) == 0xe0 || sensor_inp(MASK4_4) == 0x60 ) {// �������� �������� �������� ��������
					handle_on();//�T�[�{�d��on
					handle( 0 );
					pattern = 11;  //�ʏ한�A
					cnt1 = 0;
					break;
				}
			break;
			

            case 250:	//�E�N�����N
				handle( 50 );//�傫�߂ɂӂ�  
		        motor( 100, -50);
                if(sensor_inp(MASK4_4)==0x00 ){//�������� ��������
					pattern = 255;//�ʏ�N�����N����
					cnt1=0;		//�^�C�}�[�N���A
                }
            break;
			
			case 255:
				handle_on();//�T�[�{�d��on
				handle( 50 );//�傫�߂ɂӂ�
				motor( 100,  -50);

				if( sensor_inp(MASK1_0) == 0x80 ){//�������� ��������  (�O���C�����o)
					motor( 100 , -90 ); 
					pattern = 260; 
					cnt1 = 0;
					break;
				}
				if( sensor_inp(MASK0_1) == 0x01 ){//�������� ��������  (�����C�����o)
					motor( 100 , -90 );
					handle_free();
					pattern = 270;
					cnt1 = 0;
					break;
				}
			break;
			
			case 260:
				if( cnt1 > 90 ){	//70ms�� 152�ƍ��킹��150ms���x
					// �������� ��������  �������� �������� �������� ��������
					if( sensor_inp(MASK4_4) == 0x81  || sensor_inp(MASK4_4) == 0xc1 ||  sensor_inp(MASK4_4) == 0x83 || cnt1 > 250 ){
						handle_on();//�T�[�{�d��on
						handle( 50 );
						motor( 100 , -30 );  
						pattern = 270;  
						cnt1 = 0;
						break;
					}
				}
			break;
			
			case 270:
				if( sensor_inp(MASK4_4) == 0x07 || sensor_inp(MASK4_4) == 0x06 ) {// �������� �������� �������� ��������
					handle_on();//�T�[�{�d��on
					handle( 0 );
					pattern = 11;  //�ʏ한�A
					cnt1 = 0;
					break;
				}
			break;


////////////���[���`�F��///////////////
			case 300:
				if( check_crossline() ) {       /* �N���X���C���`�F�b�N */
                    pattern = 20;
                    break;
                }
				if(cnt1>30){
					pattern=305;
				}
			break;
			
			
            //���[���`�F���W�����i�����[���j
            case 305:    /* �P�{�ڂ̍��n�[�t���C�����o���̏��� */
                led_out( 0x1 );
				motor(80,100);
                handle( -5 );
                pattern = 310;
                cnt1 = 0;
            break;

            case 310:   /* ���n�[�t���C����̃g���[�X�A���[���`�F���W(in�R�[�X) */
                switch( sensor_inp(MASK4_4) ) {
                    case 0x06://�������� ���������@�܂�����
                        handle( 0 );
                    break;

                    case 0x07://�������� ��������
                        handle( 0 );
                    break;

                    case 0x03://�������� ��������   �E��聨���Ȃ�
						handle( 1 );
                    break;

                    case 0x83://�������� ��������   �E��聨���Ȃ�
						handle( 1 );
                    break;

                    case 0x81://�������� ��������   �E��聨���Ȃ�
						handle( 2 );
                    break;

                    case 0xc1://�������� ��������   �E��聨���Ȃ�
                        handle( 3 );
                    break;

                    case 0x01://�������� ��������   �E��聨���Ȃ�   (�ǉ�)
						handle( 3 );
                    break;

                    case 0xc0://�������� ��������   �E��聨���Ȃ�
						handle( 7 );
                    break;

                    case 0x0e://�������� ��������
                        handle( 0 );
                    break;

                    case 0x0c://�������� ��������   ����聨�E�Ȃ�
						handle( -1 );
                    break;

                    case 0x1c://�������� ��������   ����聨�E�Ȃ�
						handle( -1 );
                    break;

                    case 0x18://�������� ��������   ����聨�E�Ȃ�
						handle( -2 );
                    break;

                    case 0x38://�������� ��������   ����聨�E�Ȃ�
						handle( -4 );
                    break;

                    case 0x30://�������� ��������   ����聨�E�Ȃ�
						handle( -4 );
                    break;

                    case 0x70://�������� ��������   ����聨�E�Ȃ�
						handle( -7 );
                    break;
                }
				
				//���x����
				if(sp >= harfTargetSpeed+10){
					motor(-10,-10);
				}
				else if(sp >= harfTargetSpeed+3){
					motor(0,0);
				}
				
				else if(sp <= harfTargetSpeed-3){
					motor(60,60);
				}
				else if(sp <= harfTargetSpeed-10){
					motor(80,80);
				}
				
				else{
					motor(50,50);
				}
				
			    if( sensor_inp(MASK4_4) == 0x00 ) {
					handle_on();//�T�[�{�d��on
                    handle( -37 );	 //�傫���U��
                    motor( 30 , 30 );
                    pattern = 316;
                    cnt1 = 0;
					break;
                }
            break;

            case 316:
                if(cnt1 > 10){
					handle_on();//�T�[�{�d��on
                    handle( -30 );	 //-18�ł́A���肬���ւ��ʂ�
                    motor( -10 ,80 );
                    pattern = 320;
                    cnt1 = 0;
                }
            break;

                
            case 320:  /* �����[���`�F���W��̖߂�����1(�C���R�[�X�j */
                if(( sensor_inp(MASK3_0) == 0x80) || ( sensor_inp(MASK3_0) == 0xc0)) {	//�������� ��������  �������� ��������
                    handle_on();//�T�[�{�d��on
					handle( 45 );
                    motor( 100 ,-90 );
                    pattern = 325;
                    cnt1 = 0;
                }
            break;

            case 325:  /* �����[���`�F���W��߂�����2(�C���R�[�X�j */
                if( cnt1 > 20 ) {
					handle_on();//�T�[�{�d��on
                    handle( 45 );
                    motor( 100 ,-90 );
                    pattern = 330;
                    cnt1 = 0;
                }
            break;

            case 330: /* �����[���`�F���W���肵����ʏ�g���[�X�� (�C���R�[�X�j*/
                if( cnt1 > 60 ) { 
					if( (sensor_inp(MASK3_3)== 0x06) || (sensor_inp(MASK3_3)== 0x02)|| (sensor_inp(MASK_left)== 0x04)  ||  (sensor_inp(MASK4_4)== 0x38) || (sensor_inp(MASK4_4)== 0x18) || (sensor_inp(MASK4_4)== 0x1c) || (sensor_inp(MASK4_4)== 0x30)|| (sensor_inp(MASK4_4)== 0x70) ||(sensor_inp(MASK4_4)== 0x60)) {
	                    led_out( 0x0 );
	                    pattern = 11;
	                }
				}
            break;
			
			case 400:
			if( check_crossline() ) {       /* ��Ȃ������N���X���C���`�F�b�N */
                pattern = 20;
                break;
            }
			if(cnt1>30){
				pattern = 405;
			}
			break;
			
            //���[���`�F���W�����i�E���[���j
            case 405:/* �P�{�ڂ̉E�n�[�t���C�����o���̏��� */
                led_out( 0x2 );
				motor(100,80);
                handle( 5 );
                pattern = 410;
                cnt1 = 0;
            break;

            case 410: /* �E�n�[�t���C����̃g���[�X�A���[���`�F���W */
                switch( sensor_inp(MASK4_4) ) {
                    case 0x60://�������� ���������@�܂�����
						handle( 0 );
                    break;

                    case 0xe0://�������� ��������
						handle( 0 );
                    break;

                    case 0xc0://�������� ��������   ����聨�E�Ȃ�
						handle( -1 );
                    break;

                    case 0xc1://�������� ��������   ����聨�E�Ȃ�
                        handle( -1 );
                    break;

                    case 0x81://�������� ��������   ����聨�E�Ȃ�
						handle( -2 );
                    break;

                    case 0x83://�������� ��������   ����聨�E�Ȃ�   (�ǉ�)
						handle( -3 );
                    break;

                    case 0x03://�������� ��������   ����聨�E�Ȃ�
						handle( -3 );
                    break;

                    case 0x07://�������� ��������   ����聨�E�Ȃ��@�i�ǉ��j
						handle( -7 );
                    break;

                    case 0x70://�������� ��������
						handle( 0 );
                    break;

                    case 0x30://�������� ��������   �E��聨���Ȃ�
						handle( 1 );
                    break;

                    case 0x38://�������� ��������   �E��聨���Ȃ�
						handle( 1 );
                    break;

                    case 0x18://�������� ��������   �E��聨���Ȃ�
						handle( 2 );
                    break;

                    case 0x1c://�������� ��������   �E��聨���Ȃ�
						handle( 4 );
                    break;

                    case 0x0c://�������� ��������   �E��聨���Ȃ�
						handle( 4 );
                    break;

                    case 0x0e://�������� ��������   �E��聨���Ȃ�
						handle( 7 );
                    break;
                }
				
				//���x����
				if(sp >= harfTargetSpeed+10){
					motor(-10,-10);
				}
				else if(sp >= harfTargetSpeed+3){
					motor(0,0);
				}
				
				else if(sp <= harfTargetSpeed-3){
					motor(60,60);
				}
				else if(sp <= harfTargetSpeed-10){
					motor(80,80);
				}
				
				else{
					motor(50,50);
				}
				
			    if( sensor_inp(MASK4_4) == 0x00 ) {
                    handle_on();//�T�[�{�d��on
					handle( 37 );	 //�傫���U��
                    motor( 30, 30 );
                    pattern = 416;
                    cnt1 = 0;
					break;
                }
            break;

            case 416:
                if(cnt1 > 10){// cnt1 >80
                    handle_on();//�T�[�{�d��on
					handle( 30 );	 //18�ł́A���肬���ւ��ʂ�
                    motor( 80 ,-10 );
                    pattern = 420;
                    cnt1 = 0;

                }
            break;
			
            case 420:  /* �E���[���`�F���W��̖߂�����1 */
                if(( sensor_inp(MASK0_3) == 0x01) || ( sensor_inp(MASK0_3) == 0x03)){ 	//�������� ��������  �������� ��������
                    handle_on();//�T�[�{�d��on
					handle( -45 );
                    motor( -90 ,100 );
                    pattern = 425;
                    cnt1 = 0;
                }
            break;

            case 425:  /* �E���[���`�F���W��߂�����2 */
                if( cnt1 > 20 ) {
					handle_on();//�T�[�{�d��on
                    handle( -45 );
                    motor( -90 ,100 );
                    pattern = 430;
                    cnt1 = 0;
                }
            break;

            case 430: /* �E���[���`�F���W���肵����ʏ�g���[�X�� (�C���R�[�X�j*/
                if( cnt1 > 60 ) { 
					if( (sensor_inp(MASK3_3)== 0x60) || (sensor_inp(MASK3_3)== 0x40) || (sensor_inp(MASK_right)== 0x20) ||    (sensor_inp(MASK4_4)== 0x38) || (sensor_inp(MASK4_4)== 0x18) || (sensor_inp(MASK4_4)== 0x1c) || (sensor_inp(MASK4_4)== 0x0c)|| (sensor_inp(MASK4_4)== 0x0e)||(sensor_inp(MASK4_4)== 0x06)) {
	                    led_out( 0x0 );
	                    pattern = 11;
	                }
				}
            break;
			

			case 102:   /* �f�o�b�O�p */
                motor( 0, 0 );
            break;

            case 101:   /* �E�ւ����ۂ̎�����~������́A�K�����̏������s���Ă������� */
                handle_free();
                motor( 0, 0 );
            break;

            case 100:/* �m�F�p */
                handle_free();
                motor( 0 ,0 );
                timer(500);
                while(pushsw_get()==0){
                    if( cnt1 < 50 ) {              /* LED�_�ŏ��� */
                        led_out( 0x1 );
                    }
                    else if( cnt1 < 100 ) {
                        led_out( 0x0 );
                    }
                    else {
                        cnt1 = 0;
                    }
                }
                if(pushsw_get()==1){
                    for(i=log_ct+1;i<500;i++){
                        printf( "%4d     ",i-log_ct);
                        printf( "d_0=%03d d_1=%03d d_2=%03d d_3=%03d d_4=%03d d_5=%03d fin \n",
                        log_se[i],log_pattern[i],log_a[i],log_b[i],log_c[i],log_d[i]);
                        timer(10);
                    }
                    for(i=0;i<=log_ct;i++){
                        printf( "%4d     ",i+(500-log_ct));
                        printf( "d_0=%03d d_1=%03d d_2=%03d d_3=%03d d_4=%03d d_5=%03d fin \n",
                        log_se[i],log_pattern[i],log_a[i],log_b[i],log_c[i],log_d[i]);
                        timer(10);
                    }
                    while(1);
                }
            break;
            
            case 200:
                motor(0,0);
                switch( sensor_inp(MASK4_4) ) {
                    case 0x00:
                        handle( 0 );
                        motor( 0,0 );
                    break ;

                    case 0x18://�������� ��������
                        handle( 0 );
                    break;

                    //�����@�E�Ȃ�����
                    case 0x1c://�������� ��������
                        handle( 2 );
                    break;

                    case 0x0c://�������� ��������
                        handle( 4 );
                    break;

                    case 0x0e://�������� ��������
                        handle( 8);
                    break;

                    case 0x06://�������� ��������
                        handle( 10 );
                    break;

                    case 0x07://�������� ��������
                        handle( 12 );
                    break;

                    case 0x03://�������� ��������
                        handle( 14 );
                    break;

                    //�E���@���Ȃ�����
                    case 0x38: //�������� ��������
                        handle( -2 );
                    break;

                    case 0x30: //�������� ��������
                        handle( -4 );
                    break;

                    case 0x70://�������� ��������
                        handle( -8 );
                    break;

                    case 0x60://�������� ��������
                        handle( -10 );
                    break;

                    case 0xc0://�������� ��������
                        handle( -12 );
                    break;

                    case 0xe0://�������� ��������
                        handle( -14 );
                    break;

                    default:

                    break;
                }	
            break;
        
            default: /* �ǂ�ł��Ȃ��ꍇ�͑ҋ@��Ԃɖ߂� */
                pattern = 0;
            break;
        }
    }
}

/************************************************************************/
/* R8C/38A �X�y�V�����t�@���N�V�������W�X�^(SFR)�̏�����                */
/************************************************************************/
void init( void ){
    int i;

    /* �N���b�N��XIN�N���b�N(20MHz)�ɕύX */
    prc0  = 1;                          /* �v���e�N�g����               */
    cm13  = 1;                          /* P4_6,P4_7��XIN-XOUT�[�q�ɂ���*/
    cm05  = 0;                          /* XIN�N���b�N���U              */
    for(i=0; i<50; i++ );               /* ���肷��܂ŏ����҂�(��10ms) */
    ocd2  = 0;                          /* �V�X�e���N���b�N��XIN�ɂ���  */
    prc0  = 0;                          /* �v���e�N�gON                 */

    /* �|�[�g�̓��o�͐ݒ� */
    prc2 = 1;                           /* PD0�̃v���e�N�g����          */
    pd0 = 0x00;                         /* 7-0:�Z���T���Ver.5          */
    pd1 = 0xd0;                         /* 5:RXD0 4:TXD0 3-0:DIP SW     */
    p2  = 0xc0;
    pd2 = 0xfe;                         /* 7-0:���[�^�h���C�u���Ver.5  */

    pd3 = 0xfe;                         /*  0:�G���R�[�_(���́j */
 //   pd3 = 0xff;                         /*                              */

    p4  = 0x20;                         /* P4_5��LED:�����͓_��         */
    pd4 = 0xb8;                         /* 7:XOUT 6:XIN 5:LED 2:VREF    */
    pd5 = 0x7f;                         /* 7-0:LCD/microSD���          */
    pd6 = 0xef;                         /* 4-0:LCD/microSD���          */
	
    pd7 = 0x7f;//0xff                         /*                              */
	
    pd8 = 0xff;                         /*                              */
    pd9 = 0x3f;                         /*                              */
    pur0 = 0x04;                        /* P1_3�`P1_0�̃v���A�b�vON     */

    /* �^�C�}RB�̐ݒ� */
    /* ���荞�ݎ��� = 1 / 20[MHz]   * (TRBPRE+1) * (TRBPR+1)
                    = 1 / (20*10^6) * 200        * 100
                    = 0.001[s] = 1[ms]
    */
    trbmr  = 0x00;                      /* ���샂�[�h�A������ݒ�       */
    trbpre = 200-1;                     /* �v���X�P�[�����W�X�^         */
    trbpr  = 100-1;                     /* �v���C�}�����W�X�^           */
    trbic  = 0x07;                      /* ���荞�ݗD�惌�x���ݒ�       */
    trbcr  = 0x01;                      /* �J�E���g�J�n                 */



   /* �^�C�}RG �^�C�}���[�h(���G�b�W�ŃJ�E���g)�̐ݒ�@�G���R�[�_�̃J�E���^�ݒ� */

   timsr = 0x40; /* TRGCLKA�[�q P3_0�Ɋ��蓖�Ă� */
   trgcr = 0x15; /* TRGCLKA�[�q�̗��G�b�W�ŃJ�E���g*/
   trgmr = 0x80; /* TRG�̃J�E���g�J�n */


    /* �^�C�}RD ���Z�b�g����PWM���[�h�̐ݒ�*/
    /* PWM���� = 1 / 20[MHz]   * �J�E���g�\�[�X * (TRDGRA0+1)
               = 1 / (20*10^6) * 8              * 40000
               = 0.016[s] = 16[ms]
    */
    trdpsr0 = 0x08;                     /* TRDIOB0,C0,D0�[�q�ݒ�        */
    trdpsr1 = 0x05;                     /* TRDIOA1,B1,C1,D1�[�q�ݒ�     */
    trdmr   = 0xf0;                     /* �o�b�t�@���W�X�^�ݒ�         */
    trdfcr  = 0x01;                     /* ���Z�b�g����PWM���[�h�ɐݒ�  */
    trdcr0  = 0x23;                     /* �\�[�X�J�E���g�̑I��:f8      */
    trdgra0 = trdgrc0 = PWM_CYCLE;      /* ����                         */
    trdgrb0 = trdgrd0 = 0;              /* P2_2�[�q��ON���ݒ�           */
    trdgra1 = trdgrc1 = 0;              /* P2_4�[�q��ON���ݒ�           */
    trdgrb1 = trdgrd1 = SERVO_CENTER_W; /* P2_5�[�q��ON���ݒ�           */
    trdoer1 = 0xcd;                     /* �o�͒[�q�̑I��               */
    trdstr  = 0x0d;                     /* TRD0�J�E���g�J�n             */
}

/************************************************************************/
/* �^�C�}RB ���荞�ݏ���                                                */
/************************************************************************/
#pragma interrupt intTRB(vect=24)
void intTRB( void ){
   static long cnt_time=0;
   static long cnt_time2=0;
   int t,b;
   static int uEncoderTotal;


   cnt0++;
   cnt1++;
   cnt_kasokudo++;


    /* �E�֎��̒�~�����i�f�W�^���Z���T�j */
    b = sensor_inp( MASK3_3 );
    if( b == 0xe7 ) {
        check_sen_cnt++;
        if( check_sen_cnt >= 100 ) {
            pattern = 101;
        }
    }
    else {
        check_sen_cnt = 0;
    }

    //�X�g���[�g�J�E���g
    if(angle_log<10 && angle_log>-10){//�X�g���[�g���̃J�E���g
   		streat_ct++;
    }
    else{
   		streat_ct=0;
    }
    if(streat_ct>200){	//�X�g���[�g�A�����s���i400ms��̏����j
	   streat_flag=1;	//�X�g���[�g�A�����s�m�F�t���O 1:ON 0:OFF  400ms=1m���x
	   break_count=0;	//�u���[�L�̐��J�E���g�i�X�g���[�g400ms��N���A�j
	   corner_flag=0;	//�R�[�i�[�t���O�N���A
    }
	
	//�����x�v�Z
	if(cnt_kasokudo > 50){
		kasokudo = iEncoderTotal - old_kyori;
		old_kyori = iEncoderTotal;
		cnt_kasokudo = 0;
	}
	
   /*                  */
   /*10ms�Ԋu�̏���       */
   /*���s�����E���s���x�擾*/
   /*                  */
    if(flag_st==1){//�X�^�[�g��@�@flag_st:true(�X�^�[�g��)
        cnt_time++;
        if((cnt_time%10)==0){      //10ms��
		  	//�G���R�[�_�擾�֌W
	  		t=trg;							//�}�p�ϐ��ɑ��
			iEncoder = t - uEncoderBuff;		//�S�̂���ߋ��̒l������
			iEncoderTotal += iEncoder;	//�g�[�^���̂������L�^
			uEncoderBuff = t;		//t�L���p


			/* 4�_�̑��x�̈ړ����όv�Zsp_buf[4]*/       
			sp_sum =(sp_sum + iEncoder) -sp_buf[sp_count];
			sp_buf[sp_count]=iEncoder;
			sp_count++;
			sp_count = sp_count & 0x03;
			sp = sp_sum >> 2 ;

            cnt_time=0;

            log_se[log_ct]= ((~p0) & 0xfe) | !p7_7;			
            log_pattern[log_ct]=pattern;
            log_a[log_ct]=angle_log;  //cnt_kura
            log_b[log_ct]=accele_l_log;//kuratime;//accele_r_log  log_shinsei  log_cksp  out_log
            log_c[log_ct]=accele_r_log; //accele_l_log
            log_d[log_ct]=sp;//sp;  //sp streat_flag cnt_kura  break_count
            if(log_ct<500)log_ct++;//cnt_kura
            else log_ct=0;
        }
    }
  // �v�b�V��SW���쎞	�X�^�[�g500ms�o�ߌ�
    if(pushsw_get()==1 && cnt_syu > 500 ){
        pattern=100;
        flag_st=0;//���O�擾��~
    }
   /* �I���^�C�}�[ �X�^�[�g����@�v�b�V��SW����h�~�i�����ƃX�^�[�g�Ɠ�����pattern=100�j*/
    if(syu_flag==1){
        if(cnt_syu<1000)cnt_syu++;
    }

}
/************************************************************************/
/* �^�C�}�{��                                                           */
/* �����@ �^�C�}�l 1=1ms                                                */
/************************************************************************/
void timer( unsigned long timer_set ){
    cnt0 = 0;
    while( cnt0 < timer_set );
}
/************************************************************************/
/* �Z���T��Ԍ��o                                                       */
/* �����@ �}�X�N�l                                                      */
/* �߂�l �Z���T�l                                                      */
/************************************************************************/
unsigned char sensor_inp( unsigned char mask )
{
    unsigned char sensor;

    sensor  = (~p0) & 0xfe;

    sensor  |= !p7_7;


    sensor &= mask;

    return sensor;
}



/************************************************************************/
/* �N���X���C�����o����                                                 */
/* �߂�l 0:�N���X���C���Ȃ� 1:����                                     */
/************************************************************************/
int check_crossline( void )
{
    unsigned char b;
    int ret;

    ret = 0;
    b = sensor_inp(MASK3_3);
    if( b==0xe7 ) {
        ret = 1;
    }
    return ret;
}
/************************************************************************/
/* �n�[�t���C����̎��ԃJ�E���g  
/* �����@ �t���O                                             */
/* �߂�l �J�E���g                                                 */
/************************************************************************/
int ck_timer(int sp_cnt )
{
	int ret;
		ret=0;
	if(sp_cnt>=1){
		ret++;
	}
	return ret;
}
/************************************************************************/
/* �E�n�[�t���C�����o����                                               */
/* �߂�l 0:�Ȃ� 1:����                                                 */
/************************************************************************/
int check_rightline( void )
{
    unsigned char b;
    int ret;

    ret = 0;
    b = sensor_inp(MASK4_4);
    if( b==0x0f || b==0x1f || b==0x3f || b==0x7f) { /* �I�ǉ��E�ύX�I   */
        ret = 1;
    }
    return ret;
}

/************************************************************************/
/* ���n�[�t���C�����o����                                               */
/* �߂�l 0:�Ȃ� 1:����                                                 */
/************************************************************************/
int check_leftline( void )
{
    unsigned char b;
    int ret;

    ret = 0;
    b = sensor_inp(MASK4_4);
    if( b==0xf0 || b==0xf8 || b==0xfc || b==0xfe) { /* �I�ǉ��E�ύX�I   */
       ret = 1;
    }
    return ret;
}
/************************************************************************/
/* �f�B�b�v�X�C�b�`�l�ǂݍ��݂P(�X�s�[�h����)                           */
/* �߂�l �X�C�b�`�l 0�`15                                              */
/************************************************************************/
unsigned char dipsw_gets( void )
{
    unsigned char sw1;

    sw1 = p1 & 0x07;                     /* P1_3�`P1_1�ǂݍ���           */

    return  sw1;
}
/************************************************************************/
/* �f�B�b�v�X�C�b�`�l�ǂݍ���2(in-out����)                              */
/* �߂�l �X�C�b�`�l 0�`15                                              */
/************************************************************************/
unsigned char dipsw_get( void )
{
    unsigned char sw2;

    sw2 = p1 & 0x08;                     /* P1_3�`P1_0�ǂݍ���           */

    return  sw2;
}

/************************************************************************/
/* �v�b�V���X�C�b�`�l�ǂݍ���                                           */
/* �߂�l �X�C�b�`�l ON:1 OFF:0                                         */
/************************************************************************/
unsigned char pushsw_get( void )
{
    unsigned char sw;

    sw  = ~p2;                          /* �X�C�b�`�̂���|�[�g�ǂݍ��� */
    sw &= 0x01;

    return  sw;
}

/************************************************************************/
/* �X�^�[�g�o�[���o�Z���T�ǂݍ���                                       */
/* �߂�l �Z���T�l ON(�o�[����):1 OFF(�Ȃ�):0                           */
/************************************************************************/
unsigned char startbar_get( void )
{
    unsigned char b;

    b  = !p7_7;//~p0;                           /* �X�^�[�g�o�[�M���ǂݍ���     */
    //b &= 0x01;

    return  b;
}

/************************************************************************/
/* LED����                                                              */
/* �����@�X�C�b�`�l LED0:bit0 LED1:bit1  "0":���� "1":�_��              */
/* ��)0x3��LED1:ON LED0:ON  0x2��LED1:ON LED0:OFF                       */
/************************************************************************/
void led_out( unsigned char led )
{
    unsigned char data;

    led = ~led;
    led <<= 6;
    data = p2 & 0x3f;
    p2 = data | led;
}

/************************************************************************/
/* ���[�^���x����                                                       */
/* �����@ �����[�^:-100�`100�A�E���[�^:-100�`100                        */
/*        0�Œ�~�A100�Ő��]100%�A-100�ŋt�]100%                        */
/* �߂�l �Ȃ�                                                          */
/************************************************************************/
void motor( int accele_l, int accele_r )
{
   //int    sw_data;

   accele_r_log=accele_r;
   accele_l_log=accele_l;
    
   //sw_data=(dipsw_gets()&0x03)*5+85;
   //accele_l = accele_l * sw_data / 100;
   //accele_r = accele_r * sw_data / 100;
	
   leftMotorBuff  = accele_l;          /* �o�b�t�@�ɕۑ�               */
   rightMotorBuff = accele_r;          /* �o�b�t�@�ɕۑ�               */
   

       /* �����[�^���� */
        if( accele_l >= 0 ) {
            p2 &= 0xfd;
            trdgrd0 = (long)( PWM_CYCLE - 1 ) * accele_l / 100;
        }
		else {
	        p2 |= 0x02;
	        trdgrd0 = (long)( PWM_CYCLE - 1 ) * ( -accele_l ) / 100;
        }

       /* �E���[�^���� */
        if( accele_r >= 0 ) {
            p2 &= 0xf7;
            trdgrc1 = (long)( PWM_CYCLE - 1 ) * accele_r / 100;
        }
	    else {
            p2 |= 0x08;
            trdgrc1 = (long)( PWM_CYCLE - 1 ) * ( -accele_r ) / 100;
        }


}

/************************************************************************/
/* �T�[�{�n���h������                                                   */
/* �����@ �T�[�{����p�x�F-90�`90                                       */
/*        -90�ō���90�x�A0�ł܂������A90�ŉE��90�x��]                  */
/************************************************************************/
void handle( int angle )
{
    handleBuff = angle;                 /* �o�b�t�@�ɕۑ�               */
    angle_buff = angle;                 /* ���݂̊p�x�ۑ�               */
   angle_log=angle;
    /* �T�[�{�����E�t�ɓ����ꍇ�́A�u-�v���u+�v�ɑւ��Ă������� */
    trdgrd1 = SERVO_CENTER_W + angle * HANDLE_STEP;
}

/************************************************************************/
/* �T�[�{�n���h������@�t���[                                           */
/* �����@ �Ȃ�                                                          */
/*                                                                      */
/************************************************************************/
void handle_free( void )
{
    trdoer1 |= 0x20;                    /* �T�[�{�pPWM(P2_5)��OFF�ɐݒ� */
}
/************************************************************************/
/* �T�[�{�n���h������@���䃂�[�h                                           */
/* �����@ �Ȃ�                                                          */
/*                                                                      */
/************************************************************************/
void handle_on( void )
{
    trdoer1 &= 0xdf;                    /* �T�[�{�pPWM(P2_5)��ON�ɐݒ� **0* **** */
}
/************************************************************************/
/* �O�ւ�PWM����A���ւ�PWM������o���@�n���h���p�x�͌��݂̒l���g�p     */
/* �����@ �O��PWM                                                       */
/* �߂�l ����PWM                                                       */
/************************************************************************/
int diff( int pwm )
{
    int ret;

    if( pwm >= 0 ) {
        /* PWM�l�����̐��Ȃ� */
        if( angle_buff < 0 ) {
            angle_buff = -angle_buff;
        }
        ret = revolution_difference[angle_buff] * pwm / 100;
    } else {
        /* PWM�l�����̐��Ȃ� */
        ret = pwm;                      /* ���̂܂ܕԂ�                 */
    }
    return ret;
}
/************************************************************************/
/* ��~��Ԑ���                                                         */
/* �����@ �����[�^�̏�ԁFBRAKE or FREE                                 */
/*     �@ �E���[�^�̏�ԁFBRAKE or FREE                                 */
/************************************************************************/
void motor_mode( int mode_l, int mode_r )
{
    if( mode_l ) {                      /* �����[�^�`�F�b�N             */
        p8_1 = 1;                       /* �t���[����                   */
    } else {
        p8_1 = 0;                       /* �u���[�L����                 */
    }
    if( mode_r ) {                      /* �E���[�^�`�F�b�N             */
        p8_0 = 1;                       /* �t���[����                   */
    } else {
        p8_0 = 0;                       /* �u���[�L����                 */
    }
}
/************************************************************************/
/* end of file                                                          */
/************************************************************************/
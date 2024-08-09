/****************************************************************************/
/* 対象マイコン 	R8C/38A													*/
/* ﾌｧｲﾙ内容     2023_BASIC_OKAKOプログラム									*/
/* Date         2019.10.01                           　                     */
/* Copyright    ジャパンマイコンカーラリー実行委員会						*/
//前輪駆動用
//エンコーダ：72パルス（アップダウンカウント）
/****************************************************************************/

/****************************************************************************/
/* このプログラムは、下記基板に対応しています。								*/
/*・RY_R8C38ボード															*/
/*・モータドライブ基板Ver.5													*/
/*・センサ基板Ver.5															*/
/****************************************************************************/

/*======================================*/
/* インクルード 						*/
/*======================================*/
#include <stdio.h>
#include <stdlib.h>

#include "sfr_r838a.h"                  /* R8C/38A SFRの定義ファイル    */
#include "printf_lib.h"                 /* printf使用ライブラリ         */

/*======================================*/
/* シンボル定義							*/
/*======================================*/
/* 定数設定 */

#define PWM_CYCLE      29999 //27500 // 12ms:     10ms:25000 1mm:2500  (16ms)
// 10000                                        /* モータPWMの周期     2ms */

/* JR 539*/
#define SERVO_CENTER_W   3377 //サーボセンター　下：左 上：右
		  
#define HANDLE_STEP     22    /* 1゜分の値   */

/* マスク値設定 ×：マスクあり(無効)  ○：マスク無し(有効)       	*/
#define MASK1_0         0X80        		/* ○××××××× 	*/
#define MASK0_1         0x01        		/* ×××××××○ 	*/
#define MASK1_1         0x18        		/* ×××○○××× 	*/
#define MASK1         	0x81        		/* ○××××××○ 	*/
#define MASK_right      0x20        		/* ××○××××× 	*/
#define MASK_left      	0x04        		/* ×××××○×× 	*/
#define MASK2_0         0x30        		/* ××○○×××× 	*/
#define MASK0_2         0x0c        		/* ××××○○×× 	*/
#define MASK2_4         0x3f        		/* ××○○○○○○ 	*/
#define MASK4_2         0xfc        		/* ○○○○○○×× 	*/
#define MASK3_3         0xe7        		/* ○○○××○○○ 	*/
#define MASK0_3         0x07        		/* ×××××○○○ 	*/
#define MASK3_0         0xe0        		/* ○○○××××× 	*/
#define MASK4_0         0xf0        		/* ○○○○×××× 	*/
#define MASK0_4         0x0f        		/* ××××○○○○ 	*/
#define MASK4_4         0xff        		/* ○○○○○○○○ 	*/

#define LEFT 1	//コーナーブレーキ時　ハンドリング方向記憶
#define RIGHT 2 //コーナーブレーキ時　ハンドリング方向記憶

#define TargetSpeed 90

/*======================================*/
/* プロトタイプ宣言                     */
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
/* グローバル変数の宣言                 */
/*======================================*/
unsigned long   cnt0;                   /* timer関数用                  */
unsigned long   cnt1;                   /* main内で使用                 */
unsigned long   cnt_kasokudo;           /* 加速度計算時に使用           */
unsigned long   cnt_syu;                /* 終了タイマー                 */
unsigned long   check_sen_cnt;          /* タイマ用                     */
unsigned int    pattern;                /* パターン番号                 */
int             servo_center;           /* サーボセンタ値               */
int             syu_flag = 0;           /* 終了フラグ                   */
int 			corner_flag=0;			/* 切り替えし用フラグ      LEFT or RIGHT  */
int             brake_time;	      	    //直線走行後ブレーキ時間(割り込み内でインクリメント)
int				brake_power_flag = 0;   /* 坂下のコーナーブレーキ対策   */

/* 加速度計算関連 */
unsigned long   cnt_kasokudo;           /* 加速度計算時に使用           */
unsigned long   kasokudo=0;        		/* 加速度                       */
unsigned long   old_kyori=0;            /* 前の走行距                   */

/* LCD関連 */
int             lcd_pattern = 4;        /* 初期画面設定                 */
int             base_set;

/* 現在の状態保存用 */
int             handleBuff;             /* 現在のハンドル角度記録       */
int             leftMotorBuff;          /* 現在の左モータPWM値記録      */
int             rightMotorBuff;         /* 現在の右モータPWM値記録      */

/* エンコーダ関連 */
unsigned long   iEncoderTotal=0;        /*ENC  積算値                   */
signed int      iEncoder=0;             /*ENC  現在値                   */
unsigned int    uEncoderBuff=0;         /*ENC  前回値保存               */
signed long     streat_ct=0;     //(割り込みないでインクリメント)
signed int      break_count=0;

/* 角度関連 */
int             angle_buff;             /* 現在ハンドル角度保持用       */

//走行ログ関係
int             flag_st=0;       //走行時確認フラグ 0:停止 1:走行
int             log_ct=0;
int             angle_log,accele_r_log,accele_l_log;
int             log_pattern[500];
int             log_se[500];
int             log_a[500];
int             log_b[500];
int             log_c[500];
int             log_d[500];

//速度制御関係
const int crankTargetSpeed = 45;//GoPro:38 
const int harfTargetSpeed = 80;

//未使用
const int revolution_difference[] = {   /* 角度から内輪、外輪回転差計算 */
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

/* 速度移動平均算出用変数*/
signed int      sp;						/* 速度　移動平均値           */
signed int      sp_sum=0;				/* 速度　移動平均値演算用変数 */
signed int      sp_buf[4];				/* 速度　移動平均値演算用変数 */
signed int      sp_count=0;				/* 速度　移動平均値演算用変数 */

//関数プロトタイプ宣言
void check_line(void);

/************************************************************************/
/* メインプログラム                                                     */
/************************************************************************/
void main( void ){
    int             i;                  /*ログ出力時の回り値            */
    /* マイコン機能の初期化 */
    init();                             /* 初期化                       */
    init_uart0_printf( SPEED_9600 );    /* UART0とprintf関連の初期化    */
    asm(" fset I ");                    /* 全体の割り込み許可           */
 	while( pushsw_get()) {
		base_set = 1;
        lcd_pattern = 1;
    }
   handle_on();/*サーボモータon*/
    /* マイコンカーの状態初期化 */
    handle( 0 );
    motor( 0, 0 );

	while( 1 ) {
        switch( pattern ) {
			/*****************************************************************
			    パターンについて
			    0：スイッチ入力待ち
			    1：スタートバーが開いたかチェック
			    11：通常トレース
			    12：右へ大曲げの終わりのチェック
			    13：左へ大曲げの終わりのチェック
			    20：クロスライン検出時の処理
			    25：クロスライン後のトレース、クランク検出
			   150：左クランク処理 
			   250：右クランク処理
			   300：左ハーフライン検出時の処理
			   400：右ハーフライン検出時の処理
			*****************************************************************/
            case 0:         /* スイッチ入力待ち */
                if( pushsw_get()) {
                    led_out( 0x0 );
                    pattern = 5;
                    cnt1 = 0;
                    cnt0 = 0;
                    break;
                }
                if( cnt1 < 100 ) {              /* LED点滅処理*/
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
                    flag_st=1;                     //ログ取得開始
                    cnt1 = 0;
                }
            break;
			
			case 7:                         //オート、、ゲートまで行くやつ！
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

                    case 0x18://○○○● ●○○○/* センタ→まっすぐ*/
                        handle( 0 );
                    break;

                    case 0x1c://○○○● ●●○○ /* 微妙に左寄り→右へ微曲げ */
                        handle( 2 );
                    break;

         	        case 0x0c://○○○○ ●●○○
                        handle( 3 );
                    break;

           	        case 0x0e://○○○○ ●●●○
                        handle( 4 ); 
                    break;

                    case 0x38: //○○●● ●○○○/* 微妙に右寄り→左へ微曲げ */
                        handle( -2 );
                    break;

			        case 0x30: //○○●● ○○○○
                        handle( -3 ); 
                    break;

           	        case 0x70://○●●● ○○○○
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
					

            case 1:        /* スタートバーが開いたかチェック */
                if( !startbar_get() ) { /* スタート！！ */
                    led_out( 0x0 );
                    syu_flag = 1;//スタート確認()
                    flag_st=1;   //ログ取得開始
                    pattern = 11;
                    cnt1 = 0;
                    break;
                }
                if( cnt1 < 50 ) {               /* LED点滅処理      */
                    led_out( 0x1 );
                }
                else if( cnt1 < 100 ) {
                    led_out( 0x2 );
                }
                else {
                    cnt1 = 0;
                }
            break;

            case 11:	//通常トレース
				handle_on();//サーボ電源on
	            led_out( 0x0 );
                if( check_crossline() ) {       /* 大曲げ中もクロスラインチェック */
                    pattern = 20;
					cnt1=0;
                    break;
                }
                if( check_rightline() ) {       /* 右ハーフラインチェック       */
                    pattern = 400;
					cnt1=0;
                    break;
                }
                if( check_leftline() ) {        /* 左ハーフラインチェック       */
                    pattern = 300;
					cnt1=0;
                    break;
                }
				
		        switch( sensor_inp(MASK4_4) ) {
			        case 0x00:
				        handle( 0 );
				        motor( 0,0 );
				    break ;

                    case 0x18://○○○● ●○○○/* センタ→まっすぐ*/
                        handle( 0 );
                        motor( 100,100 );
                    break;

                    //左より　右曲げ処理
                    case 0x1c://○○○● ●●○○ /* 微妙に左寄り→右へ微曲げ */
                        handle( 1 );
                        motor( 100 ,100 );
                    break;

         	        case 0x0c://○○○○ ●●○○
                        handle( 3 );
                        motor( 100 ,100 );
                    break;

           	        case 0x0e://○○○○ ●●●○
                        handle( 6 ); 
                        motor( 100 ,100 );
                    break;

			        case 0x06://○○○○ ○●●○/* 少し左寄り→右へ小曲げ */
                        handle( 10 );
                        motor( 100 ,100 );
			            pattern = 12;
				        //ストレート後のブレーキへ
				        if(sp >= 45 && break_count <= 2){	//前回左ブレーキ:LEFT　前回右ブレーキ:RIGHT
					        pattern = 112;
		           	        cnt1 = 0;//ブレーキカウント時間クリア
				        }
                    break;

				    //大きく右曲げモードへ
                    case 0x07://○○○○ ○●●●/* 中くらい左寄り→右へ中曲げ */
			            pattern = 12;
				        //ストレート後のブレーキへ
				        if(sp >= 45 && break_count <= 2){	//前回左ブレーキ:LEFT　前回右ブレーキ:RIGHT
					        pattern = 112;
                	        cnt1 = 0;//ブレーキカウント時間クリア
				        }
                    break;

				    //大きく右曲げモードへ
                    case 0x03://○○○○ ○○●●/* 大きく左寄り→右へ大曲げ */
			            pattern = 12;
                        //ストレート後のブレーキへ
				        if(sp >= 45 && break_count <= 2){	//前回左ブレーキ:LEFT　前回右ブレーキ:RIGHT
					        pattern = 112;
                	        cnt1 = 0;//ブレーキカウント時間クリア
				        }
                    break;

                    //右より　左曲げ処理
                    case 0x38: //○○●● ●○○○/* 微妙に右寄り→左へ微曲げ */
                        handle( -1 );
                        motor( 100 ,100 );
                    break;

			        case 0x30: //○○●● ○○○○
                        handle( -3 ); 
                        motor( 100 ,100 );//92 ,100
                    break;

           	        case 0x70://○●●● ○○○○
                        handle( -6 ); 
                        motor( 100 ,100 );
                    break;

                    case 0x60://○●●○ ○○○○/* 少し右寄り→左へ小曲げ */
                        handle( -10 );
                        motor( 100 ,100 );
			            pattern = 13;
				        //ストレート後のブレーキへ
				        if(sp >= 45 && break_count <= 2){	//前回左ブレーキ:LEFT　前回右ブレーキ:RIGHT
					        pattern = 113;
                	        cnt1 = 0;//ブレーキカウント時間クリア
				        }
                    break;

                    //大きく左曲げモードへ
                    case 0xc0://●●●○ ○○○○/* 大きく右寄り→左へ大曲げ */
                        pattern = 13;
                        //ストレート後のブレーキへ
                        if(sp >= 45 && break_count <= 2){	//前回左ブレーキ:LEFT　前回右ブレーキ:RIGHT
                            pattern = 113;
                            cnt1 = 0;//ブレーキカウント時間クリア
                        }
                    break;

                    case 0xe0://●●○○ ○○○○/* 中くらい右寄り→左へ中曲げ */
                        pattern = 13;
                        //ストレート後のブレーキへ
                        if(sp >= 45 && break_count <= 2){	//前回左ブレーキ:LEFT　前回右ブレーキ:RIGHT
                            pattern = 113;
                            cnt1 = 0;//ブレーキカウント時間クリア
                        }
                    break;

                    default:

                    break;
                }
		  
				if(sp > 90){  //速度制御 GoPro:65
					motor( 1 ,1 );
		    	}
		  
            break;

            //直線後のブレーキ処理(左より　右曲げ処理)
            case 112:
                if( check_crossline() ) {       /* 大曲げ中もクロスラインチェック */
                    pattern = 20;
					cnt1=0;
                    break;
                }
                if( check_rightline() ) {       /* 右ハーフラインチェック       */
                    pattern = 400;
					cnt1=0;
                    break;
                }
                if( check_leftline() ) {        /* 左ハーフラインチェック       */
                    pattern = 300;
					cnt1=0;
                    break;
                }

                /* 右曲げ時の直線ブレーキ処理*///////////////////////////////////////////////////////////////////////////////
                if(break_count < 1){//ブレーキ1回目
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
                }                               // ブレーキ2回目
                else if(sp >= 50){              // 速い時
                    handle_on();//サーボ電源on
					handle( 45 );
                    motor( -100 ,-100 );
                    brake_time=40;
					brake_power_flag = 0;
                }
				else{                           // そうでもない時
                    handle_on();//サーボ電源on
					handle( 30 );
                    motor( 0 ,-50 );
                    brake_time=30;
					brake_power_flag = 0;
                }
				//////////////////////////////////////////////////////////////////////////////////////////////////
				
				
                //○○●● ○○○○               ○○●● ●○○○              ○○○● ●○○○              ○○○● ●●○○             ○○○○ ●●○○
                if(sensor_inp(MASK4_4)==0x30 || sensor_inp(MASK4_4)==0x38 || sensor_inp(MASK4_4)==0x18 || sensor_inp(MASK4_4)==0x1c || sensor_inp(MASK4_4)==0x0c){
                    pattern = 11;
                    streat_flag=0;//ストレートフラグクリア
                    corner_flag=RIGHT;//コーナーフラグ＝右曲げ
                    break_count++;
                }

                if(cnt1>brake_time){//ブレーキ時間ms
                    pattern=12;
                    streat_flag=0;//ストレートフラグクリア
                    corner_flag=RIGHT;//コーナーフラグ＝右曲げ
                    break_count++;
                }
            break;

            //直線後のブレーキ処理(右より　左曲げ処理)
            case 113:
                if( check_crossline() ) {       /* 大曲げ中もクロスラインチェック */
                    pattern = 20;
					cnt1=0;
                    break;
                }
                if( check_rightline() ) {       /* 右ハーフラインチェック       */
                    pattern = 400;
					cnt1=0;
                    break;
                }
                if( check_leftline() ) {        /* 左ハーフラインチェック       *///ｼﾆﾅ抹茶
				
                    pattern = 300;
					cnt1=0;
                    break;
                }

                /* 左曲げ時の直線ブレーキ処理*/
                if(break_count < 1){//ブレーキ1回目///////////////////////////////////////////////////////////////////////
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
                }                               // ブレーキ2回目
                else if(sp >= 48){              // 速い時
                    handle_on();//サーボ電源on
					handle( -45 );
                    motor( -100 ,50 );
                    brake_time=40;
					brake_power_flag = 0;
                }
                else{                           // そうでもない時
                    handle_on();//サーボ電源on
					handle( -30 );
                    motor( -50 ,0 );
                    brake_time=30;
					brake_power_flag = 0;
                }
				///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
				
                //○○●● ○○○○               ○○●● ●○○○                  ○○● ●○○○              ○○○○ ●●○○       ○○○● ●●○○
                if(sensor_inp(MASK4_4)==0x30 || sensor_inp(MASK4_4)==0x38 || sensor_inp(MASK4_4)==0x18 || sensor_inp(MASK4_4)==0x0c || sensor_inp(MASK4_4)==0x1c) {
                    pattern = 11;
                    streat_flag=0;//ストレートフラグクリア
                    corner_flag=LEFT;//コーナーフラグ＝左曲げ
                    break_count++;
                }
                if(cnt1>brake_time){//ブレーキ時間ms
                    pattern=13;
                    streat_flag=0;//ストレートフラグクリア
                    corner_flag=LEFT;//コーナーフラグ＝左曲げ
                    break_count++;
                }
            break;

            //大きく右曲げモードへ	○○○○ ○○●●　後の処理
            case 12:
				handle_on();//サーボ電源on
                if( check_crossline() ) {       /* 大曲げ中もクロスラインチェック */
                    pattern = 20;
					cnt1=0;
                    break;
                }
                switch( sensor_inp(MASK3_3) ) {
                    case 0x04://○○○○ ○●○○	//pattern=11　復帰
                    case 0x06://○○○○ ○●●○	//pattern=11　復帰
                        pattern = 11;
                    break;

                    case 0x07:  //○○○○ ○●●●
                        handle( 10 );
                        motor( 100, 50 );
                    break;

                    case 0x03:  //○○○○ ○○●●
                        handle( 15 );
                        motor( 100, 45 );
                    break;

                    case 0x83:// ●○○○ ○○●●
                        handle( 21 );
                        motor( 100, 40 );
                    break;

                    case 0x81://●○○○ ○○○●
                        handle( 22 );
                        motor( 100, 35 );
                    break;

                    case 0xc1://●●○○ ○○○● 追加
                        handle( 22 );
                        motor( 100, 30 );
                    break;

                    case 0x80://●○○○ ○○○○  /* 80追加 */
                        pattern=14;
                        handle( 23 );
                        motor( 100, 30 );
                    break;

                    case 0xc0://●●○○ ○○○○  /* c0追加 */
                        pattern=14;
                        handle( 23 );
                        motor( 100, 30 );
                    break;

					case 0xe0://●●●○ ○○○○  /* c0追加 */
                        pattern=14;
                        handle( 24 );
                        motor( 100, 30 );
                    break;

                    //●●●○ ○○○○   //e0 をカット
                    default :
                    
                    break;
                }
            break;

            case 14://減速処理（大外れ時の減速処理）パターン12より
				handle_on();//サーボ電源on
                switch( sensor_inp(MASK3_3) ) {
                    case 0x83:// ●○○○ ○○●●
                        pattern=12;
                        handle( 24 );
                        motor( 100, 30 );
                    break;

                    case 0x81://●○○○ ○○○●
                        pattern=12;
                        handle( 27 );
                        motor( 100, 30 );
                    break;

                    case 0xc1://●●○○ ○○○●追加
                        pattern=12;
                        handle( 29 );
                        motor( 100, 20 );
                    break;

                    case 0xc0://●●○○ ○○○○  /* c0追加 */
                        handle( 31 );
                        motor( 100, 10 );
                    break;

					case 0xe0://●●●○ ○○○○  /* c0追加 */
                        handle( 33 );
                        motor( 100, 0 );
                    break;
					
					case 0x60://○●●○ ○○○○  /* c0追加 */
                        handle( 35 );
                        motor( 100, -100 );
                    break;
					
					case 0x70://○●●● ○○○○  /* c0追加 */
                        handle( 45 );
                        motor( 100, -100 );
                    break;
					
                    default :
                    
                    break;
                }
            break;

    //大きく左曲げモードへ	●●○○ ○○○○　後の処理
            case 13:
				handle_on();//サーボ電源on
                if( check_crossline() ) {       /* 大曲げ中もクロスラインチェック */
                    pattern = 20;
					cnt1=0;
                    break;
                }

                switch( sensor_inp(MASK3_3) ) {
                    case 0x20:	//○○●○ ○○○○//pattern=11　復帰
                    case 0x60:	//○●●○ ○○○○//pattern=11　復帰
                        pattern = 11;
                    break;

                    case 0xe0:	//●●●○ ○○○○
                        handle( -10 );
                        motor( 50, 100 );
                    break;

                    case 0xc0:  //●●○○ ○○○○
                        handle( -15 );
                        motor( 45, 100 );
                    break;

                    case 0xc1:  //●●○○ ○○○●
                        handle( -21 );
                        motor( 40, 100 );
                    break;

                    case 0x81 :  //●○○○ ○○○●
                        handle( -22 );
                        motor( 35, 100 );
                    break;

                    case 0x83:    //●○○○ ○○●●
                        handle( -22 );
                        motor( 30, 100 );
                    break;

                    case 0x01:	//○○○○ ○○○●
                        pattern=15;
                        handle( -23 );
                        motor( 30, 100 );
                    break;
                
                    case 0x03:	//○○○○ ○○●●
                        pattern=15;
                        handle( -23 );
                        motor( 30, 100 );
                    break;

					case 0x07:	//○○○○ ○●●●
                        pattern=15;
                        handle( -25 );
                        motor( 30, 100 );
                    break;

                    default :
                    
                    break;
                }
            break;

            case 15://（大外れ時の減速処理）パターン13より
				handle_on();//サーボ電源on
                switch( sensor_inp(MASK3_3) ) {
                    case 0xc1:	//●●○○ ○○○●
                        pattern = 13;
                        handle( -25 );
                        motor( 30, 100 );
                    break;

                    case 0x81:  //●○○○ ○○○●
                        pattern = 13;
                        handle( -27 );
                        motor( 30, 100 );
                    break;

                    case 0x83:	//●○○○ ○○●●
                        pattern = 13;
                        handle( -29 );
                        motor( 20, 100 );
                    break;

                    case 0x03:	//○○○○ ○○●●
                        handle( -31 );
                        motor( 10, 100 );
                    break;
					
					case 0x07:	//○○○○ ○●●●
                        handle( -33 );
                        motor( 0, 100 );
                    break;
					
					case 0x06:	//○○○○ ○●●○
                        handle( -35 );
                        motor( -100, 100 );
                    break;
					
					case 0x0e:	//○○○○ ●●●○
                        handle( -45 );
                        motor( -100, 100 );
                    break;
					
                    default :

                    break;
                }
            break;


////////////クランク/////////////
            case 20:/* クロスライン処理 */
				handle_on();//サーボ電源on
                cnt1 = 0;
				led_out(0x0);
				pattern=25;
            break;

            case 25:
				handle_on();//サーボ電源on
				if(cnt1>35){/////////////////////ゴヨミ
					led_out(0x0);
					pattern=120;
					cnt1 = 0;
				}
				switch( sensor_inp(MASK4_4) ) {
					case 0x18:// ○○○● ●○○○/* センタ→まっすぐ*/
	                    handle( 0 );

	                break;

	                case 0x1c://○○●● ●○○○ /* 微妙に左寄り→右へ微曲げ */
	                    handle( 2 );
	                    motor( 100 ,100 );

					break;

	                case 0x0c://○○●● ○○○○
	                    handle( 4 );
	               		motor( 100 ,100 );

	                break;

	                case 0x0e://○●●● ○○○○
	                    handle( 7 );
	       				motor( 100,100 );

	                break;

	                case 0x06://○●●○ ○○○○/* 少し左寄り→右へ小曲げ */
	                    handle( 10 );
	                    motor( 100 ,100 );

	                break;

	                case 0x38://○○○● ●●○○/* 微妙に右寄り→左へ微曲げ */
	                    handle( -2 );
	                    motor( 100 ,100 );

	                break;

	                case 0x30://○○○○ ●●○○
	                    handle( -4 );
	                    motor( 100 ,100 );

	                break;

	                case 0x70://○○○○ ●●●○
	                    handle( -7 );
	                    motor( 100 ,100 );
						
	                break;

	                case 0x60://○○○○ ○●●○/* 少し右寄り→左へ小曲げ */
	                    handle( -10 );
	                    motor( 100 ,100 );

	                break;
				}
			break;
			
				
			case 120:
		        /* 左クランクと判断→左クランククリア処理へ */
				if( sensor_inp(MASK4_0)==0xf0 || sensor_inp(MASK4_0)==0xe0) {   /* ●●●●　○○○○  ●●●○　○○○○*/
		            handle_on();//サーボ電源on
					handle( -50 );//大きめにふる  
		            motor( -50, 100);
		            pattern = 150;//左クランク判定
		            cnt1 = 0; 
		            break;
				}
		        /* 右クランクと判断→右クランククリア処理へ */
				if( sensor_inp(MASK0_4)==0x0f || sensor_inp(MASK0_4)==0x07){	/* ○○○○　●●●●  ○○○○　○●●●*/
					handle_on();//サーボ電源on
					handle( 50 );//大きめにふる  
		            motor( 100, -50);
		            pattern = 250;//右クランク判定
		            cnt1 = 0;					
		            break;
				}


				if (sp >= crankTargetSpeed+10) {            // クランク前の速度制御↓
					motor( -60, -60);
				}
				else if(sp >= crankTargetSpeed+5){
					motor( -40, -40 );
				}
				else if(sp >= crankTargetSpeed+2){
					motor( -10, -10 );
				}
				
				else if(sp <= crankTargetSpeed-2){         // 減速しすぎた時対策↓
					motor( 30, 30 );
				}
				else if(sp <= crankTargetSpeed-6){
					motor( 50, 50 );
				}
				else{                                      // 進入時の速度
					motor( 20, 20);
				}
				
		 		switch( sensor_inp(MASK4_4) ) {		//クロスラインからハーフライン検出までのトレース
					case 0x18://○○○● ●○○○
						handle( 0 );
					break;

					case 0x1c://○○○● ●●○○
						handle( 2 );
					break;

					case 0x0c://○○○○ ●●○○
						handle( 4 );
					break;

					case 0x0e://○○○○ ●●●○
						handle( 7 );
					break;

					case 0x06://○○○○ ○●●○
						handle( 9 );
					break;

					case 0x07://○○○○ ○●●●
						handle( 11 );
					break;

					case 0x38://○○●● ●○○○
						handle( -2 );
					break;

					case 0x30://○○●● ○○○○
						handle( -4 );
					break;

					case 0x70://○●●● ○○○○
						handle( -7 );
					break;

					case 0x60://○●●○ ○○○○
						handle( -9 );
					break;

					case 0xe0://●●●○ ○○○○
						handle( -11 );
					break;
				}
			break;

            case 150:	//左クランク
				handle( -50 );//大きめにふる  
		        motor( -50, 100);
                if(sensor_inp(MASK4_4)==0x00 ){//○○○○ ○○○○
					pattern = 155;//通常クランク判定
					cnt1=0;		//タイマークリア
                }
            break;
			
			
			case 155:
				handle_on();//サーボ電源on
				handle( -50 );//大きめにふる
				motor( -50, 100);

					
				if( sensor_inp(MASK0_1) == 0x01 ){//○○○○ ○○○●  (外ライン検出)
					motor( -90 , 100 );
					pattern = 160;
					cnt1 = 0;
					break;
				}   		
				if( sensor_inp(MASK1_0) == 0x80 ){//●○○○ ○○○○  (内ライン検出)
					motor( -90 , 100 ); 
					handle_free();
					pattern = 170; 
					cnt1 = 0;
					break;
				}
			break;
			
			
			case 160:
				if( cnt1 > 90 ){	//70ms後 152と合わせて150ms程度
					// ●○○○ ○○○●  ●●○○ ○○○● ●○○○ ○○●●
					if( sensor_inp(MASK4_4) == 0x81  || sensor_inp(MASK4_4) == 0xc1 ||  sensor_inp(MASK4_4) == 0x83 || cnt1 > 250 ){
						handle_on();//サーボ電源on
						handle( -50 );
						motor( -30 , 100 );
						pattern = 170;  
						cnt1 = 0;
						break;
					}
				}
			break;

					
			case 170:
				if( sensor_inp(MASK4_4) == 0xe0 || sensor_inp(MASK4_4) == 0x60 ) {// ●●●○ ○○○○ ○●●○ ○○○○
					handle_on();//サーボ電源on
					handle( 0 );
					pattern = 11;  //通常復帰
					cnt1 = 0;
					break;
				}
			break;
			

            case 250:	//右クランク
				handle( 50 );//大きめにふる  
		        motor( 100, -50);
                if(sensor_inp(MASK4_4)==0x00 ){//○○○○ ○○○○
					pattern = 255;//通常クランク判定
					cnt1=0;		//タイマークリア
                }
            break;
			
			case 255:
				handle_on();//サーボ電源on
				handle( 50 );//大きめにふる
				motor( 100,  -50);

				if( sensor_inp(MASK1_0) == 0x80 ){//●○○○ ○○○○  (外ライン検出)
					motor( 100 , -90 ); 
					pattern = 260; 
					cnt1 = 0;
					break;
				}
				if( sensor_inp(MASK0_1) == 0x01 ){//○○○○ ○○○●  (内ライン検出)
					motor( 100 , -90 );
					handle_free();
					pattern = 270;
					cnt1 = 0;
					break;
				}
			break;
			
			case 260:
				if( cnt1 > 90 ){	//70ms後 152と合わせて150ms程度
					// ●○○○ ○○○●  ●●○○ ○○○● ●○○○ ○○●●
					if( sensor_inp(MASK4_4) == 0x81  || sensor_inp(MASK4_4) == 0xc1 ||  sensor_inp(MASK4_4) == 0x83 || cnt1 > 250 ){
						handle_on();//サーボ電源on
						handle( 50 );
						motor( 100 , -30 );  
						pattern = 270;  
						cnt1 = 0;
						break;
					}
				}
			break;
			
			case 270:
				if( sensor_inp(MASK4_4) == 0x07 || sensor_inp(MASK4_4) == 0x06 ) {// ○○○○ ○●●● ○○○○ ○●●○
					handle_on();//サーボ電源on
					handle( 0 );
					pattern = 11;  //通常復帰
					cnt1 = 0;
					break;
				}
			break;


////////////レーンチェン///////////////
			case 300:
				if( check_crossline() ) {       /* クロスラインチェック */
                    pattern = 20;
                    break;
                }
				if(cnt1>30){
					pattern=305;
				}
			break;
			
			
            //レーンチェンジ処理（左レーン）
            case 305:    /* １本目の左ハーフライン検出時の処理 */
                led_out( 0x1 );
				motor(80,100);
                handle( -5 );
                pattern = 310;
                cnt1 = 0;
            break;

            case 310:   /* 左ハーフライン後のトレース、レーンチェンジ(inコース) */
                switch( sensor_inp(MASK4_4) ) {
                    case 0x06://○○○○ ○●●○　まっすぐ
                        handle( 0 );
                    break;

                    case 0x07://○○○○ ○●●●
                        handle( 0 );
                    break;

                    case 0x03://○○○○ ○○●●   右寄り→左曲げ
						handle( 1 );
                    break;

                    case 0x83://●○○○ ○○●●   右寄り→左曲げ
						handle( 1 );
                    break;

                    case 0x81://●○○○ ○○○●   右寄り→左曲げ
						handle( 2 );
                    break;

                    case 0xc1://●●○○ ○○○●   右寄り→左曲げ
                        handle( 3 );
                    break;

                    case 0x01://○○○○ ○○○●   右寄り→左曲げ   (追加)
						handle( 3 );
                    break;

                    case 0xc0://●●○○ ○○○○   右寄り→左曲げ
						handle( 7 );
                    break;

                    case 0x0e://○○○○ ●●●○
                        handle( 0 );
                    break;

                    case 0x0c://○○○○ ●●○○   左寄り→右曲げ
						handle( -1 );
                    break;

                    case 0x1c://○○○● ●●○○   左寄り→右曲げ
						handle( -1 );
                    break;

                    case 0x18://○○○● ●○○○   左寄り→右曲げ
						handle( -2 );
                    break;

                    case 0x38://○○●● ●○○○   左寄り→右曲げ
						handle( -4 );
                    break;

                    case 0x30://○○●● ○○○○   左寄り→右曲げ
						handle( -4 );
                    break;

                    case 0x70://○●●● ○○○○   左寄り→右曲げ
						handle( -7 );
                    break;
                }
				
				//速度制御
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
					handle_on();//サーボ電源on
                    handle( -37 );	 //大きく振る
                    motor( 30 , 30 );
                    pattern = 316;
                    cnt1 = 0;
					break;
                }
            break;

            case 316:
                if(cnt1 > 10){
					handle_on();//サーボ電源on
                    handle( -30 );	 //-18では、ぎりぎり後輪が通る
                    motor( -10 ,80 );
                    pattern = 320;
                    cnt1 = 0;
                }
            break;

                
            case 320:  /* 左レーンチェンジ後の戻し動作1(インコース） */
                if(( sensor_inp(MASK3_0) == 0x80) || ( sensor_inp(MASK3_0) == 0xc0)) {	//●○○○ ○○○○  ●●○○ ○○○○
                    handle_on();//サーボ電源on
					handle( 45 );
                    motor( 100 ,-90 );
                    pattern = 325;
                    cnt1 = 0;
                }
            break;

            case 325:  /* 左レーンチェンジ後戻し動作2(インコース） */
                if( cnt1 > 20 ) {
					handle_on();//サーボ電源on
                    handle( 45 );
                    motor( 100 ,-90 );
                    pattern = 330;
                    cnt1 = 0;
                }
            break;

            case 330: /* 左レーンチェンジ安定したら通常トレースへ (インコース）*/
                if( cnt1 > 60 ) { 
					if( (sensor_inp(MASK3_3)== 0x06) || (sensor_inp(MASK3_3)== 0x02)|| (sensor_inp(MASK_left)== 0x04)  ||  (sensor_inp(MASK4_4)== 0x38) || (sensor_inp(MASK4_4)== 0x18) || (sensor_inp(MASK4_4)== 0x1c) || (sensor_inp(MASK4_4)== 0x30)|| (sensor_inp(MASK4_4)== 0x70) ||(sensor_inp(MASK4_4)== 0x60)) {
	                    led_out( 0x0 );
	                    pattern = 11;
	                }
				}
            break;
			
			case 400:
			if( check_crossline() ) {       /* 大曲げ中もクロスラインチェック */
                pattern = 20;
                break;
            }
			if(cnt1>30){
				pattern = 405;
			}
			break;
			
            //レーンチェンジ処理（右レーン）
            case 405:/* １本目の右ハーフライン検出時の処理 */
                led_out( 0x2 );
				motor(100,80);
                handle( 5 );
                pattern = 410;
                cnt1 = 0;
            break;

            case 410: /* 右ハーフライン後のトレース、レーンチェンジ */
                switch( sensor_inp(MASK4_4) ) {
                    case 0x60://○●●○ ○○○○　まっすぐ
						handle( 0 );
                    break;

                    case 0xe0://●●●○ ○○○○
						handle( 0 );
                    break;

                    case 0xc0://●●○○ ○○○○   左寄り→右曲げ
						handle( -1 );
                    break;

                    case 0xc1://●●○○ ○○○●   左寄り→右曲げ
                        handle( -1 );
                    break;

                    case 0x81://●○○○ ○○○●   左寄り→右曲げ
						handle( -2 );
                    break;

                    case 0x83://●○○○ ○○●●   左寄り→右曲げ   (追加)
						handle( -3 );
                    break;

                    case 0x03://○○○○ ○○●●   左寄り→右曲げ
						handle( -3 );
                    break;

                    case 0x07://○○○○ ○●●●   左寄り→右曲げ　（追加）
						handle( -7 );
                    break;

                    case 0x70://○●●● ○○○○
						handle( 0 );
                    break;

                    case 0x30://○○●● ○○○○   右寄り→左曲げ
						handle( 1 );
                    break;

                    case 0x38://○○●● ●○○○   右寄り→左曲げ
						handle( 1 );
                    break;

                    case 0x18://○○○● ●○○○   右寄り→左曲げ
						handle( 2 );
                    break;

                    case 0x1c://○○○● ●●○○   右寄り→左曲げ
						handle( 4 );
                    break;

                    case 0x0c://○○○○ ●●○○   右寄り→左曲げ
						handle( 4 );
                    break;

                    case 0x0e://○○○○ ●●●○   右寄り→左曲げ
						handle( 7 );
                    break;
                }
				
				//速度制御
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
                    handle_on();//サーボ電源on
					handle( 37 );	 //大きく振る
                    motor( 30, 30 );
                    pattern = 416;
                    cnt1 = 0;
					break;
                }
            break;

            case 416:
                if(cnt1 > 10){// cnt1 >80
                    handle_on();//サーボ電源on
					handle( 30 );	 //18では、ぎりぎり後輪が通る
                    motor( 80 ,-10 );
                    pattern = 420;
                    cnt1 = 0;

                }
            break;
			
            case 420:  /* 右レーンチェンジ後の戻し動作1 */
                if(( sensor_inp(MASK0_3) == 0x01) || ( sensor_inp(MASK0_3) == 0x03)){ 	//○○○○ ○○○●  ○○○○ ○○●●
                    handle_on();//サーボ電源on
					handle( -45 );
                    motor( -90 ,100 );
                    pattern = 425;
                    cnt1 = 0;
                }
            break;

            case 425:  /* 右レーンチェンジ後戻し動作2 */
                if( cnt1 > 20 ) {
					handle_on();//サーボ電源on
                    handle( -45 );
                    motor( -90 ,100 );
                    pattern = 430;
                    cnt1 = 0;
                }
            break;

            case 430: /* 右レーンチェンジ安定したら通常トレースへ (インコース）*/
                if( cnt1 > 60 ) { 
					if( (sensor_inp(MASK3_3)== 0x60) || (sensor_inp(MASK3_3)== 0x40) || (sensor_inp(MASK_right)== 0x20) ||    (sensor_inp(MASK4_4)== 0x38) || (sensor_inp(MASK4_4)== 0x18) || (sensor_inp(MASK4_4)== 0x1c) || (sensor_inp(MASK4_4)== 0x0c)|| (sensor_inp(MASK4_4)== 0x0e)||(sensor_inp(MASK4_4)== 0x06)) {
	                    led_out( 0x0 );
	                    pattern = 11;
	                }
				}
            break;
			

			case 102:   /* デバッグ用 */
                motor( 0, 0 );
            break;

            case 101:   /* 脱輪した際の自動停止処理後は、必ずこの処理を行ってください */
                handle_free();
                motor( 0, 0 );
            break;

            case 100:/* 確認用 */
                handle_free();
                motor( 0 ,0 );
                timer(500);
                while(pushsw_get()==0){
                    if( cnt1 < 50 ) {              /* LED点滅処理 */
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

                    case 0x18://○○○● ●○○○
                        handle( 0 );
                    break;

                    //左より　右曲げ処理
                    case 0x1c://○○○● ●●○○
                        handle( 2 );
                    break;

                    case 0x0c://○○○○ ●●○○
                        handle( 4 );
                    break;

                    case 0x0e://○○○○ ●●●○
                        handle( 8);
                    break;

                    case 0x06://○○○○ ○●●○
                        handle( 10 );
                    break;

                    case 0x07://○○○○ ○●●●
                        handle( 12 );
                    break;

                    case 0x03://○○○○ ○○●●
                        handle( 14 );
                    break;

                    //右より　左曲げ処理
                    case 0x38: //○○●● ●○○○
                        handle( -2 );
                    break;

                    case 0x30: //○○●● ○○○○
                        handle( -4 );
                    break;

                    case 0x70://○●●● ○○○○
                        handle( -8 );
                    break;

                    case 0x60://○●●○ ○○○○
                        handle( -10 );
                    break;

                    case 0xc0://●●●○ ○○○○
                        handle( -12 );
                    break;

                    case 0xe0://●●○○ ○○○○
                        handle( -14 );
                    break;

                    default:

                    break;
                }	
            break;
        
            default: /* どれでもない場合は待機状態に戻す */
                pattern = 0;
            break;
        }
    }
}

/************************************************************************/
/* R8C/38A スペシャルファンクションレジスタ(SFR)の初期化                */
/************************************************************************/
void init( void ){
    int i;

    /* クロックをXINクロック(20MHz)に変更 */
    prc0  = 1;                          /* プロテクト解除               */
    cm13  = 1;                          /* P4_6,P4_7をXIN-XOUT端子にする*/
    cm05  = 0;                          /* XINクロック発振              */
    for(i=0; i<50; i++ );               /* 安定するまで少し待つ(約10ms) */
    ocd2  = 0;                          /* システムクロックをXINにする  */
    prc0  = 0;                          /* プロテクトON                 */

    /* ポートの入出力設定 */
    prc2 = 1;                           /* PD0のプロテクト解除          */
    pd0 = 0x00;                         /* 7-0:センサ基板Ver.5          */
    pd1 = 0xd0;                         /* 5:RXD0 4:TXD0 3-0:DIP SW     */
    p2  = 0xc0;
    pd2 = 0xfe;                         /* 7-0:モータドライブ基板Ver.5  */

    pd3 = 0xfe;                         /*  0:エンコーダ(入力） */
 //   pd3 = 0xff;                         /*                              */

    p4  = 0x20;                         /* P4_5のLED:初期は点灯         */
    pd4 = 0xb8;                         /* 7:XOUT 6:XIN 5:LED 2:VREF    */
    pd5 = 0x7f;                         /* 7-0:LCD/microSD基板          */
    pd6 = 0xef;                         /* 4-0:LCD/microSD基板          */
	
    pd7 = 0x7f;//0xff                         /*                              */
	
    pd8 = 0xff;                         /*                              */
    pd9 = 0x3f;                         /*                              */
    pur0 = 0x04;                        /* P1_3～P1_0のプルアップON     */

    /* タイマRBの設定 */
    /* 割り込み周期 = 1 / 20[MHz]   * (TRBPRE+1) * (TRBPR+1)
                    = 1 / (20*10^6) * 200        * 100
                    = 0.001[s] = 1[ms]
    */
    trbmr  = 0x00;                      /* 動作モード、分周比設定       */
    trbpre = 200-1;                     /* プリスケーラレジスタ         */
    trbpr  = 100-1;                     /* プライマリレジスタ           */
    trbic  = 0x07;                      /* 割り込み優先レベル設定       */
    trbcr  = 0x01;                      /* カウント開始                 */



   /* タイマRG タイマモード(両エッジでカウント)の設定　エンコーダのカウンタ設定 */

   timsr = 0x40; /* TRGCLKA端子 P3_0に割り当てる */
   trgcr = 0x15; /* TRGCLKA端子の両エッジでカウント*/
   trgmr = 0x80; /* TRGのカウント開始 */


    /* タイマRD リセット同期PWMモードの設定*/
    /* PWM周期 = 1 / 20[MHz]   * カウントソース * (TRDGRA0+1)
               = 1 / (20*10^6) * 8              * 40000
               = 0.016[s] = 16[ms]
    */
    trdpsr0 = 0x08;                     /* TRDIOB0,C0,D0端子設定        */
    trdpsr1 = 0x05;                     /* TRDIOA1,B1,C1,D1端子設定     */
    trdmr   = 0xf0;                     /* バッファレジスタ設定         */
    trdfcr  = 0x01;                     /* リセット同期PWMモードに設定  */
    trdcr0  = 0x23;                     /* ソースカウントの選択:f8      */
    trdgra0 = trdgrc0 = PWM_CYCLE;      /* 周期                         */
    trdgrb0 = trdgrd0 = 0;              /* P2_2端子のON幅設定           */
    trdgra1 = trdgrc1 = 0;              /* P2_4端子のON幅設定           */
    trdgrb1 = trdgrd1 = SERVO_CENTER_W; /* P2_5端子のON幅設定           */
    trdoer1 = 0xcd;                     /* 出力端子の選択               */
    trdstr  = 0x0d;                     /* TRD0カウント開始             */
}

/************************************************************************/
/* タイマRB 割り込み処理                                                */
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


    /* 脱輪時の停止処理（デジタルセンサ） */
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

    //ストレートカウント
    if(angle_log<10 && angle_log>-10){//ストレート時のカウント
   		streat_ct++;
    }
    else{
   		streat_ct=0;
    }
    if(streat_ct>200){	//ストレート連続走行時（400ms後の処理）
	   streat_flag=1;	//ストレート連続走行確認フラグ 1:ON 0:OFF  400ms=1m程度
	   break_count=0;	//ブレーキの数カウント（ストレート400ms後クリア）
	   corner_flag=0;	//コーナーフラグクリア
    }
	
	//加速度計算
	if(cnt_kasokudo > 50){
		kasokudo = iEncoderTotal - old_kyori;
		old_kyori = iEncoderTotal;
		cnt_kasokudo = 0;
	}
	
   /*                  */
   /*10ms間隔の処理       */
   /*走行距離・走行速度取得*/
   /*                  */
    if(flag_st==1){//スタート後　　flag_st:true(スタート時)
        cnt_time++;
        if((cnt_time%10)==0){      //10ms毎
		  	//エンコーダ取得関係
	  		t=trg;							//凡用変数に代入
			iEncoder = t - uEncoderBuff;		//全体から過去の値を引く
			iEncoderTotal += iEncoder;	//トータルのきょりを記録
			uEncoderBuff = t;		//t記憶用


			/* 4点の速度の移動平均計算sp_buf[4]*/       
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
  // プッシュSW操作時	スタート500ms経過後
    if(pushsw_get()==1 && cnt_syu > 500 ){
        pattern=100;
        flag_st=0;//ログ取得停止
    }
   /* 終了タイマー スタート直後　プッシュSW操作防止（無いとスタートと同時にpattern=100）*/
    if(syu_flag==1){
        if(cnt_syu<1000)cnt_syu++;
    }

}
/************************************************************************/
/* タイマ本体                                                           */
/* 引数　 タイマ値 1=1ms                                                */
/************************************************************************/
void timer( unsigned long timer_set ){
    cnt0 = 0;
    while( cnt0 < timer_set );
}
/************************************************************************/
/* センサ状態検出                                                       */
/* 引数　 マスク値                                                      */
/* 戻り値 センサ値                                                      */
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
/* クロスライン検出処理                                                 */
/* 戻り値 0:クロスラインなし 1:あり                                     */
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
/* ハーフライン後の時間カウント  
/* 引数　 フラグ                                             */
/* 戻り値 カウント                                                 */
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
/* 右ハーフライン検出処理                                               */
/* 戻り値 0:なし 1:あり                                                 */
/************************************************************************/
int check_rightline( void )
{
    unsigned char b;
    int ret;

    ret = 0;
    b = sensor_inp(MASK4_4);
    if( b==0x0f || b==0x1f || b==0x3f || b==0x7f) { /* ！追加・変更！   */
        ret = 1;
    }
    return ret;
}

/************************************************************************/
/* 左ハーフライン検出処理                                               */
/* 戻り値 0:なし 1:あり                                                 */
/************************************************************************/
int check_leftline( void )
{
    unsigned char b;
    int ret;

    ret = 0;
    b = sensor_inp(MASK4_4);
    if( b==0xf0 || b==0xf8 || b==0xfc || b==0xfe) { /* ！追加・変更！   */
       ret = 1;
    }
    return ret;
}
/************************************************************************/
/* ディップスイッチ値読み込み１(スピード制御)                           */
/* 戻り値 スイッチ値 0～15                                              */
/************************************************************************/
unsigned char dipsw_gets( void )
{
    unsigned char sw1;

    sw1 = p1 & 0x07;                     /* P1_3～P1_1読み込み           */

    return  sw1;
}
/************************************************************************/
/* ディップスイッチ値読み込み2(in-out制御)                              */
/* 戻り値 スイッチ値 0～15                                              */
/************************************************************************/
unsigned char dipsw_get( void )
{
    unsigned char sw2;

    sw2 = p1 & 0x08;                     /* P1_3～P1_0読み込み           */

    return  sw2;
}

/************************************************************************/
/* プッシュスイッチ値読み込み                                           */
/* 戻り値 スイッチ値 ON:1 OFF:0                                         */
/************************************************************************/
unsigned char pushsw_get( void )
{
    unsigned char sw;

    sw  = ~p2;                          /* スイッチのあるポート読み込み */
    sw &= 0x01;

    return  sw;
}

/************************************************************************/
/* スタートバー検出センサ読み込み                                       */
/* 戻り値 センサ値 ON(バーあり):1 OFF(なし):0                           */
/************************************************************************/
unsigned char startbar_get( void )
{
    unsigned char b;

    b  = !p7_7;//~p0;                           /* スタートバー信号読み込み     */
    //b &= 0x01;

    return  b;
}

/************************************************************************/
/* LED制御                                                              */
/* 引数　スイッチ値 LED0:bit0 LED1:bit1  "0":消灯 "1":点灯              */
/* 例)0x3→LED1:ON LED0:ON  0x2→LED1:ON LED0:OFF                       */
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
/* モータ速度制御                                                       */
/* 引数　 左モータ:-100～100、右モータ:-100～100                        */
/*        0で停止、100で正転100%、-100で逆転100%                        */
/* 戻り値 なし                                                          */
/************************************************************************/
void motor( int accele_l, int accele_r )
{
   //int    sw_data;

   accele_r_log=accele_r;
   accele_l_log=accele_l;
    
   //sw_data=(dipsw_gets()&0x03)*5+85;
   //accele_l = accele_l * sw_data / 100;
   //accele_r = accele_r * sw_data / 100;
	
   leftMotorBuff  = accele_l;          /* バッファに保存               */
   rightMotorBuff = accele_r;          /* バッファに保存               */
   

       /* 左モータ制御 */
        if( accele_l >= 0 ) {
            p2 &= 0xfd;
            trdgrd0 = (long)( PWM_CYCLE - 1 ) * accele_l / 100;
        }
		else {
	        p2 |= 0x02;
	        trdgrd0 = (long)( PWM_CYCLE - 1 ) * ( -accele_l ) / 100;
        }

       /* 右モータ制御 */
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
/* サーボハンドル操作                                                   */
/* 引数　 サーボ操作角度：-90～90                                       */
/*        -90で左へ90度、0でまっすぐ、90で右へ90度回転                  */
/************************************************************************/
void handle( int angle )
{
    handleBuff = angle;                 /* バッファに保存               */
    angle_buff = angle;                 /* 現在の角度保存               */
   angle_log=angle;
    /* サーボが左右逆に動く場合は、「-」を「+」に替えてください */
    trdgrd1 = SERVO_CENTER_W + angle * HANDLE_STEP;
}

/************************************************************************/
/* サーボハンドル操作　フリー                                           */
/* 引数　 なし                                                          */
/*                                                                      */
/************************************************************************/
void handle_free( void )
{
    trdoer1 |= 0x20;                    /* サーボ用PWM(P2_5)をOFFに設定 */
}
/************************************************************************/
/* サーボハンドル操作　制御モード                                           */
/* 引数　 なし                                                          */
/*                                                                      */
/************************************************************************/
void handle_on( void )
{
    trdoer1 &= 0xdf;                    /* サーボ用PWM(P2_5)をONに設定 **0* **** */
}
/************************************************************************/
/* 外輪のPWMから、内輪のPWMを割り出す　ハンドル角度は現在の値を使用     */
/* 引数　 外輪PWM                                                       */
/* 戻り値 内輪PWM                                                       */
/************************************************************************/
int diff( int pwm )
{
    int ret;

    if( pwm >= 0 ) {
        /* PWM値が正の数なら */
        if( angle_buff < 0 ) {
            angle_buff = -angle_buff;
        }
        ret = revolution_difference[angle_buff] * pwm / 100;
    } else {
        /* PWM値が負の数なら */
        ret = pwm;                      /* そのまま返す                 */
    }
    return ret;
}
/************************************************************************/
/* 停止状態制御                                                         */
/* 引数　 左モータの状態：BRAKE or FREE                                 */
/*     　 右モータの状態：BRAKE or FREE                                 */
/************************************************************************/
void motor_mode( int mode_l, int mode_r )
{
    if( mode_l ) {                      /* 左モータチェック             */
        p8_1 = 1;                       /* フリー動作                   */
    } else {
        p8_1 = 0;                       /* ブレーキ動作                 */
    }
    if( mode_r ) {                      /* 右モータチェック             */
        p8_0 = 1;                       /* フリー動作                   */
    } else {
        p8_0 = 0;                       /* ブレーキ動作                 */
    }
}
/************************************************************************/
/* end of file                                                          */
/************************************************************************/
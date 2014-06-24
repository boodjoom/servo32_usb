/*
**
**                           Main.c
**
**
**********************************************************************/
/*
   Last committed:     $Revision: 00 $
   Last changed by:    $Author: $
   Last changed date:  $Date:  $
   ID:                 $Id:  $

**********************************************************************/
#include "stm32f10x_conf.h"

#include "servo.h"
#include "string.h"
#include "stdio.h"

#define LINEMAX 512
#define USED_PORTC 0
#define USED_PORTB 1
#define USED_PORTA 2
#define USED_PORTF 3
#define USED_PORTD 4
//char rx_buffer[LINEMAX];
//uint16_t rx_counter;
volatile char line_buffer[LINEMAX + 1]; // Holding buffer with space for terminating NUL
volatile int line_valid = 0;
volatile int line_length = 0;

uint16_t commands[2][MAX_SERVO];

uint8_t servo_state=0;
uint16_t cur_ocr=0;
ServoArray servos;

static void
Delay(__IO uint32_t nTime);

static void
TimingDelay_Decrement(void);

void
SysTick_Handler(void);

void init_timer(void);
void TIM4_IRQHandler(void);

void init_usart1(void);
void USART1_IRQHandler(void);
void ProcessLine(void);
void USART2_IRQHandler(void);

/* ----- SysTick definitions ----------------------------------------------- */

#define SYSTICK_FREQUENCY_HZ       1000

int fputc(int ch, FILE * f) {
  /* Transmit the character using USART1 */
  USART_SendData(USART1, (uint8_t) ch);

  /* Wait until transmit finishes */
  while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);

  return ch;
}

void putch(char c)
{
    /* Wait until transmit finishes */
  while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
    /* Transmit the character using USART1 */
  USART_SendData(USART1, c);
}

void putsu(char * str)
{
  int i;
  for(i=0;i<strlen(str);i++)
    putch(str[i]);
}

int
main(void)
{

  /*
   * At this stage the microcontroller clock setting is already configured,
   * this is done through SystemInit() function which is called from startup
   * file (startup_cm.c) before to branch to application main.
   * To reconfigure the default setting of SystemInit() function, refer to
   * system_stm32f0xx.c file
   */

  /* Use SysTick as reference for the timer */
  SysTick_Config(SystemCoreClock / SYSTICK_FREQUENCY_HZ);

  //GPIO_InitTypeDef GPIO_InitStructure;

  /* Configure pin in output push/pull mode */
  //GPIO_InitStructure.GPIO_Pin = (1 << BLINK_PIN);
  //GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  //GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
 // GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
 // GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  //GPIO_Init(BLINK_PORT, &GPIO_InitStructure);

  /* Turn on led by setting the pin low */
  //GPIO_ResetBits(BLINK_PORT, (1 << BLINK_PIN));
  /* Turn off led by setting the pin high */
  //GPIO_SetBits(BLINK_PORT, (1 << BLINK_PIN));

  //
  //init usart1
  //
  init_usart1();

  //
  //init servos
  //
  servos._port_action[USED_PORTC]=GPIOC;
  servos._port_action[USED_PORTB]=GPIOB;
  servos._port_action[USED_PORTA]=GPIOA;
  servos._port_action[USED_PORTD]=GPIOD;
  servos._port_action[USED_PORTF]=GPIOF;

  uint8_t s_num=0;
  //servo #1
  servos._array[s_num]._port=GPIOC;
  servos._array[s_num]._RCC_AHBPeriph=RCC_APB2Periph_GPIOC;
  servos._array[s_num]._pin=GPIO_Pin_5;
  //servo #2
  s_num=1;
  servos._array[s_num]._port=GPIOB;
  servos._array[s_num]._RCC_AHBPeriph=RCC_APB2Periph_GPIOB;
  servos._array[s_num]._pin=GPIO_Pin_0;
  //servo #3
  s_num=2;
  servos._array[s_num]._port=GPIOB;
  servos._array[s_num]._RCC_AHBPeriph=RCC_APB2Periph_GPIOB;
  servos._array[s_num]._pin=GPIO_Pin_1;
  //servo #4
  s_num=3;
  servos._array[s_num]._port=GPIOB;
  servos._array[s_num]._RCC_AHBPeriph=RCC_APB2Periph_GPIOB;
  servos._array[s_num]._pin=GPIO_Pin_12;
  //servo #5
  s_num=4;
  servos._array[s_num]._port=GPIOB;
  servos._array[s_num]._RCC_AHBPeriph=RCC_APB2Periph_GPIOB;
  servos._array[s_num]._pin=GPIO_Pin_13;
  //servo #6
  s_num=5;
  servos._array[s_num]._port=GPIOB;
  servos._array[s_num]._RCC_AHBPeriph=RCC_APB2Periph_GPIOB;
  servos._array[s_num]._pin=GPIO_Pin_14;
  //servo #7
  s_num=6;
  servos._array[s_num]._port=GPIOB;
  servos._array[s_num]._RCC_AHBPeriph=RCC_APB2Periph_GPIOB;
  servos._array[s_num]._pin=GPIO_Pin_15;
  //servo #8
  s_num=7;
  servos._array[s_num]._port=GPIOC;
  servos._array[s_num]._RCC_AHBPeriph=RCC_APB2Periph_GPIOC;
  servos._array[s_num]._pin=GPIO_Pin_6;
  //servo #9
  s_num=8;
  servos._array[s_num]._port=GPIOC;
  servos._array[s_num]._RCC_AHBPeriph=RCC_APB2Periph_GPIOC;
  servos._array[s_num]._pin=GPIO_Pin_7;
  //servo #10
  s_num=9;
  servos._array[s_num]._port=GPIOC;
  servos._array[s_num]._RCC_AHBPeriph=RCC_APB2Periph_GPIOC;
  servos._array[s_num]._pin=GPIO_Pin_8;
  //servo #11
  s_num=10;
  servos._array[s_num]._port=GPIOC;
  servos._array[s_num]._RCC_AHBPeriph=RCC_APB2Periph_GPIOC;
  servos._array[s_num]._pin=GPIO_Pin_9;
  //servo #12
  s_num=11;
  servos._array[s_num]._port=GPIOA;
  servos._array[s_num]._RCC_AHBPeriph=RCC_APB2Periph_GPIOA;
  servos._array[s_num]._pin=GPIO_Pin_8;
  //servo #13
  s_num=12;
  servos._array[s_num]._port=GPIOA;
  servos._array[s_num]._RCC_AHBPeriph=RCC_APB2Periph_GPIOA;
  servos._array[s_num]._pin=GPIO_Pin_14;
  //servo #14
  s_num=13;
  servos._array[s_num]._port=GPIOA;
  servos._array[s_num]._RCC_AHBPeriph=RCC_APB2Periph_GPIOA;
  servos._array[s_num]._pin=GPIO_Pin_15;

  //servo #15
  s_num=14;
  servos._array[s_num]._port=GPIOC;
  servos._array[s_num]._RCC_AHBPeriph=RCC_APB2Periph_GPIOC;
  servos._array[s_num]._pin=GPIO_Pin_10;
  //servo #16
  s_num=15;
  servos._array[s_num]._port=GPIOC;
  servos._array[s_num]._RCC_AHBPeriph=RCC_APB2Periph_GPIOC;
  servos._array[s_num]._pin=GPIO_Pin_11;
  //servo #17
  s_num=16;
  servos._array[s_num]._port=GPIOC;
  servos._array[s_num]._RCC_AHBPeriph=RCC_APB2Periph_GPIOC;
  servos._array[s_num]._pin=GPIO_Pin_12;

  //servo #18
  s_num=17;
  servos._array[s_num]._port=GPIOD;
  servos._array[s_num]._RCC_AHBPeriph=RCC_APB2Periph_GPIOD;
  servos._array[s_num]._pin=GPIO_Pin_2;

  //servo #19
  s_num=18;
  servos._array[s_num]._port=GPIOB;
  servos._array[s_num]._RCC_AHBPeriph=RCC_APB2Periph_GPIOB;
  servos._array[s_num]._pin=GPIO_Pin_3;
   //servo #20
  s_num=19;
  servos._array[s_num]._port=GPIOB;
  servos._array[s_num]._RCC_AHBPeriph=RCC_APB2Periph_GPIOB;
  servos._array[s_num]._pin=GPIO_Pin_4;
   //servo 21
  s_num=20;
  servos._array[s_num]._port=GPIOB;
  servos._array[s_num]._RCC_AHBPeriph=RCC_APB2Periph_GPIOB;
  servos._array[s_num]._pin=GPIO_Pin_5;
   //servo #22
  s_num=21;
  servos._array[s_num]._port=GPIOB;
  servos._array[s_num]._RCC_AHBPeriph=RCC_APB2Periph_GPIOB;
  servos._array[s_num]._pin=GPIO_Pin_6;
   //servo #23
  s_num=22;
  servos._array[s_num]._port=GPIOB;
  servos._array[s_num]._RCC_AHBPeriph=RCC_APB2Periph_GPIOB;
  servos._array[s_num]._pin=GPIO_Pin_7;
   //servo #24
  s_num=23;
  servos._array[s_num]._port=GPIOB;
  servos._array[s_num]._RCC_AHBPeriph=RCC_APB2Periph_GPIOB;
  servos._array[s_num]._pin=GPIO_Pin_8;
   //servo #25
  s_num=24;
  servos._array[s_num]._port=GPIOB;
  servos._array[s_num]._RCC_AHBPeriph=RCC_APB2Periph_GPIOB;
  servos._array[s_num]._pin=GPIO_Pin_9;
    //servo #26
  s_num=25;
  servos._array[s_num]._port=GPIOC;
  servos._array[s_num]._RCC_AHBPeriph=RCC_APB2Periph_GPIOC;
  servos._array[s_num]._pin=GPIO_Pin_13;

   //servo #27
  s_num=26;
  servos._array[s_num]._port=GPIOA;
  servos._array[s_num]._RCC_AHBPeriph=RCC_APB2Periph_GPIOA;
  servos._array[s_num]._pin=GPIO_Pin_0;
   //servo #28
  s_num=27;
  servos._array[s_num]._port=GPIOA;
  servos._array[s_num]._RCC_AHBPeriph=RCC_APB2Periph_GPIOA;
  servos._array[s_num]._pin=GPIO_Pin_1;
   //servo #29
  s_num=28;
  servos._array[s_num]._port=GPIOA;
  servos._array[s_num]._RCC_AHBPeriph=RCC_APB2Periph_GPIOA;
  servos._array[s_num]._pin=GPIO_Pin_2;
   //servo #30
  s_num=29;
  servos._array[s_num]._port=GPIOA;
  servos._array[s_num]._RCC_AHBPeriph=RCC_APB2Periph_GPIOA;
  servos._array[s_num]._pin=GPIO_Pin_3;
   //servo #31
  s_num=30;
  servos._array[s_num]._port=GPIOF;
  servos._array[s_num]._RCC_AHBPeriph=RCC_APB2Periph_GPIOF;
  servos._array[s_num]._pin=GPIO_Pin_5;
   //servo #32
  s_num=31;
  servos._array[s_num]._port=GPIOF;
  servos._array[s_num]._RCC_AHBPeriph=RCC_APB2Periph_GPIOF;
  servos._array[s_num]._pin=GPIO_Pin_4;

  ServoArrayInit(&servos);

  //GPIO_ResetBits(BLINK_PORT, (1 << BLINK_PIN));
  //Delay(50);
  /* Turn off led by setting the pin high */
  //GPIO_SetBits(BLINK_PORT, (1 << BLINK_PIN));
  //Delay(50);
  init_timer();
  //printf("start");
  /* Infinite loop */
  //do __NOP;
  while (1)
    {/*
	   GPIO_ResetBits(BLINK_PORT, (1 << BLINK_PIN));
	   ServoArraySetPosition(&servos,0,1000,0);
	   ServoArraySetPosition(&servos,1,1000,0);
	   Delay(1000);
	   GPIO_SetBits(BLINK_PORT, (1 << BLINK_PIN));
     ServoArraySetPosition(&servos,0,2000,0);
	   ServoArraySetPosition(&servos,1,2000,0);
	   Delay(1000);*/

	   __WFI(); // Wait for an interrupt rather than grind endlessly

      if (line_valid && line_length>5) // A new line has arrived
      {
          ProcessLine(); // Do something with the line
          line_valid = 0; // clear pending flag
      }
    }

}

// ----------------------------------------------------------------------------

static __IO uint32_t uwTimingDelay;

/**
 * @brief  Inserts a delay time.
 * @param  nTime: specifies the delay time length, in SysTick ticks.
 * @retval None
 * void

 */
void
Delay(__IO uint32_t nTime)
{
  uwTimingDelay = nTime;

  while (uwTimingDelay != 0)
    ;
}


/**
 * @brief  Decrements the TimingDelay variable.
 * @param  None
 * @retval None
 */
void
TimingDelay_Decrement(void)
{
  if (uwTimingDelay != 0x00)
    {
      uwTimingDelay--;
    }
}

// ----------------------------------------------------------------------------

/**
 * @brief  This function is the SysTick Handler.
 * @param  None
 * @retval None
 */
void
SysTick_Handler(void)
{
  TimingDelay_Decrement();
}

// ----------------------------------------------------------------------------

void init_usart1()
{
  USART_InitTypeDef USART_InitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);

  /* Configure USART1 pins:  Rx and Tx ----------------------------*/
GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_9; //TX and RTS pins
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10; //RX and CTS pins
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  USART_InitStructure.USART_BaudRate = 9600;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  USART_Init(USART1, &USART_InitStructure);

  USART_Cmd(USART1,ENABLE);
/* Enable USART1 Receive interrupt */
  USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
  NVIC_EnableIRQ(USART1_IRQn);
}

void USART1_IRQHandler(void)
{
  static char rx_buffer[LINEMAX];   // Local holding buffer to build line
  static int rx_index = 0;

  if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET) // Received character?
  {
    char rx =  USART_ReceiveData(USART1);
    if ((rx == '\r') || (rx == '\n')) // Is this an end-of-line condition, either will suffice?
    {
        if (rx_index != 0) // Line has some content?
        {
                memcpy((void *)line_buffer, rx_buffer, rx_index); // Copy to static line buffer from dynamic receive buffer
                line_buffer[rx_index] = 0; // Add terminating NUL
                line_valid = 1; // flag new line valid for processing
                line_length=rx_index+1;
            rx_index = 0; // Reset content pointer
        }
    }
    else
    {
        if (rx_index == LINEMAX) // If overflows pull back to start
            rx_index = 0;

        rx_buffer[rx_index++] = rx; // Copy to buffer and increment
    }
  }
}

void ProcessLine(void)
{
  char* next = line_buffer;
  int id,pwm_us,duration_us,scan_rez,commands_len=0;
  duration_us=0;
  while(1)
  {
  if(line_length<2)
    break;
  if(next[0]!='#')
  {
    if(next[0]=='T')
    {
        scan_rez=sscanf(next,"T%d%s",&duration_us,next);
        if(scan_rez<1)
          break;
        if(scan_rez<2)
          line_length=0;
        continue;
    }
    else break;
  }
  scan_rez=sscanf(next,"#%dP%d%s",&id,&pwm_us,next);
  if(scan_rez<2)
    break;
  if(scan_rez<3)
    line_length=0;
  if(id>=1 && id<=MAX_SERVO && pwm_us>=500 && pwm_us<=2500)
    {
      commands[0][commands_len]=id;
      commands[1][commands_len]=pwm_us;
      commands_len++;
    }
  if(line_length!=0)
    line_length=strlen(next);
  else break;
  }
  for(id=0;id<commands_len;id++)
    ServoArraySetPosition(&servos,commands[0][id]-1,commands[1][id],duration_us);
}

void init_timer()
{
  /* ���� �٧ѧҧ�ӧѧ֧� �٧ѧ�ѧܧ�ڧ��ӧѧ�� ��ѧۧާ֧� */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

  /* ���ߧڧ�ڧѧݧڧ٧ڧ��֧� �ҧѧ٧�ӧ�� ��ѧۧާ֧�: �է֧ݧڧ�֧ݧ� 24000, ��֧�ڧ�� 500 �ާ�.
   * �����ԧڧ� ��ѧ�ѧާ֧��� �����ܧ���� TIM_TimeBaseInitTypeDef
   * �ߧ� �ڧާ֧�� ��ާ��ݧ� �էݧ� �ҧѧ٧�ӧ�� ��ѧۧާ֧���.
   */
  TIM_TimeBaseInitTypeDef base_timer;
  TIM_TimeBaseStructInit(&base_timer);
  base_timer.TIM_Prescaler = 72 - 1;
  base_timer.TIM_Period = 2000;
  TIM_TimeBaseInit(TIM4, &base_timer);

  /* ���ѧ٧�֧�ѧ֧� ���֧��ӧѧߧڧ� ��� ��ҧߧ�ӧݧ֧ߧڧ� (�� �էѧߧߧ�� ��ݧ��ѧ� -
   * ��� ��֧�֧��ݧߧ֧ߧڧ�) ���ק��ڧܧ� ��ѧۧާ֧�� TIM6.
   */
  TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);
  /* ���ܧݧ��ѧ֧� ��ѧۧާ֧� */
  TIM_Cmd(TIM4, ENABLE);

  /* ���ѧ٧�֧�ѧ֧� ��ҧ�ѧҧ��ܧ� ���֧��ӧѧߧڧ� ��� ��֧�֧��ݧߧ֧ߧڧ� ���ק��ڧܧ�
   * ��ѧۧާ֧�� TIM6. ���ѧ� ���ݧ��ڧݧ���, ���� ���� �ا� ���֧��ӧѧߧڧ�
   * ���ӧ֧�ѧ֧� �� �٧� ��������֧ߧڧ� ������.
   */
  NVIC_EnableIRQ(TIM4_IRQn);

}

void TIM4_IRQHandler()
{
  /* ���ѧ� �ܧѧ� ����� ��ҧ�ѧҧ���ڧ� �ӧ�٧�ӧѧ֧��� �� �էݧ� ������, �ߧ�اߧ� ����ӧ֧����,
   * ����ڧ٧��ݧ� �ݧ� ���֧��ӧѧߧڧ� ��� ��֧�֧��ݧߧ֧ߧڧ� ���ק��ڧܧ� ��ѧۧާ֧�� TIM6.
   */
	uint8_t i,j;
  if (TIM_GetITStatus(TIM4, TIM_IT_Update) != RESET)
  {
    /* ����ڧ�ѧ֧� �ҧڧ� ��ҧ�ѧҧѧ��ӧѧ֧ާ�ԧ� ���֧��ӧѧߧڧ� */
    TIM_ClearITPendingBit(TIM4, TIM_IT_Update);

    //cur_ocr=servos._ocr_action[servo_state];
    if (servo_state!=0)                // ����ݧ� �ߧ� �ߧ�ݧ֧ӧ�� �������ߧڧ� ���
    {
      TIM4->ARR=servos._ocr_action[servo_state]-servos._ocr_action[servo_state-1];
      for(i=0;i<USED_PORTS;i++)
        {
          GPIO_ResetBits(servos._port_action[i],servos._pin_action[i][servo_state]);
        }
        //GPIO_SetBits(BLINK_PORT, (1 << BLINK_PIN));
      if (servos._ocr_action[servo_state] == OCR_MAX)                // ����ݧ� �٧ߧѧ�֧ߧڧ� ���ѧӧߧ֧ߧڧ� ��ѧӧߧ� FF �٧ߧѧ�ڧ� ���� �٧ѧԧݧ��ܧ�
      {          // �� �ާ� �է���ڧԧݧ� �ܧ�ߧ�� ��ѧҧݧڧ��. �� ����� ��ҧߧ�ݧڧ�� �ѧӧ��ާѧ�
        servo_state = 0;            // ������ѧӧݧ�֧� �ߧ�ݧ֧ӧ�� �������ߧڧ�.

        //if (servos._servo_need_update==1)		// ����ݧ� �������ڧ� ���ڧܧѧ� ��ҧߧ�ӧڧ�� ��ѧҧݧڧ�� �ѧӧ��ާѧ��
        if (1)
        {
          ServoArrayNextStep(&servos,100);
          ServoArraySort(&servos);
          ServoArrayUpdate(&servos);
          servos._servo_need_update=0;
        }
      }else servo_state++;
    }
    else						// ����ݧ֧ӧ�� �������ߧڧ� �ѧӧ��ާѧ��. ����ӧ�� ��ڧܧ�
    {
      //cur_ocr=servos._ocr_action[servo_state];
      TIM4->ARR=servos._ocr_action[servo_state];		// ���֧�֧� ��֧�ӧ�� �ӧ�է֧�اܧ�.
      for(i=0;i<USED_PORTS;i++)
        for(j=1;j<MAX_SERVO+1;j++)
        {
          GPIO_SetBits(servos._port_action[i],servos._pin_action[i][j]);
        }				// ������ѧӧ�ݧ֧� �ӧ�� ��֧�ӧ�ܧѧߧѧݧ� �� 1 - �ߧѧ�ѧݧ� �ڧާ��ݧ���
      //GPIO_SetBits(servos._port_action[0],servos._pin_action[0][1]);
      //GPIO_SetBits(servos._port_action[0],servos._pin_action[0][2]);
      servo_state++;				// ���ӧ֧ݧڧ�ڧӧѧ֧� �������ߧڧ� �ܧ�ߧ֧�ߧ�ԧ� �ѧӧ��ާѧ��.
      //GPIO_ResetBits(BLINK_PORT, (1 << BLINK_PIN));
    }
  }
}


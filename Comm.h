#define CH1 23
#define CH2 22
#define CH3 17
#define CH4 16
#define CH5 24
#define CH6 26
#define CH7 27
#define CH8 28

#define LED0 13
#define LED1 31
#define LED2 33

void InitComm();
void intChannel1();
void intChannel2();
void intChannel3();
void intChannel4();
void intChannel5();
void intChannel6();
void RC_refine();
short remote_An[7];
unsigned long timer_1, timer_2, timer_3, timer_4, timer_5, timer_6;
short  timer_channel_1, timer_channel_2, timer_channel_3, timer_channel_4, esc_timer, esc_loop_timer;
bool last_channel_1, last_channel_2, last_channel_3, last_channel_4, last_channel_5, last_channel_6;
short  receiver_input_channel_[7];



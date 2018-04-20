void InitComm()
{
  Serial.begin(500000);
  Serial1.begin(115200);
  pinMode(LED0, OUTPUT);
  pinMode(LED2, OUTPUT);
  digitalWrite(LED2, LOW);
  digitalWrite(LED0, HIGH);
  delay(5);
  pinMode(CH1, INPUT);
  pinMode(CH2, INPUT);
  pinMode(CH3, INPUT);
  pinMode(CH4, INPUT);
  pinMode(CH5, INPUT);
  pinMode(CH6, INPUT);
  attachInterrupt(CH1, intChannel1, CHANGE);
  attachInterrupt(CH2, intChannel2, CHANGE);
  attachInterrupt(CH3, intChannel3, CHANGE);
  attachInterrupt(CH4, intChannel4, CHANGE);
  attachInterrupt(CH5, intChannel5, CHANGE);
  attachInterrupt(CH6, intChannel6, CHANGE);
  delay(50);
  remote_An[1] = 9375;
  remote_An[2] = 9375;
  remote_An[3] = 5900;
  remote_An[4] = 9375;
  remote_An[5] = 6250;
  remote_An[6] = 12500;
}
void RC_refine()
{
  for (uint8_t i = 1; i < 7; i++)
  {
    remote_An[i] -= remote_An[i] / 5;
    remote_An[i] += receiver_input_channel_[i];
    receiver_input_channel_[i] = remote_An[i] / 5;
  }
  Copter.RCsignal.ROLL = receiver_input_channel_[1];
  Copter.RCsignal.PITCH = receiver_input_channel_[2];
  Copter.RCsignal.THROTTLE = receiver_input_channel_[3];
  Copter.RCsignal.YAW = receiver_input_channel_[4];
  Copter.RCsignal.MODE = receiver_input_channel_[5];
  Copter.RCsignal.SWITCH = receiver_input_channel_[6];
}
void intChannel1()
{
  if (last_channel_1 == 0 && digitalReadFast(CH1) == HIGH)
  {
    last_channel_1 = 1;
    timer_1 = micros();
  }
  if (last_channel_1 == 1 && digitalReadFast(CH1) == LOW)
  {
    last_channel_1 = 0;
    receiver_input_channel_[1] = micros() - timer_1;
  }
}

void intChannel2()
{
  if (last_channel_2 == 0 && digitalReadFast(CH2) == HIGH)
  {
    last_channel_2 = 1;
    timer_2 = micros();
  }
  if (last_channel_2 == 1 && digitalReadFast(CH2) == LOW)
  {
    last_channel_2 = 0;
    receiver_input_channel_[2] = micros() - timer_2;
  }
}

void intChannel3()
{
  if (last_channel_3 == 0 && digitalReadFast(CH3) == HIGH)
  {
    last_channel_3 = 1;
    timer_3 = micros();
  }
  if (last_channel_3 == 1 && digitalReadFast(CH3) == LOW)
  {
    last_channel_3 = 0;
    receiver_input_channel_[3] = micros() - timer_3;
  }
}

void intChannel4()
{
  if (last_channel_4 == 0 && digitalReadFast(CH4) == HIGH)
  {
    last_channel_4 = 1;
    timer_4 = micros();
  }
  if (last_channel_4 == 1 && digitalReadFast(CH4) == LOW)
  {
    last_channel_4 = 0;
    receiver_input_channel_[4] = micros() - timer_4;
  }
}

void intChannel5()
{
  if (last_channel_5 == 0 && digitalReadFast(CH5) == HIGH)
  {
    last_channel_5 = 1;
    timer_5 = micros();
  }
  if (last_channel_5 == 1 && digitalReadFast(CH5) == LOW)
  {
    last_channel_5 = 0;
    receiver_input_channel_[5] = micros() - timer_5;
  }
}

void intChannel6()
{
  if (last_channel_6 == 0 && digitalReadFast(CH6) == HIGH)
  {
    last_channel_6 = 1;
    timer_6 = micros();
  }
  if (last_channel_6 == 1 && digitalReadFast(CH6) == LOW)
  {
    last_channel_6 = 0;
    receiver_input_channel_[6] = micros() - timer_6;
  }
  if (receiver_input_channel_[6] > 1700)
  {
    Copter.Motor_stop();
  }
}


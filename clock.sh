#!/bin/bash
#pwmFrequency in Hz = 19.2e6 Hz /pwmClock /pwmRange
#example 800Hz = 19.2e6 Hz / 240 /100
gpio mode 23 pwm
gpio pwm-ms
gpio pwmc 240
gpio pwmr 100 # 1.25ms / 200 = 6.25 us per unit
gpio pwm 23 50 #duty num

EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title ""
Date ""
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
Wire Wire Line
	3050 1850 3050 1750
Wire Wire Line
	3050 1750 3150 1750
Wire Wire Line
	3150 1850 3150 1750
Connection ~ 3150 1750
Wire Wire Line
	3150 1750 3200 1750
$Comp
L power:+3.3V #PWR0109
U 1 1 5F1DFD71
P 3200 1600
F 0 "#PWR0109" H 3200 1450 50  0001 C CNN
F 1 "+3.3V" H 3215 1773 50  0000 C CNN
F 2 "" H 3200 1600 50  0001 C CNN
F 3 "" H 3200 1600 50  0001 C CNN
	1    3200 1600
	1    0    0    -1  
$EndComp
Wire Wire Line
	3200 1600 3200 1750
$Comp
L Device:C C2
U 1 1 5F1E5D99
P 1550 3250
F 0 "C2" H 1665 3296 50  0000 L CNN
F 1 "18pF" H 1665 3205 50  0000 L CNN
F 2 "Capacitor_SMD:C_0402_1005Metric" H 1588 3100 50  0001 C CNN
F 3 "~" H 1550 3250 50  0001 C CNN
	1    1550 3250
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0110
U 1 1 5F1E5D9F
P 1350 3550
F 0 "#PWR0110" H 1350 3300 50  0001 C CNN
F 1 "GND" H 1355 3377 50  0000 C CNN
F 2 "" H 1350 3550 50  0001 C CNN
F 3 "" H 1350 3550 50  0001 C CNN
	1    1350 3550
	1    0    0    -1  
$EndComp
Wire Wire Line
	1150 3400 1150 3550
Wire Wire Line
	1150 3550 1350 3550
Wire Wire Line
	1350 3550 1550 3550
Wire Wire Line
	1550 3550 1550 3400
Connection ~ 1350 3550
$Comp
L Device:Crystal_GND24 Y1
U 1 1 5F1E5D85
P 1350 2650
F 0 "Y1" V 1304 2894 50  0000 L CNN
F 1 "Crystal_GND24" V 1395 2894 50  0000 L CNN
F 2 "Crystal:Crystal_SMD_3225-4Pin_3.2x2.5mm" H 1350 2650 50  0001 C CNN
F 3 "~" H 1350 2650 50  0001 C CNN
	1    1350 2650
	0    1    1    0   
$EndComp
Wire Wire Line
	1550 2650 1550 3100
Wire Wire Line
	1150 2650 1150 3100
Wire Wire Line
	2350 2750 2650 2750
Wire Wire Line
	2650 2850 2300 2850
Wire Wire Line
	1350 2500 1650 2500
Wire Wire Line
	1800 2450 1800 2700
Wire Wire Line
	1800 2700 2350 2700
Wire Wire Line
	2350 2700 2350 2750
$Comp
L Device:LED D1
U 1 1 5F24A4FD
P 6000 6350
F 0 "D1" V 6039 6232 50  0000 R CNN
F 1 "LED" V 5948 6232 50  0000 R CNN
F 2 "LED_SMD:LED_0603_1608Metric" H 6000 6350 50  0001 C CNN
F 3 "~" H 6000 6350 50  0001 C CNN
	1    6000 6350
	0    -1   -1   0   
$EndComp
$Comp
L Device:R R3
U 1 1 5F24B22B
P 6000 6050
F 0 "R3" V 5793 6050 50  0000 C CNN
F 1 "R" V 5884 6050 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric_Pad1.05x0.95mm_HandSolder" V 5930 6050 50  0001 C CNN
F 3 "~" H 6000 6050 50  0001 C CNN
	1    6000 6050
	-1   0    0    1   
$EndComp
$Comp
L power:GND #PWR0113
U 1 1 5F24F516
P 6000 6500
F 0 "#PWR0113" H 6000 6250 50  0001 C CNN
F 1 "GND" H 6005 6327 50  0000 C CNN
F 2 "" H 6000 6500 50  0001 C CNN
F 3 "" H 6000 6500 50  0001 C CNN
	1    6000 6500
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR0114
U 1 1 5F24FD13
P 6000 5900
F 0 "#PWR0114" H 6000 5750 50  0001 C CNN
F 1 "+3.3V" H 6015 6073 50  0000 C CNN
F 2 "" H 6000 5900 50  0001 C CNN
F 3 "" H 6000 5900 50  0001 C CNN
	1    6000 5900
	1    0    0    -1  
$EndComp
$Comp
L Device:C C7
U 1 1 5F254B2B
P 5050 6200
F 0 "C7" H 5165 6246 50  0000 L CNN
F 1 "C" H 5165 6155 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric_Pad1.05x0.95mm_HandSolder" H 5088 6050 50  0001 C CNN
F 3 "~" H 5050 6200 50  0001 C CNN
	1    5050 6200
	1    0    0    -1  
$EndComp
$Comp
L Device:C C5
U 1 1 5F25537C
P 4200 6200
F 0 "C5" H 4315 6246 50  0000 L CNN
F 1 "C" H 4315 6155 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric_Pad1.05x0.95mm_HandSolder" H 4238 6050 50  0001 C CNN
F 3 "~" H 4200 6200 50  0001 C CNN
	1    4200 6200
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0115
U 1 1 5F255BE0
P 4650 6450
F 0 "#PWR0115" H 4650 6200 50  0001 C CNN
F 1 "GND" H 4655 6277 50  0000 C CNN
F 2 "" H 4650 6450 50  0001 C CNN
F 3 "" H 4650 6450 50  0001 C CNN
	1    4650 6450
	1    0    0    -1  
$EndComp
Wire Wire Line
	4200 6350 4650 6350
Connection ~ 4650 6350
Wire Wire Line
	4650 6350 5050 6350
Wire Wire Line
	5050 6050 4950 6050
Wire Wire Line
	4350 6050 4200 6050
$Comp
L power:+3.3V #PWR0117
U 1 1 5F262A55
P 5450 6050
F 0 "#PWR0117" H 5450 5900 50  0001 C CNN
F 1 "+3.3V" H 5465 6223 50  0000 C CNN
F 2 "" H 5450 6050 50  0001 C CNN
F 3 "" H 5450 6050 50  0001 C CNN
	1    5450 6050
	1    0    0    -1  
$EndComp
$Comp
L Device:CP C4
U 1 1 5F26360B
P 3850 6200
F 0 "C4" H 3968 6246 50  0000 L CNN
F 1 "CP" H 3968 6155 50  0000 L CNN
F 2 "Capacitor_SMD:C_1206_3216Metric" H 3888 6050 50  0001 C CNN
F 3 "~" H 3850 6200 50  0001 C CNN
	1    3850 6200
	1    0    0    -1  
$EndComp
$Comp
L Device:CP C9
U 1 1 5F26443B
P 5450 6200
F 0 "C9" H 5568 6246 50  0000 L CNN
F 1 "CP" H 5568 6155 50  0000 L CNN
F 2 "Capacitor_SMD:C_1206_3216Metric" H 5488 6050 50  0001 C CNN
F 3 "~" H 5450 6200 50  0001 C CNN
	1    5450 6200
	1    0    0    -1  
$EndComp
Wire Wire Line
	3850 6050 4200 6050
Connection ~ 4200 6050
Wire Wire Line
	4200 6350 3850 6350
Connection ~ 4200 6350
Wire Wire Line
	5050 6050 5450 6050
Connection ~ 5050 6050
Wire Wire Line
	5450 6350 5050 6350
Connection ~ 5050 6350
Connection ~ 5450 6050
$Comp
L power:GND #PWR0129
U 1 1 5F45E150
P 3050 3650
F 0 "#PWR0129" H 3050 3400 50  0001 C CNN
F 1 "GND" H 3055 3477 50  0000 C CNN
F 2 "" H 3050 3650 50  0001 C CNN
F 3 "" H 3050 3650 50  0001 C CNN
	1    3050 3650
	1    0    0    -1  
$EndComp
Wire Wire Line
	3750 1150 3750 1200
Wire Wire Line
	3400 1150 3400 1200
Wire Wire Line
	3400 1200 3750 1200
Connection ~ 3750 1200
Wire Wire Line
	3750 1200 3950 1200
$Comp
L power:GND #PWR0130
U 1 1 5F488B10
P 3950 1200
F 0 "#PWR0130" H 3950 950 50  0001 C CNN
F 1 "GND" H 3955 1027 50  0000 C CNN
F 2 "" H 3950 1200 50  0001 C CNN
F 3 "" H 3950 1200 50  0001 C CNN
	1    3950 1200
	1    0    0    -1  
$EndComp
Connection ~ 3950 1200
$Comp
L power:+3.3V #PWR0131
U 1 1 5F4895A0
P 3950 850
F 0 "#PWR0131" H 3950 700 50  0001 C CNN
F 1 "+3.3V" H 3965 1023 50  0000 C CNN
F 2 "" H 3950 850 50  0001 C CNN
F 3 "" H 3950 850 50  0001 C CNN
	1    3950 850 
	1    0    0    -1  
$EndComp
Connection ~ 3950 850 
$Comp
L Switch:SW_Push SW1
U 1 1 5F4A3EC7
P 8600 3300
F 0 "SW1" H 8600 3585 50  0000 C CNN
F 1 "SW_Push" H 8600 3494 50  0000 C CNN
F 2 "Button_Switch_SMD:SW_Push_1P1T_NO_6x6mm_H9.5mm" H 8600 3500 50  0001 C CNN
F 3 "~" H 8600 3500 50  0001 C CNN
	1    8600 3300
	1    0    0    -1  
$EndComp
$Comp
L Device:C C3
U 1 1 5F4AD302
P 8800 3550
F 0 "C3" H 8915 3596 50  0000 L CNN
F 1 "C" H 8915 3505 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric_Pad1.05x0.95mm_HandSolder" H 8838 3400 50  0001 C CNN
F 3 "~" H 8800 3550 50  0001 C CNN
	1    8800 3550
	1    0    0    -1  
$EndComp
$Comp
L Device:R R2
U 1 1 5F4AE03A
P 8800 3050
F 0 "R2" H 8870 3096 50  0000 L CNN
F 1 "R" H 8870 3005 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric_Pad1.05x0.95mm_HandSolder" V 8730 3050 50  0001 C CNN
F 3 "~" H 8800 3050 50  0001 C CNN
	1    8800 3050
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0132
U 1 1 5F4AE9D9
P 8800 3800
F 0 "#PWR0132" H 8800 3550 50  0001 C CNN
F 1 "GND" H 8805 3627 50  0000 C CNN
F 2 "" H 8800 3800 50  0001 C CNN
F 3 "" H 8800 3800 50  0001 C CNN
	1    8800 3800
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR0133
U 1 1 5F4AF34F
P 8800 2800
F 0 "#PWR0133" H 8800 2650 50  0001 C CNN
F 1 "+3.3V" H 8815 2973 50  0000 C CNN
F 2 "" H 8800 2800 50  0001 C CNN
F 3 "" H 8800 2800 50  0001 C CNN
	1    8800 2800
	1    0    0    -1  
$EndComp
Wire Wire Line
	8800 3700 8800 3750
Wire Wire Line
	8800 3400 8800 3300
Connection ~ 8800 3300
Wire Wire Line
	8800 3300 8800 3200
Wire Wire Line
	8800 2800 8800 2900
Wire Wire Line
	8400 3300 8400 3750
Wire Wire Line
	8400 3750 8800 3750
Connection ~ 8800 3750
Wire Wire Line
	8800 3750 8800 3800
Text GLabel 9050 3300 2    50   Input ~ 0
NRST
Wire Wire Line
	8800 3300 9050 3300
Text GLabel 2400 2050 0    50   Input ~ 0
NRST
Wire Wire Line
	2650 2050 2400 2050
Wire Wire Line
	4650 6350 4650 6450
Text GLabel 4000 3150 2    50   Input ~ 0
SWCLK
Wire Wire Line
	4000 3150 3850 3150
Wire Wire Line
	3850 3050 4000 3050
Text GLabel 4000 3050 2    50   Input ~ 0
SWDIO
$Comp
L Connector:Conn_01x04_Female J5
U 1 1 5F310E28
P 4700 5250
F 0 "J5" H 4728 5226 50  0000 L CNN
F 1 "Conn_01x04_Female" H 4728 5135 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x04_P2.54mm_Vertical" H 4700 5250 50  0001 C CNN
F 3 "~" H 4700 5250 50  0001 C CNN
	1    4700 5250
	1    0    0    -1  
$EndComp
Text GLabel 4350 5250 0    50   Input ~ 0
SWCLK
Wire Wire Line
	4350 5250 4500 5250
Wire Wire Line
	4500 5350 4350 5350
Text GLabel 4350 5350 0    50   Input ~ 0
SWDIO
$Comp
L power:+3.3V #PWR0137
U 1 1 5F33BFFB
P 4400 5050
F 0 "#PWR0137" H 4400 4900 50  0001 C CNN
F 1 "+3.3V" H 4415 5223 50  0000 C CNN
F 2 "" H 4400 5050 50  0001 C CNN
F 3 "" H 4400 5050 50  0001 C CNN
	1    4400 5050
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0138
U 1 1 5F33C780
P 4350 5550
F 0 "#PWR0138" H 4350 5300 50  0001 C CNN
F 1 "GND" H 4355 5377 50  0000 C CNN
F 2 "" H 4350 5550 50  0001 C CNN
F 3 "" H 4350 5550 50  0001 C CNN
	1    4350 5550
	1    0    0    -1  
$EndComp
Wire Wire Line
	4400 5050 4400 5150
Wire Wire Line
	4400 5150 4500 5150
Wire Wire Line
	4500 5450 4350 5450
Wire Wire Line
	4350 5450 4350 5550
$Comp
L Device:C C1
U 1 1 5F1E5D93
P 1150 3250
F 0 "C1" H 1265 3296 50  0000 L CNN
F 1 "18pF" H 1265 3205 50  0000 L CNN
F 2 "Capacitor_SMD:C_0402_1005Metric" H 1188 3100 50  0001 C CNN
F 3 "~" H 1150 3250 50  0001 C CNN
	1    1150 3250
	1    0    0    -1  
$EndComp
Wire Wire Line
	1800 2450 1650 2450
Wire Wire Line
	1650 2450 1650 2500
Wire Wire Line
	2300 2800 2300 2850
Wire Wire Line
	1350 2800 2300 2800
$Comp
L Device:C C15
U 1 1 5F3FC447
P 4900 1000
F 0 "C15" H 5015 1046 50  0000 L CNN
F 1 "1u" H 5015 955 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric_Pad1.05x0.95mm_HandSolder" H 4938 850 50  0001 C CNN
F 3 "~" H 4900 1000 50  0001 C CNN
	1    4900 1000
	1    0    0    -1  
$EndComp
Wire Wire Line
	4900 1150 4900 1200
$Comp
L Regulator_Linear:AMS1117-3.3 U1
U 1 1 5F494CDA
P 4650 6050
F 0 "U1" H 4650 6292 50  0000 C CNN
F 1 "AMS1117-3.3" H 4650 6201 50  0000 C CNN
F 2 "Package_TO_SOT_SMD:SOT-223-3_TabPin2" H 4650 6250 50  0001 C CNN
F 3 "http://www.advanced-monolithic.com/pdf/ds1117.pdf" H 4750 5800 50  0001 C CNN
	1    4650 6050
	1    0    0    -1  
$EndComp
Text GLabel 4050 2350 2    50   Input ~ 0
Button3
Text GLabel 4050 2250 2    50   Input ~ 0
Button2
Text GLabel 4050 2150 2    50   Input ~ 0
Button1
Wire Wire Line
	4050 2350 3850 2350
Wire Wire Line
	3850 2250 4050 2250
Wire Wire Line
	3850 2150 4050 2150
$Comp
L Switch:SW_Push SW2
U 1 1 5F777D71
P 8700 1400
F 0 "SW2" H 8700 1685 50  0000 C CNN
F 1 "SW_Push" H 8700 1594 50  0000 C CNN
F 2 "Button_Switch_SMD:SW_Push_1P1T_NO_6x6mm_H9.5mm" H 8700 1600 50  0001 C CNN
F 3 "~" H 8700 1600 50  0001 C CNN
	1    8700 1400
	1    0    0    -1  
$EndComp
$Comp
L Device:C C14
U 1 1 5F777D77
P 8900 1650
F 0 "C14" H 9015 1696 50  0000 L CNN
F 1 "C" H 9015 1605 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric_Pad1.05x0.95mm_HandSolder" H 8938 1500 50  0001 C CNN
F 3 "~" H 8900 1650 50  0001 C CNN
	1    8900 1650
	1    0    0    -1  
$EndComp
$Comp
L Device:R R10
U 1 1 5F777D7D
P 8900 1150
F 0 "R10" H 8970 1196 50  0000 L CNN
F 1 "R" H 8970 1105 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric_Pad1.05x0.95mm_HandSolder" V 8830 1150 50  0001 C CNN
F 3 "~" H 8900 1150 50  0001 C CNN
	1    8900 1150
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0119
U 1 1 5F777D83
P 8900 1900
F 0 "#PWR0119" H 8900 1650 50  0001 C CNN
F 1 "GND" H 8905 1727 50  0000 C CNN
F 2 "" H 8900 1900 50  0001 C CNN
F 3 "" H 8900 1900 50  0001 C CNN
	1    8900 1900
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR0120
U 1 1 5F777D89
P 8900 900
F 0 "#PWR0120" H 8900 750 50  0001 C CNN
F 1 "+3.3V" H 8915 1073 50  0000 C CNN
F 2 "" H 8900 900 50  0001 C CNN
F 3 "" H 8900 900 50  0001 C CNN
	1    8900 900 
	1    0    0    -1  
$EndComp
Wire Wire Line
	8900 1800 8900 1850
Wire Wire Line
	8900 1500 8900 1400
Connection ~ 8900 1400
Wire Wire Line
	8900 1400 8900 1300
Wire Wire Line
	8900 900  8900 1000
Wire Wire Line
	8500 1400 8500 1850
Wire Wire Line
	8500 1850 8900 1850
Connection ~ 8900 1850
Wire Wire Line
	8900 1850 8900 1900
$Comp
L power:GND #PWR0147
U 1 1 5F8D6F84
P 5500 7950
F 0 "#PWR0147" H 5500 7700 50  0001 C CNN
F 1 "GND" H 5505 7777 50  0000 C CNN
F 2 "" H 5500 7950 50  0001 C CNN
F 3 "" H 5500 7950 50  0001 C CNN
	1    5500 7950
	1    0    0    -1  
$EndComp
$Comp
L Device:C C6
U 1 1 5F47143F
P 3400 1000
F 0 "C6" H 3515 1046 50  0000 L CNN
F 1 "100n" H 3515 955 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric_Pad1.05x0.95mm_HandSolder" H 3438 850 50  0001 C CNN
F 3 "~" H 3400 1000 50  0001 C CNN
	1    3400 1000
	1    0    0    -1  
$EndComp
Wire Wire Line
	3750 850  3950 850 
Wire Wire Line
	3400 850  3750 850 
Connection ~ 3750 850 
$Comp
L Device:C C8
U 1 1 5F471439
P 3750 1000
F 0 "C8" H 3865 1046 50  0000 L CNN
F 1 "100n" H 3865 955 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric_Pad1.05x0.95mm_HandSolder" H 3788 850 50  0001 C CNN
F 3 "~" H 3750 1000 50  0001 C CNN
	1    3750 1000
	1    0    0    -1  
$EndComp
Wire Wire Line
	2700 5150 2950 5150
Connection ~ 2700 5150
Wire Wire Line
	2400 5150 2700 5150
Wire Wire Line
	2750 5250 2950 5250
Wire Wire Line
	1850 5250 2450 5250
$Comp
L Device:R R5
U 1 1 5F5095DE
P 2600 5250
F 0 "R5" H 2670 5296 50  0000 L CNN
F 1 "22R" H 2670 5205 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 2530 5250 50  0001 C CNN
F 3 "~" H 2600 5250 50  0001 C CNN
	1    2600 5250
	0    1    1    0   
$EndComp
$Comp
L Device:R R4
U 1 1 5F502DD6
P 2250 5150
F 0 "R4" H 2320 5196 50  0000 L CNN
F 1 "22R" H 2320 5105 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 2180 5150 50  0001 C CNN
F 3 "~" H 2250 5150 50  0001 C CNN
	1    2250 5150
	0    1    1    0   
$EndComp
Text GLabel 2950 5150 2    50   Input ~ 0
USB_D+
Wire Wire Line
	2700 5150 2700 5000
$Comp
L power:+3.3V #PWR0124
U 1 1 5F99EA10
P 2700 4450
F 0 "#PWR0124" H 2700 4300 50  0001 C CNN
F 1 "+3.3V" H 2715 4623 50  0000 C CNN
F 2 "" H 2700 4450 50  0001 C CNN
F 3 "" H 2700 4450 50  0001 C CNN
	1    2700 4450
	1    0    0    -1  
$EndComp
Wire Wire Line
	2700 4700 2700 4450
$Comp
L Device:R R1
U 1 1 5F97C2D0
P 2700 4850
F 0 "R1" H 2770 4896 50  0000 L CNN
F 1 "1.5k" H 2770 4805 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 2630 4850 50  0001 C CNN
F 3 "~" H 2700 4850 50  0001 C CNN
	1    2700 4850
	-1   0    0    1   
$EndComp
Text GLabel 2950 5250 2    50   Input ~ 0
USB_D-
$Comp
L power:GND #PWR0112
U 1 1 5F2480AE
P 1550 5550
F 0 "#PWR0112" H 1550 5300 50  0001 C CNN
F 1 "GND" H 1555 5377 50  0000 C CNN
F 2 "" H 1550 5550 50  0001 C CNN
F 3 "" H 1550 5550 50  0001 C CNN
	1    1550 5550
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR0111
U 1 1 5F247937
P 2050 4750
F 0 "#PWR0111" H 2050 4600 50  0001 C CNN
F 1 "+5V" H 2065 4923 50  0000 C CNN
F 2 "" H 2050 4750 50  0001 C CNN
F 3 "" H 2050 4750 50  0001 C CNN
	1    2050 4750
	1    0    0    -1  
$EndComp
Wire Wire Line
	2050 4950 2050 4750
Wire Wire Line
	1850 4950 2050 4950
Wire Wire Line
	1850 5150 2100 5150
$Comp
L Connector:USB_B_Micro J1
U 1 1 5F23AAA0
P 1550 5150
F 0 "J1" H 1607 5617 50  0000 C CNN
F 1 "USB_B_Micro" H 1607 5526 50  0000 C CNN
F 2 "Connector_USB:USB_Micro-B_Molex-105017-0001" H 1700 5100 50  0001 C CNN
F 3 "~" H 1700 5100 50  0001 C CNN
	1    1550 5150
	1    0    0    -1  
$EndComp
$Comp
L MCU_ST_STM32F0:STM32F042F6Px U2
U 1 1 5F65B582
P 3250 2550
F 0 "U2" H 3250 1661 50  0000 C CNN
F 1 "STM32F042F6Px" H 3250 1570 50  0000 C CNN
F 2 "Package_SO:TSSOP-20_4.4x6.5mm_P0.65mm" H 2750 1850 50  0001 R CNN
F 3 "http://www.st.com/st-web-ui/static/active/en/resource/technical/document/datasheet/DM00105814.pdf" H 3250 2550 50  0001 C CNN
	1    3250 2550
	1    0    0    -1  
$EndComp
Wire Wire Line
	3950 1200 4150 1200
Connection ~ 4150 1200
Wire Wire Line
	4150 1150 4150 1200
Wire Wire Line
	3950 850  4150 850 
Connection ~ 4150 850 
$Comp
L Device:C C10
U 1 1 5F478AF1
P 4150 1000
F 0 "C10" H 4265 1046 50  0000 L CNN
F 1 "100n" H 4265 955 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric_Pad1.05x0.95mm_HandSolder" H 4188 850 50  0001 C CNN
F 3 "~" H 4150 1000 50  0001 C CNN
	1    4150 1000
	1    0    0    -1  
$EndComp
Wire Wire Line
	4150 1200 4900 1200
Wire Wire Line
	4150 850  4900 850 
Text GLabel 4050 2950 2    50   Input ~ 0
USB_D+
Text GLabel 4050 2850 2    50   Input ~ 0
USB_D-
Wire Wire Line
	3850 2850 4050 2850
Wire Wire Line
	3850 2950 4050 2950
Wire Wire Line
	3050 3350 3050 3650
Text GLabel 4050 2750 2    50   Input ~ 0
Button6
Text GLabel 4050 2650 2    50   Input ~ 0
Button5
Text GLabel 4050 2550 2    50   Input ~ 0
Button4
Wire Wire Line
	4050 2750 3850 2750
Wire Wire Line
	3850 2650 4050 2650
Wire Wire Line
	3850 2550 4050 2550
Text GLabel 2450 3050 0    50   Input ~ 0
Button7
Wire Wire Line
	2650 3050 2450 3050
Text GLabel 2450 3150 0    50   Input ~ 0
LED
Wire Wire Line
	2650 3150 2450 3150
Text GLabel 4050 2050 2    50   Input ~ 0
Sampling_cap1
Wire Wire Line
	3850 2050 4050 2050
Text GLabel 4050 2450 2    50   Input ~ 0
Sampling_cap2
Wire Wire Line
	3850 2450 4050 2450
$Comp
L Device:LED D2
U 1 1 5F8059A3
P 6800 6300
F 0 "D2" V 6839 6182 50  0000 R CNN
F 1 "LED" V 6748 6182 50  0000 R CNN
F 2 "LED_SMD:LED_0603_1608Metric" H 6800 6300 50  0001 C CNN
F 3 "~" H 6800 6300 50  0001 C CNN
	1    6800 6300
	0    -1   -1   0   
$EndComp
$Comp
L Device:R R6
U 1 1 5F8059A9
P 6800 6000
F 0 "R6" V 6593 6000 50  0000 C CNN
F 1 "R" V 6684 6000 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric_Pad1.05x0.95mm_HandSolder" V 6730 6000 50  0001 C CNN
F 3 "~" H 6800 6000 50  0001 C CNN
	1    6800 6000
	-1   0    0    1   
$EndComp
$Comp
L power:GND #PWR0101
U 1 1 5F8059AF
P 6800 6450
F 0 "#PWR0101" H 6800 6200 50  0001 C CNN
F 1 "GND" H 6805 6277 50  0000 C CNN
F 2 "" H 6800 6450 50  0001 C CNN
F 3 "" H 6800 6450 50  0001 C CNN
	1    6800 6450
	1    0    0    -1  
$EndComp
Text GLabel 6800 5650 1    50   Input ~ 0
LED
Wire Wire Line
	6800 5850 6800 5650
$Comp
L Switch:SW_Push SW5
U 1 1 5F81568E
P 5700 2550
F 0 "SW5" H 5700 2835 50  0000 C CNN
F 1 "SW_Push" H 5700 2744 50  0000 C CNN
F 2 "touch_keypad:cap_button" H 5700 2750 50  0001 C CNN
F 3 "~" H 5700 2750 50  0001 C CNN
	1    5700 2550
	1    0    0    -1  
$EndComp
$Comp
L Switch:SW_Push SW3
U 1 1 5F823933
P 5650 3200
F 0 "SW3" H 5650 3485 50  0000 C CNN
F 1 "SW_Push" H 5650 3394 50  0000 C CNN
F 2 "touch_keypad:cap_button" H 5650 3400 50  0001 C CNN
F 3 "~" H 5650 3400 50  0001 C CNN
	1    5650 3200
	1    0    0    -1  
$EndComp
$Comp
L Switch:SW_Push SW4
U 1 1 5F82716F
P 5550 4050
F 0 "SW4" H 5550 4335 50  0000 C CNN
F 1 "SW_Push" H 5550 4244 50  0000 C CNN
F 2 "touch_keypad:cap_button" H 5550 4250 50  0001 C CNN
F 3 "~" H 5550 4250 50  0001 C CNN
	1    5550 4050
	1    0    0    -1  
$EndComp
$Comp
L Switch:SW_Push SW6
U 1 1 5F82A899
P 6400 2250
F 0 "SW6" H 6400 2535 50  0000 C CNN
F 1 "SW_Push" H 6400 2444 50  0000 C CNN
F 2 "touch_keypad:cap_button" H 6400 2450 50  0001 C CNN
F 3 "~" H 6400 2450 50  0001 C CNN
	1    6400 2250
	1    0    0    -1  
$EndComp
$Comp
L Switch:SW_Push SW11
U 1 1 5F83265E
P 7050 2250
F 0 "SW11" H 7050 2535 50  0000 C CNN
F 1 "SW_Push" H 7050 2444 50  0000 C CNN
F 2 "touch_keypad:cap_button" H 7050 2450 50  0001 C CNN
F 3 "~" H 7050 2450 50  0001 C CNN
	1    7050 2250
	1    0    0    -1  
$EndComp
$Comp
L Switch:SW_Push SW14
U 1 1 5F83ADB0
P 7700 2250
F 0 "SW14" H 7700 2535 50  0000 C CNN
F 1 "SW_Push" H 7700 2444 50  0000 C CNN
F 2 "touch_keypad:cap_button" H 7700 2450 50  0001 C CNN
F 3 "~" H 7700 2450 50  0001 C CNN
	1    7700 2250
	1    0    0    -1  
$EndComp
$Comp
L Switch:SW_Push SW10
U 1 1 5F842FF4
P 7000 2750
F 0 "SW10" H 7000 3035 50  0000 C CNN
F 1 "SW_Push" H 7000 2944 50  0000 C CNN
F 2 "touch_keypad:TouchSlider-2_10x10mm" H 7000 2950 50  0001 C CNN
F 3 "~" H 7000 2950 50  0001 C CNN
	1    7000 2750
	1    0    0    -1  
$EndComp
$Comp
L Switch:SW_Push SW15
U 1 1 5F84686E
P 7700 2750
F 0 "SW15" H 7700 3035 50  0000 C CNN
F 1 "SW_Push" H 7700 2944 50  0000 C CNN
F 2 "touch_keypad:TouchSlider-2_10x10mm" H 7700 2950 50  0001 C CNN
F 3 "~" H 7700 2950 50  0001 C CNN
	1    7700 2750
	1    0    0    -1  
$EndComp
$Comp
L Switch:SW_Push SW16
U 1 1 5F84A071
P 7700 3300
F 0 "SW16" H 7700 3585 50  0000 C CNN
F 1 "SW_Push" H 7700 3494 50  0000 C CNN
F 2 "touch_keypad:TouchSlider-2_10x10mm" H 7700 3500 50  0001 C CNN
F 3 "~" H 7700 3500 50  0001 C CNN
	1    7700 3300
	1    0    0    -1  
$EndComp
$Comp
L Switch:SW_Push SW12
U 1 1 5F84D730
P 7000 3350
F 0 "SW12" H 7000 3635 50  0000 C CNN
F 1 "SW_Push" H 7000 3544 50  0000 C CNN
F 2 "touch_keypad:TouchSlider-2_10x10mm" H 7000 3550 50  0001 C CNN
F 3 "~" H 7000 3550 50  0001 C CNN
	1    7000 3350
	1    0    0    -1  
$EndComp
$Comp
L Switch:SW_Push SW8
U 1 1 5F850E2B
P 6200 3400
F 0 "SW8" H 6200 3685 50  0000 C CNN
F 1 "SW_Push" H 6200 3594 50  0000 C CNN
F 2 "touch_keypad:TouchSlider-2_10x10mm" H 6200 3600 50  0001 C CNN
F 3 "~" H 6200 3600 50  0001 C CNN
	1    6200 3400
	1    0    0    -1  
$EndComp
$Comp
L Switch:SW_Push SW9
U 1 1 5F854695
P 5900 4300
F 0 "SW9" H 5900 4585 50  0000 C CNN
F 1 "SW_Push" H 5900 4494 50  0000 C CNN
F 2 "touch_keypad:TouchSlider-2_10x10mm" H 5900 4500 50  0001 C CNN
F 3 "~" H 5900 4500 50  0001 C CNN
	1    5900 4300
	1    0    0    -1  
$EndComp
$Comp
L Switch:SW_Push SW13
U 1 1 5F857CC8
P 6950 4200
F 0 "SW13" H 6950 4485 50  0000 C CNN
F 1 "SW_Push" H 6950 4394 50  0000 C CNN
F 2 "touch_keypad:TouchSlider-2_10x10mm" H 6950 4400 50  0001 C CNN
F 3 "~" H 6950 4400 50  0001 C CNN
	1    6950 4200
	1    0    0    -1  
$EndComp
$Comp
L Switch:SW_Push SW17
U 1 1 5F85B322
P 7800 4250
F 0 "SW17" H 7800 4535 50  0000 C CNN
F 1 "SW_Push" H 7800 4444 50  0000 C CNN
F 2 "touch_keypad:TouchSlider-2_10x10mm" H 7800 4450 50  0001 C CNN
F 3 "~" H 7800 4450 50  0001 C CNN
	1    7800 4250
	1    0    0    -1  
$EndComp
Text GLabel 5150 4050 0    50   Input ~ 0
Button3
Text GLabel 5250 3200 0    50   Input ~ 0
Button2
Text GLabel 5300 2550 0    50   Input ~ 0
Button1
Wire Wire Line
	5150 4050 5300 4050
Wire Wire Line
	5450 3200 5250 3200
Wire Wire Line
	5500 2550 5450 2550
Text GLabel 7500 1450 1    50   Input ~ 0
Button6
Text GLabel 6800 1500 1    50   Input ~ 0
Button5
Text GLabel 6200 1500 1    50   Input ~ 0
Button4
Wire Wire Line
	6800 1500 6800 2250
Wire Wire Line
	6800 2250 6850 2250
Wire Wire Line
	7500 1450 7500 2250
Wire Wire Line
	6200 1500 6200 2250
Text GLabel 9300 1400 2    50   Input ~ 0
Button7
Wire Wire Line
	8900 1400 9300 1400
$Comp
L Switch:SW_Push SW7
U 1 1 5F83E787
P 6200 2750
F 0 "SW7" H 6200 3035 50  0000 C CNN
F 1 "SW_Push" H 6200 2944 50  0000 C CNN
F 2 "touch_keypad:TouchSlider-2_10x10mm" H 6200 2950 50  0001 C CNN
F 3 "~" H 6200 2950 50  0001 C CNN
	1    6200 2750
	1    0    0    -1  
$EndComp
NoConn ~ 5900 2550
Wire Wire Line
	5450 2750 5450 2550
Connection ~ 5450 2550
Wire Wire Line
	5450 2550 5300 2550
Wire Wire Line
	5450 2750 5750 2750
Wire Wire Line
	6400 2750 6400 2400
Wire Wire Line
	6400 2400 6200 2400
Wire Wire Line
	6200 2400 6200 2250
Connection ~ 6200 2250
NoConn ~ 6600 2250
NoConn ~ 5850 3200
Wire Wire Line
	5450 3200 5450 3400
Wire Wire Line
	5450 3400 5750 3400
Connection ~ 5450 3200
Wire Wire Line
	6400 2750 6400 3400
Connection ~ 6400 2750
Wire Wire Line
	5300 4050 5300 4300
Wire Wire Line
	5300 4300 5500 4300
Connection ~ 5300 4050
Wire Wire Line
	5300 4050 5350 4050
Wire Wire Line
	6400 3400 6400 4300
Wire Wire Line
	6100 4300 6400 4300
Connection ~ 6400 3400
NoConn ~ 5750 4050
Wire Wire Line
	5750 2750 5750 2900
Wire Wire Line
	5750 2900 6750 2900
Wire Wire Line
	6750 2900 6750 2750
Wire Wire Line
	6750 2750 6800 2750
Connection ~ 5750 2750
Wire Wire Line
	5750 2750 6000 2750
Wire Wire Line
	6750 2900 7400 2900
Wire Wire Line
	7400 2900 7400 2750
Wire Wire Line
	7400 2750 7500 2750
Connection ~ 6750 2900
NoConn ~ 7250 2250
Wire Wire Line
	6800 2250 6800 2400
Wire Wire Line
	6800 2400 7200 2400
Connection ~ 6800 2250
Wire Wire Line
	7500 2250 7500 2400
Wire Wire Line
	7500 2400 7900 2400
Wire Wire Line
	7900 2400 7900 2750
Connection ~ 7500 2250
NoConn ~ 7900 2250
Wire Wire Line
	5750 3400 5750 3500
Wire Wire Line
	5750 3500 6800 3500
Wire Wire Line
	6800 3500 6800 3350
Connection ~ 5750 3400
Wire Wire Line
	5750 3400 6000 3400
Wire Wire Line
	7200 2400 7200 2750
Connection ~ 7200 2750
Wire Wire Line
	7200 2750 7200 3350
Wire Wire Line
	6800 3500 7450 3500
Wire Wire Line
	7450 3500 7450 3300
Wire Wire Line
	7450 3300 7500 3300
Connection ~ 6800 3500
Wire Wire Line
	7900 2750 7900 3300
Connection ~ 7900 2750
Wire Wire Line
	5500 4300 5500 4500
Wire Wire Line
	5500 4500 6750 4500
Wire Wire Line
	6750 4500 6750 4200
Connection ~ 5500 4300
Wire Wire Line
	5500 4300 5700 4300
Wire Wire Line
	7200 3350 7200 4200
Wire Wire Line
	7200 4200 7150 4200
Connection ~ 7200 3350
Wire Wire Line
	6750 4500 7600 4500
Wire Wire Line
	7600 4500 7600 4250
Connection ~ 6750 4500
Wire Wire Line
	7900 3300 7900 3800
Wire Wire Line
	7900 3800 8000 3800
Wire Wire Line
	8000 3800 8000 4250
Connection ~ 7900 3300
$Comp
L power:+5V #PWR?
U 1 1 5F5A334C
P 3850 6050
F 0 "#PWR?" H 3850 5900 50  0001 C CNN
F 1 "+5V" H 3865 6223 50  0000 C CNN
F 2 "" H 3850 6050 50  0001 C CNN
F 3 "" H 3850 6050 50  0001 C CNN
	1    3850 6050
	1    0    0    -1  
$EndComp
$EndSCHEMATC

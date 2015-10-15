EESchema Schematic File Version 2
LIBS:power
LIBS:device
LIBS:transistors
LIBS:conn
LIBS:linear
LIBS:regul
LIBS:74xx
LIBS:cmos4000
LIBS:adc-dac
LIBS:memory
LIBS:xilinx
LIBS:microcontrollers
LIBS:dsp
LIBS:microchip
LIBS:analog_switches
LIBS:motorola
LIBS:texas
LIBS:intel
LIBS:audio
LIBS:interface
LIBS:digital-audio
LIBS:philips
LIBS:display
LIBS:cypress
LIBS:siliconi
LIBS:opto
LIBS:atmel
LIBS:contrib
LIBS:valves
LIBS:BACONdriver
LIBS:BACONdriver-cache
EELAYER 25 0
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
$Comp
L CONN_01X04 P1
U 1 1 561F005D
P 900 1100
F 0 "P1" H 900 1350 50  0000 C CNN
F 1 "Batt_Mon" V 1000 1100 50  0000 C CNN
F 2 "" H 900 1100 60  0000 C CNN
F 3 "" H 900 1100 60  0000 C CNN
	1    900  1100
	-1   0    0    1   
$EndComp
Text Label 1150 1150 0    60   ~ 0
Cell1
Text Label 1150 1050 0    60   ~ 0
Cell2
Text Label 1150 950  0    60   ~ 0
Cell3
$Comp
L CONN_01X02 P2
U 1 1 561F01B2
P 900 2050
F 0 "P2" H 900 2200 50  0000 C CNN
F 1 "Batt" V 1000 2050 50  0000 C CNN
F 2 "" H 900 2050 60  0000 C CNN
F 3 "" H 900 2050 60  0000 C CNN
	1    900  2050
	-1   0    0    1   
$EndComp
$Comp
L GND #PWR4
U 1 1 561F01FA
P 1500 2300
F 0 "#PWR4" H 1500 2050 50  0001 C CNN
F 1 "GND" H 1500 2150 50  0000 C CNN
F 2 "" H 1500 2300 60  0000 C CNN
F 3 "" H 1500 2300 60  0000 C CNN
	1    1500 2300
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR2
U 1 1 561F0218
P 1200 1250
F 0 "#PWR2" H 1200 1000 50  0001 C CNN
F 1 "GND" H 1200 1100 50  0000 C CNN
F 2 "" H 1200 1250 60  0000 C CNN
F 3 "" H 1200 1250 60  0000 C CNN
	1    1200 1250
	1    0    0    -1  
$EndComp
$Comp
L +BATT #PWR5
U 1 1 561F0240
P 1750 2300
F 0 "#PWR5" H 1750 2150 50  0001 C CNN
F 1 "+BATT" H 1750 2440 50  0000 C CNN
F 2 "" H 1750 2300 60  0000 C CNN
F 3 "" H 1750 2300 60  0000 C CNN
	1    1750 2300
	-1   0    0    1   
$EndComp
$Comp
L PWR_FLAG #FLG1
U 1 1 561F0278
P 1200 2000
F 0 "#FLG1" H 1200 2095 50  0001 C CNN
F 1 "PWR_FLAG" H 1200 2180 50  0000 C CNN
F 2 "" H 1200 2000 60  0000 C CNN
F 3 "" H 1200 2000 60  0000 C CNN
	1    1200 2000
	1    0    0    -1  
$EndComp
$Comp
L PWR_FLAG #FLG2
U 1 1 561F0292
P 1200 2100
F 0 "#FLG2" H 1200 2195 50  0001 C CNN
F 1 "PWR_FLAG" H 1200 2280 50  0000 C CNN
F 2 "" H 1200 2100 60  0000 C CNN
F 3 "" H 1200 2100 60  0000 C CNN
	1    1200 2100
	-1   0    0    1   
$EndComp
Text Notes 1300 3800 0    60   ~ 0
Connectors
Text Notes 5350 2600 0    60   ~ 0
Battery Cell Monitor
$Comp
L LM358 U4
U 1 1 561F09C7
P 5000 1550
F 0 "U4" H 4950 1750 60  0000 L CNN
F 1 "LM358" H 4950 1300 60  0000 L CNN
F 2 "" H 5000 1550 60  0000 C CNN
F 3 "" H 5000 1550 60  0000 C CNN
	1    5000 1550
	1    0    0    1   
$EndComp
$Comp
L LM358 U4
U 2 1 561F09F6
P 7200 1550
F 0 "U4" H 7150 1750 60  0000 L CNN
F 1 "LM358" H 7150 1300 60  0000 L CNN
F 2 "" H 7200 1550 60  0000 C CNN
F 3 "" H 7200 1550 60  0000 C CNN
	2    7200 1550
	1    0    0    1   
$EndComp
$Comp
L Metro_Mini U5
U 1 1 561F2659
P 9700 1650
F 0 "U5" H 9700 2450 60  0000 C CNN
F 1 "Metro_Mini" H 9700 850 60  0000 C CNN
F 2 "" H 9700 1650 60  0000 C CNN
F 3 "" H 9700 1650 60  0000 C CNN
	1    9700 1650
	1    0    0    -1  
$EndComp
Text Label 10300 1500 0    60   ~ 0
A0
Text Label 10300 1400 0    60   ~ 0
A1
Text Label 10300 1300 0    60   ~ 0
A2
$Comp
L GND #PWR13
U 1 1 561F2D8D
P 4900 1150
F 0 "#PWR13" H 4900 900 50  0001 C CNN
F 1 "GND" H 4900 1000 50  0000 C CNN
F 2 "" H 4900 1150 60  0000 C CNN
F 3 "" H 4900 1150 60  0000 C CNN
	1    4900 1150
	-1   0    0    1   
$EndComp
$Comp
L GND #PWR16
U 1 1 561F2DAB
P 7100 1150
F 0 "#PWR16" H 7100 900 50  0001 C CNN
F 1 "GND" H 7100 1000 50  0000 C CNN
F 2 "" H 7100 1150 60  0000 C CNN
F 3 "" H 7100 1150 60  0000 C CNN
	1    7100 1150
	-1   0    0    1   
$EndComp
$Comp
L +5V #PWR17
U 1 1 561F2DFB
P 7100 1950
F 0 "#PWR17" H 7100 1800 50  0001 C CNN
F 1 "+5V" H 7100 2090 50  0000 C CNN
F 2 "" H 7100 1950 60  0000 C CNN
F 3 "" H 7100 1950 60  0000 C CNN
	1    7100 1950
	-1   0    0    1   
$EndComp
$Comp
L +5V #PWR14
U 1 1 561F2E19
P 4900 1950
F 0 "#PWR14" H 4900 1800 50  0001 C CNN
F 1 "+5V" H 4900 2090 50  0000 C CNN
F 2 "" H 4900 1950 60  0000 C CNN
F 3 "" H 4900 1950 60  0000 C CNN
	1    4900 1950
	-1   0    0    1   
$EndComp
$Comp
L +5V #PWR19
U 1 1 561F2E5A
P 10300 2000
F 0 "#PWR19" H 10300 1850 50  0001 C CNN
F 1 "+5V" H 10300 2140 50  0000 C CNN
F 2 "" H 10300 2000 60  0000 C CNN
F 3 "" H 10300 2000 60  0000 C CNN
	1    10300 2000
	0    1    1    0   
$EndComp
$Comp
L +3.3V #PWR21
U 1 1 561F2E7D
P 10450 2100
F 0 "#PWR21" H 10450 1950 50  0001 C CNN
F 1 "+3.3V" H 10450 2240 50  0000 C CNN
F 2 "" H 10450 2100 60  0000 C CNN
F 3 "" H 10450 2100 60  0000 C CNN
	1    10450 2100
	0    1    1    0   
$EndComp
$Comp
L GND #PWR18
U 1 1 561F2F9C
P 10300 1800
F 0 "#PWR18" H 10300 1550 50  0001 C CNN
F 1 "GND" H 10300 1650 50  0000 C CNN
F 2 "" H 10300 1800 60  0000 C CNN
F 3 "" H 10300 1800 60  0000 C CNN
	1    10300 1800
	0    -1   -1   0   
$EndComp
$Comp
L R R8
U 1 1 561F32DB
P 6600 1900
F 0 "R8" V 6680 1900 50  0000 C CNN
F 1 "10K" V 6600 1900 50  0000 C CNN
F 2 "" V 6530 1900 30  0000 C CNN
F 3 "" H 6600 1900 30  0000 C CNN
	1    6600 1900
	-1   0    0    1   
$EndComp
$Comp
L R R6
U 1 1 561F338A
P 6350 1650
F 0 "R6" V 6430 1650 50  0000 C CNN
F 1 "10K" V 6350 1650 50  0000 C CNN
F 2 "" V 6280 1650 30  0000 C CNN
F 3 "" H 6350 1650 30  0000 C CNN
	1    6350 1650
	0    1    1    0   
$EndComp
$Comp
L R R5
U 1 1 561F33DA
P 6350 1450
F 0 "R5" V 6430 1450 50  0000 C CNN
F 1 "10K" V 6350 1450 50  0000 C CNN
F 2 "" V 6280 1450 30  0000 C CNN
F 3 "" H 6350 1450 30  0000 C CNN
	1    6350 1450
	0    1    1    0   
$EndComp
$Comp
L R R7
U 1 1 561F3407
P 6600 1200
F 0 "R7" V 6680 1200 50  0000 C CNN
F 1 "10K" V 6600 1200 50  0000 C CNN
F 2 "" V 6530 1200 30  0000 C CNN
F 3 "" H 6600 1200 30  0000 C CNN
	1    6600 1200
	1    0    0    -1  
$EndComp
$Comp
L R R4
U 1 1 561F36CE
P 4400 1900
F 0 "R4" V 4480 1900 50  0000 C CNN
F 1 "10K" V 4400 1900 50  0000 C CNN
F 2 "" V 4330 1900 30  0000 C CNN
F 3 "" H 4400 1900 30  0000 C CNN
	1    4400 1900
	-1   0    0    1   
$EndComp
$Comp
L R R2
U 1 1 561F36D4
P 4150 1650
F 0 "R2" V 4230 1650 50  0000 C CNN
F 1 "10K" V 4150 1650 50  0000 C CNN
F 2 "" V 4080 1650 30  0000 C CNN
F 3 "" H 4150 1650 30  0000 C CNN
	1    4150 1650
	0    1    1    0   
$EndComp
$Comp
L R R1
U 1 1 561F36DA
P 4150 1450
F 0 "R1" V 4230 1450 50  0000 C CNN
F 1 "10K" V 4150 1450 50  0000 C CNN
F 2 "" V 4080 1450 30  0000 C CNN
F 3 "" H 4150 1450 30  0000 C CNN
	1    4150 1450
	0    1    1    0   
$EndComp
$Comp
L R R3
U 1 1 561F36E0
P 4400 1200
F 0 "R3" V 4480 1200 50  0000 C CNN
F 1 "10K" V 4400 1200 50  0000 C CNN
F 2 "" V 4330 1200 30  0000 C CNN
F 3 "" H 4400 1200 30  0000 C CNN
	1    4400 1200
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR15
U 1 1 561F3A25
P 6600 2050
F 0 "#PWR15" H 6600 1800 50  0001 C CNN
F 1 "GND" H 6600 1900 50  0000 C CNN
F 2 "" H 6600 2050 60  0000 C CNN
F 3 "" H 6600 2050 60  0000 C CNN
	1    6600 2050
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR12
U 1 1 561F3A53
P 4400 2050
F 0 "#PWR12" H 4400 1800 50  0001 C CNN
F 1 "GND" H 4400 1900 50  0000 C CNN
F 2 "" H 4400 2050 60  0000 C CNN
F 3 "" H 4400 2050 60  0000 C CNN
	1    4400 2050
	1    0    0    -1  
$EndComp
Text Label 3900 1650 2    60   ~ 0
Cell1
Text Label 3900 1450 2    60   ~ 0
Cell2
Text Label 6100 1650 2    60   ~ 0
Cell2
Text Label 6100 1450 2    60   ~ 0
Cell3
Text Label 5600 1550 0    60   ~ 0
A1
Text Label 7800 1550 0    60   ~ 0
A2
Text Label 4050 1900 0    60   ~ 0
A0
$Comp
L +BATT #PWR20
U 1 1 561F4FB3
P 10450 1700
F 0 "#PWR20" H 10450 1550 50  0001 C CNN
F 1 "+BATT" H 10450 1840 50  0000 C CNN
F 2 "" H 10450 1700 60  0000 C CNN
F 3 "" H 10450 1700 60  0000 C CNN
	1    10450 1700
	0    1    1    0   
$EndComp
NoConn ~ 10200 1600
NoConn ~ 10200 2200
NoConn ~ 10200 2300
$Comp
L ACS712_Mod U3
U 1 1 561F3887
P 2250 2100
F 0 "U3" H 2250 2350 60  0000 C CNN
F 1 "ACS712_Mod" H 2250 1800 60  0000 C CNN
F 2 "" H 2300 2100 60  0000 C CNN
F 3 "" H 2300 2100 60  0000 C CNN
	1    2250 2100
	-1   0    0    1   
$EndComp
$Comp
L ACS712_Mod U1
U 1 1 561F3904
P 1550 3100
F 0 "U1" H 1550 3350 60  0000 C CNN
F 1 "ACS712_Mod" H 1550 2800 60  0000 C CNN
F 2 "" H 1600 3100 60  0000 C CNN
F 3 "" H 1600 3100 60  0000 C CNN
	1    1550 3100
	-1   0    0    1   
$EndComp
$Comp
L VNH2SP30 U2
U 1 1 561F3981
P 2050 4400
F 0 "U2" H 2050 4700 60  0000 C CNN
F 1 "VNH2SP30" H 2050 4450 60  0000 C CNN
F 2 "" H 2050 4400 60  0000 C CNN
F 3 "" H 2050 4400 60  0000 C CNN
	1    2050 4400
	1    0    0    -1  
$EndComp
$Comp
L CONN_01X02 P3
U 1 1 561F3A29
P 900 3250
F 0 "P3" H 900 3400 50  0000 C CNN
F 1 "Motor" V 1000 3250 50  0000 C CNN
F 2 "" H 900 3250 60  0000 C CNN
F 3 "" H 900 3250 60  0000 C CNN
	1    900  3250
	-1   0    0    1   
$EndComp
Text Label 1100 3000 2    60   ~ 0
DriveA
Text Label 1150 3450 0    60   ~ 0
DriveB
$Comp
L GND #PWR10
U 1 1 561F53D1
P 2700 2000
F 0 "#PWR10" H 2700 1750 50  0001 C CNN
F 1 "GND" H 2700 1850 50  0000 C CNN
F 2 "" H 2700 2000 60  0000 C CNN
F 3 "" H 2700 2000 60  0000 C CNN
	1    2700 2000
	0    -1   -1   0   
$EndComp
$Comp
L GND #PWR6
U 1 1 561F5407
P 2000 3000
F 0 "#PWR6" H 2000 2750 50  0001 C CNN
F 1 "GND" H 2000 2850 50  0000 C CNN
F 2 "" H 2000 3000 60  0000 C CNN
F 3 "" H 2000 3000 60  0000 C CNN
	1    2000 3000
	0    -1   -1   0   
$EndComp
$Comp
L +5V #PWR7
U 1 1 561F543D
P 2000 3200
F 0 "#PWR7" H 2000 3050 50  0001 C CNN
F 1 "+5V" H 2000 3340 50  0000 C CNN
F 2 "" H 2000 3200 60  0000 C CNN
F 3 "" H 2000 3200 60  0000 C CNN
	1    2000 3200
	0    1    1    0   
$EndComp
$Comp
L +5V #PWR11
U 1 1 561F5473
P 2700 2200
F 0 "#PWR11" H 2700 2050 50  0001 C CNN
F 1 "+5V" H 2700 2340 50  0000 C CNN
F 2 "" H 2700 2200 60  0000 C CNN
F 3 "" H 2700 2200 60  0000 C CNN
	1    2700 2200
	0    1    1    0   
$EndComp
Text Label 2950 2100 0    60   ~ 0
A3
Text Label 2250 3100 0    60   ~ 0
A4
Text Label 10300 1200 0    60   ~ 0
A3
Text Label 10300 1100 0    60   ~ 0
A4
$Comp
L GND #PWR3
U 1 1 561F737B
P 1200 4300
F 0 "#PWR3" H 1200 4050 50  0001 C CNN
F 1 "GND" H 1200 4150 50  0000 C CNN
F 2 "" H 1200 4300 60  0000 C CNN
F 3 "" H 1200 4300 60  0000 C CNN
	1    1200 4300
	0    1    1    0   
$EndComp
$Comp
L GND #PWR8
U 1 1 561F73B1
P 2250 5050
F 0 "#PWR8" H 2250 4800 50  0001 C CNN
F 1 "GND" H 2250 4900 50  0000 C CNN
F 2 "" H 2250 5050 60  0000 C CNN
F 3 "" H 2250 5050 60  0000 C CNN
	1    2250 5050
	1    0    0    -1  
$EndComp
$Comp
L +5V #PWR9
U 1 1 561F73E7
P 2350 4900
F 0 "#PWR9" H 2350 4750 50  0001 C CNN
F 1 "+5V" H 2350 5040 50  0000 C CNN
F 2 "" H 2350 4900 60  0000 C CNN
F 3 "" H 2350 4900 60  0000 C CNN
	1    2350 4900
	-1   0    0    1   
$EndComp
$Comp
L +BATT #PWR1
U 1 1 561F7979
P 1050 4400
F 0 "#PWR1" H 1050 4250 50  0001 C CNN
F 1 "+BATT" H 1050 4540 50  0000 C CNN
F 2 "" H 1050 4400 60  0000 C CNN
F 3 "" H 1050 4400 60  0000 C CNN
	1    1050 4400
	0    -1   -1   0   
$EndComp
Wire Wire Line
	1100 950  1150 950 
Wire Wire Line
	1100 1050 1150 1050
Wire Wire Line
	1100 1150 1150 1150
Wire Wire Line
	1100 1250 1200 1250
Wire Wire Line
	1100 2000 1800 2000
Connection ~ 1200 2000
Wire Wire Line
	1100 2100 1500 2100
Connection ~ 1200 2100
Wire Wire Line
	10200 1300 10300 1300
Wire Wire Line
	10200 1400 10300 1400
Wire Wire Line
	10200 1500 10300 1500
Wire Wire Line
	10200 2000 10300 2000
Wire Wire Line
	10200 2100 10450 2100
Wire Wire Line
	10200 1900 10200 1800
Wire Wire Line
	4300 1650 4500 1650
Wire Wire Line
	4400 1650 4400 1750
Connection ~ 4400 1650
Wire Wire Line
	4300 1450 4500 1450
Wire Wire Line
	4400 1350 4400 1450
Connection ~ 4400 1450
Wire Wire Line
	6500 1450 6700 1450
Wire Wire Line
	6700 1650 6500 1650
Wire Wire Line
	6600 1750 6600 1650
Connection ~ 6600 1650
Wire Wire Line
	6600 1350 6600 1450
Connection ~ 6600 1450
Wire Wire Line
	4400 900  4400 1050
Wire Wire Line
	5500 900  5500 1550
Wire Wire Line
	6600 1050 6600 900 
Wire Wire Line
	6600 900  7700 900 
Wire Wire Line
	7700 900  7700 1550
Wire Wire Line
	4400 900  5500 900 
Wire Wire Line
	3900 1450 4000 1450
Wire Wire Line
	3900 1650 4000 1650
Wire Wire Line
	3950 1650 3950 1900
Wire Wire Line
	3950 1900 4050 1900
Connection ~ 3950 1650
Wire Wire Line
	6100 1450 6200 1450
Wire Wire Line
	6100 1650 6200 1650
Wire Wire Line
	7700 1550 7800 1550
Wire Wire Line
	10450 1700 10200 1700
Wire Wire Line
	10200 1800 10300 1800
Wire Wire Line
	1500 2100 1500 2300
Wire Wire Line
	1750 2300 1750 2200
Wire Wire Line
	1750 2200 1800 2200
Wire Wire Line
	1100 3300 1100 3450
Wire Wire Line
	2000 3100 2250 3100
Wire Wire Line
	2700 2100 2950 2100
Wire Wire Line
	1100 3450 1150 3450
Wire Wire Line
	10300 1200 10200 1200
Wire Wire Line
	10200 1100 10300 1100
Wire Wire Line
	2250 4900 2250 5050
Wire Wire Line
	1200 4400 1050 4400
Text Label 2850 4300 0    60   ~ 0
DriveA
Text Label 2850 4400 0    60   ~ 0
DriveB
Text Label 1950 4900 3    60   ~ 0
INA
Text Label 1850 4900 3    60   ~ 0
INB
Text Label 1750 4900 3    60   ~ 0
PWM
Text Label 9100 1200 2    60   ~ 0
INA
Text Label 9100 1300 2    60   ~ 0
PWD
Text Label 9100 1400 2    60   ~ 0
INB
Wire Wire Line
	9100 1200 9200 1200
Wire Wire Line
	9200 1300 9100 1300
Wire Wire Line
	9100 1400 9200 1400
$Comp
L CONN_01X05 P4
U 1 1 561F9439
P 2150 1150
F 0 "P4" H 2150 1450 50  0000 C CNN
F 1 "Stearing" V 2250 1150 50  0000 C CNN
F 2 "" H 2150 1150 60  0000 C CNN
F 3 "" H 2150 1150 60  0000 C CNN
	1    2150 1150
	-1   0    0    1   
$EndComp
Wire Notes Line
	500  3900 3400 3900
Wire Notes Line
	3400 3900 3400 500 
Wire Notes Line
	3400 2700 8500 2700
Wire Notes Line
	8500 2700 8500 500 
$EndSCHEMATC
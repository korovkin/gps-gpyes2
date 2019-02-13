# gps-gpyes2
playing with the Stratux GPYes 2.0 u-blox 8 GPS unit. 

## example:

playing with [Stratux GPYes 2.0 u-blox 8 GPS](https://www.amazon.com/gp/product/B0716BK5NT) plugged into a MacBook Pro.

```
go run main.go -port_name /dev/tty.usbmodem14101

2019/02/13 12:41:26.203144 main.go:62: LINE:           59 $GNRMC,,V,,,,,,,,,,N*4D
2019/02/13 12:41:26.203199 main.go:92: => RMC: TIME: 00:00:00.0000 POS:       0.00000000       0.00000000 SPEED: 0 COURSE: 0
2019/02/13 12:41:26.203217 main.go:62: LINE:           60 $GNVTG,,,,,,,,,N*2E
2019/02/13 12:41:26.203248 main.go:62: LINE:           61 $GNGGA,,,,,,0,00,99.99,,,,,,*56
2019/02/13 12:41:26.203271 main.go:62: LINE:           62 $GNGSA,A,1,,,,,,,,,,,,,99.99,99.99,99.99*2E
2019/02/13 12:41:26.203310 main.go:62: LINE:           63 $GNGSA,A,1,,,,,,,,,,,,,99.99,99.99,99.99*2E
2019/02/13 12:41:26.203449 main.go:62: LINE:           64 $GPGSV,1,1,00*79
2019/02/13 12:41:26.203462 main.go:76: => GSV: MSG: 1 1 SATs: 0
2019/02/13 12:41:26.203595 main.go:62: LINE:           65 $GLGSV,1,1,00*65
2019/02/13 12:41:26.203605 main.go:76: => GSV: MSG: 1 1 SATs: 0
2019/02/13 12:41:26.203646 main.go:62: LINE:           66 $GNGLL,,,,,,V,N*7A
2019/02/13 12:41:26.203664 main.go:119: => GLL: TIME: 00:00:00.0000 POS:       0.00000000       0.00000000

2019/02/13 12:41:27.203544 main.go:62: LINE:           67 $GNRMC,,V,,,,,,,,,,N*4D
2019/02/13 12:41:27.203618 main.go:92: => RMC: TIME: 00:00:00.0000 POS:       0.00000000       0.00000000 SPEED: 0 COURSE: 0
2019/02/13 12:41:27.203634 main.go:62: LINE:           68 $GNVTG,,,,,,,,,N*2E
2019/02/13 12:41:27.203683 main.go:62: LINE:           69 $GNGGA,,,,,,0,00,99.99,,,,,,*56
2019/02/13 12:41:27.203740 main.go:62: LINE:           70 $GNGSA,A,1,,,,,,,,,,,,,99.99,99.99,99.99*2E
2019/02/13 12:41:27.203787 main.go:62: LINE:           71 $GNGSA,A,1,,,,,,,,,,,,,99.99,99.99,99.99*2E
2019/02/13 12:41:27.203811 main.go:62: LINE:           72 $GPGSV,1,1,00*79
2019/02/13 12:41:27.203825 main.go:76: => GSV: MSG: 1 1 SATs: 0
2019/02/13 12:41:27.203855 main.go:62: LINE:           73 $GLGSV,1,1,00*65
2019/02/13 12:41:27.203867 main.go:76: => GSV: MSG: 1 1 SATs: 0
2019/02/13 12:41:27.203877 main.go:62: LINE:           74 $GNGLL,,,,,,V,N*7A
2019/02/13 12:41:27.203914 main.go:119: => GLL: TIME: 00:00:00.0000 POS:       0.00000000       0.00000000

```

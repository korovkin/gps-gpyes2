package main

import (
	"bufio"
	"bytes"
	"encoding/json"
	"flag"
	"fmt"
	"io"
	"log"
	"time"

	"github.com/adrianmo/go-nmea"
	"github.com/jacobsa/go-serial/serial"
)

func ToJsonBytes(v interface{}) []byte {
	buf := new(bytes.Buffer)
	enc := json.NewEncoder(buf)
	enc.SetEscapeHTML(false)
	enc.SetIndent("", " ")
	err := enc.Encode(v)
	if err == nil {
		return buf.Bytes()
	}
	return []byte("{}")
}

func ToJsonString(v interface{}) string {
	bytes := ToJsonBytes(v)
	return string(bytes)
}

func chksumUBX(msg []byte) []byte {
	ret := make([]byte, 2)
	for i := 0; i < len(msg); i++ {
		ret[0] = ret[0] + msg[i]
		ret[1] = ret[1] + ret[0]
	}
	return ret
}

func makeUBXCFG(class, id byte, msglen uint16, msg []byte) []byte {
	ret := make([]byte, 6)
	ret[0] = 0xB5
	ret[1] = 0x62
	ret[2] = class
	ret[3] = id
	ret[4] = byte(msglen & 0xFF)
	ret[5] = byte((msglen >> 8) & 0xFF)
	ret = append(ret, msg...)
	chk := chksumUBX(ret[2:])
	ret = append(ret, chk[0])
	ret = append(ret, chk[1])
	return ret
}

func makeNMEACmd(cmd string) []byte {
	chk_sum := byte(0)
	for i := range cmd {
		chk_sum = chk_sum ^ byte(cmd[i])
	}
	return []byte(fmt.Sprintf("$%s*%02x\x0d\x0a", cmd, chk_sum))
}

func initGPSSerial(device io.ReadWriteCloser) bool {
	// Set 10 Hz update to make gpsattitude more responsive for ublox7.
	updatespeed := []byte{0x64, 0x00, 0x01, 0x00, 0x01, 0x00} // 10 Hz

	// Set navigation settings.
	nav := make([]byte, 36)
	nav[0] = 0x05 // Set dyn and fixMode only.
	nav[1] = 0x00
	nav[2] = 0x07 // "Airborne with >2g Acceleration".
	nav[3] = 0x02 // 3D only.
	device.Write(makeUBXCFG(0x06, 0x24, 36, nav))

	// Turn off "time pulse" (usually drives an LED).
	if false {
		tp5 := make([]byte, 32)
		tp5[4] = 0x32
		tp5[8] = 0x40
		tp5[9] = 0x42
		tp5[10] = 0x0F
		tp5[12] = 0x40
		tp5[13] = 0x42
		tp5[14] = 0x0F
		tp5[28] = 0xE7
		device.Write(makeUBXCFG(0x06, 0x31, 32, tp5))
	}

	// GNSS configuration CFG-GNSS for ublox 7 higher, p. 125 (v8)
	// Notes: ublox8 is multi-GNSS capable (simultaneous decoding of GPS and GLONASS, or
	// GPS and Galileo) if SBAS (e.g. WAAS) is unavailable. This may provide robustness
	// against jamming / interference on one set of frequencies. However, this will drop the
	// position reporting rate to 5 Hz during times multi-GNSS is in use. This shouldn't affect
	// gpsattitude too much --  without WAAS corrections, the algorithm could get jumpy at higher
	// sampling rates.

	cfgGnss := []byte{0x00, 0x20, 0x20, 0x06}
	gps := []byte{0x00, 0x08, 0x10, 0x00, 0x01, 0x00, 0x01, 0x01}  // enable GPS with 8-16 tracking channels
	sbas := []byte{0x01, 0x02, 0x03, 0x00, 0x01, 0x00, 0x01, 0x01} // enable SBAS (WAAS) with 2-3 tracking channels
	beidou := []byte{0x03, 0x00, 0x10, 0x00, 0x00, 0x00, 0x01, 0x01}
	qzss := []byte{0x05, 0x00, 0x03, 0x00, 0x00, 0x00, 0x01, 0x01}
	glonass := []byte{0x06, 0x04, 0x0E, 0x00, 0x00, 0x00, 0x01, 0x01} // this disables GLONASS
	galileo := []byte{0x02, 0x04, 0x08, 0x00, 0x00, 0x00, 0x01, 0x01} // this disables Galileo

	gps = []byte{0x00, 0x10, 0x10, 0x00, 0x01, 0x00, 0x01, 0x01}     // enable GPS with 8-16 tracking channels
	glonass = []byte{0x06, 0x10, 0x10, 0x00, 0x00, 0x00, 0x01, 0x01} // this disables GLONASS
	sbas = []byte{0x01, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00}    // enable SBAS (WAAS) with 2-3 tracking channels
	beidou = []byte{0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}
	qzss = []byte{0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}
	galileo = []byte{0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00} // this disables Galileo

	updatespeed = []byte{0x06, 0x00, 0xF4, 0x01, 0x01, 0x00} // Nav speed 2Hz

	cfgGnss = append(cfgGnss, gps...)
	cfgGnss = append(cfgGnss, sbas...)
	cfgGnss = append(cfgGnss, beidou...)
	cfgGnss = append(cfgGnss, qzss...)
	cfgGnss = append(cfgGnss, glonass...)
	cfgGnss = append(cfgGnss, galileo...)
	device.Write(makeUBXCFG(0x06, 0x3E, uint16(len(cfgGnss)), cfgGnss))

	device.Write(makeUBXCFG(0x06, 0x16, 8, []byte{0x01, 0x07, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00}))
	device.Write(makeUBXCFG(0x06, 0x08, 6, updatespeed))

	x := byte(0x01)
	device.Write(makeUBXCFG(0x06, 0x01, 8, []byte{0xF0, 0x00, x, x, x, x, x, x})) // GGA
	device.Write(makeUBXCFG(0x06, 0x01, 8, []byte{0xF0, 0x01, x, x, x, x, x, x})) // GLL
	device.Write(makeUBXCFG(0x06, 0x01, 8, []byte{0xF0, 0x02, x, x, x, x, x, x})) // GSA
	device.Write(makeUBXCFG(0x06, 0x01, 8, []byte{0xF0, 0x03, x, x, x, x, x, x})) // GSV
	device.Write(makeUBXCFG(0x06, 0x01, 8, []byte{0xF0, 0x04, x, x, x, x, x, x})) // RMC
	device.Write(makeUBXCFG(0x06, 0x01, 8, []byte{0xF0, 0x08, x, x, x, x, x, x})) // ZDA
	device.Write(makeUBXCFG(0x06, 0x01, 8, []byte{0xF0, 0x41, x, x, x, x, x, x})) // TXT
	device.Write(makeUBXCFG(0x06, 0x01, 8, []byte{0xF0, 0x05, x, x, x, x, x, x})) // VGT
	device.Write(makeUBXCFG(0x06, 0x01, 8, []byte{0xF0, 0x06, x, x, x, x, x, x})) // GRS
	device.Write(makeUBXCFG(0x06, 0x01, 8, []byte{0xF0, 0x07, x, x, x, x, x, x})) // GST
	device.Write(makeUBXCFG(0x06, 0x01, 8, []byte{0xF0, 0x08, x, x, x, x, x, x})) // ZDA
	device.Write(makeUBXCFG(0x06, 0x01, 8, []byte{0xF0, 0x09, x, x, x, x, x, x})) // GBS
	device.Write(makeUBXCFG(0x06, 0x01, 8, []byte{0xF0, 0x0A, x, x, x, x, x, x})) // DTM
	device.Write(makeUBXCFG(0x06, 0x01, 8, []byte{0xF0, 0x0D, x, x, x, x, x, x})) // GNS
	device.Write(makeUBXCFG(0x06, 0x01, 8, []byte{0xF0, 0x0F, x, x, x, x, x, x})) // VLW

	x = byte(0x00)
	device.Write(makeUBXCFG(0x06, 0x01, 8, []byte{0xF1, 0x00, x, x, x, x, x, x})) // Ublox,0
	device.Write(makeUBXCFG(0x06, 0x01, 8, []byte{0xF1, 0x03, x, x, x, x, x, x})) // Ublox,3
	device.Write(makeUBXCFG(0x06, 0x01, 8, []byte{0xF1, 0x04, x, x, x, x, x, x})) // Ublox,4

	// Reconfigure serial port.
	cfg := make([]byte, 20)
	cfg[0] = 0x01 // portID.
	cfg[1] = 0x00 // res0.
	cfg[2] = 0x00 // res1.
	cfg[3] = 0x00 // res1.

	//      [   7   ] [   6   ] [   5   ] [   4   ]
	//	0000 0000 0000 0000 0000 10x0 1100 0000
	// UART mode. 0 stop bits, no parity, 8 data bits. Little endian order.
	cfg[4] = 0xC0
	cfg[5] = 0x08
	cfg[6] = 0x00
	cfg[7] = 0x00

	// Baud rate. Little endian order.
	bdrt := uint32(38400)
	cfg[11] = byte((bdrt >> 24) & 0xFF)
	cfg[10] = byte((bdrt >> 16) & 0xFF)
	cfg[9] = byte((bdrt >> 8) & 0xFF)
	cfg[8] = byte(bdrt & 0xFF)

	// inProtoMask. NMEA and UBX. Little endian.
	cfg[12] = 0x03
	cfg[13] = 0x00

	// outProtoMask. NMEA. Little endian.
	cfg[14] = 0x02
	cfg[15] = 0x00

	cfg[16] = 0x00 // flags.
	cfg[17] = 0x00 // flags.

	cfg[18] = 0x00 //pad.
	cfg[19] = 0x00 //pad.

	device.Write(makeUBXCFG(0x06, 0x00, 20, cfg))

	sleepPeriod := 250 * time.Millisecond
	log.Println("=> sleep:", sleepPeriod)
	time.Sleep(sleepPeriod)
	return true
}

func main() {
	log.SetFlags(log.Ltime | log.Lshortfile | log.Lmicroseconds | log.Ldate)

	port_name := flag.String(
		"port_name",
		"/dev/tty.usbmodem14101",
		"device name to connect to")
	flag.Parse()

	options := serial.OpenOptions{
		PortName:        *port_name,
		BaudRate:        38400,
		DataBits:        8,
		StopBits:        1,
		MinimumReadSize: 4,
	}

	log.Println("CONNECTING TO:", options.PortName, ToJsonString(options))

	port, err := serial.Open(options)
	if err != nil {
		log.Fatalf("ERROR: serial.Open: %v", err)
	}

	initGPSSerial(port)
	port.Close()

	port, err = serial.Open(options)
	if err != nil {
		log.Fatalf("ERROR: serial.Open: %v", err)
	}

	scanner := bufio.NewScanner(port)
	N := 0
	for scanner.Scan() {
		line := scanner.Text()

		log.Println("LINE:",
			fmt.Sprintf("%-12d", N),
			line)

		N += 1

		s, err := nmea.Parse(line)
		if err != nil {
			// log.Println("ERRROR: LINE:", line)
			// log.Println("ERRROR: REASON:", err.Error())
			continue
		}
		// log.Println("=>", ToJsonString(s))

		if m, ok := s.(nmea.GSV); ok {
			log.Println("=> GSV:",
				"MSG:", m.MessageNumber, m.TotalMessages,
				"SATs:", m.NumberSVsInView,
			)

			for _, sat := range m.Info {
				log.Println("      SAT:",
					fmt.Sprintf("%4d", sat.SVPRNNumber),
					"SNR:", fmt.Sprintf("%5d", sat.SNR),
					"ELEV:", fmt.Sprintf("%5d", sat.Elevation),
					"AZ:", fmt.Sprintf("%5d", sat.Azimuth),
				)
			}
		}

		if m, ok := s.(nmea.RMC); ok {
			log.Println("=> RMC:",
				"TIME:", m.Time,
				"POS:",
				fmt.Sprintf("%16.10f", m.Latitude),
				",",
				fmt.Sprintf("%16.10f", m.Longitude),
				"SPEED:", fmt.Sprintf("%.1f", m.Speed),
				"COURSE:", fmt.Sprintf("%.1f", m.Course),
			)
		}

		if m, ok := s.(nmea.GNS); ok {
			log.Println("=> GNS:",
				"TIME:", m.Time,
				"SVs", m.SVs,
				"AGE:", m.Age,
				"POS:",
				fmt.Sprintf("%16.10f", m.Latitude),
				",",
				fmt.Sprintf("%16.10f", m.Longitude),
				"ALT:", m.Altitude,
				"MODE:", m.Mode,
			)
		}

		if m, ok := s.(nmea.GLL); ok {
			log.Println("=> GLL:",
				"TIME:", m.Time,
				"POS:",
				fmt.Sprintf("%16.10f", m.Latitude),
				",",
				fmt.Sprintf("%16.10f", m.Longitude),
				"\n",
			)

		}
	}
}

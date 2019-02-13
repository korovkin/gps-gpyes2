package main

import (
	"bufio"
	"bytes"
	"encoding/json"
	"flag"
	"fmt"
	"log"

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

func main() {
	log.SetFlags(log.Ltime | log.Lshortfile | log.Lmicroseconds | log.Ldate)

	port_name := flag.String(
		"port_name",
		"/dev/tty.usbmodem14101",
		"device name to connect to")
	flag.Parse()

	options := serial.OpenOptions{
		PortName:        *port_name,
		BaudRate:        19200,
		DataBits:        8,
		StopBits:        1,
		MinimumReadSize: 4,
	}

	log.Println("CONNECTING TO:", options.PortName, ToJsonString(options))

	port, err := serial.Open(options)
	if err != nil {
		log.Fatalf("ERROR: serial.Open: %v", err)
	}
	defer port.Close()

	scanner := bufio.NewScanner(port)
	N := 0
	for scanner.Scan() {

		line := scanner.Text()
		log.Println("LINE:",
			fmt.Sprintf("%12d", N),
			line)
		N += 1

		s, err := nmea.Parse(line)
		if err != nil {
			log.Println("ERRROR: LINE:", line)
			log.Println("ERRROR: REASON:", err.Error())
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
					"NUM:", sat.SVPRNNumber,
					"SNR:", sat.SNR,
					"ELEV:", sat.Elevation,
					"AZ:", sat.Azimuth,
				)
			}
		}

		if m, ok := s.(nmea.RMC); ok {
			log.Println("=> RMC:",
				"TIME:", m.Time,
				"POS:",
				// nmea.FormatGPS(m.Latitude),
				// nmea.FormatGPS(m.Longitude),
				fmt.Sprintf("%16.8f", m.Latitude),
				fmt.Sprintf("%16.8f", m.Longitude),
				"SPEED:", m.Speed,
				"COURSE:", m.Course,
			)
		}

		if m, ok := s.(nmea.GNS); ok {
			log.Println("=> GNS:",
				"TIME:", m.Time,
				"SVs", m.SVs,
				"AGE:", m.Age,
				"POS:",
				fmt.Sprintf("%16.8f", m.Latitude),
				fmt.Sprintf("%16.8f", m.Longitude),
				"ALT:", m.Altitude,
				"MODE:", m.Mode,
				"FIELDS:", m.Fields,
			)
		}

		if m, ok := s.(nmea.GLL); ok {
			log.Println("=> GLL:",
				"TIME:", m.Time,
				"POS:",
				fmt.Sprintf("%16.8f", m.Latitude),
				fmt.Sprintf("%16.8f", m.Longitude),
				"\n",
			)

		}
	}
}

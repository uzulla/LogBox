// logbo-dump receives the JSON datagrams broadcast by the logger and prints them.
package main

import (
	"encoding/json"
	"flag"
	"fmt"
	"log"
	"net"
	"time"
)

type xyz struct{ X, Y, Z float64 }

type message struct {
	Type string `json:"type"`

	// type=sample
	Seq      uint32   `json:"seq"`
	GNSS     bool     `json:"gnss"`
	Time     *string  `json:"time"`
	Fix      int      `json:"fix"`
	FixOK    bool     `json:"fix_ok"`
	Sats     int      `json:"sats"`
	Lat      float64  `json:"lat"`
	Lon      float64  `json:"lon"`
	HAcc     float64  `json:"h_acc_m"`
	SpeedMps float64  `json:"speed_mps"`
	Heading  float64  `json:"heading_deg"`
	G        *xyz     `json:"g"`
	Gyro     *xyz     `json:"gyro_dps"`
	PressHPa *float64 `json:"press_hpa"`

	// type=status
	Records  uint32 `json:"records"`
	Rec      int    `json:"rec"`
	Log      string `json:"log"`
	WifiRSSI int    `json:"wifi_rssi"`
	FreeHeap uint32 `json:"free_heap"`
}

type statusGNSS struct {
	GNSS struct {
		Configured     bool   `json:"configured"`
		Baud           uint32 `json:"baud"`
		PVT            uint32 `json:"pvt"`
		ChecksumErrors uint32 `json:"checksum_errors"`
	} `json:"gnss"`
}

func main() {
	addr := flag.String("listen", ":5005", "UDP address to listen on")
	raw := flag.Bool("raw", false, "print the received JSON as is")
	flag.Parse()

	conn, err := net.ListenPacket("udp4", *addr)
	if err != nil {
		log.Fatal(err)
	}
	defer conn.Close()
	log.Printf("listening on %s", conn.LocalAddr())

	var lastSeq uint32
	var haveSeq bool
	buf := make([]byte, 2048)
	for {
		n, from, err := conn.ReadFrom(buf)
		if err != nil {
			log.Fatal(err)
		}
		now := time.Now().Format("15:04:05.000")
		if *raw {
			fmt.Printf("%s %s %s\n", now, from, buf[:n])
			continue
		}
		var m message
		if err := json.Unmarshal(buf[:n], &m); err != nil {
			log.Printf("%s: invalid JSON: %v: %q", from, err, buf[:n])
			continue
		}
		switch m.Type {
		case "sample":
			if haveSeq && m.Seq != lastSeq+1 && m.Seq != 0 {
				log.Printf("lost %d datagram(s) (seq %d -> %d)", m.Seq-lastSeq-1, lastSeq, m.Seq)
			}
			lastSeq, haveSeq = m.Seq, true
			t := "-"
			if m.Time != nil {
				t = *m.Time
			}
			g, gy := xyz{}, xyz{}
			if m.G != nil {
				g = *m.G
			}
			if m.Gyro != nil {
				gy = *m.Gyro
			}
			p := "-"
			if m.PressHPa != nil {
				p = fmt.Sprintf("%.2fhPa", *m.PressHPa)
			}
			fmt.Printf("%s #%d %s gnss=%v fix=%d ok=%v sats=%d lat=%.7f lon=%.7f hacc=%.2fm speed=%.2fkm/h hdg=%.1f g=(%.2f,%.2f,%.2f) gyro=(%.1f,%.1f,%.1f) %s\n",
				now, m.Seq, t, m.GNSS, m.Fix, m.FixOK, m.Sats, m.Lat, m.Lon, m.HAcc,
				m.SpeedMps*3.6, m.Heading, g.X, g.Y, g.Z, gy.X, gy.Y, gy.Z, p)
		case "status":
			var st statusGNSS
			_ = json.Unmarshal(buf[:n], &st)
			fmt.Printf("%s [status from %s] records=%d gnss(configured=%v baud=%d pvt=%d cksum_err=%d) rec=%d log=%q rssi=%d heap=%d\n",
				now, from, m.Records, st.GNSS.Configured, st.GNSS.Baud, st.GNSS.PVT, st.GNSS.ChecksumErrors,
				m.Rec, m.Log, m.WifiRSSI, m.FreeHeap)
		default:
			fmt.Printf("%s %s unknown: %s\n", now, from, buf[:n])
		}
	}
}

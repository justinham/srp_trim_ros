import pyshark
from collections import defaultdict
from datetime import datetime

# 1. Define your PCAP file and the exact Wireshark filter you found
pcap_file = '/home/yz4d3h/Downloads/PCAP_04201_2026_02_24_16_33_38_cv2x0_tx_rx_000.pcap'
display_filter = 'frame contains 00:80:10 && frame contains 00:29'

def check_packet_rate(file_path, filter_str, min_packets=8):
    # Initialize the capture with the display filter applied
    cap = pyshark.FileCapture(
        file_path,
        display_filter=filter_str,
        use_json=True,
        include_raw=True,
        keep_packets=False
    )

    # Dictionary to store counts: {integer_second: count}
    packets_per_second = defaultdict(int)

    print(f"Analyzing packets matching: {filter_str}...")

    try:
        for packet in cap:
            ts = packet.sniff_timestamp
            if "T" in ts:
                dt = datetime.fromisoformat(ts.replace("Z", "+00:00"))
                timestamp = dt.timestamp()
            else:
                timestamp = float(ts)

            second = int(timestamp)
            packets_per_second[second] += 1

            raw = packet.get_raw_packet()
            print(ts, raw.hex())

        cap.close()
    except Exception as e:
        print(f"Error reading packets: {e}")

    # 2. Check and report results
    all_met = True
    sorted_seconds = sorted(packets_per_second.keys())

    if not sorted_seconds:
        print("No packets found matching the filter.")
        return

    print(f"\n--- Analysis Results (Threshold: {min_packets} pkts/sec) ---")
    for sec in range(min(sorted_seconds), max(sorted_seconds) + 1):
        count = packets_per_second.get(sec, 0)
        status = "PASS" if count >= min_packets else "FAIL"
        print(f"Second {sec}: {count} packets [{status}]")

        if count < min_packets:
            all_met = False

    if all_met:
        print("\nSUCCESS: All seconds met the minimum packet requirement.")
    else:
        print("\nWARNING: Some seconds failed the 8 packet/sec threshold.")

if __name__ == "__main__":
    check_packet_rate(pcap_file, display_filter)
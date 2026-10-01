#!/usr/bin/env python3
import argparse
import pandas as pd
import numpy as np
from pathlib import Path

def main():
    ap = argparse.ArgumentParser(description="Hysteretic latching filter on heading based on speed.")
    ap.add_argument("csv", help="Input CSV path")
    ap.add_argument("--heading-col", default="heading (rad)", help="Heading column name (degrees)")
    ap.add_argument("--speed-col", default="calculated speed (km/h)", help="Speed column name")
    ap.add_argument("--time-col", default="timestamp (s) \{PC Timestamp\}", help="Timestamp column name")
    ap.add_argument("--latch-speed", type=float, default=1.4,
                    help="Latch when speed < this value")
    ap.add_argument("--unlatch-speed", type=float, default=2.0,
                    help="Unlatch when speed >= this value (must be > latch-speed)")
    ap.add_argument("--output-range", choices=["0-360", "-180_180"], default="0-360",
                    help="Angle wrap range for output")
    args = ap.parse_args()

    if not (args.unlatch_speed > args.latch_speed):
        raise ValueError("unlatch-speed must be strictly greater than latch-speed to create hysteresis.")

    in_path = Path(args.csv)
    df = pd.read_csv(in_path)

    # Sort by time if available (important for stateful filtering)
    if args.time_col in df.columns:
        df = df.sort_values(args.time_col).reset_index(drop=True)

    # Checks
    for col in [args.heading_col, args.speed_col]:
        if col not in df.columns:
            raise ValueError(f"Column '{col}' not found in CSV.")

    # Inputs
    heading_rad = pd.to_numeric(df[args.heading_col], errors="coerce").to_numpy()
    speed = pd.to_numeric(df[args.speed_col], errors="coerce").to_numpy()

    n = len(heading_rad)
    out_unwrapped = np.empty(n, dtype=float)

    # State for hysteretic latching
    latched_on = False
    latched_value = None
    last_live = None  # last "live" (unlatched) unwrapped heading

    for i in range(n):
        h = heading_rad[i]
        v = speed[i]

        # Treat NaN speeds as "no transition" band (i.e., stay in current state)
        in_latch_band = (not np.isnan(v)) and (v < args.latch_speed)
        in_unlatch_band = (not np.isnan(v)) and (v >= args.unlatch_speed)

        if latched_on:
            if in_unlatch_band:
                # Transition: unlatch, resume live
                latched_on = False
                last_live = h
                out_unwrapped[i] = h
            else:
                # Stay latched (also within hysteresis band or NaN speed)
                # If latched_value wasn't set yet (rare), fall back to h
                if latched_value is None:
                    latched_value = h
                out_unwrapped[i] = latched_value
        else:
            # Currently unlatched (live)
            if in_latch_band:
                # Transition: latch, freeze current live heading
                latched_on = True
                latched_value = last_live if last_live is not None else h
                out_unwrapped[i] = latched_value
            else:
                # Stay unlatched (live)
                out_unwrapped[i] = h
                last_live = h

    # Wrap output to requested range
    if args.output_range == "0-360":
        out_deg = (np.rad2deg(out_unwrapped) % 360.0)
    else:
        out_deg = ((np.rad2deg(out_unwrapped) + 180) % 360) - 180

    # Replace the heading column with filtered values
    df[args.heading_col] = out_unwrapped

    # Save next to input file, same folder
    out_path = in_path.with_name(f"{in_path.stem}_latched{in_path.suffix}")
    df.to_csv(out_path, index=False)
    print(f"Saved latched CSV to: {out_path}")

if __name__ == "__main__":
    main()

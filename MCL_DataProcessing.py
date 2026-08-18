"""
General post-processor for MCL bench-test data.

Python recreation of General_PostProc_OndrejTest_V1dot0_TEAMS.m
(Daphne Psarra 2023, updated Genevieve Kenney-Dwyer 2024, Ondrej 2026).

Reads 8 calibrated pressure channels + a flow channel from a TDMS file,
plots the aortic / valve / venous / atrial waveforms, estimates arterial
and atrial compliance and systemic vascular resistance (SVR), and appends
a summary row to a results CSV.

Cleaned up relative to the MATLAB source:
  - dropped a dead initial tdmsread() call that was immediately overwritten
  - dropped the unused top-level pressure_scale/offset (every channel has
    its own scale/offset; only the shared flow scale/offset was ever used)
  - dropped the Butterworth low-pass filter: it was designed and applied to
    every channel but the filtered signal was never plotted or used
  - dropped findpeaks() calls for channels whose peaks were never used
    (only Pressure 2 and Pressure 8 peaks feed the compliance calculations)
  - dropped ~100 lines of commented-out alternate SVR-logging code and an
    unused venous-compliance block
  - replaced repeated subplot/axhline boilerplate with plot_panels()
  - file paths, voltage and pulse are now CLI overridable instead of
    requiring source edits per test run
  - the tdms "Flow" channel's calibration (flow_scale=10 in the MATLAB
    source) was found to disagree with the separately-logged flow-meter
    CSV by 5-7x; it's now plotted raw/uncalibrated with the CSV average
    annotated alongside it instead of silently mislabeling the y-axis

Usage
-----
Single run — either or both of --tdms/--flow-csv may be given; BPM and
voltage are parsed from whichever filename is provided unless overridden:
    python MCL_DataProcessing.py --tdms path/to/TBH26_60BPM_5V....tdms
    python MCL_DataProcessing.py --tdms ...tdms --flow-csv ...csv

Batch mode — recursively finds *.tdms/*.csv under a directory, pairs files
that share a parent folder and a BPM/voltage parsed from their filenames,
and appends one row per pairing to --output-log (plot windows suppressed):
    python MCL_DataProcessing.py --dir MCL_DATA/
"""

from __future__ import annotations

import argparse
import re
import sys
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from nptdms import TdmsFile
from scipy.signal import find_peaks

# --------------------------------------------------------------------------
# Defaults — override any of these from the command line, see --help
# --------------------------------------------------------------------------
DEFAULT_GROUP_NAME = "All Data"
DEFAULT_OUTPUT_LOG = "data_log_SVR_1.csv"

# 1-indexed, inclusive rows of the flow CSV's first data column to average
FLOW_AVG_ROWS = (1000, 27000)

SAMPLING_FREQUENCY_HZ = 100
PLOT_RANGE = range(300, 1900)  # main waveform window, samples
MAP_RANGE = range(400, 800)  # window used for the max-MAP estimate

# mmHg = raw * scale + offset, per pressure channel
PRESSURE_CALIBRATION = {
    1: (64.02, -33.29),
    2: (63.70, -33.76),
    3: (63.70, -30.57),
    4: (63.39, -32.33),
    5: (63.39, -29.79),
    6: (63.39, -29.79),
    7: (63.24, -29.09),
    8: (63.55, -31.13),
}
# NOTE: the tdms "Flow" channel has no validated calibration. The script's
# old flow_scale=10 constant (and even the DAQ's own recorded scale of 7.5)
# produce values 5-7x higher than the separately-logged flow-meter CSV for
# the same test run, so the channel is plotted raw/uncalibrated below and
# the CSV-derived average is used for CO/SVR instead.

# Compliance-chamber geometry / fluid properties
G = 9.81  # m/s^2
K_ADIABATIC = 1.4  # adiabatic coefficient of air
RHO = 997  # kg/m^3, working liquid density
MMHG_PER_M = 0.00750062 * 1000  # Pa -> mmHg, applied to rho*g*h in metres

ARTERY_CHAMBER = dict(stopper_diameter_mm=104, stopper_height_mm=140, liquid_height_mm=100)
ATRIUM_CHAMBER = dict(stopper_diameter_mm=150, stopper_height_mm=550, liquid_height_mm=20)

# Test filenames encode their run params, e.g. "TBH26_60BPM_5V 18Aug2026
# 12h24m05s.tdms" or "..._FLOW_60BPM_1.5V_...csv" -> (bpm=60, voltage=5.0)
FILENAME_PATTERN = re.compile(r"(\d+)BPM_(\d+(?:\.\d+)?)V", re.IGNORECASE)


def parse_bpm_voltage(filename: str) -> tuple[int, float] | None:
    match = FILENAME_PATTERN.search(filename)
    if match is None:
        return None
    return int(match.group(1)), float(match.group(2))


# --------------------------------------------------------------------------
# Data loading
# --------------------------------------------------------------------------
def read_channel(group, name: str, scale: float, offset: float) -> np.ndarray:
    if name not in [ch.name for ch in group.channels()]:
        available = ", ".join(ch.name for ch in group.channels())
        raise LookupError(f"Channel '{name}' not found in group '{group.name}'. Available: {available}")
    return group[name][:] * scale + offset


def average_flow_from_csv(path: Path, row_range: tuple[int, int]) -> float:
    """Mean of the first data column over a 1-indexed inclusive row range.

    Matches MATLAB's readmatrix()+range indexing: readmatrix auto-skips a
    non-numeric header row, and rows are addressed 1-indexed from there.
    """
    start, end = row_range
    flow_df = pd.read_csv(path)
    column_a = flow_df.iloc[:, 0]
    return column_a.iloc[start - 1 : end].mean(skipna=True)


# --------------------------------------------------------------------------
# Signal analysis
# --------------------------------------------------------------------------
def peak_trough_means(signal: np.ndarray, peak_distance: int, trough_distance: int) -> tuple[float, float]:
    """Mean of the local maxima and mean of the local minima of a windowed signal."""
    peak_idx, _ = find_peaks(signal, distance=peak_distance)
    trough_idx, _ = find_peaks(-signal, distance=trough_distance)
    return signal[peak_idx].mean(), -signal[trough_idx].mean()


def hydrostatic_pressure_mmhg(liquid_height_mm: float) -> float:
    return RHO * G * (liquid_height_mm / 1000) * 0.00750062


def air_volume_ml(diameter_mm: float, height_mm: float) -> float:
    return np.pi * (diameter_mm / 2) ** 2 * height_mm * 0.001


def pneumatic_compliance(chamber: dict, p_max: float, p_min: float) -> float:
    """Isentropic gas-spring compliance estimate C = dV/dP (ml/mmHg)."""
    v0 = air_volume_ml(chamber["stopper_diameter_mm"], chamber["stopper_height_mm"])
    dp = p_max - p_min
    p0 = p_min + 760 - hydrostatic_pressure_mmhg(chamber["liquid_height_mm"])
    return v0 * (1 - (p0 / (p0 + dp)) ** (1 / K_ADIABATIC)) / dp


# --------------------------------------------------------------------------
# Plotting
# --------------------------------------------------------------------------
def plot_panels(fig_name: str, time: np.ndarray, panels: list[dict]) -> plt.Figure:
    """panels: [{signal, title, ylabel='Pressure (mmHg)', color=None, hlines=[(value, label), ...]}]"""
    fig, axes = plt.subplots(len(panels), 1, num=fig_name, figsize=(8, 3 * len(panels)))
    axes = np.atleast_1d(axes)
    for ax, panel in zip(axes, panels):
        ax.plot(time, panel["signal"], color=panel.get("color"))
        ax.set_title(panel["title"])
        ax.set_xlabel("Time (s)")
        ax.set_ylabel(panel.get("ylabel", "Pressure (mmHg)"))
        for value, label in panel.get("hlines", []):
            ax.axhline(value, linestyle="--", color="r", label=label)
        if panel.get("hlines"):
            ax.legend()
        if panel.get("annotation"):
            ax.text(
                0.02,
                0.95,
                panel["annotation"],
                transform=ax.transAxes,
                va="top",
                ha="left",
                fontsize=8,
                bbox=dict(boxstyle="round", facecolor="white", alpha=0.8),
            )
    fig.tight_layout()
    return fig


# --------------------------------------------------------------------------
# Per-dataset processing
# --------------------------------------------------------------------------
def process_dataset(
    tdms_path: Path | None,
    flow_csv_path: Path | None,
    pulse: int,
    voltage: float,
    group_name: str,
    output_log: Path,
    show: bool,
    show_tap28: bool = True,
) -> None:
    """Process one (tdms, flow-csv) pairing, either of which may be absent, and
    append one row to output_log."""
    if tdms_path is None and flow_csv_path is None:
        raise ValueError("need at least one of tdms_path/flow_csv_path")

    print(f"--- Pulse={pulse} BPM, Voltage={voltage} V ---")

    average_flow_value = None
    if flow_csv_path is not None:
        average_flow_value = average_flow_from_csv(flow_csv_path, FLOW_AVG_ROWS)  # ml/min
        print(f"Average flow (rows {FLOW_AVG_ROWS[0]}-{FLOW_AVG_ROWS[1]}): {average_flow_value:.4f} ml/min")
    else:
        print("No flow CSV provided — CO/SVR will be left blank")

    pressure2_mean = pressure8_mean = None
    figs: list[plt.Figure] = []

    if tdms_path is not None:
        group = TdmsFile.read(tdms_path)[group_name]

        time = np.array(PLOT_RANGE) / SAMPLING_FREQUENCY_HZ
        plot_slice = slice(PLOT_RANGE.start, PLOT_RANGE.stop)
        map_slice = slice(MAP_RANGE.start, MAP_RANGE.stop)

        pressures = {
            n: read_channel(group, f"Pressure {n}", *PRESSURE_CALIBRATION[n])[plot_slice] for n in range(1, 9)
        }
        flow_raw = read_channel(group, "Flow", 1, 0)[plot_slice]  # uncalibrated, see NOTE above

        pressure2_mean = pressures[2].mean()
        max_map = pressures[2][map_slice].mean() if MAP_RANGE.stop <= len(pressures[2]) else np.nan
        max_pressure2, min_pressure2 = peak_trough_means(pressures[2], peak_distance=50, trough_distance=50)
        max_pressure8, min_pressure8 = peak_trough_means(pressures[8], peak_distance=49, trough_distance=48)
        pressure3_mean, pressure4_mean = pressures[3].mean(), pressures[4].mean()
        pressure5_mean, pressure6_mean = pressures[5].mean(), pressures[6].mean()
        max_pressure5, _ = peak_trough_means(pressures[5], peak_distance=1, trough_distance=1)
        max_pressure6, _ = peak_trough_means(pressures[6], peak_distance=1, trough_distance=1)
        pressure8_mean = pressures[8].mean()

        print(f"Pressure 2 mean (MAP): {pressure2_mean:.2f} mmHg")
        print(f"Max MAP (rows {MAP_RANGE.start}-{MAP_RANGE.stop}): {max_map:.2f} mmHg")

        if show_tap28:
            figs.append(
                plot_panels(
                    "Pressure Taps 2 & 8",
                    time,
                    [
                        dict(signal=pressures[2], title="Pressure 2 (MAP)", color="tab:blue"),
                        dict(signal=pressures[8], title="Pressure 8 (RAP)", color="tab:red"),
                    ],
                )
            )

        if average_flow_value is not None:
            flow_annotation = (
                f"Flow-meter avg (CSV): {average_flow_value / 1000:.2f} L/min\n"
                f"tdms channel calibration unvalidated —\n"
                f"see NOTE in source"
            )
        else:
            flow_annotation = "No flow CSV provided.\ntdms channel calibration unvalidated — see NOTE in source"

        figs.append(
            plot_panels(
                "Aortic waveforms (arterial compliance inputs)",
                time,
                [
                    dict(signal=pressures[1], title="Pressure 1", color="black"),
                    dict(signal=pressures[2], title="Pressure 2", color="black"),
                ],
            )
        )
        figs.append(
            plot_panels(
                "Diaphragm valve resistance",
                time,
                [
                    dict(
                        signal=pressures[3], title="Pressure 3", hlines=[(pressure3_mean, f"{pressure3_mean:.2f} mmHg")]
                    ),
                    dict(
                        signal=pressures[4], title="Pressure 4", hlines=[(pressure4_mean, f"{pressure4_mean:.2f} mmHg")]
                    ),
                    dict(
                        signal=flow_raw,
                        title="Flow Rate (tdms channel, uncalibrated)",
                        ylabel="Flow (raw signal)",
                        annotation=flow_annotation,
                    ),
                ],
            )
        )
        figs.append(
            plot_panels(
                "Venous waveforms",
                time,
                [
                    dict(
                        signal=pressures[5],
                        title="Pressure 5",
                        hlines=[(pressure5_mean, f"{pressure5_mean:.2f}"), (max_pressure5, f"{max_pressure5:.2f}")],
                    ),
                    dict(
                        signal=pressures[6],
                        title="Pressure 6",
                        hlines=[(pressure6_mean, f"{pressure6_mean:.2f}"), (max_pressure6, f"{max_pressure6:.2f}")],
                    ),
                ],
            )
        )
        figs.append(
            plot_panels(
                "Atrial waveforms (atrial compliance inputs)",
                time,
                [
                    dict(signal=pressures[7], title="Pressure 7", color="black"),
                    dict(signal=pressures[8], title="Pressure 8", color="black"),
                ],
            )
        )

        c_arterial = pneumatic_compliance(ARTERY_CHAMBER, max_pressure2, min_pressure2)
        c_atrial = pneumatic_compliance(ATRIUM_CHAMBER, max_pressure8, min_pressure8)
        print(f"Arterial compliance: {c_arterial:.4f} ml/mmHg")
        print(f"Atrial compliance: {c_atrial:.4f} ml/mmHg")
    else:
        print("No tdms file provided — AoP/RAP/compliance/plots will be left blank")

    svr = None
    if pressure2_mean is not None and pressure8_mean is not None and average_flow_value is not None:
        svr = ((pressure2_mean - pressure8_mean) / (average_flow_value / 1000)) * 80
        print(f"SVR: {svr:.4f} dyn.s/cm^5")

    row = pd.DataFrame(
        [
            {
                "Pulse": pulse,
                "Voltage": voltage,
                "AoP": pressure2_mean,
                "RAP": pressure8_mean,
                "CO": average_flow_value,
                "SVR": svr,
                "TDMS_File": tdms_path.name if tdms_path is not None else "",
                "Flow_CSV_File": flow_csv_path.name if flow_csv_path is not None else "",
            }
        ]
    )
    file_exists = output_log.exists()
    row.to_csv(output_log, mode="a" if file_exists else "w", header=not file_exists, index=False)
    print(f"Logged result to {output_log}")

    if show:
        plt.show()
    else:
        for fig in figs:
            plt.close(fig)


# --------------------------------------------------------------------------
# Batch mode
# --------------------------------------------------------------------------
def run_batch(root: Path, group_name: str, output_log: Path, show_tap28: bool, show: bool) -> None:
    if not root.exists():
        sys.exit(f"Error: directory not found: {root}")

    candidates = sorted(root.rglob("*.tdms")) + sorted(
        p for p in root.rglob("*.csv") if p.resolve() != output_log.resolve()
    )

    groups: dict[tuple[Path, int, float], dict[str, list[Path]]] = {}
    skipped = []
    for path in candidates:
        parsed = parse_bpm_voltage(path.name)
        if parsed is None:
            skipped.append(path)
            continue
        key = (path.parent, *parsed)
        bucket = groups.setdefault(key, {"tdms": [], "csv": []})
        bucket["tdms" if path.suffix.lower() == ".tdms" else "csv"].append(path)

    if skipped:
        print(f"Skipping {len(skipped)} file(s) — could not parse BPM/voltage from filename:")
        for path in skipped:
            print(f"  {path}")

    if not groups:
        sys.exit(f"Error: no BPM/voltage-parseable .tdms or .csv files found under {root}")

    for (parent, pulse, voltage), bucket in sorted(groups.items(), key=lambda kv: (str(kv[0][0]), kv[0][1], kv[0][2])):
        tdms_matches, csv_matches = bucket["tdms"], bucket["csv"]
        if len(tdms_matches) > 1 or len(csv_matches) > 1:
            print(
                f"Warning: ambiguous match for {pulse}BPM_{voltage}V in {parent} "
                f"({len(tdms_matches)} tdms, {len(csv_matches)} csv) — skipping, disambiguate filenames"
            )
            continue

        tdms_path = tdms_matches[0] if tdms_matches else None
        csv_path = csv_matches[0] if csv_matches else None
        print(f"\n=== {pulse} BPM, {voltage} V  ({parent}) ===")
        try:
            process_dataset(
                tdms_path, csv_path, pulse, voltage, group_name, output_log, show=show, show_tap28=show_tap28
            )
        except (LookupError, ValueError) as exc:
            print(f"  Error processing this group: {exc}")


# --------------------------------------------------------------------------
# Main
# --------------------------------------------------------------------------
def parse_args(argv=None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__.strip().splitlines()[0])
    parser.add_argument("--tdms", type=Path, default=None, help="pressure/flow TDMS file (optional)")
    parser.add_argument("--flow-csv", type=Path, default=None, help="separate flow-meter log CSV (optional)")
    parser.add_argument(
        "--dir",
        type=Path,
        default=None,
        help="batch mode: recursively process all tdms/csv under this directory, "
        "paired by BPM/voltage parsed from filenames; cannot be combined with --tdms/--flow-csv",
    )
    parser.add_argument("--group", default=DEFAULT_GROUP_NAME, help="TDMS channel group name")
    parser.add_argument("--output-log", type=Path, default=Path(DEFAULT_OUTPUT_LOG), help="results CSV to append to")
    parser.add_argument(
        "--voltage", type=float, default=None, help="pump drive voltage; parsed from filename if omitted"
    )
    parser.add_argument("--pulse", type=int, default=None, help="pump rate (BPM); parsed from filename if omitted")
    parser.add_argument(
        "--no-show", action="store_true", help="save/compute but don't open plot windows (ignored in --dir mode)"
    )
    parser.add_argument(
        "--show",
        action="store_true",
        help="--dir mode only: open plot windows per dataset, pausing (blocking on each "
        "dataset's windows until closed) before moving to the next one; off by default",
    )
    parser.add_argument(
        "--tap28-plot",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="show a combined Pressure 2 & 8 window per dataset (default: on; use --no-tap28-plot to disable)",
    )
    return parser.parse_args(argv)


def resolve_pulse_voltage(
    pulse: int | None, voltage: float | None, tdms_path: Path | None, flow_csv_path: Path | None
) -> tuple[int, float]:
    if pulse is not None and voltage is not None:
        return pulse, voltage

    parsed = None
    for path in (tdms_path, flow_csv_path):
        if path is not None:
            parsed = parse_bpm_voltage(path.name)
            if parsed is not None:
                break

    if parsed is None:
        sys.exit("Error: could not parse BPM/voltage from filename — pass --pulse and --voltage explicitly")

    parsed_pulse, parsed_voltage = parsed
    return (pulse if pulse is not None else parsed_pulse, voltage if voltage is not None else parsed_voltage)


def main(argv=None) -> None:
    args = parse_args(argv)

    if args.dir is not None:
        if args.tdms is not None or args.flow_csv is not None:
            sys.exit("Error: --dir cannot be combined with --tdms/--flow-csv")
        run_batch(args.dir, args.group, args.output_log, args.tap28_plot, args.show)
        return

    if args.tdms is None and args.flow_csv is None:
        sys.exit("Error: provide --tdms and/or --flow-csv (or --dir for batch mode)")

    for path in (args.tdms, args.flow_csv):
        if path is not None and not path.exists():
            sys.exit(f"Error: file not found: {path}")

    pulse, voltage = resolve_pulse_voltage(args.pulse, args.voltage, args.tdms, args.flow_csv)

    process_dataset(
        args.tdms,
        args.flow_csv,
        pulse,
        voltage,
        args.group,
        args.output_log,
        show=not args.no_show,
        show_tap28=args.tap28_plot,
    )


if __name__ == "__main__":
    try:
        main()
    except (LookupError, ValueError) as exc:
        sys.exit(f"Error: {exc}")

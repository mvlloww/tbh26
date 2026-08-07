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
"""

from __future__ import annotations

import argparse
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
DEFAULT_TDMS_FILE = "1.5V_FILLING_ALLOWED 12Jun2026 12h18m38s.tdms"
DEFAULT_FLOW_CSV_FILE = "20260724-TBH26_FLOW_60BPM_5V_Test124Jul2026 12h50m31s.csv"
DEFAULT_GROUP_NAME = "All Data"
DEFAULT_OUTPUT_LOG = "data_log_SVR_1.csv"
DEFAULT_VOLTAGE = 5
DEFAULT_PULSE_BPM = 60

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
# Main
# --------------------------------------------------------------------------
def parse_args(argv=None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__.strip().splitlines()[0])
    parser.add_argument("--tdms", type=Path, default=Path(DEFAULT_TDMS_FILE), help="pressure/flow TDMS file")
    parser.add_argument(
        "--flow-csv", type=Path, default=Path(DEFAULT_FLOW_CSV_FILE), help="separate flow-meter log CSV"
    )
    parser.add_argument("--group", default=DEFAULT_GROUP_NAME, help="TDMS channel group name")
    parser.add_argument("--output-log", type=Path, default=Path(DEFAULT_OUTPUT_LOG), help="results CSV to append to")
    parser.add_argument("--voltage", type=float, default=DEFAULT_VOLTAGE, help="pump drive voltage, for the log row")
    parser.add_argument("--pulse", type=int, default=DEFAULT_PULSE_BPM, help="pump rate (BPM), for the log row")
    parser.add_argument("--no-show", action="store_true", help="save/compute but don't open plot windows")
    return parser.parse_args(argv)


def main(argv=None) -> None:
    args = parse_args(argv)

    for path in (args.tdms, args.flow_csv):
        if not path.exists():
            sys.exit(f"Error: file not found: {path}")

    group = TdmsFile.read(args.tdms)[args.group]

    average_flow_value = average_flow_from_csv(args.flow_csv, FLOW_AVG_ROWS)  # ml/min
    average_flow_lpm = average_flow_value / 1000
    print(f"Average flow (rows {FLOW_AVG_ROWS[0]}-{FLOW_AVG_ROWS[1]}): {average_flow_value:.4f} ml/min")

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

    plot_panels(
        "Aortic waveforms (arterial compliance inputs)",
        time,
        [
            dict(signal=pressures[1], title="Pressure 1", color="black"),
            dict(signal=pressures[2], title="Pressure 2", color="black"),
        ],
    )
    plot_panels(
        "Diaphragm valve resistance",
        time,
        [
            dict(signal=pressures[3], title="Pressure 3", hlines=[(pressure3_mean, f"{pressure3_mean:.2f} mmHg")]),
            dict(signal=pressures[4], title="Pressure 4", hlines=[(pressure4_mean, f"{pressure4_mean:.2f} mmHg")]),
            dict(
                signal=flow_raw,
                title="Flow Rate (tdms channel, uncalibrated)",
                ylabel="Flow (raw signal)",
                annotation=f"Flow-meter avg (CSV): {average_flow_lpm:.2f} L/min\n"
                f"tdms channel calibration unvalidated —\n"
                f"see NOTE in source",
            ),
        ],
    )
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
    plot_panels(
        "Atrial waveforms (atrial compliance inputs)",
        time,
        [
            dict(signal=pressures[7], title="Pressure 7", color="black"),
            dict(signal=pressures[8], title="Pressure 8", color="black"),
        ],
    )

    c_arterial = pneumatic_compliance(ARTERY_CHAMBER, max_pressure2, min_pressure2)
    c_atrial = pneumatic_compliance(ATRIUM_CHAMBER, max_pressure8, min_pressure8)
    print(f"Arterial compliance: {c_arterial:.4f} ml/mmHg")
    print(f"Atrial compliance: {c_atrial:.4f} ml/mmHg")

    svr = ((pressure2_mean - pressure8_mean) / (average_flow_value / 1000)) * 80
    print(f"SVR: {svr:.4f} dyn.s/cm^5")

    row = pd.DataFrame(
        [
            {
                "Pulse": args.pulse,
                "Voltage": args.voltage,
                "AoP": pressure2_mean,
                "RAP": pressure8_mean,
                "CO": average_flow_value,
                "SVR": svr,
            }
        ]
    )
    file_exists = args.output_log.exists()
    row.to_csv(args.output_log, mode="a" if file_exists else "w", header=not file_exists, index=False)
    print(f"Logged result to {args.output_log}")

    if not args.no_show:
        plt.show()


if __name__ == "__main__":
    try:
        main()
    except (LookupError, ValueError) as exc:
        sys.exit(f"Error: {exc}")


<span id="readme-top"></span>

# Mu3e IP Library
<p>
  <a href="#license">
    <img src="https://img.shields.io/badge/license-Academic%20Evaluation-lightgrey.svg" alt="License badge" height="30" />
  </a>
  <a href="#">
    <img src="https://img.shields.io/badge/build-passing-brightgreen.svg" alt="Build status badge" height="30" />
  </a>
  <a href="#">
    <img src="https://img.shields.io/badge/platform-FPGA-blue.svg" alt="Platform badge" height="30" />
  </a>
</p>

<p align="middle">
  <a href="https://github.com/yifeng-ethz/mu3e-ip-cores">
    <img src="./quartus_system/logo/Logo_drawing300.png" alt="Mu3e Logo" width="150" />
  </a>
</p>


---

## Mu3e Experiment

The [Mu3e experiment](https://www.psi.ch/de/mu3e) at PSI pursues one of the most sensitive tests of charged lepton flavour violation: the decay $\mu^{+} \rightarrow e^{+} e^{-} e^{+}$. Within the Standard Model this channel is suppressed to branching ratios below $10^{-54}$, so any observation would point directly to new physics such as heavy neutral leptons, supersymmetric particles, or multi-TeV gauge bosons. Mu3e therefore aims for single-event sensitivities close to $\mathcal{O}(10^{-16})$ in Phase I and below $\mathcal{O}(10^{-16})$ in the Phase II upgrade, advancing $\mathcal{O}(10^{4})$ comparing to latest experiment SINDRUM in 1980s.

The collaboration also records so-called candle modes that are predicted by the Standard Model and serve as in-situ calibrations. The internal-conversion decay $\mu^{+} \rightarrow e^{+} e^{-} e^{+} \nu \bar{\nu} |\ B \approx 3.4 \times 10^{-5}$ ($3e2\nu$) and the rarer five-charged-lepton channel $\mu^{+} \rightarrow e^{+} e^{-} e^{+} e^{-} e^{+} \nu \bar{\nu}\ |\ B \approx (3.9–4.0) \times 10^{-10}$ ($5e2\nu$) provide precise control samples to validate momentum reconstruction, timing alignment, and background modelling. To note, $5e2\nu$ is the rarest measurable muon decay and Mu3e will be the first to observe it.

Achieving the design sensitivity requires an ultra-light detector composed of HV-MAPS MuPix pixels, scintillating fibre and tile timing layers, and a 1 T solenoidal field for momemtum and vertex resoluting on track-by-track bases (acceptance close to cover the full Michel spectrum, i.e., 10 MeV to 53 MeV) of $10^{8}$ muons per second in Phase I. The planned Phase II upgrade pushes the rate to $10^{9}$ muons per second using the High-Intensity Muon Beam (HiMB) line, demanding redesign part of the front-end chip with higher bandwidth and upgrade our triggerless DAQ capable to real-time reconstruct multi-terabit-per-second data streams. The IP cores in this repository includes the FPGA building blocks that make that readout strategy feasible.

---

## IP List

Each Mu3e IP core resides in its own subdirectory of this repository or in an external submodule. Cores marked as **Prototype** have no `VERSION` property and are still under development.

<table>
  <thead>
    <tr>
      <th rowspan="2">IP Name</th>
      <th rowspan="2">Description</th>
      <th colspan="2">Signoff</th>
    </tr>
    <tr>
      <th>DV</th>
      <th>SYN</th>
    </tr>
  </thead>
  <tbody>
    <tr>
      <td><a href="https://github.com/yifeng-ethz/slow-control_hub"><strong>Slow-Control Hub</strong></a></td>
      <td>Converts Mu3e slow-control packets into Avalon Memory-Mapped transactions and handles burst count, address and response timing. <img src="https://img.shields.io/badge/version-26.6.9.0414-blue" alt="version 26.6.9.0414 badge" /></td>
      <td align="center"><a href="https://github.com/yifeng-ethz/slow-control_hub/blob/1fe83feac36337492bbc69728c66c958cafd36ae/doc/VERIFICATION_SIGNOFF.md"><img src="https://img.shields.io/badge/-%E2%9C%85-brightgreen?style=flat-square" alt="DV signoff available" /></a></td>
      <td align="center"><a href="https://github.com/yifeng-ethz/slow-control_hub/blob/1fe83feac36337492bbc69728c66c958cafd36ae/doc/SYNTHESIS_SIGNOFF.md"><img src="https://img.shields.io/badge/-%E2%9C%85-brightgreen?style=flat-square" alt="SYN signoff available" /></a></td>
    </tr>
    <tr>
      <td><a href="https://github.com/yifeng-ethz/onewire_temp_sense"><strong>Onewire Temperature Sensor Controller</strong></a></td>
      <td>Periodically polls 1-Wire temperature sensors; implements reset, presence detect and bit-level timing. <img src="https://img.shields.io/badge/version-24.0.918-blue" alt="version 24.0.918 badge" /></td>
      <td align="center">—</td>
      <td align="center">—</td>
    </tr>
    <tr>
      <td><a href="https://github.com/yifeng-ethz/mutrig_frame_deassembly"><strong>MuTRiG Frame Deassembly</strong></a></td>
      <td>Parses MuTRiG frames into header and hit payloads and flags individual hit errors and frame CRC errors. <img src="https://img.shields.io/badge/version-26.0.1.0415-blue" alt="version 26.0.1.0415 badge" /></td>
      <td align="center"><a href="https://github.com/yifeng-ethz/mutrig_frame_deassembly/blob/8af676e18d86f87c34ba98de33f120e04e44bfa3/doc/SIGNOFF.md"><img src="https://img.shields.io/badge/-%E2%9C%85-brightgreen?style=flat-square" alt="DV signoff available" /></a></td>
      <td align="center"><a href="https://github.com/yifeng-ethz/mutrig_frame_deassembly/blob/8af676e18d86f87c34ba98de33f120e04e44bfa3/doc/SIGNOFF.md"><img src="https://img.shields.io/badge/-%E2%9C%85-brightgreen?style=flat-square" alt="SYN signoff available" /></a></td>
    </tr>
    <tr>
      <td><a href="https://github.com/yifeng-ethz/CAM"><strong>CAM (Content Addressable Memory)</strong></a></td>
      <td>Primitive content-addressable memory core. Use as a building block for caches, correlators and address decoders. <img src="https://img.shields.io/badge/version-17.0.2-blue" alt="version 17.0.2 badge" /></td>
      <td align="center">—</td>
      <td align="center">—</td>
    </tr>
    <tr>
      <td><a href="https://github.com/yifeng-ethz/mutrig_timestamp_processor"><strong>MuTRiG Timestamp Processor</strong></a></td>
      <td>Tracks MuTRiG timestamp overflow and maps MuTRiG-local timestamps to global timestamps. <img src="https://img.shields.io/badge/version-26.0.1.0415-blue" alt="version 26.0.1.0415 badge" /></td>
      <td align="center"><a href="https://github.com/yifeng-ethz/mutrig_timestamp_processor/blob/98691c1d1b43420c8fcd0a063271c39ff26d2af2/tb/DV_REPORT.md"><img src="https://img.shields.io/badge/-%E2%9C%85-brightgreen?style=flat-square" alt="DV signoff available" /></a></td>
      <td align="center">—</td>
    </tr>
    <tr>
      <td><a href="https://github.com/yifeng-ethz/histogram_statistics"><strong>Histogram Statistics</strong></a></td>
      <td>Builds histograms from a selected data stream using SAR bin calculation and DP-RAM counters. <img src="https://img.shields.io/badge/version-26.1.1.0416-blue" alt="version 26.1.1.0416 badge" /></td>
      <td align="center"><a href="https://github.com/yifeng-ethz/histogram_statistics/blob/5a39f5159c4b1fcb506c1bcd4bf28e497b0b9600/VERIFICATION_SIGNOFF.md"><img src="https://img.shields.io/badge/-%E2%9C%85-brightgreen?style=flat-square" alt="DV signoff available" /></a></td>
      <td align="center"><a href="https://github.com/yifeng-ethz/histogram_statistics/blob/5a39f5159c4b1fcb506c1bcd4bf28e497b0b9600/SYNTHESIS_SIGNOFF.md"><img src="https://img.shields.io/badge/-%E2%9C%85-brightgreen?style=flat-square" alt="SYN signoff available" /></a></td>
    </tr>
    <tr>
      <td><a href="https://github.com/yifeng-ethz/mutrig_controller"><strong>MuTRiG Controller</strong></a></td>
      <td>SPI master for configuring MuTRiG ASICs. Automatically scans T- and E-thresholds and stores results locally. <img src="https://img.shields.io/badge/version-24.0.817-blue" alt="version 24.0.817 badge" /></td>
      <td align="center">—</td>
      <td align="center">—</td>
    </tr>
    <tr>
      <td><a href="https://github.com/yifeng-ethz/charge_injection"><strong>Charge Injection (Analog Pulser)</strong></a></td>
      <td>Generates calibration pulses with arbitrary frequency and duration for TDC injection tests. <img src="https://img.shields.io/badge/version-4.0.5-blue" alt="version 4.0.5 badge" /></td>
      <td align="center">—</td>
      <td align="center">—</td>
    </tr>
    <tr>
      <td><a href="https://github.com/yifeng-ethz/charge_injection"><strong>Charge Injection (MuTRiG Injector)</strong></a></td>
      <td>Produces digital and analog pulses to verify MuTRiG operation when used with DAB boards. <img src="https://img.shields.io/badge/version-26.0.0326-blue" alt="version 26.0.0326 badge" /></td>
      <td align="center">—</td>
      <td align="center">—</td>
    </tr>
    <tr>
      <td><a href="https://github.com/yifeng-ethz/alt_temp_sense_controller"><strong>Altera Temperature Sensor Controller</strong></a></td>
      <td>Wraps the on-chip <code>alt_temp_sense</code> IP on 28 nm devices and stores the last temperature result. <img src="https://img.shields.io/badge/version-1.1-blue" alt="version 1.1 badge" /></td>
      <td align="center">—</td>
      <td align="center">—</td>
    </tr>
    <tr>
      <td><a href="https://github.com/yifeng-ethz/high_performance_counter_array"><strong>High Performance Counter Array</strong></a></td>
      <td>Parallel counters supporting concurrent inputs with Avalon-MM readout. Features synchronous clear and reset. <img src="https://img.shields.io/badge/version-1.4.1-blue" alt="version 1.4.1 badge" /></td>
      <td align="center">—</td>
      <td align="center">—</td>
    </tr>
    <tr>
      <td><a href="https://github.com/yifeng-ethz/mutrig_channel_counter_fabric"><strong>MuTRiG Channel Counter Fabric</strong></a></td>
      <td>Connects hit type 0 from the frame deassembly IP to the counter array and decodes channel IDs into one-hot update signals. <img src="https://img.shields.io/badge/version-1.0.12-blue" alt="version 1.0.12 badge" /></td>
      <td align="center">—</td>
      <td align="center">—</td>
    </tr>
    <tr>
      <td><a href="https://github.com/yifeng-ethz/lvds_error_counter_fabric"><strong>LVDS Error Counter Fabric</strong></a></td>
      <td>Accumulates parity and decode error counts from the LVDS receiver sideband. <img src="https://img.shields.io/badge/version-1.0.5-blue" alt="version 1.0.5 badge" /></td>
      <td align="center">—</td>
      <td align="center">—</td>
    </tr>
    <tr>
      <td><a href="https://github.com/yifeng-ethz/firefly_xcvr_i2c_master"><strong>Firefly Transceiver I2C Master</strong></a></td>
      <td>Interfaces with the Samtec Firefly optical transceiver module via I2C and periodically reads temperature and RX power. <img src="https://img.shields.io/badge/version-26.0.0330-blue" alt="version 26.0.0330 badge" /></td>
      <td align="center">—</td>
      <td align="center">—</td>
    </tr>
    <tr>
      <td><a href="https://github.com/yifeng-ethz/ip_8b10b_decoder"><strong>IP 8b/10b Decoder</strong></a></td>
      <td>Standard 8b/10b decoder for parallel LVDS <code>rxout</code> data and decoding/parity error derivation. <img src="https://img.shields.io/badge/version-1.3.1-blue" alt="version 1.3.1 badge" /></td>
      <td align="center">—</td>
      <td align="center">—</td>
    </tr>
    <tr>
      <td><a href="https://github.com/yifeng-ethz/mutrig_reset_controller"><strong>MuTRiG Reset Controller</strong></a></td>
      <td>Issues reset pulses for the MuTRiG based on run-state changes and provides programmable phase shift via <code>alt_pll_reconfig</code>. <img src="https://img.shields.io/badge/version-1.0.8-blue" alt="version 1.0.8 badge" /></td>
      <td align="center">—</td>
      <td align="center">—</td>
    </tr>
    <tr>
      <td><a href="https://github.com/yifeng-ethz/ring-buffer_cam"><strong>Ring-buffer CAM</strong></a></td>
      <td>Circular buffer variant of CAM with push-to-stack write semantics and cache-like read-through. Used to build the hit stack. <img src="https://img.shields.io/badge/version-26.1.4.0402-blue" alt="version 26.1.4.0402 badge" /></td>
      <td align="center"><a href="https://github.com/yifeng-ethz/ring-buffer_cam/blob/3c512dd6fd8c2e287dd77fecef9e07f30165d0a3/doc/SIGNOFF.md"><img src="https://img.shields.io/badge/-%E2%9C%85-brightgreen?style=flat-square" alt="DV signoff available" /></a></td>
      <td align="center"><a href="https://github.com/yifeng-ethz/ring-buffer_cam/blob/3c512dd6fd8c2e287dd77fecef9e07f30165d0a3/doc/SIGNOFF.md"><img src="https://img.shields.io/badge/-%E2%9C%85-brightgreen?style=flat-square" alt="SYN signoff available" /></a></td>
    </tr>
    <tr>
      <td><a href="https://github.com/yifeng-ethz/feb_frame_assembly"><strong>Frontend-Board Frame Assembly</strong></a></td>
      <td>Assembles time-interleaved subframes from the ring-buffer CAM into Mu3e-standard data frames and schedules packet transmission. <img src="https://img.shields.io/badge/version-26.0.0328-blue" alt="version 26.0.0328 badge" /></td>
      <td align="center">—</td>
      <td align="center">—</td>
    </tr>
    <tr>
      <td><a href="https://github.com/yifeng-ethz/feb_max10_comm"><strong>FEB MAX10 Communication Bridge</strong></a></td>
      <td>FEB-side Arria V bridge that stages one flash page, crosses it into the MAX10 link domain, and preserves the downstream FEBSPI programming contract. <img src="https://img.shields.io/badge/version-0.1.0-blue" alt="version 0.1.0 badge" /></td>
      <td align="center">—</td>
      <td align="center">—</td>
    </tr>
    <tr>
      <td><a href="https://github.com/yifeng-ethz/mu3e_lvds_controller"><strong>Mu3e LVDS Controller</strong></a></td>
      <td>Provides high-speed LVDS links to the MuPix sensors using FPGA vendor IP, including 28 nm LVDS RX and Pro variants. <img src="https://img.shields.io/badge/version-25.1.0630-blue" alt="version 25.1.0630 badge" /></td>
      <td align="center">—</td>
      <td align="center">—</td>
    </tr>
    <tr>
      <td><a href="https://github.com/yifeng-ethz/mupix_inbound"><strong>MuPix Inbound</strong></a></td>
      <td>Deserializes data from MuPix chips, decodes and buffers hits. <img src="https://img.shields.io/badge/status-Prototype-lightgrey" alt="Prototype badge" /></td>
      <td align="center">—</td>
      <td align="center">—</td>
    </tr>
    <tr>
      <td><a href="https://github.com/yifeng-ethz/packet_scheduler"><strong>Packet Scheduler</strong></a></td>
      <td>Orders packets via an interface adapter and ordered-priority queues to achieve deterministic DAQ multiplexing. <img src="https://img.shields.io/badge/version-26.3.10.0414-blue" alt="version 26.3.10.0414 badge" /></td>
      <td align="center"><a href="https://github.com/yifeng-ethz/packet_scheduler/blob/bf59a0dc13b90c8d66e0beacf07a4698d332c471/doc/SIGNOFF.md"><img src="https://img.shields.io/badge/-%E2%9C%85-brightgreen?style=flat-square" alt="DV signoff available" /></a></td>
      <td align="center"><a href="https://github.com/yifeng-ethz/packet_scheduler/blob/bf59a0dc13b90c8d66e0beacf07a4698d332c471/doc/SIGNOFF.md"><img src="https://img.shields.io/badge/-%E2%9C%85-brightgreen?style=flat-square" alt="SYN signoff available" /></a></td>
    </tr>
    <tr>
      <td><a href="https://github.com/yifeng-ethz/emulator_mutrig"><strong>MuTRiG Emulator</strong></a></td>
      <td>FPGA emulator of MuTRiG 3 ASIC digital output. Produces 8b/1k frames bit-compatible with real ASIC output for FPGA-internal verification. <img src="https://img.shields.io/badge/version-26.0.3.0416-blue" alt="version 26.0.3.0416 badge" /></td>
      <td align="center"><a href="https://github.com/yifeng-ethz/emulator_mutrig/blob/e763a56020202d942338aca538fb752a8994b4e8/doc/SIGNOFF.md"><img src="https://img.shields.io/badge/-%E2%9C%85-brightgreen?style=flat-square" alt="DV signoff available" /></a></td>
      <td align="center"><a href="https://github.com/yifeng-ethz/emulator_mutrig/blob/e763a56020202d942338aca538fb752a8994b4e8/doc/SIGNOFF.md"><img src="https://img.shields.io/badge/-%E2%9C%85-brightgreen?style=flat-square" alt="SYN signoff available" /></a></td>
    </tr>
    <tr>
      <td><a href="./mutrig_lane_source_mux/"><strong>MuTRiG Lane Source Mux</strong></a></td>
      <td>Static per-lane selector that switches FEB datapath inputs between real MuTRiG decode streams and emulator streams during DAQ bring-up. <img src="https://img.shields.io/badge/version-1.0-blue" alt="version 1.0 badge" /></td>
      <td align="center">—</td>
      <td align="center">—</td>
    </tr>
    <tr>
      <td><a href="https://github.com/yifeng-ethz/run-control_mgmt"><strong>Run-Control Management</strong></a></td>
      <td>Manages run-state transitions for Mu3e subsystems and issues control signals. <img src="https://img.shields.io/badge/version-26.1.0.0413-blue" alt="version 26.1.0.0413 badge" /></td>
      <td align="center">—</td>
      <td align="center">—</td>
    </tr>
  </tbody>
</table>

`Signoff` links are checked one by one against the current checked-in file in each IP. Because many entries in this catalog are git submodules, the landing page uses absolute GitHub blob URLs pinned to the current submodule commit so the rendered web page resolves correctly. When an IP publishes a master `SIGNOFF.md`, both `DV` and `SYN` intentionally point to that master dashboard. `—` means no synthesis or verification signoff document is currently indexed from this top-level catalog.

<p align="right">(<a href="#readme-top">back to top</a>)</p>

---

## About the Project

The Mu3e detector relies on fully monolithic HV-MAPS MuPix sensors with $\mathbf{80 \times 80\ \mu m^{2}}$ pixels, providing per-hit timestamps of $\sigma\approx 20\ \mathrm{ns}$ with $\mathbf{50\ \mu m}$ thickness and serialising zero-suppressed data at up to $\mathbf{1.25\ \mathrm{Gbit/s}}$ per link. Timing layers combine $\sim 250\ \mu\mathrm{m}$ scintillating fibres read by SiPM arrays and fast plastic scintillator tiles to reach $\mathbf{< 100\ \mathrm{ps}}$ time-of-flight resolution. On L2 and L3, all sub-detectors **Front-End Boards** (FEBs) feed custom **Detector Adapter Boards** (DABs) that fan out 3380 1.25 Gbps low-voltage differential links (LVDS). These FEBs upload via optical FireFly transceivers on L4 to route traffic to **Switching Boards** (SWBs) populated with Arria 10 FPGAs for link aggregation. A full detector slice already moves in excess of $\mathbf{1.5\ \mathrm{Tbit/s}}$ during Phase I operation, with Phase II designs targeting several multi-terabit fabrics to handle the HiMB rate.

The firmware stack orchestrated by this repository spans the complete readout chain: packet schedulers for G/D/1 deterministic multiplexing, MuTRiG slow-control SPI and configuration hardware core, timestamp processors that unify the MuTRiG counting scheme (625 MHz) into the Mu3e counting scheme (125 MHz), histogramming for online monitoring, and Altera Network-on-Chip (NoC) that stitch the system together. Reference Platform Designer TCL scripts and hardware abstraction packages ensure each IP can be regenerated reproducibly, while synthesizable testbenches and Python drivers enable continuous regression testing in ModelSim/Questa environments. The codebase currently comprises more than $\mathbf{300 k}$ lines of HDL, TCL, and support scripts spread across the IP submodules. After system generation, each subdetector typically end up with synthesis RTL of $\mathcal{O}(1 M)$. 

---

## Built With

- **Intel Quartus Prime** – for FPGA synthesis and Platform Designer integration
- **ModelSim / Questa** – for simulation and verification
- **Python** – used in simulation testbenches and tooling
- **Tcl** – for Platform Designer scripts and System Console debugging

<p align="right">(<a href="#readme-top">back to top</a>)</p>

---

## Directory Structure

```text
mu3e-ip-cores/
├── CAM/                        # Content addressable memory core
├── alt_temp_sense_controller/  # On‑chip temperature diode wrapper
├── charge_injection/           # Analog pulser and MuTRiG injector variants
├── feb_frame_assembly/         # Frontend-board frame assembly
├── feb_max10_comm/             # FEB-side MAX10 flash programming bridge
├── firefly_xcvr_i2c_master/    # I2C master for Samtec Firefly transceiver
├── high_performance_counter_array/
├── histogram_statistics/
├── ip_8b10b_decoder/
├── lvds_error_counter_fabric/
├── emulator_mutrig/            # MuTRiG 3 ASIC emulator (8b/1k frame generator)
├── misc/                       # Local debug/helper IPs kept outside submodules
├── mu3e_lvds_controller/
├── mupix_inbound/
├── mutrig_channel_counter_fabric/
├── mutrig_controller/
├── mutrig_lane_source_mux/     # Static selector between real and emulated MuTRiG lanes
├── mutrig_frame_assembly/
├── mutrig_frame_deassembly/
├── mutrig_reset_controller/
├── mutrig_timestamp_processor/
├── onewire_temp_sense/         # 1-Wire controllers plus FEB v1 bridge helper
├── packet_scheduler/
├── quartus_system/             # Qsys wrapper, portable IP catalogs, and catalog scripts
├── ring-buffer_cam/
├── run-control_mgmt/
├── scripts/                    # Shared DV/report automation used across IPs
├── slow-control_hub/
└── system_console_toolkit/     # Shared System Console toolkits and board bring-up flows
```

<p align="right">(<a href="#readme-top">back to top</a>)</p>

---

## Getting Started

### Prerequisites

- **Quartus Prime 18.1** or later (18.1 recommended)
- **Python 3.10+** (for testbenches)
- **ModelSim / Questa** for simulation
- **Tcl 8.5+** for integration scripts

### Installation

1. Clone this repository and its submodules:
   ```bash
   git clone --recursive https://github.com/yifeng-ethz/mu3e-ip-cores.git
   cd mu3e-ip-cores
   ```
2. Export the repository root used by the portable Platform Designer catalogs.
   ```bash
   export MU3E_IP_CORES_ROOT=$(pwd)
   ```
3. (optional) Regenerate the normalized Platform Designer catalogs used by Quartus and downstream tooling.
   ```bash
   ./quartus_system/regenerate_ipx_catalog.sh
   ```
   This refreshes the canonical `quartus_system/components.ipx` catalog and the identical compatibility alias `quartus_system/mu3e_ip_cores.ipx`.
4. Add the catalog directory to Quartus / Platform Designer search paths.
   ```bash
   export QSYS_IP_FILE_PATH=${QSYS_IP_FILE_PATH:+$QSYS_IP_FILE_PATH:}$MU3E_IP_CORES_ROOT/quartus_system
   ```

<p align="right">(<a href="#readme-top">back to top</a>)</p>

---

## Usage

Detailed integration instructions are provided in each core’s subdirectory.  In general:
- GUI:
  1. Open Platform Designer (formerly Qsys) in Quartus.
  2. Add `quartus_system/` as an **IP search path** so Platform Designer picks up the normalized Mu3e catalog.
- CLI:
  1. Configure parameters (e.g., burst widths, channel counts) as required for your design.
    ```bash
    qsys-edit <path_to_your_system>.qsys # or leave it empty
    ```
  2. Export generated system top RTL file and connect the IP into your top‑level design or testbench.
    ```bash
    qsys-generate <path_to_your_system>.qsys --search-path="$MU3E_IP_CORES_ROOT/quartus_system,$" --synthesis=VHDL --output-directory=./generated
    ```

    

    

For simulation, the subdirectories include VHDL testbenches and Python scripts demonstrating correct operation.

---

## Contributing

Contributions to the Mu3e IP Library are welcome for academic and research purposes under the Academic Evaluation License.

As several cores and algorithms are under active patent filing, contributions must follow the rules below:

1.	Scope — Contributions are accepted for research, validation, and reproducibility. Commercial or derivative use is not permitted without written consent.
2.	Fork and Branch — Fork this repository under your academic or institutional account. Create a feature branch (e.g., feature/my_lab_fix).
3.	Document Clearly — Use descriptive commit messages. Include a concise technical summary of changes and your institutional affiliation.
4.	Submit a Pull Request — Open a PR describing your improvement, test results, or bug fix. The maintainer will review for technical compatibility and possible overlap with ongoing patent filings.
5.	Contributor Agreement — By submitting a contribution, you agree that:
	- You retain copyright of your original code.
	- You grant us a perpetual, non-exclusive right to incorporate, modify, or relicense your contribution under the same Academic Evaluation License.
	- You acknowledge that no patent rights are transferred or implied and that underlying Mu3e IP patent rights remain solely with us.

For commercial collaboration or technology-transfer discussions, please contact one of us:

  yifenwan@phys.ethz.ch \
  rwallny@phys.ethz.ch \
  mkoepp@phys.ethz.ch \
  schoning@physi.uni-heidelberg.de \
  niberger@uni-mainz.de \
    ...
    
---

## License

Mu3e IP Cores — Academic Evaluation License
Copyright (c) 2025 Mu3e Collaboration, Yifeng Wang, et al. All rights reserved.

Permission is hereby granted, free of charge, to academic researchers,
universities, and non-commercial institutions to use and evaluate this
source code for research, teaching, and internal experimental purposes only.

The following conditions apply:

1. Redistribution of this source code, in whole or in part, is prohibited.
2. Use, modification, or integration of this source code or any derivative
   work in a commercial product, service, or hardware design is strictly
   prohibited without prior written consent from the copyright holder.
3. Any publication, report, or presentation using results obtained from
   this code must include the acknowledgment:
       “Mu3e IP Cores © 2025 Mu3e Collaboration, used under Academic Evaluation License.”
4. No patent rights are granted, explicitly or implicitly. All patent rights
   related to the algorithms, architectures, and designs implemented in this
   repository are fully reserved by the author. Patent filings are ongoing.
5. This software is provided “AS IS”, without warranty of any kind,
   express or implied, including but not limited to the warranties of
   merchantability, fitness for a particular purpose, and non-infringement.

For commercial licensing, technology transfer, or collaboration inquiries,
please contact yifenwan@phys.ethz.ch or visit https://transfer.ethz.ch 

---

## Acknowledgments

- Mu3e Collaboration at PSI
  - [**ETH Zürich – Rainer Wallny Group**](https://wallny-group.phys.ethz.ch)
  - Université de Genève
  - Universität Zürich
  - Paul Scherrer Institut
  - Universität Heidelberg
  - Karlsruhe Institut für Technologie
  - Johannes Gutenberg-Universität Mainz
  - University of Bristol
  - University of Liverpool
  - University of Oxford
  - University College London

<p align="right">(<a href="#readme-top">back to top</a>)</p>

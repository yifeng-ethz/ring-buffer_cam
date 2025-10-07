
# Mu3e IP Library
[![License](https://img.shields.io/badge/license-Academic%20Evaluation-lightgrey.svg)](#license)
[![Build Status](https://img.shields.io/badge/build-passing-brightgreen.svg)](#)  
[![Platform](https://img.shields.io/badge/platform-FPGA-blue.svg)](#)

<div align="center">
  <a href="https://github.com/yifeng-ethz/mu3e-ip-cores">
    <img src="./figures/mu3e_logo.png" alt="Mu3e Logo" width="120" />
  </a>
  
  <h3 align="center">Mu3e IP Library</h3>
</div>

---

## IP List

Each Mu3e IP core resides in its own subdirectory of this repository or in an external submodule. Cores marked as **Prototype** have no `VERSION` property and are still under development.

| IP Name | Description | Version / Status |
|---|---|---|
| [**Slow‑Control Hub**](https://github.com/yifeng-ethz/slow-control_hub) | Converts Mu3e slow‑control packets into Avalon Memory‑Mapped transactions and handles burst count, address and response timing. | ![v25.0.0806](https://img.shields.io/badge/version-25.0.0806-blue) |
| [**Onewire Temperature Sensor Controller**](https://github.com/yifeng-ethz/onewire_temp_sense) | Periodically polls 1‑Wire temperature sensors; implements reset, presence detect and bit‑level timing. | ![v24.0.918](https://img.shields.io/badge/version-24.0.918-blue) |
| [**MuTRiG Frame Deassembly**](https://github.com/yifeng-ethz/mutrig_frame_deassembly) | Parses MuTRiG frames into header and hit payloads and flags individual hit errors and frame CRC errors. | ![v24.0.1021](https://img.shields.io/badge/version-24.0.1021-blue) |
| [**CAM (Content Addressable Memory)**](https://github.com/yifeng-ethz/CAM) | Primitive content‑addressable memory core.  Use as a building block for caches, correlators and address decoders. | ![v17.0.2](https://img.shields.io/badge/version-17.0.2-blue) |
| [**MuTRiG Timestamp Processor**](https://github.com/yifeng-ethz/mutrig_timestamp_processor) | Tracks MuTRiG timestamp overflow and maps MuTRiG‑local timestamps to global timestamps. | ![v25.0.0306](https://img.shields.io/badge/version-25.0.0306-blue) |
| [**Histogram Statistics**](https://github.com/yifeng-ethz/histogram_statistics) | Builds histograms from a selected data stream using SAR bin calculation and DP‑RAM counters. | ![v25.0.0306](https://img.shields.io/badge/version-25.0.0306-blue) |
| [**MuTRiG Controller**](https://github.com/yifeng-ethz/mutrig_controller) | SPI master for configuring MuTRiG ASICs.  Automatically scans T‑thresholds and stores results locally. | ![v24.0.1028](https://img.shields.io/badge/version-24.0.1028-blue) |
| [**Charge Injection (Analog Pulser)**](https://github.com/yifeng-ethz/charge_injection) | Generates calibration pulses with arbitrary frequency and duration for TDC injection tests. | ![v4.0.5](https://img.shields.io/badge/version-4.0.5-blue) |
| [**Charge Injection (MuTRiG Injector)**](https://github.com/yifeng-ethz/charge_injection) | Produces digital and analog pulses to verify MuTRiG operation when used with DAB boards. | ![v25.0.0710](https://img.shields.io/badge/version-25.0.0710-blue) |
| [**Altera Temperature Sensor Controller**](https://github.com/yifeng-ethz/alt_temp_sense_controller) | Wraps the on‑chip `alt_temp_sense` IP on 28 nm devices and stores the last temperature result. | ![v1.0](https://img.shields.io/badge/version-1.0-blue) |
| [**High Performance Counter Array**](https://github.com/yifeng-ethz/high_performance_counter_array) | Parallel counters supporting concurrent inputs with Avalon‑MM readout.  Features synchronous clear and reset. | ![v1.4.1](https://img.shields.io/badge/version-1.4.1-blue) |
| [**MuTRiG Channel Counter Fabric**](https://github.com/yifeng-ethz/mutrig_channel_counter_fabric) | Connects *hit type 0* from the frame deassembly IP to the counter array.  Decodes channel IDs into one‑hot update signals. | ![v1.0.12](https://img.shields.io/badge/version-1.0.12-blue) |
| [**LVDS Error Counter Fabric**](https://github.com/yifeng-ethz/lvds_error_counter_fabric) | Accumulates parity and decode error counts from the LVDS receiver sideband. | ![v1.0.5](https://img.shields.io/badge/version-1.0.5-blue) |
| [**Firefly Transceiver I2C Master**](https://github.com/yifeng-ethz/firefly_xcvr_i2c_master) | Interfaces with the Samtec Firefly optical transceiver module via I2C.  Periodically reads temperature and RX power and can be halted. | ![v1.9.6](https://img.shields.io/badge/version-1.9.6-blue) |
| [**IP 8b/10b Decoder**](https://github.com/yifeng-ethz/ip_8b10b_decoder) | Standard 8b/10b decoder for parallel LVDS `rxout` data.  Derives parity and decoding errors. | ![v1.3.1](https://img.shields.io/badge/version-1.3.1-blue) |
| [**MuTRiG Reset Controller**](https://github.com/yifeng-ethz/mutrig_reset_controller) | Issues reset pulses for the MuTRiG based on run‑state changes.  Provides programmable phase shift via `alt_pll_reconfig`. | ![v1.0.8](https://img.shields.io/badge/version-1.0.8-blue) |
| [**Ring‑buffer CAM**](https://github.com/yifeng-ethz/ring-buffer_cam) | Circular buffer variant of CAM with push‑to‑stack write semantics and cache‑like read‑through.  Used to build the hit stack. | ![Protected](https://img.shields.io/badge/status-Protected-red) |
| [**Frontend‑Board Frame Assembly**](https://github.com/yifeng-ethz/feb_frame_assembly) | Assembles time‑interleaved subframes from the ring‑buffer CAM into Mu3e‑standard data frames and schedules packet transmission. | ![v25.0.0710](https://img.shields.io/badge/version-25.0.0710-blue) |
| [**Mu3e LVDS Controller**](https://github.com/yifeng-ethz/mu3e_lvds_controller) | Provides high‑speed LVDS links to the MuPix sensors using FPGA vendor IP.  Includes 28 nm LVDS RX and Pro variants. | ![v25.1.0630](https://img.shields.io/badge/version-25.1.0630-blue) |
| [**MuPix Inbound**](https://github.com/yifeng-ethz/mupix_inbound) | Deserializes data from MuPix chips, decodes and buffers hits. | ![Prototype](https://img.shields.io/badge/status-Prototype-lightgrey) |
| [**Packet Scheduler**](https://github.com/yifeng-ethz/packet_scheduler) | Orders packets via an interface adapter and an ordered priority queue to achieve deterministic throughput. | ![v25.0.0723](https://img.shields.io/badge/version-25.0.0723-blue) |
| [**Run‑Control Management**](https://github.com/yifeng-ethz/run-control_mgmt) | Manages run‑state transitions for Mu3e subsystems and issues control signals. | ![v24.0.1125](https://img.shields.io/badge/version-24.0.1125-blue) |

<p align="right">(<a href="#readme-top">back to top</a>)</p>

---

## About the Project

The Mu3e experiment searches for the rare decay $\mu^{+} \rightarrow e^{+} e^{-} e^{+}$ with unprecedented sensitivity.  A high-throughput, triggerless data acquisition (DAQ) system is required to read out the MuPix pixel sensors and MuTRiG tracking chips.  This repository houses reusable FPGA IP cores developed for Mu3e.  They cover front-end interfacing, slow-control, data processing and utilities.  Each subdirectory contains the source code, documentation and simulation testbenches for an individual core.

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
├── charge_injection/           # Analog pulser and MuTRiG injector
├── feb_frame_assembly/         # Frontend-board frame assembly
├── firefly_xcvr_i2c_master/    # I2C master for Samtec Firefly transceiver
├── high_performance_counter_array/
├── histogram_statistics/
├── ip_8b10b_decoder/
├── lvds_error_counter_fabric/
├── mu3e_lvds_controller/
├── mupix_inbound/
├── mutrig_channel_counter_fabric/
├── mutrig_controller/
├── mutrig_frame_deassembly/
├── mutrig_reset_controller/
├── mutrig_timestamp_processor/
├── onewire_temp_sense_controller/
├── packet_scheduler/
├── ring-buffer_cam/
├── run-control_mgmt/
└── slow-control_hub/
```

<p align="right">(<a href="#readme-top">back to top</a>)</p>

---

## Getting Started

### Prerequisites

- **Quartus Prime 18.1** or later (20.1 recommended)
- **Python 3.10+** (for testbenches)
- **ModelSim / Questa** for simulation
- **Tcl 8.5+** for integration scripts

### Installation

1. Clone this repository and its submodules:
   ```bash
   git clone --recursive https://github.com/yifeng-ethz/mu3e-ip-cores.git
   ```
2. Add the desired IP cores to your Quartus project.
3. For each core, run the provided Platform Designer script (the `*_hw.tcl` file) to generate the IP.
4. Use the provided VHDL or system-level wrappers as integration examples.

<p align="right">(<a href="#readme-top">back to top</a>)</p>

---

## Usage

Detailed integration instructions are provided in each core’s subdirectory.  In general:

1. Open Platform Designer (formerly Qsys) in Quartus.
2. Select **New Custom IP Component** and run the corresponding `*_hw.tcl` script to instantiate the IP.
3. Configure parameters (e.g., burst widths, channel counts) as required for your design.
4. Export interfaces and connect the IP into your top‑level design or testbench.

For simulation, the subdirectories include VHDL testbenches and Python scripts demonstrating correct operation.

---

## Contributing

Contributions to the Mu3e IP library are welcome.  To propose a change:

1. Fork this repository.
2. Create a feature branch for your improvement.
3. Commit your changes with clear messages.
4. Open a pull request describing the improvement or bug fix.

---

## License

Mu3e IP Cores — Academic Evaluation License
Copyright (c) 2025 Yifeng Wang. All rights reserved.

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
       “Mu3e IP Cores © 2025 Yifeng Wang, used under Academic Evaluation License.”
4. No patent rights are granted, explicitly or implicitly. All patent rights
   related to the algorithms, architectures, and designs implemented in this
   repository are fully reserved by the author. Patent filings are ongoing.
5. This software is provided “AS IS”, without warranty of any kind,
   express or implied, including but not limited to the warranties of
   merchantability, fitness for a particular purpose, and non-infringement.

For commercial licensing, technology transfer, or collaboration inquiries,
please contact:
    yifenwan@phys.ethz.ch

---

## Acknowledgments

- Mu3e Collaboration at PSI
  - **ETH Zürich – Rainer Wallny Group**
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

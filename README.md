# OAI 5G + TSN Scheduler (CBS + TAS Integration)

This repository contains a prototype integration of **Time-Sensitive Networking (TSN) features** —
**Credit-Based Shaper (CBS)** and **Time-Aware Shaper (TAS)** — into the 
`gNB_scheduler.c` of the OpenAirInterface 5G NR gNB MAC scheduler.

## Features
- CBS credit-based shaping per traffic class
- TAS gate control (open/close windows for traffic classes)
- Logging of frame, slot, credit, gate state, and burst counters
- Enforces bounded Jitter with CBS logic

## Directory
- `NR_MAC_gNB/` – modified scheduler files from OAI


## Usage
1. Clone this repo:
   ```bash
   git clone https://github.com/AadiEC/oai-5g-tsn-scheduler.git

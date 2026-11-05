# EM Waves Visualizer — Design

Date: 2026-11-05
Status: Draft for review

## 1. Purpose

A teaching/demo tool that lets students *see* Wi-Fi radiation in augmented reality.
An ESP32-S3 plugged into an Android phone measures the real signal strength of a chosen
2.4 GHz router; the phone renders the router as a glowing "lamp" of false-color light that
fills the room, the way the human eye turns light frequency into color and intensity into
brightness.

Success criteria:
- A student opens a URL, plugs in the sensor, picks a network, taps the router, and sees the glow in under a minute.
- The glow visibly responds (within ~0.5 s) to walking away from the router or blocking it with a body.
- The HUD explains what is shown: frequency → hue, intensity → brightness, expected vs measured signal.

Intuition and clarity matter more than measurement precision.

## 2. Decisions and scope

| Decision | Choice |
|---|---|
| Goal | Education / demo |
| Measured quantity | RSSI of one 2.4 GHz Wi-Fi access point |
| Sensor | ESP32-S3 over USB (OTG), promiscuous mode; **required** in v1 |
| Platform | Web app — Chrome on ARCore-capable Android (AR); desktop Chrome for sensor test page |
| Source location | User taps the router in AR to anchor it |
| Number of sources | One |
| Visualization | Volumetric glow: hue = channel frequency, brightness = intensity |

Out of scope for v1 (possible later): multiple sources, Bluetooth sources, CSI/phase data,
5 GHz (ESP32-S3 is 2.4 GHz only), phone-only RSSI fallback, depth-based surface lighting,
headphone-coil magnetic "listener", iOS, a user-facing simulated demo mode.

## 3. Architecture

Two deliverables: ESP32-S3 firmware and a static web app.

```
ESP32-S3 radio → USB serial (JSON lines) → sensorLink → protocol → smoother → waveModel → waveRenderer + hud
                                                         WebXR pose + router anchor ─────┘
```

### 3.1 Firmware (Arduino-ESP32, C++, PlatformIO)

USB CDC serial (native USB on the S3), 115200 baud nominal, one JSON object per line.

Commands (app → board), newline-terminated text:
- `HELLO` → `{"type":"hello","fw":"1.0.0","proto":1}`
- `SCAN` → one `{"type":"ap","ssid":"School","bssid":"aa:bb:cc:dd:ee:ff","channel":6,"rssi":-52}` per network, then `{"type":"scan_done","count":N}`
- `TRACK <bssid> <channel>` → `{"type":"tracking","bssid":"…","channel":6}` then a stream of
  `{"type":"rssi","rssi":-48,"n":7,"t":123456}` at ~20 Hz (50 ms windows; `rssi` = mean of packets
  whose transmitter address equals `bssid`; `n` = packet count, `n:0` windows are still sent with `rssi:null`; `t` = ms since boot)
- `STOP` → `{"type":"stopped"}`
- Errors → `{"type":"error","msg":"…"}`

Tracking: Wi-Fi in promiscuous mode, fixed channel, filter frames by addr2 == bssid (beacons ~10/s plus data).
Channel change handling: if no packets for 3 s, the firmware rescans for the same BSSID; if found on another
channel it switches and emits `{"type":"channel","channel":11}`.

Code layout: `firmware/src/main.cpp` (radio, serial I/O) and `firmware/lib/core/` (pure C++: JSON
formatting, windowed averaging, command parsing) so core logic compiles and is tested natively.

### 3.2 Web app (TypeScript, Vite, three.js)

Hosted as a static site on GitHub Pages (HTTPS, required by WebXR/WebUSB). Pages:
- **Main app** — connect → pick network → AR.
- **Sensor test** (desktop or phone, no AR) — live RSSI graph.
- **Install firmware** — ESP Web Tools button (intended for desktop Chrome).

Modules (`web/src/`), each one file with one job:
1. `sensorLink.ts` — `SensorSource` interface (`connect()`, `send(cmd)`, `onMessage`, `onDisconnect`).
   Implementation uses Web Serial on desktop and WebUSB via `web-serial-polyfill` on Android.
   A `FakeSensorSource` (dev/test only, enabled with `?fake=1`) emits simulated messages.
   Scenarios for E2E tests via `?fake=<scenario>`: `ok` (same as `1`), `badversion`, `nosignal`
   (stops packets after 2 s), `disconnect` (drops after 2 s, returns after 2 s), `channelchange`, `malformed`.
2. `protocol.ts` — parse JSON lines into typed messages; build command strings. Pure.
3. `smoother.ts` — exponential moving average over RSSI (time constant ~300 ms); ignores `rssi:null`;
   flags "no signal" after 3 s without packets. Pure.
4. `waveModel.ts` — pure functions:
   - channel → frequency: `f = 2407 + 5·ch` MHz (ch 1–13); wavelength `λ = c / f`.
   - expected RSSI at distance d: `P(d) = P1m − 20·log10(d)` (free space, d clamped ≥ 0.2 m).
   - calibration: `P1m = measured + 20·log10(d)` using the current AR distance (user is prompted to stand ~1 m away); default `P1m = −40 dBm`.
   - brightness factor `k = 10^((measured − expected)/10)` clamped to [0.05, 2].
   - hue: channel 1 → red (0°) … channel 13 → violet (~270°), linear.
5. `arSession.ts` — WebXR `immersive-ar` with `hit-test`, `anchors`, `dom-overlay`. Tap → hit-test →
   anchor router. No hit → place 1 m in front of camera. Re-place button. Exposes router pose and phone pose.
6. `waveRenderer.ts` — three.js full-screen pass, additively blended over the camera feed.
   Per pixel: ray from camera; energy along the ray from a point source at the anchor, integrated
   analytically over t ∈ [0, 10 m]: `E = (1/d)·(atan(t1/d) − atan(t0/d))`, where d is the ray's perpendicular
   distance to the source and t0, t1 are the segment ends relative to the closest point. Color =
   `hue(channel)`, intensity = `E · k · gain`, then soft tone mapping (`1 − exp(−x)`) so the near field
   doesn't clip and the far field stays visible. A TypeScript twin of the shader formula is unit-tested.
7. `hud.ts` — DOM overlay: network name and channel; hue legend with the eye analogy text;
   expected vs measured dBm and difference with explanation (e.g. "−12 dB: something is absorbing the
   signal"); distance to router; wavelength (e.g. 12.3 cm); Calibrate button; status banners.
8. `networkPicker.ts` — Connect sensor → `HELLO` (version check) → `SCAN` list sorted by RSSI → pick → `TRACK` → "Start AR".

## 4. Error handling

- **Capability check on load:** WebXR AR, WebUSB/Web Serial. Missing → explanatory message; desktop gets the sensor test page.
- **Sensor:** not connected/permission denied → "Connect sensor" with OTG hint; bad/missing `HELLO` or `proto` mismatch → link to install page; unplug mid-session → glow freezes and greys, banner, auto-reconnect and re-`TRACK` on replug; malformed lines are skipped and counted.
- **Signal:** no packets for 3 s → "No signal from <network>", glow fades, Rescan button; channel change handled by firmware (`channel` message updates hue/wavelength); hidden SSIDs listed by BSSID.
- **AR:** tracking lost → hide glow, "move slowly / point at textured surfaces" hint; no hit-test surface → 1 m fallback placement; uncalibrated → default P1m and a calibration prompt.

## 5. Testing

- **Unit (Vitest):** `protocol`, `smoother` (convergence, null handling, timeout), `waveModel` (channel 6 → 2437 MHz → 12.3 cm; free-space values; calibration; k clamping; hue), glow integral vs numerical integration on random rays.
- **Integration (Vitest):** `sensorLink` + `networkPicker` flow against a fake serial port replaying recorded lines: handshake, scan, track, disconnect/reconnect, version mismatch.
- **Firmware:** PlatformIO native tests for `lib/core` (JSON formatting, averaging windows, command parsing). Radio behavior verified manually via serial monitor (`SCAN` lists networks; `TRACK` streams ~20 Hz).
- **E2E (Playwright):** run against the Vite dev server with `?fake=<scenario>`, in desktop Chromium and
  mobile emulation (Pixel 7). Covers everything outside the live AR session:
  - capability check — `navigator.xr` / `navigator.usb` / `navigator.serial` removed via `addInitScript` → explanatory message shown;
  - picker flow — Connect → network list sorted by RSSI → pick → "Start AR" enabled;
  - sensor test page — live RSSI graph updates;
  - error states — `badversion` → install-page link; `nosignal` → "No signal" banner + Rescan; `disconnect` → banner then auto-recovery; `channelchange` → HUD channel/wavelength update; `malformed` → no crash;
  - install page — ESP Web Tools button and manifest present.
  The AR session itself (camera, hit-test, glow) cannot run in Playwright and stays manual. HUD elements are
  checked in the DOM by role/text, not by screenshots.
- **Manual on device:** walk toward/away (measured roughly follows expected); body-block (glow dims, difference shown); unplug/replug recovery; covered camera (tracking-lost hint).
- **Dev without hardware:** `?fake=1` uses `FakeSensorSource`.

## 6. Repository layout

```
firmware/        PlatformIO project (src/, lib/core/, test/)
web/             Vite + TypeScript app (src/, test/ for Vitest, e2e/ for Playwright, public/firmware/ for ESP Web Tools manifest + binary)
docs/superpowers/specs/
```

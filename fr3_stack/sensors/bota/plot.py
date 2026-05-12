"""Live plot of CSV-from-stdin in a browser (port 8988).

Auto-detects 7-col (compensated only) vs 13-col (with ``*_raw`` channels)
streams from the header and shows a Comp/Raw/Both toggle for the 13-col case.
"""
from __future__ import annotations

import json
import os
import socket
import sys
import threading
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

REFRESH_HZ  = 10
MAX_SAMPLES = 10000     # window = MAX_SAMPLES / publisher_hz
PORT        = int(os.environ.get("PLOT_PORT", "8988"))


COMP_KEYS = ("fx", "fy", "fz", "tx", "ty", "tz")
RAW_KEYS  = ("fx_raw", "fy_raw", "fz_raw", "tx_raw", "ty_raw", "tz_raw")


class Buffer:
    """Thread-safe ring buffer; one deque per CSV column.

    ``has_raw`` flips True on a 13-col header so raw deques fill and the
    JSON snapshot exposes them for the browser toggle.
    """
    def __init__(self, n: int = MAX_SAMPLES):
        self.n = n
        self.cols: dict[str, deque[float]] = {
            k: deque(maxlen=n) for k in (("t",) + COMP_KEYS + RAW_KEYS)
        }
        self.t0: float | None = None
        self.lock = threading.Lock()
        self.n_rows = 0
        self.has_raw = False

    def push(self, t: float, comp: tuple[float, ...],
             raw: tuple[float, ...] | None = None) -> None:
        with self.lock:
            if self.t0 is None:
                self.t0 = t
            self.cols["t"].append(t - self.t0)
            for k, v in zip(COMP_KEYS, comp):
                self.cols[k].append(v)
            if raw is not None:
                for k, v in zip(RAW_KEYS, raw):
                    self.cols[k].append(v)
            self.n_rows += 1

    def snapshot(self) -> dict[str, list[float]]:
        with self.lock:
            keys = ("t",) + COMP_KEYS + (RAW_KEYS if self.has_raw else ())
            return {k: list(self.cols[k]) for k in keys}


BUFFER = Buffer()


def reader_thread() -> None:
    """Parse CSV from stdin into BUFFER. First line is the header; column
    count picks the schema. Malformed later rows are silently dropped."""
    header_seen = False
    n_cols = 7
    for line in iter(sys.stdin.readline, ""):
        line = line.strip()
        if not line:
            continue
        if not header_seen:
            header_seen = True
            cols = [c.strip() for c in line.split(",")]
            n_cols = len(cols)
            BUFFER.has_raw = (n_cols >= 13)
            print(f"[plot] header ({n_cols} cols, has_raw={BUFFER.has_raw}): {line}",
                  file=sys.stderr, flush=True)
            continue
        try:
            parts = line.split(",")
            if len(parts) < n_cols:
                continue
            vals = [float(p) for p in parts[:n_cols]]
        except (ValueError, IndexError):
            continue
        t = vals[0]
        comp = tuple(vals[1:7])
        raw = tuple(vals[7:13]) if n_cols >= 13 else None
        BUFFER.push(t, comp, raw)
    print("[plot] stdin closed", file=sys.stderr, flush=True)


HTML = r"""<!doctype html>
<html><head>
<meta charset="utf-8"/>
<title>Bota F/T Sensor — Live</title>
<script src="https://cdn.plot.ly/plotly-2.35.2.min.js"></script>
<style>
  body { font-family: sans-serif; margin: 0; padding: 12px;
         background: #fff; color: #222; }
  h2 { margin: 0 0 6px 0; font-weight: 500; font-size: 18px; text-align: center; }
  #status { font-size: 12px; padding: 2px 8px 8px 8px; color: #555; text-align: center; }
  #mode-toggle { display: none; text-align: center; padding: 4px 0 8px 0;
                 font-size: 13px; color:#333; }
  #mode-toggle label { margin: 0 12px; cursor: pointer; }
  .plot { width: 100%; height: 320px; }
</style>
</head><body>
<h2>Bota F/T Sensor — Live</h2>
<div id="status">connecting…</div>
<div id="mode-toggle">
  <label><input type="radio" name="mode" value="comp" checked> Compensated</label>
  <label><input type="radio" name="mode" value="raw"> Raw</label>
  <label><input type="radio" name="mode" value="both"> Both</label>
</div>
<div id="force"  class="plot"></div>
<div id="torque" class="plot"></div>
<script>
const GRID = "rgba(0,0,0,0.15)";
const layoutBase = {
  paper_bgcolor:"#fff", plot_bgcolor:"#fff",
  font:{color:"#222", size:12},
  margin:{l:60, r:20, t:10, b:30},
  legend:{x:1, y:1, xanchor:"right", yanchor:"top",
          bgcolor:"rgba(255,255,255,0.7)", bordercolor:"#ccc", borderwidth:1},
  showlegend:true,
};
const fLayout = {
  ...layoutBase,
  yaxis:{title:"Force (N)",  gridcolor:GRID, zerolinecolor:GRID},
  xaxis:{showticklabels:false, gridcolor:GRID, zerolinecolor:GRID},
};
const tLayout = {
  ...layoutBase,
  yaxis:{title:"Torque (Nm)", gridcolor:GRID, zerolinecolor:GRID},
  xaxis:{title:"Time (s)",    gridcolor:GRID, zerolinecolor:GRID},
};
const COLORS = {x:"red", y:"green", z:"blue"};

function compTraces(d, axisLetter /* "f" | "t" */) {
  return ["x","y","z"].map(a => ({
    x:d.t, y:d[axisLetter+a],
    name:axisLetter.toUpperCase()+a,
    line:{color:COLORS[a], width:1.5},
  }));
}
function rawTraces(d, axisLetter, dashed) {
  return ["x","y","z"].map(a => ({
    x:d.t, y:d[axisLetter+a+"_raw"],
    name:axisLetter.toUpperCase()+a+(dashed?" raw":""),
    line:{color:COLORS[a], width:dashed?1:1.5,
          dash:dashed?"dot":"solid"},
  }));
}

Plotly.newPlot("force",  compTraces({t:[],fx:[],fy:[],fz:[]}, "f"), fLayout, {displayModeBar:false});
Plotly.newPlot("torque", compTraces({t:[],tx:[],ty:[],tz:[]}, "t"), tLayout, {displayModeBar:false});

function selectedMode() {
  const el = document.querySelector('input[name="mode"]:checked');
  return el ? el.value : "comp";
}

async function tick() {
  try {
    const r = await fetch("/data");
    const d = await r.json();
    const n = d.t.length;
    const hasRaw = ("fx_raw" in d);
    document.getElementById("mode-toggle").style.display = hasRaw ? "block" : "none";
    const mode = hasRaw ? selectedMode() : "comp";
    document.getElementById("status").textContent =
      `${n} samples buffered (received ${d.n_rows} total) — ${mode}`;

    let fT, tT;
    if (mode === "raw") {
      fT = rawTraces(d, "f", false);
      tT = rawTraces(d, "t", false);
    } else if (mode === "both") {
      fT = [...compTraces(d, "f"), ...rawTraces(d, "f", true)];
      tT = [...compTraces(d, "t"), ...rawTraces(d, "t", true)];
    } else {
      fT = compTraces(d, "f");
      tT = compTraces(d, "t");
    }
    Plotly.react("force",  fT, fLayout, {displayModeBar:false});
    Plotly.react("torque", tT, tLayout, {displayModeBar:false});
  } catch (e) {
    document.getElementById("status").textContent = "error: " + e;
  }
}
// Re-render on toggle change — snappier than waiting for the next poll.
document.querySelectorAll('input[name="mode"]').forEach(
  el => el.addEventListener("change", tick));
setInterval(tick, __REFRESH_MS__);
tick();
</script>
</body></html>
""".replace("__REFRESH_MS__", str(int(1000 / REFRESH_HZ)))


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *args, **kwargs):
        pass

    def do_GET(self):
        if self.path == "/" or self.path.startswith("/index"):
            body = HTML.encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
        elif self.path.startswith("/data"):
            snap = BUFFER.snapshot()
            snap["n_rows"] = BUFFER.n_rows
            body = json.dumps(snap).encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
        else:
            self.send_error(404)


def lan_ip() -> str:
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
        s.close()
        return ip
    except OSError:
        return socket.gethostname()


def main() -> None:
    threading.Thread(target=reader_thread, daemon=True).start()
    server = ThreadingHTTPServer(("0.0.0.0", PORT), Handler)
    print(f"\n  Open in your browser:  http://{lan_ip()}:{PORT}/\n",
          file=sys.stderr, flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()


if __name__ == "__main__":
    main()

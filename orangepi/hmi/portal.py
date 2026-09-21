#!/usr/bin/env python3
"""
Captive-portal responder for the Pi 5 field AP.

Runs on port 80. While the Pi is hosting its wifi AP, NetworkManager's shared
dnsmasq resolves every hostname to the Pi (see
/etc/NetworkManager/dnsmasq-shared.d/captive.conf), so all client HTTP requests
land here. We answer the OS captive-portal detection URLs with a redirect (not
the success response they expect), which makes phones/laptops pop up the
"Sign in to network" page showing our landing screen. From there the user taps
through to the real HMI dashboard on :3000.

Standalone Flask (no ROS). Harmless when the Pi is in home-wifi mode — it just
serves the landing page on :80 to anyone who asks.
"""

from flask import Flask, redirect, Response

app = Flask(__name__)

# The Pi's address while it is the AP gateway (NetworkManager "shared" default).
HMI_URL = "http://10.42.0.1:3000"

LANDING = f"""<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>SLAM Rig — Connect</title>
  <style>
    body {{ font-family: -apple-system, system-ui, sans-serif; background:#0d1117;
           color:#e6edf3; margin:0; display:flex; min-height:100vh;
           align-items:center; justify-content:center; }}
    .card {{ text-align:center; padding:2rem 1.5rem; max-width:420px; }}
    h1 {{ font-size:1.4rem; margin:0 0 .5rem; }}
    p {{ color:#9da7b3; margin:.25rem 0 1.5rem; }}
    a.btn {{ display:inline-block; background:#2f81f7; color:#fff; text-decoration:none;
             font-weight:600; padding:.9rem 1.6rem; border-radius:10px; font-size:1.1rem; }}
    .hint {{ font-size:.8rem; color:#6e7681; margin-top:1.25rem; }}
  </style>
</head>
<body>
  <div class="card">
    <h1>SLAM Rig Connected</h1>
    <p>You're on the rig's wifi. Open the control panel to run scans and recordings.</p>
    <a class="btn" href="{HMI_URL}">Open Control Panel</a>
    <div class="hint">If the button doesn't load the dashboard, open a browser and go to<br><b>{HMI_URL}</b></div>
  </div>
</body>
</html>"""


# OS captive-portal detection endpoints -> redirect so the "Sign in" popup appears.
@app.route("/generate_204")            # Android / Chrome
@app.route("/gen_204")                 # Android (older)
@app.route("/hotspot-detect.html")     # iOS / macOS
@app.route("/library/test/success.html")
@app.route("/connecttest.txt")         # Windows
@app.route("/ncsi.txt")                # Windows (NCSI)
@app.route("/redirect")                # Windows
def detect():
    return redirect("/", code=302)


# Everything else -> the landing page (DNS sends all names here in AP mode).
@app.route("/", defaults={"path": ""})
@app.route("/<path:path>")
def catch_all(path):
    return Response(LANDING, mimetype="text/html")


if __name__ == "__main__":
    app.run(host="0.0.0.0", port=80, threaded=True)

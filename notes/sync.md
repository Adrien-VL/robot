**For your Mac client (host), the easiest and most reliable way is to use macOS's built-in time sync + make sure the robot is also well-synced.**

### Option 1: Quick & Recommended (Both sync to internet NTP — works great if both have internet)

**On your Mac:**
1. Go to **System Settings → General → Date & Time**.
2. Turn on **"Set date and time automatically"**.
3. Choose a reliable server like:
   - `time.apple.com`
   - `time.google.com`
   - or `pool.ntp.org`

macOS will keep itself synced automatically.

**On the robot (Ubuntu/Linux):**
```bash
sudo apt update
sudo apt install chrony
sudo systemctl enable --now chrony
```

Edit `/etc/chrony/chrony.conf` (add or uncomment):
```
pool pool.ntp.org iburst
```

Then restart:
```bash
sudo systemctl restart chrony
chronyc sources     # should show good sources
chronyc tracking    # check offset (should be < 100ms ideally)
```

### Option 2: Robot as time server (better if robot has better clock or no internet sometimes)

**On the robot (make it NTP server):**
Edit `/etc/chrony/chrony.conf` and add near the end:
```conf
# Act as server
local stratum 8
allow all   # or your specific subnet, e.g. 192.168.1.0/24
```

Restart chrony:
```bash
sudo systemctl restart chrony
```

**On your Mac:**
macOS built-in NTP client doesn't easily point to a custom server via GUI for local IPs. Two easy workarounds:

**A. Use Terminal (one-time or script):**
```bash
sudo sntp -sS robot-ip-address   # e.g. 192.168.1.100
```

**B. Install ChronyControl on Mac (best long-term):**
- Download **ChronyControl** from: https://whatroute.net/chronycontrol.html
- It installs a proper `chronyd` on macOS.
- Configure it to use your robot's IP as server.

### Quick check after syncing
On **both** machines run:
```bash
date
```

The times should be within < 100–200 ms of each other.

Then on your Mac host:
```bash
ros2 run tf2_tools view_frames
ros2 topic echo /tf --once | head -n 20
```

Let me know the output of `chronyc tracking` (on robot) and `date` on both machines if you're still seeing timestamp drops. This usually fixes the RViz "earlier than transform cache" errors completely.
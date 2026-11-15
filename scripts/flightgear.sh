#!/bin/sh
set -e

# fgfs on the PATH, else the macOS app bundle. Override with FGFS=/path/to/fgfs
FGFS="${FGFS:-$(command -v fgfs || echo /Applications/FlightGear.app/Contents/MacOS/FlightGear)}"

exec "$FGFS" \
  --fdm=null --native-fdm=socket,in,60,,5550,udp \
  --aircraft=c172p --timeofday=noon \
  --lat=37.325776 --lon=-79.978586 --altitude=3000 --heading=0 --in-air \
  '--prop:/consumables/fuel/tank[0]/capacity-gal_us=30.8' \
  '--prop:/consumables/fuel/tank[0]/density-ppg=6.0' \
  '--prop:/consumables/fuel/tank[1]/capacity-gal_us=30.8' \
  '--prop:/consumables/fuel/tank[1]/density-ppg=6.0' \
  "$@"

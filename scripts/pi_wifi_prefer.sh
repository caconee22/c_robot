#!/bin/bash
# Raspberry Pi: move back to the preferred Wi-Fi when it is in range.
# NetworkManager picks the higher autoconnect-priority only when it connects;
# once on the fallback (307) it stays there. Run every 30 s by
# c-robot-wifi-prefer.timer.
PREFERRED=caconee22
active=$(nmcli -t -f NAME,TYPE con show --active | awk -F: '$2=="802-11-wireless"{print $1; exit}')
[ "$active" = "$PREFERRED" ] && exit 0
# --rescan auto: reuse a scan younger than 30 s, otherwise scan now.
if nmcli -t -f SSID dev wifi list --rescan auto | grep -qx "$PREFERRED"; then
  echo "switching from '${active:-none}' to $PREFERRED"
  nmcli con up "$PREFERRED" || echo "could not switch, staying on '${active:-none}'"
fi

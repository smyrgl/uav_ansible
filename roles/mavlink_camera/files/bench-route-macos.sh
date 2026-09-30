#!/bin/sh
# Mac-only, temporary host route. No subnet/default route changes.
set -eu
destination=192.168.144.1
gateway=${2:-192.168.1.42}   # the Jetson's wifi address: a UniFi DHCP reservation, see group_vars
case "$(uname -s)" in Darwin) ;; *) echo 'This helper is for macOS only.' >&2; exit 1;; esac
case "${1:-status}" in
  status) /sbin/route -n get "$destination" ;;
  up)
    current=$(/sbin/route -n get "$destination")
    if printf '%s\n' "$current" | /usr/bin/grep -q "destination: $destination$"; then
      echo 'A host route already exists. Inspect it before replacing it:'
      printf '%s\n' "$current"
      exit 0
    fi
    /usr/bin/sudo /sbin/route -n add -host "$destination" "$gateway"
    /sbin/route -n get "$destination"
    ;;
  down)
    current=$(/sbin/route -n get "$destination")
    if ! printf '%s\n' "$current" | /usr/bin/grep -q "destination: $destination$" ||
       ! printf '%s\n' "$current" | /usr/bin/grep -q "gateway: $gateway$"; then
      echo 'Refusing to remove a route that does not match this bench configuration.' >&2
      exit 1
    fi
    /usr/bin/sudo /sbin/route -n delete -host "$destination" "$gateway"
    ;;
  *) echo "Usage: $0 {up|down|status} [Jetson-Wi-Fi-IP]" >&2; exit 2;;
esac

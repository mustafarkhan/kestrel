#!/usr/bin/env bash
# A private certificate authority for one site, and a certificate for each end of the
# SAPIENT link. Mutual TLS 1.3: only holders of a certificate this CA signed can register
# with the fusion node, report to it, or send tasks and traffic to the aircraft.
#
#   deploy/pki.sh <output-dir> fusion kestrel-1 radar-1 ...
#
# Keep ca.key offline once the certificates are made. Give each device only its own
# <name>.crt and <name>.key, plus ca.crt. To retire a device, make a new CA and reissue:
# there is no revocation list.
set -euo pipefail
OUT="${1:?usage: pki.sh <output-dir> <name>...}"; shift
DAYS="${KESTREL_PKI_DAYS:-365}"
mkdir -p "$OUT"; cd "$OUT"; umask 077
if [ ! -f ca.key ]; then
  openssl req -x509 -newkey ec -pkeyopt ec_paramgen_curve:P-384 -nodes -keyout ca.key -out ca.crt \
    -days "$DAYS" -subj "/CN=kestrel-site-ca"
fi
for NAME in "$@"; do
  openssl req -newkey ec -pkeyopt ec_paramgen_curve:P-384 -nodes -keyout "$NAME.key" -out "$NAME.csr" \
    -subj "/CN=$NAME"
  openssl x509 -req -in "$NAME.csr" -CA ca.crt -CAkey ca.key -CAcreateserial -out "$NAME.crt" -days "$DAYS"
  rm "$NAME.csr"
  echo "issued $OUT/$NAME.crt (valid $DAYS days)"
done

#!/bin/bash

cd "$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"

echo ""
echo "Downloading simulation environments..."
echo ""

ggID='1V4MWCD3qYLr3RFLLATbqAh_lXKcCU39C'
ggURL='https://drive.usercontent.google.com/download'

filename="$(curl -sc /tmp/gcokie "${ggURL}?id=${ggID}" | grep -o '="uc-name.*</span>' | sed 's/.*">//;s/<.a> .*//')"
curl -Lb /tmp/gcokie "${ggURL}?id=${ggID}&confirm=xxx" -o ${filename}

echo ""
echo "Unzipping files..."
echo ""

unzip "${filename}"

rm "${filename}"

echo ""
echo "Done, simulation environments are kept in 'src/vehicle_simulator/mesh'."
echo ""

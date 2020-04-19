#!/bin/bash
source /etc/os-release || echo 'Warning: /etc/os-release was not found'

if [[ "$ID" == 'arch' ]]; then
  echo "Detected Arch Linux distribution."
  sudo pacman -S git libuv openssl gcc cmake make
else
  if [[ ! " $ID_LIKE " == *' debian '* ]]; then
    echo 'Error: unidentified Linux distribution. Modify this script to add support for your distribution.'
    exit 1
  fi
  echo "Detected Debian-like distribution. Using apt."
  sudo apt-get update
  sudo apt-get install git libuv1-dev libssl-dev gcc g++ cmake make
fi

git clone https://github.com/uWebSockets/uWebSockets 
cd uWebSockets
git checkout e94b6e1
mkdir build
cd build
cmake ..
make 
sudo make install
cd ../..
sudo ln -s /usr/lib64/libuWS.so /usr/lib/libuWS.so
sudo rm -r uWebSockets

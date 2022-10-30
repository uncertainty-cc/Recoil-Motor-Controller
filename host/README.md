## Pi CAN Adapter

```bash
sudo apt update
sudo apt install python3-pip
```

```bash
pip3 install python-can
```

```bash
sudo nano /boot/config.txt
```

```
dtparam=spi=on
dtoverlay=mcp2515-can0,oscillator=12000000,interrupt=25,spimaxfrequency=1000000
```


```bash
reboot
```


```bash
dmesg | grep -i '\(can\|spi\)'
```
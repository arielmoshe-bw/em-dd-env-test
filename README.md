# Environment Testing Steering Motor

Version 1.0.0

Env. Testing Steering Motor

Check the following documents for more info:

1.[Steering Motor - BLDC Direct Drive](https://bw-robotics.atlassian.net/wiki/spaces/RH/pages/379224384/Steering+Motor+-+BLDC+Direct+Drive)

2.[Env. Testing Steering Motor](https://bw-robotics.atlassian.net/wiki/spaces/RH/pages/436994564/Env.+Testing+Steering+Motor)

To run use executable "em-dd-env-test/Host/pythonProject/dist/host_script".

python libs -
pyserial
matplotlib
pyinstaller

pip install -r requirements.txt

running pyinstaller on Windows after installing requirements - python -m PyInstaller host_script.py
running script on Windows - change dev/tty0 to a specific com port "COMX"

# Environment Testing Steering Motor

Version 1.0.0

Env. Testing Steering Motor

Check the following documents for more info:

1.[Steering Motor - BLDC Direct Drive](https://bw-robotics.atlassian.net/wiki/spaces/RH/pages/379224384/Steering+Motor+-+BLDC+Direct+Drive)

2.[Env. Testing Steering Motor](https://bw-robotics.atlassian.net/wiki/spaces/RH/pages/436994564/Env.+Testing+Steering+Motor)

Run script -
1) pip install -r requirements.txt
2) open terminal in the directory em-dd-env-test/Host/pythonProject/
3) generate executable -
   Windows:
   run command "python -m PyInstaller host_script.py"
   Ubuntu:
   run command "pyinstaller --onefile host_script.py"
4) change directory em-dd-env-test/Host/pythonProject/dist and run host_script executble.

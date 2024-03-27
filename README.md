# Environment Testing Steering Motor

Version 1.0.0

Env. Testing Steering Motor

Check the following documents for more info:

1.[Steering Motor - BLDC Direct Drive](https://bw-robotics.atlassian.net/wiki/spaces/RH/pages/379224384/Steering+Motor+-+BLDC+Direct+Drive)

2.[Env. Testing Steering Motor](https://bw-robotics.atlassian.net/wiki/spaces/RH/pages/436994564/Env.+Testing+Steering+Motor)

3. Autopilot motor software V5.4 for configuring motor firmware parameters - [download link](https://www.dcmotorkeya.com/download.html)
   
Run script -
1) Install Arduino IDE for usb drivers - [download link](https://www.microsoft.com/store/productId/9MSSZTT1N39L?ocid=pdpshare)
2) Install Python 3.8 - [download link](https://www.microsoft.com/store/productId/9MSSZTT1N39L?ocid=pdpshare)
3) Only For Windows: Install Microsoft Visual C++ Redistributables: Download and install the appropriate version (32-bit or 64-bit) that matches your Python installation from the official Microsoft website: [download link](https://www.dcmotorkeya.com/download.html]https://learn.microsoft.com/en-us/cpp/windows/latest-supported-vc-redist?view=msvc-170)
4) pip install -r requirements.txt
5) open terminal in the directory em-dd-env-test/Host/pythonProject/
6) generate executable -
   
   Windows:
   run command "python -m PyInstaller host_script.py"
   
   Ubuntu:
   run command "pyinstaller --onefile host_script.py"
7) change directory em-dd-env-test/Host/pythonProject/dist and run host_script executble.

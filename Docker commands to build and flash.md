**Docker commands to build and flash**:



**First time users**:

1. Download and install Docker for Windows
2. Run Docker and login
3. Install/Update WSL for Windows
4. Open cmd in the source folder and run **docker compose build - let it install the dependencies**
5. Install usbipd for windows using the command - **winget install usbipd**



**Build**:

1\. Run **docker compose run --rm builder**



**Flash (Run this every session)**:

1\. Open PowerShell in **Admin mode**

&#x09;1. Run **usbipd list** - find the STLink's BUSID, e.g. 2-4

&#x09;2. Run **usbipd bind --busid 2-4** - first time per device only (Admin)

2\. Open PowerShell inside your code folder

&#x09;1. Run **usbipd attach --wsl --busid 2-4** - forward it into WSL2

3\. Run **docker compose run --rm flasher**


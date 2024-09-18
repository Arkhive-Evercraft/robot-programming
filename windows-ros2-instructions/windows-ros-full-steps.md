# Installing ROS2 Foxy on Windows

> Windows 10 is required as a minimum as ROS only officially supports Windows 10. This guide was made while installing it on Windows 11.
> 
> Windows PowerShell v2+ minimum 
>
>**OpenSSL**
>
> The ROS for Windows installation page lists ***OpenSSL*** **v1.1.1n** as a requirement to be installed. The page it suggests to download the installer from does not have this installer. It may have been removed due to security vulerabilities.
>
> An appropriate ***OpenSSL*** version can still be downloaded using ***Chocolatey***. This is just some information if you are trying to follow along with the official ROS instructions. For this, we will be installing ***OpenSSL*** **v1.1.1.3**
> 
> Win32 or Light versions, or the v3.X.Y installers are not compatible with ROS so we must use a version of 1.1.1.
>
> **Chocolatey installation script**
>
> The installation script that is written below in this guide comes from the ***Chocolatey*** website at https://chocolatey.org/install.
> - The required installation is under the **individual** installation option
> <img src="images/choco-indiv-install.png" style="border: 2px grey solid"/>
> - It's a good idea to check installation scripts are valid and from a verifiable source so I've included this link so you can check the validity of the installation script.

---

## Step 1: Install Chocolatey

***Chocolatey*** is a package manager for Windows and will be used throughout this guide to install all dependencies and packages for ROS.

1. Open Start menu search / press Windows key and  type:
    ```
    Windows Powershell
    ```
2. Right click ***Windows PowerShell*** and select *Run as Administrator*

    <img src="images/windows-powershell-admin.png" height="350" width="auto" style="border: 2px grey solid"/>
    
3. Click *Yes* to the prompt

    <img src="images/windows-powershell-yes.png" height="300" width="auto" style="border: 2px grey solid"/>

4. Execute the ***Chocolatey*** installation script by running the following command in the ***PowerShell*** terminal:
    ```
    Set-ExecutionPolicy Bypass -Scope Process -Force; [System.Net.ServicePointManager]::SecurityProtocol = [System.Net.ServicePointManager]::SecurityProtocol -bor 3072; iex ((New-Object System.Net.WebClient).DownloadString('https://community.chocolatey.org/install.ps1'))
    ```
    <img src="images/chocolatey-installation-cmd.png" style="border: 2px grey solid"/>

    <img src="images/chocolatey-installation.png" style="border: 2px grey solid"/>

5. If you don't see any errors, you are ready to use ***Chocolatey***! 
Type `choco` to check that it has been installed.

    <img src="images/chocolatey-install-complete.png" style="border: 2px grey solid"/>

1. Close and reopen ***PowerShell***, making sure to *Run as administrator*.

    <img src="images/windows-powershell-admin.png" height="350" width="auto" style="border: 2px grey solid"/>

---

## Step 2: Install Python
> The specific version of ***Python*** required is 3.8.3.

1. In the ***PowerShell*** terminal, run the following:
    ```
	choco install -y python --version=3.8.3
    ```
    ![Chocolatey Python Installation](images/chocolatey-python-install.png)

---

## Step 3: Install Visual C++ Distributables
1. In the ***PowerShell*** terminal, run the following:
    ```
    choco install -y vcredist2013 vcredist140
    ```

    <img src="images/choco-vcpp-install.png" style="border: 2px grey solid"/>

    > The previous ***Python*** installation may have already installed one or more of these packages, which will cause the following message:

    <img src="images/vcpp-already-install.png" style="border: 2px grey solid"/>
    
    > This is fine to ignore; There is no need to reinstall any packages you already have.

---

## Step 4: Install OpenSSL

1. In the ***PowerShell*** terminal, run the following:
    ```
    choco install openssl --version=1.1.1.3
    ```

2. Type `Y` when prompted and press Enter
    
    ![Chocolatey Installation of OpenSSL v1.1.1.3](images/choco-openssl-install.png)

    > It should say **Installing 64-bit openssl** after you press Enter
3. Type the following command into the ***PowerShell*** terminal and press Enter:
    ```
    setx /m OPENSSL_CONF "C:\Program Files\OpenSSL-Win64\bin\openssl.cfg"
    ```
    ![Set OpenSSL Environment Variable](images/openssl-env-var.png)
    > This sets an enviroment variable that will persist across different sessions.

---

## Step 5: Install Visual Studio
> **[ ! ] IMPORTANT**<br>
> This is an installation of ***Visual Studio***, not ***Visual Studio Code***.
>
> Microsoft didn't do a thorough think before naming these two programs the way they did.
>
> ***Visual Studio*** has both a paid version and a free version. This install is for the free version.
>
> The free version is called ***Community***.

1. Go to https://visualstudio.microsoft.com/free-developer-offers/ and download ***Community***

    <img src="images/visual-studio-community.png" height="200" width="auto"/>
2. Navigate to where the installer downloaded to and run it once it has finished downloading

    <img src="images/vs-community-in-downloads.png" height="200" width="auto"/>
    
    Just click *Close* if prompted to update
    ![Visual Studio Community Installer Window](images/vs-community-installer-init-window.png)

3. Click on *Modify* in the Visual Studio Community section (above *Launch*)
    ![Visual Studio Community Installer Highlight Modify Button](images/vs-community-modify-install.png)


4. In the *Modify* window, scroll down to **Desktop development with C++** and make sure the checkbox is ticked.
    ![Visual Studio Community Desktop Development with C++ Highlight Checkbox](images/vs-community-desktop-cpp.png)

5. In the **Installation details** menu to the right, untick **C++ Clang tools for windows** and **C++ Cmake Tools for windows**

    <img src="images/vs-community-highlight-clang.png" height="350" width="auto"/>
    <img src="images/vs-community-highlight-cmake.png" height="350" width="auto"/>


    Then click *Modify*

    <img src="images/vs-community-modify.png" height="100" width="auto"/>
6. Allow installer to finish the installation
    ![Visual Studio Community Installation](images/vs-community-installer-modifying.png)

---

## Step 6: Install OpenCV
> ***OpenCV*** is a package that is used for computer vision and image processing. This is not an essential installation but some examples provided by ROS Foxy use ***OpenCV*** and require it to be installed.

1. You can download a precompiled version of ***OpenCV*** **3.4.6** from:
    ```
    https://github.com/ros2/ros2/releases/download/opencv-archives/opencv-3.4.6-vc16.VS2019.zip
    ```
    **Note: This is a .zip link and will automatically download the folder if pasted into the browser**

2. Extract the .zip to **Program Files** by right-clicking the .zip and clicking on *Extract All...*

    <img src="images/open-cv-extract.png" height="350" width="auto"/>

- Click *Browse* to change the extraction path
    <img src="images/open-cv-extract-browse.png" height="200" width="auto"/>
    > You can uncheck **Show extracted files when complete** if you want.

- Navigate to **C:\\** and click on **Program Files** to select it and click *Select Folder* down the bottom right
    ![OpenCV Extraction Highlight Program Files Extraction](images/open-cv-extract-program-files.png)

- Alternatively, you can just directly type the file path into the text box
    ![OpenCV Extraction Destination Folder Dialogue Window](images/open-cv-extract-dest-dialogue.png)

    
- Click *Extract* to extract the files

- Click *Continue* to allow files to be extracted to **Program Files**
    ![OpenCV Extraction Destination Folder Requires Admin Permission](images/open-cv-extract-admin-prompt.png)

3. Assuming you unpacked it to **C:\Program Files\opencv**

    Type the following command into the ***PowerShell*** terminal and press enter:
    ```
    setx /m OpenCV_DIR "C:\Program Files\opencv"
    ```
    > This is setting an environment variable for ***OpenCV*** so it knows where to find the ***OpenCV*** files

    ![OpenCV Set Environment Variable](images/open-cv-env-var.png)

---

## Step 7: Install Dependencies (CMake and Git Packages)

### Install CMake 
1. Type the following command into the ***PowerShell*** terminal and press enter:
    ```
    choco install -y cmake
    ```

    ![Chocolatey CMake Command in Terminal](images/choco-cmake-install.png)

### Download Git packages
1. Go to https://github.com/ros2/choco-packages/releases/latest

    ![Git Packages to Download for Chocolatey](images/git-packages.png)

2. Download all except **log4cxx.0.10.0-2.nupkg** and the **source code** folders:
    - asio.1.12.1.nupkg
	- bullet.2.89.0.nupkg
	- cunit.2.1.3.nupkg
	- eigen-3.3.4.nupkg
	- tinyxml-usestl.2.6.2.nupkg
	- tinyxml2.6.0.0.nupkg
	- log4cxx.0.10.0.nupkg

3. In the ***PowerShell*** terminal, copy and paste the following command but **do not press enter yet**:
    ```
    choco install -y -s <PATH\TO\DOWNLOADS> asio cunit eigen tinyxml-usestl tinyxml2 log4cxx bullet
    ```

4. Navigate to the folder that the packages were downloaded to so that you can click on the folder itself.
  - Right-click the folder and select *Copy as path* (or Ctrl + Shift + C)
    > e.g. The files were downloaded into the **Downloads** folder, so I copy the path of the **Downloads** folder

    ![Copy Path of Downloads Folder Using Right Click Menu](images/git-packages-copy-path.png)

- Replace `<PATH\TO\DOWNLOADS>` with the copied filepath

    ![Chocolatey Installation Original Command with Text To Substitute](images/git-packages-orig-command.png)

    ![Chocolatey Installation Changed Command](images/git-packages-changed-command.png)

1. Press Enter to install the packages

    ![Output of Chocolatey Installation of Git Packages](images/git-packages-installed.png)

2. Close and reopen ***PowerShell*** again, making sure to *Run as administrator*.

    ![Windows PowerShell Administrator Mode](images/windows-powershell-admin.png)

### Python Command Line Tools
1. Run the following command into the ***PowerShell*** Terminal and press Enter to upgrade pip
    ```
    python -m pip install --upgrade pip
    ```
2. Run the following command into the ***PowerShell*** terminal and press Enter
    ```
    python -m pip install -U catkin_pkg cryptography empy ifcfg lark-parser lxml netifaces numpy opencv-python pyparsing pyyaml setuptools rosdistro
    ```

    ![Python Command Line Tool Installation Command](images/python-cmd-tools-install.png)

    > **[ ! ] Important** <br>
    > If you encounter the following error, your version of pip needs to be upgraded

    ![Error During Installation Requiring Pip Upgrade](images/pip-upgrade-error.png)

    Type the following command into the ***PowerShell*** Terminal and press Enter to upgrade pip 
    ```
    python -m pip install --upgrade pip
    ```
    - Once pip is upgraded, try executing the command at step 2 again

### RQt Dependencies
1. Paste the following command into the ***PowerShell*** terminal and press enter
    ```
    python -m pip install -U pydot PyQt5
    ```

    ![Python Installation Command for RQt](images/python-pyqt-install-command.png)

2. Use ***Chocolatey*** to install ***Graphviz*** (required to use RQt Graph)
- Paste the following command into the ***PowerShell*** terminal and press Enter
    ```
    choco install graphviz
    ```
- Type `Y` and press enter when prompted

    ![Chocolatey Installation Command for Graphviz](images/choco-graphviz-install-command.png)

---

## Step 8: Adding Variables to PATH
1. Click on Start button / press Windows key and search for:
	```
	Environment Variables
	```

	![Start Menu Search for Environment Variables Setting](images/edit-env-variables-1.png)

2. Open **Edit the system environment variables**
3. In the **System Properties** window that pops up, click on *Environment Variables*

	![Set Environment Variable System Properties Window Highlight Environment Variables](images/edit-env-variables-2.png)


4. In the bottom panel called **System Variables**, click on **Path** and then click on *Edit*
	![Set System Variables Highlight Path and Edit](images/edit-env-variables-3.png)

5. In the **Edit environment variable** window that pops up, click on the *New* button.

	![Edit Environment Variable Highlight New Button](images/edit-env-variables-4.png)

6. Create a *New* path for each of the following paths:
- C:\Program Files\OpenSSL-Win64\bin\
- C:\Program Files\opencv\x64\vc16\bin
- C:\Program Files\CMake\bin
- C:\Program Files\Graphviz\bin

1. From completing all of the installation steps up until now, these are the environment variables should be set.

	![Cropped Picture of Paths Added During Installation](images/final-env-paths-cropped.png)
	
	> Make sure to add any if they are missing. The paths should be the same as in the picture if you installed everything in the default locations in the instructions.

---

## Step 10: Install ROS2 Foxy
1. Go to https://github.com/ros2/ros2/releases and find the latest release of ROS2 Foxy
    > Currently it is at https://github.com/ros2/ros2/releases?page=2 

    ![Github Page for ROS2 Packages](images/ros-foxy-github-page.png)

2. Download the latest package for Windows, e.g., ros2-foxy-*-windows-AMD64.zip
- Right click the downloaded .zip and select *Extract All...*

    ![Right Click Menu to Extract All](images/extract-ros-foxy.png)

- Click *Browse* to change the extraction path

    ![Extract ROS Foxy Highlight Browse Button](images/extract-ros-foxy-browse.png)

- Create new folder in **C:\\** called **dev**

    ![Highlight Create New Folder](images/create-new-folder.png)

- Select **dev** and click on *Select Folder* down the bottom right 

    ![Highlight Created dev Folder](images/created-dev-folder.png)

- Click on *Extract* to extract the files. Allow permission to continue if prompted.

    ![Extraction Dialogue Highlight Extract Button](images/extract-dialogue-dev-selected.png)

3. The extracted folder has the name **ros2-windows** but the ROS2 Foxy tutorials assume it is called **ros2_foxy**. 
    - You can either keep it as **ros2-windows**, but remember to substitute **ros2-windows** where ever the tutorials mention **ros2_foxy**, or rename the folder.

- To rename the folder, right click on the folder and select *Show more options*

    ![Right Click Menu Show More Options](images/right-click-more-options.png)

- Click on Rename

    ![More Options Menu Rename Option](images/right-click-rename.png)

- Change the name to **ros2_foxy**

    ![Changed Name to ros2_foxy](images/name-change-ros-foxy.png)

4. Set up ROS environment
- Open Start menu search / press Windows key and  type:
    ```
    cmd
    ```

    ![Start Menu Search Command Prompt](images/cmd-search.png)

- Open ***Command Prompt***, paste the following command in the terminal window and press Enter
    ```
    C:\dev\ros2_foxy\local_setup.bat
    ```
    > This assumes the ROS2 package was extracted to **C:\dev** and was renamed to **ros2_foxy**. Change this path to where ever your ROS package was extracted to.
    
    > This command needs to be run every time a new Command Prompt terminal is opened.

    ![Source ROS Installation](images/cmd-source-env.png)

    > It is normal that the previous command, if nothing else went wrong, outputs “The system cannot find the path specified.” exactly once.

    ![Connext Error](images/connext-error.png)

    > When running the local_setup.bat, this error might occur. It has no impact on the installation and can be ignored unless specific support for Connext is needed.

5. Test Installation
- Open a second Command Prompt window and execute the same command as before in the new window
    ```
    C:\dev\ros2_foxy\local_setup.bat
    ```
- Make sure that any new terminal windows are **Command Prompt** and not **PowerShell**. You can choose the terminal type with the small arrow next to the tab.

    ![Change PowerShell to Command Prompt](images/choose-cmd.png)

- In one of the two terminal windows, paste the following command and press Enter
    ```
    ros2 run demo_nodes_cpp talker
    ```
- In the other terminal window, paste the following command and press Enter
    ```
    ros2 run demo_nodes_py listener
    ```
- Give permission for the executable to run if prompted
- If the two programs are running then ROS is installed. 

    ![Talker Listener ROS Demo](images/ros-demo.png)

### Troubleshooting
https://docs.ros.org/en/foxy/How-To-Guides/Installation-Troubleshooting.html#windows-troubleshooting
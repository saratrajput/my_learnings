# Programming with Julia

### 6. Installing Julia in Linux

* Go to the **julialang.org** website.
* Navigate to the download page.
* Select the 64-bit version of Linux and download it.
* Locate the downloaded file in the Downloads folder.
* Extract the file either using a file extractor or through the terminal with the command ```tar zxvf [filename]```.
* Move the extracted folder to another folder with any desired name (e.g., "julia").
    ```
    sudo mkdir -p /julia/
    sudo mv julia-*/ /julia/
    ```
* Add the Julia folder to PATH by creating a symbolic link to Julia. Use the following commands in the terminal:
    ```
    # (optional - to remove any previous installations)
    sudo rm /usr/local/bin/julia
    # Create symbolic link.
    sudo ln -s ~/[folder name]/bin/julia /usr/local/bin/julia"
    ```
* Reopen the terminal and type ```julia``` to verify that it is installed successfully.

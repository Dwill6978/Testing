# AeroSandbox Installation

In short:

* `pip install aerosandbox[full]` for a complete install.

* `pip install aerosandbox` for a lightweight (headless) installation with minimal dependencies. All optimization, numerics, and physics models are included, but optional visualization dependencies are skipped.

More detail:

1. First, install Python via the Anaconda distribution, [available here](https://www.anaconda.com/products/individual). During the installation, it is recommended that you add Python to your PATH ("[What does this mean?](https://janelbrandon.medium.com/understanding-the-path-variable-6eae0936e976)"), which can be conveniently done by checking a checkbox.
   2. The minimum-required Python version is given in [this file](./setup.py) in a line beginning with `python_requires = `. At the time of writing (July 2023), the minimum required Python version is 3.8.

2. Verify that your installation has completed correctly by opening up a Terminal (Command Prompt on Windows) and executing the following commands. You should see something similar to the below:
	
    Windows:
	
    ```shell
    C:\Users\peter>where python
    C:\ProgramData\Anaconda3\python.exe
	
    C:\Users\peter>where pip
    C:\ProgramData\Anaconda3\Scripts\pip.exe
    ```
	
    Mac/Linux:
	
    ```bash
    (base) peter@Peter-PC:/mnt/c/Users/peter$ which python
    /home/peter/anaconda3/bin/python
	
    (base) peter@Peter-PC:/mnt/c/Users/peter$ which pip
    /home/peter/anaconda3/bin/pip
    ```
	
    Basically, we're checking that the aliases `python` and `pip` both:
	
    1. refer to essentially the same folder, and
	
    2. that folder is our recently-installed Anaconda distribution.

    For reference, `pip` is [Python's built-in package manager](https://realpython.com/lessons/what-is-pip-overview/), which lets you easily use others' open-source code (such as AeroSandbox) in your own projects.

3. (Optional) Install a Python IDE, if desired. I like using [PyCharm](https://www.jetbrains.com/pycharm/), which offers a feature-rich "Community Edition" for free, or an even-better "Professional Edition" which is free for students, open-source developers, and many other groups. Another popular choice is [Visual Studio Code](https://code.visualstudio.com/), which is also free.

4. Install AeroSandbox by typing the following into your terminal:

    ```shell
    pip install aerosandbox[full]
    ```
 
    If all goes well, you should see a bunch of output as it first downloads & installs dependencies, then downloads & installs the latest version of AeroSandbox.

5. Congrats! You've installed Anaconda Python and AeroSandbox. If desired, you can test your install by following steps in the "Testing" section of this document, but it's not necessary. 

   To start learning how to use AeroSandbox on your problems, we recommend you [check out the tutorials folder here](./tutorial/).

## Testing

If desired, you can test that AeroSandbox functions properly on your system by:

1. Installing with the `[test]` flag, so:
   ```shell
   pip install aerosandbox[full,test] 
   ```
   for a new install, or, if you already have an existing installation:
   ```shell
   pip install --upgrade aerosandbox[full,test]
   ```
2. Then, as shown below: open up a terminal, launch into Python, and execute these commands:

```
C:\Users\peter>python
Python 3.9.7 (default, Sep 16 2021, 16:59:28) [MSC v.1916 64 bit (AMD64)] :: Anaconda, Inc. on win32
Type "help", "copyright", "credits" or "license" for more information.
>>> import aerosandbox as asb
>>> asb.run_tests()
```

A bunch of tests should run, hopefully terminating with a message like:

```shell
====================== 214 passed, 47 warnings in 17.89s =======================
```

If you get any failing tests, that is considered a bug, and we hope you'll report it to us [by creating an issue ticket](https://github.com/peterdsharpe/AeroSandbox/issues).

## Troubleshooting

### Permissions Errors

If you get any permissions issues, try the following:

1. Windows: [Run Command Prompt as Administrator](https://www.howtogeek.com/194041/how-to-open-the-command-prompt-as-administrator-in-windows-8.1/) and try again. If that still doesn't work, try also adding the `--user` flag to the installation command, so:
       
    ```shell
    pip install --user aerosandbox[full] 
    ```
2. Mac/Linux: Try running with `sudo`, so:

    ```bash
    sudo pip install aerosandbox[full] 
    ```
   
    If that still doesn't work, try also adding the `--user` flag to the installation command, so:

    ```bash
    sudo pip install --user aerosandbox[full]
    ```

### SSL Certificate Errors

If you get any SSL certificate errors (sometimes seen with VPN connections on very-restricted corporate networks):

Execute the following commands in your terminal:

```shell
>>> conda activate base
>>> pip install --trusted-host pypi.org --trusted-host files.pythonhosted.org aerosandbox[full]
```

The first command explicitly [puts you into a Conda environment](https://docs.conda.io/projects/conda/en/latest/user-guide/getting-started.html) called "base", which is the default environment - basically, this will ensure that whatever you install is accessible anytime you use Python on your system.

The second command is our standard install command, except the `--trusted-host` additions tell `pip` to ignore any SSL certificate errors.

(Advanced users: if this is a recurring annoyance on your network, [consider editing `pip.conf` and adding these targets to `trusted-host` under the `global` parameter](https://stackoverflow.com/questions/59287824/specifying-multiple-trusted-hosts-in-pip-conf).)

### Other Problems

For all other issues, start a discussion on the [Discussions page](https://github.com/peterdsharpe/AeroSandbox/discussions).
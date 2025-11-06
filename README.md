> **Process to Build/Compile/Load the code**


The process for uploading the code to the robot controllers is based on the use of **"Geany"** within the Raspberry Pi OS. 

> [!NOTE]
> Geany is a ***text editor and Integrated Development Environment (IDE), used for programming with Pyton on a Raspberry Pi***.

- The first thing to do to upload the code is open a new file (or an existing one) in the Geany IDE within the Raspberry Pi OS.
  
<!--imagen de como abrir un archivo de geany\-->

- Once the file is open, the code writing process is started, treating the structure as if it were Python

<!--imagen del codigo sin haber guardado el archivo\-->

- Upon completing the code, the file is save with the **_".py"_** extension.

<!--imagen del archivo con la extension .py\-->

> [!NOTE]
> Once the file is saved with the ".py" extension, ***the file is automatically detected by Geany as a Python script***.

<!--imagen del codigo con el archivo ya guardado\-->

Once the file with the ".py" extension is available, the command console of the Raspberry Pi OS must be opened. Inside the console, the following command is run: **"_python3 my_script.py", replacing "my script" with the name of the file_**. Upon completion of this step, the code will run automatically.

<!--imagen de la consola con el comando\-->

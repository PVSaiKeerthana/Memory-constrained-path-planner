# Memory-constrained-path-planner

This project is a path planner program written in C for RISC-V processors, designed such that it takes less memory and less instruction count. To run this we used the RIPES RISC-V processor simulator.

## 1. Prerequisites

You need to download the following :
- **RIPES**: A RISC-V graphical CPU simulator.
- **RISC-V GCC compiler**: To compile C code for RISC-V.
- **VS Code or any text editor**: To edit the code.
- We should install them as per operating system instructions.

## 2. Preparing the Code

1. **Copy the Source Code**  
   Copy all the C code (`file_name.c`) from this repository to your computer.

2. **Open a Terminal**  
   Go to the folder where you saved `file_name.c`.

3. **Compile for RISC-V**  
   Run this command (aftefr installing RISC-V compiler and is added to PATH):

   ```sh
   riscv32-unknown-elf-gcc -Os -o file_name.elf file_name.c
   ```

   - This creates `file_name.elf`—an executable file for RISC-V.

## 3. Running on RIPES

1. **Open RIPES**
   - Double-click to launch.

2. **Load the Program**
   - Go to `File > Open Program…`
   - Select your new `file_name.elf` file.

3. **Set Start and Goal Nodes-using Switches**
   - In RIPES, find the Find MMIO addresses: go to the I/O panel (input/output devies) which is available on the left side bar of the RIPES interface
   - then select switches which is available under devices to give start and end nodes
  - 0xF0000000—set value for START node (0 to 31) in the Switches that appeared when selected (Switches 0)
  - 0xF0000004—set value for GOAL node (0 to 31) in the Switches that appeared when selected (Switches 1)
  - For example, type 5 for Start node and 15 for Goal node.
     5 in bits is represented as- 00000101(2^2 + 2^0), 15 in bits is represented as-00001111(2^3 + 2^2 + 2^1 + 2^0)
  - Sometimes you might need to press Enter or click a button to update.
  -  Run the simulation:
     press the Run (►) button in RIPES—the program will read your input and plan the path.

## 4. Run and See Results

- Click the **Run** button (`►`) in RIPES.
- The program will:
  - Ask for start and goal nodes (using switches).
  - Find the shortest path.
  - Show instructions and path steps in the output or UART/console window.

- **Path instructions** will look like:
  ```
  [UART OUT]: 0x01
  Movement instructions:
    Turn RIGHT
    Move EAST to node ...
    ...
  Arrived at goal node ...
  ```

- You can change the start and goal node MMIO/switches and rerun to see different paths.
  NOTE : to change the start and end nodes , reset the simulator by clicking on the two circular arrows which appears on the top. and then change the switches to the new start and end nodes and then run the simulation.

## 5. Troubleshooting

- If the compiler isn’t found:  
  Make sure the RISC-V GCC folder is in your system’s PATH variable.

- If the `.elf` file doesn’t load:  
  Ensure you compiled for 32-bit RISC-V.

- If nothing happens:  
  Check that the switch values are set and you pressed “Run” in RIPES.

**We change the start and end goals and check the path by running it in Ripes**










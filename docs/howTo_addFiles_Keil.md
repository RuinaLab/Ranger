# How to add new files to Kiel uVision 3
Suppose that you would like to create a new directory  if your project, with two files:
- `newDir\test.c`
- `newDir\test.h`
 
### 1. Create files
- Create the directory and both files, either using Keil or a standard text editor

### 2. Add source code to project
- Right-click on the top level of the project in the file tree (left side of screen in Keil) and select "New Group". Give it the same name as the directory that was just created (`newIDr`).
- Now right-click on `newDir` in the file tree and select add files. Select `test.c` from the menu. Do not select the header file.
- 

### 3. Update search path for compiler
- Select the top level project in the file-tree, and then (at the top of the screen) go to: `Project->Options for target...`
- Click on the tab for C/C++ and then select the "..." symbol to the right of "Include Paths"
- Add `newDir` to the path.
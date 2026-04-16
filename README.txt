Input-code folder — incomplete student code for the class
=========================================================
This folder contains the starter/incomplete controller code for students.
Fill in the "488 TODO" sections to implement the attitude and rate PID controller.

When "Student Code" is checked in the simulator, the build uses these files
if all three are present:
  - controller_student.c
  - student_attitude_controller.c
  - student_pid.c
  - student_pid.h

If any are missing, the simulator builds from student-code/ instead.

Where the simulator stores files (packaged app)
----------------------------------------------
When you run the packaged .app, the simulator prefers to use the folder that
contains the app: logs/, input-code/, and build/ in that same folder. If it
cannot write there (e.g. app in Applications or macOS runs a read-only copy),
it uses a hidden folder in your home directory instead:

  ~/.microcart_simulator/
    logs/       - log files from each run
    input-code/ - your copy of these starter files (edit these to build)
    build/      - compiled controller.dylib

To open that folder: Finder → Go → Go to Folder… → type ~/.microcart_simulator

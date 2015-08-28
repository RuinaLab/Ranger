# How to generate error codes on Ranger

1. Edit `error_table.csv` to create an entry for the error that you would like to send.
    - `Module` is not too important, select something reasonable from the existing list, or create your own set if those don't make sense. Try to follow Ranger naming conventions (_eg._ "MB" for Main Brain).
    - `Error Name` is important. It is how you will reference this error in code.
    - `priority` is unused. Enter `0` as a placeholder.
    - `description` is the message that is sent to the LabVEIW data handler.
2. Run `ranger_parser.m` - this is very important. It copies the error codes to the source files for each of the computers on Ranger, so that they all communicated together. Now when the code compiles, all modules (including LabVIEW) will know about the new error codes.
3. The function to call (inside of the c code) is defined:
        void mb_error_occurred(ERROR_ID error_code);  

from cffi import FFI
ffibuilder = FFI()

     # cdef() expects a single string declaring the C types, functions and
     # globals needed to use the shared object. It must be in valid C syntax.
ffibuilder.cdef("""
    void init_control();
    void clean_up_control();
    int get_state(int pin);
    int set_motor(int pin, double motor_setting);
    int rc_servo_send_pulse_normalized  (int ch, double  input );
    """)
     # set_source() gives the name of the python extension module to
     # produce, and some C source code as a string.  This C code needs
     # to make the declarated functions, types and globals available,
     # so it is often just the "#include".
ffibuilder.set_source("_encodercontrol_cffi",
    """
    #include "encodercontrol_export.h"   // the C header of the library
    #include <rc/servo.h>
    """,
    sources=['encodercontrol_export.c'],
    libraries=['robotcontrol'])   # library name, for the linker

if __name__ == "__main__":
	ffibuilder.compile(verbose=True)

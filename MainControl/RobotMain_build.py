def compileMain():
     ffibuilder = FFI()

# cdef() expects a single string declaring the C types, functions and
# globals needed to use the shared object. It must be in valid C syntax.
ffibuilder.cdef("""
    int get_state(int pin);
""")
# set_source() gives the name of the python extension module to
# produce, and some C source code as a string.  This C code needs
# to make the declarated functions, types and globals available,
# so it is often just the "#include".
ffibuilder.set_source("_encodercontrol_cffi",
"""
     #include "encodercontrol.h"   // the C header of the library
""",
     libraries=['encodercontrol'])   # library name, for the linker

     if __name__ == "__main__":
          ffibuilder.compile(verbose=True)
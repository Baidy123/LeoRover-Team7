
---
events:
  -
    kind: "message-v1"
    backtrace:
      - "/usr/share/cmake-3.28/Modules/CMakeDetermineSystem.cmake:233 (message)"
      - "CMakeLists.txt:2 (project)"
    message: |
      The system is: Linux - 6.17.0-19-generic - x86_64
  -
    kind: "message-v1"
    backtrace:
      - "/usr/share/cmake-3.28/Modules/CMakeDetermineCompilerId.cmake:17 (message)"
      - "/usr/share/cmake-3.28/Modules/CMakeDetermineCompilerId.cmake:64 (__determine_compiler_id_test)"
      - "/usr/share/cmake-3.28/Modules/CMakeDetermineCCompiler.cmake:123 (CMAKE_DETERMINE_COMPILER_ID)"
      - "CMakeLists.txt:2 (project)"
    message: |
      Compiling the C compiler identification source file "CMakeCCompilerId.c" succeeded.
      Compiler: /usr/bin/cc 
      Build flags: 
      Id flags:  
      
      The output was:
      0
      
      
      Compilation of the C compiler identification source "CMakeCCompilerId.c" produced "a.out"
      
      The C compiler identification is GNU, found in:
        /home/student31/leo_ws/build/leo_lidar_sim/CMakeFiles/3.28.3/CompilerIdC/a.out
      
  -
    kind: "message-v1"
    backtrace:
      - "/usr/share/cmake-3.28/Modules/CMakeDetermineCompilerId.cmake:17 (message)"
      - "/usr/share/cmake-3.28/Modules/CMakeDetermineCompilerId.cmake:64 (__determine_compiler_id_test)"
      - "/usr/share/cmake-3.28/Modules/CMakeDetermineCXXCompiler.cmake:126 (CMAKE_DETERMINE_COMPILER_ID)"
      - "CMakeLists.txt:2 (project)"
    message: |
      Compiling the CXX compiler identification source file "CMakeCXXCompilerId.cpp" succeeded.
      Compiler: /usr/bin/c++ 
      Build flags: 
      Id flags:  
      
      The output was:
      0
      
      
      Compilation of the CXX compiler identification source "CMakeCXXCompilerId.cpp" produced "a.out"
      
      The CXX compiler identification is GNU, found in:
        /home/student31/leo_ws/build/leo_lidar_sim/CMakeFiles/3.28.3/CompilerIdCXX/a.out
      
  -
    kind: "try_compile-v1"
    backtrace:
      - "/usr/share/cmake-3.28/Modules/CMakeDetermineCompilerABI.cmake:57 (try_compile)"
      - "/usr/share/cmake-3.28/Modules/CMakeTestCCompiler.cmake:26 (CMAKE_DETERMINE_COMPILER_ABI)"
      - "CMakeLists.txt:2 (project)"
    checks:
      - "Detecting C compiler ABI info"
    directories:
      source: "/home/student31/leo_ws/build/leo_lidar_sim/CMakeFiles/CMakeScratch/TryCompile-kIIiWJ"
      binary: "/home/student31/leo_ws/build/leo_lidar_sim/CMakeFiles/CMakeScratch/TryCompile-kIIiWJ"
    cmakeVariables:
      CMAKE_C_FLAGS: ""
      CMAKE_C_FLAGS_DEBUG: "-g"
      CMAKE_EXE_LINKER_FLAGS: ""
    buildResult:
      variable: "CMAKE_C_ABI_COMPILED"
      cached: true
      stdout: |
        Change Dir: '/home/student31/leo_ws/build/leo_lidar_sim/CMakeFiles/CMakeScratch/TryCompile-kIIiWJ'
        
        Run Build Command(s): /usr/bin/cmake -E env VERBOSE=1 /usr/bin/gmake -f Makefile cmTC_b896c/fast
        /usr/bin/gmake  -f CMakeFiles/cmTC_b896c.dir/build.make CMakeFiles/cmTC_b896c.dir/build
        gmake[1]: Entering directory '/home/student31/leo_ws/build/leo_lidar_sim/CMakeFiles/CMakeScratch/TryCompile-kIIiWJ'
        Building C object CMakeFiles/cmTC_b896c.dir/CMakeCCompilerABI.c.o
        /usr/bin/cc   -v -o CMakeFiles/cmTC_b896c.dir/CMakeCCompilerABI.c.o -c /usr/share/cmake-3.28/Modules/CMakeCCompilerABI.c
        Using built-in specs.
        COLLECT_GCC=/usr/bin/cc
        OFFLOAD_TARGET_NAMES=nvptx-none:amdgcn-amdhsa
        OFFLOAD_TARGET_DEFAULT=1
        Target: x86_64-linux-gnu
        Configured with: ../src/configure -v --with-pkgversion='Ubuntu 13.3.0-6ubuntu2~24.04.1' --with-bugurl=file:///usr/share/doc/gcc-13/README.Bugs --enable-languages=c,ada,c++,go,d,fortran,objc,obj-c++,m2 --prefix=/usr --with-gcc-major-version-only --program-suffix=-13 --program-prefix=x86_64-linux-gnu- --enable-shared --enable-linker-build-id --libexecdir=/usr/libexec --without-included-gettext --enable-threads=posix --libdir=/usr/lib --enable-nls --enable-bootstrap --enable-clocale=gnu --enable-libstdcxx-debug --enable-libstdcxx-time=yes --with-default-libstdcxx-abi=new --enable-libstdcxx-backtrace --enable-gnu-unique-object --disable-vtable-verify --enable-plugin --enable-default-pie --with-system-zlib --enable-libphobos-checking=release --with-target-system-zlib=auto --enable-objc-gc=auto --enable-multiarch --disable-werror --enable-cet --with-arch-32=i686 --with-abi=m64 --with-multilib-list=m32,m64,mx32 --enable-multilib --with-tune=generic --enable-offload-targets=nvptx-none=/build/gcc-13-EldibY/gcc-13-13.3.0/debian/tmp-nvptx/usr,amdgcn-amdhsa=/build/gcc-13-EldibY/gcc-13-13.3.0/debian/tmp-gcn/usr --enable-offload-defaulted --without-cuda-driver --enable-checking=release --build=x86_64-linux-gnu --host=x86_64-linux-gnu --target=x86_64-linux-gnu --with-build-config=bootstrap-lto-lean --enable-link-serialization=2
        Thread model: posix
        Supported LTO compression algorithms: zlib zstd
        gcc version 13.3.0 (Ubuntu 13.3.0-6ubuntu2~24.04.1) 
        COLLECT_GCC_OPTIONS='-v' '-o' 'CMakeFiles/cmTC_b896c.dir/CMakeCCompilerABI.c.o' '-c' '-mtune=generic' '-march=x86-64' '-dumpdir' 'CMakeFiles/cmTC_b896c.dir/'
         /usr/libexec/gcc/x86_64-linux-gnu/13/cc1 -quiet -v -imultiarch x86_64-linux-gnu /usr/share/cmake-3.28/Modules/CMakeCCompilerABI.c -quiet -dumpdir CMakeFiles/cmTC_b896c.dir/ -dumpbase CMakeCCompilerABI.c.c -dumpbase-ext .c -mtune=generic -march=x86-64 -version -fasynchronous-unwind-tables -fstack-protector-strong -Wformat -Wformat-security -fstack-clash-protection -fcf-protection -o /tmp/ccVDVf0d.s
        GNU C17 (Ubuntu 13.3.0-6ubuntu2~24.04.1) version 13.3.0 (x86_64-linux-gnu)
        	compiled by GNU C version 13.3.0, GMP version 6.3.0, MPFR version 4.2.1, MPC version 1.3.1, isl version isl-0.26-GMP
        
        GGC heuristics: --param ggc-min-expand=100 --param ggc-min-heapsize=131072
        ignoring nonexistent directory "/usr/local/include/x86_64-linux-gnu"
        ignoring nonexistent directory "/usr/lib/gcc/x86_64-linux-gnu/13/include-fixed/x86_64-linux-gnu"
        ignoring nonexistent directory "/usr/lib/gcc/x86_64-linux-gnu/13/include-fixed"
        ignoring nonexistent directory "/usr/lib/gcc/x86_64-linux-gnu/13/../../../../x86_64-linux-gnu/include"
        #include "..." search starts here:
        #include <...> search starts here:
         /usr/lib/gcc/x86_64-linux-gnu/13/include
         /usr/local/include
         /usr/include/x86_64-linux-gnu
         /usr/include
        End of search list.
        Compiler executable checksum: b220a7f1a1f69970d969d254ad9ec166
        COLLECT_GCC_OPTIONS='-v' '-o' 'CMakeFiles/cmTC_b896c.dir/CMakeCCompilerABI.c.o' '-c' '-mtune=generic' '-march=x86-64' '-dumpdir' 'CMakeFiles/cmTC_b896c.dir/'
         as -v --64 -o CMakeFiles/cmTC_b896c.dir/CMakeCCompilerABI.c.o /tmp/ccVDVf0d.s
        GNU assembler version 2.42 (x86_64-linux-gnu) using BFD version (GNU Binutils for Ubuntu) 2.42
        COMPILER_PATH=/usr/libexec/gcc/x86_64-linux-gnu/13/:/usr/libexec/gcc/x86_64-linux-gnu/13/:/usr/libexec/gcc/x86_64-linux-gnu/:/usr/lib/gcc/x86_64-linux-gnu/13/:/usr/lib/gcc/x86_64-linux-gnu/
        LIBRARY_PATH=/usr/lib/gcc/x86_64-linux-gnu/13/:/usr/lib/gcc/x86_64-linux-gnu/13/../../../x86_64-linux-gnu/:/usr/lib/gcc/x86_64-linux-gnu/13/../../../../lib/:/lib/x86_64-linux-gnu/:/lib/../lib/:/usr/lib/x86_64-linux-gnu/:/usr/lib/../lib/:/usr/lib/gcc/x86_64-linux-gnu/13/../../../:/lib/:/usr/lib/
        COLLECT_GCC_OPTIONS='-v' '-o' 'CMakeFiles/cmTC_b896c.dir/CMakeCCompilerABI.c.o' '-c' '-mtune=generic' '-march=x86-64' '-dumpdir' 'CMakeFiles/cmTC_b896c.dir/CMakeCCompilerABI.c.'
        Linking C executable cmTC_b896c
        /usr/bin/cmake -E cmake_link_script CMakeFiles/cmTC_b896c.dir/link.txt --verbose=1
        /usr/bin/cc  -v CMakeFiles/cmTC_b896c.dir/CMakeCCompilerABI.c.o -o cmTC_b896c 
        Using built-in specs.
        COLLECT_GCC=/usr/bin/cc
        COLLECT_LTO_WRAPPER=/usr/libexec/gcc/x86_64-linux-gnu/13/lto-wrapper
        OFFLOAD_TARGET_NAMES=nvptx-none:amdgcn-amdhsa
        OFFLOAD_TARGET_DEFAULT=1
        Target: x86_64-linux-gnu
        Configured with: ../src/configure -v --with-pkgversion='Ubuntu 13.3.0-6ubuntu2~24.04.1' --with-bugurl=file:///usr/share/doc/gcc-13/README.Bugs --enable-languages=c,ada,c++,go,d,fortran,objc,obj-c++,m2 --prefix=/usr --with-gcc-major-version-only --program-suffix=-13 --program-prefix=x86_64-linux-gnu- --enable-shared --enable-linker-build-id --libexecdir=/usr/libexec --without-included-gettext --enable-threads=posix --libdir=/usr/lib --enable-nls --enable-bootstrap --enable-clocale=gnu --enable-libstdcxx-debug --enable-libstdcxx-time=yes --with-default-libstdcxx-abi=new --enable-libstdcxx-backtrace --enable-gnu-unique-object --disable-vtable-verify --enable-plugin --enable-default-pie --with-system-zlib --enable-libphobos-checking=release --with-target-system-zlib=auto --enable-objc-gc=auto --enable-multiarch --disable-werror --enable-cet --with-arch-32=i686 --with-abi=m64 --with-multilib-list=m32,m64,mx32 --enable-multilib --with-tune=generic --enable-offload-targets=nvptx-none=/build/gcc-13-EldibY/gcc-13-13.3.0/debian/tmp-nvptx/usr,amdgcn-amdhsa=/build/gcc-13-EldibY/gcc-13-13.3.0/debian/tmp-gcn/usr --enable-offload-defaulted --without-cuda-driver --enable-checking=release --build=x86_64-linux-gnu --host=x86_64-linux-gnu --target=x86_64-linux-gnu --with-build-config=bootstrap-lto-lean --enable-link-serialization=2
        Thread model: posix
        Supported LTO compression algorithms: zlib zstd
        gcc version 13.3.0 (Ubuntu 13.3.0-6ubuntu2~24.04.1) 
        COMPILER_PATH=/usr/libexec/gcc/x86_64-linux-gnu/13/:/usr/libexec/gcc/x86_64-linux-gnu/13/:/usr/libexec/gcc/x86_64-linux-gnu/:/usr/lib/gcc/x86_64-linux-gnu/13/:/usr/lib/gcc/x86_64-linux-gnu/
        LIBRARY_PATH=/usr/lib/gcc/x86_64-linux-gnu/13/:/usr/lib/gcc/x86_64-linux-gnu/13/../../../x86_64-linux-gnu/:/usr/lib/gcc/x86_64-linux-gnu/13/../../../../lib/:/lib/x86_64-linux-gnu/:/lib/../lib/:/usr/lib/x86_64-linux-gnu/:/usr/lib/../lib/:/usr/lib/gcc/x86_64-linux-gnu/13/../../../:/lib/:/usr/lib/
        COLLECT_GCC_OPTIONS='-v' '-o' 'cmTC_b896c' '-mtune=generic' '-march=x86-64' '-dumpdir' 'cmTC_b896c.'
         /usr/libexec/gcc/x86_64-linux-gnu/13/collect2 -plugin /usr/libexec/gcc/x86_64-linux-gnu/13/liblto_plugin.so -plugin-opt=/usr/libexec/gcc/x86_64-linux-gnu/13/lto-wrapper -plugin-opt=-fresolution=/tmp/ccDl1mTN.res -plugin-opt=-pass-through=-lgcc -plugin-opt=-pass-through=-lgcc_s -plugin-opt=-pass-through=-lc -plugin-opt=-pass-through=-lgcc -plugin-opt=-pass-through=-lgcc_s --build-id --eh-frame-hdr -m elf_x86_64 --hash-style=gnu --as-needed -dynamic-linker /lib64/ld-linux-x86-64.so.2 -pie -z now -z relro -o cmTC_b896c /usr/lib/gcc/x86_64-linux-gnu/13/../../../x86_64-linux-gnu/Scrt1.o /usr/lib/gcc/x86_64-linux-gnu/13/../../../x86_64-linux-gnu/crti.o /usr/lib/gcc/x86_64-linux-gnu/13/crtbeginS.o -L/usr/lib/gcc/x86_64-linux-gnu/13 -L/usr/lib/gcc/x86_64-linux-gnu/13/../../../x86_64-linux-gnu -L/usr/lib/gcc/x86_64-linux-gnu/13/../../../../lib -L/lib/x86_64-linux-gnu -L/lib/../lib -L/usr/lib/x86_64-linux-gnu -L/usr/lib/../lib -L/usr/lib/gcc/x86_64-linux-gnu/13/../../.. CMakeFiles/cmTC_b896c.dir/CMakeCCompilerABI.c.o -lgcc --push-state --as-needed -lgcc_s --pop-state -lc -lgcc --push-state --as-needed -lgcc_s --pop-state /usr/lib/gcc/x86_64-linux-gnu/13/crtendS.o /usr/lib/gcc/x86_64-linux-gnu/13/../../../x86_64-linux-gnu/crtn.o
        COLLECT_GCC_OPTIONS='-v' '-o' 'cmTC_b896c' '-mtune=generic' '-march=x86-64' '-dumpdir' 'cmTC_b896c.'
        gmake[1]: Leaving directory '/home/student31/leo_ws/build/leo_lidar_sim/CMakeFiles/CMakeScratch/TryCompile-kIIiWJ'
        
      exitCode: 0
  -
    kind: "message-v1"
    backtrace:
      - "/usr/share/cmake-3.28/Modules/CMakeDetermineCompilerABI.cmake:127 (message)"
      - "/usr/share/cmake-3.28/Modules/CMakeTestCCompiler.cmake:26 (CMAKE_DETERMINE_COMPILER_ABI)"
      - "CMakeLists.txt:2 (project)"
    message: |
      Parsed C implicit include dir info: rv=done
        found start of include info
        found start of implicit include info
          add: [/usr/lib/gcc/x86_64-linux-gnu/13/include]
          add: [/usr/local/include]
          add: [/usr/include/x86_64-linux-gnu]
          add: [/usr/include]
        end of search list found
        collapse include dir [/usr/lib/gcc/x86_64-linux-gnu/13/include] ==> [/usr/lib/gcc/x86_64-linux-gnu/13/include]
        collapse include dir [/usr/local/include] ==> [/usr/local/include]
        collapse include dir [/usr/include/x86_64-linux-gnu] ==> [/usr/include/x86_64-linux-gnu]
        collapse include dir [/usr/include] ==> [/usr/include]
        implicit include dirs: [/usr/lib/gcc/x86_64-linux-gnu/13/include;/usr/local/include;/usr/include/x86_64-linux-gnu;/usr/include]
      
      
  -
    kind: "message-v1"
    backtrace:
      - "/usr/share/cmake-3.28/Modules/CMakeDetermineCompilerABI.cmake:159 (message)"
      - "/usr/share/cmake-3.28/Modules/CMakeTestCCompiler.cmake:26 (CMAKE_DETERMINE_COMPILER_ABI)"
      - "CMakeLists.txt:2 (project)"
    message: |
      Parsed C implicit link information:
        link line regex: [^( *|.*[/\\])(ld|CMAKE_LINK_STARTFILE-NOTFOUND|([^/\\]+-)?ld|collect2)[^/\\]*( |$)]
        ignore line: [Change Dir: '/home/student31/leo_ws/build/leo_lidar_sim/CMakeFiles/CMakeScratch/TryCompile-kIIiWJ']
        ignore line: []
        ignore line: [Run Build Command(s): /usr/bin/cmake -E env VERBOSE=1 /usr/bin/gmake -f Makefile cmTC_b896c/fast]
        ignore line: [/usr/bin/gmake  -f CMakeFiles/cmTC_b896c.dir/build.make CMakeFiles/cmTC_b896c.dir/build]
        ignore line: [gmake[1]: Entering directory '/home/student31/leo_ws/build/leo_lidar_sim/CMakeFiles/CMakeScratch/TryCompile-kIIiWJ']
        ignore line: [Building C object CMakeFiles/cmTC_b896c.dir/CMakeCCompilerABI.c.o]
        ignore line: [/usr/bin/cc   -v -o CMakeFiles/cmTC_b896c.dir/CMakeCCompilerABI.c.o -c /usr/share/cmake-3.28/Modules/CMakeCCompilerABI.c]
        ignore line: [Using built-in specs.]
        ignore line: [COLLECT_GCC=/usr/bin/cc]
        ignore line: [OFFLOAD_TARGET_NAMES=nvptx-none:amdgcn-amdhsa]
        ignore line: [OFFLOAD_TARGET_DEFAULT=1]
        ignore line: [Target: x86_64-linux-gnu]
        ignore line: [Configured with: ../src/configure -v --with-pkgversion='Ubuntu 13.3.0-6ubuntu2~24.04.1' --with-bugurl=file:///usr/share/doc/gcc-13/README.Bugs --enable-languages=c ada c++ go d fortran objc obj-c++ m2 --prefix=/usr --with-gcc-major-version-only --program-suffix=-13 --program-prefix=x86_64-linux-gnu- --enable-shared --enable-linker-build-id --libexecdir=/usr/libexec --without-included-gettext --enable-threads=posix --libdir=/usr/lib --enable-nls --enable-bootstrap --enable-clocale=gnu --enable-libstdcxx-debug --enable-libstdcxx-time=yes --with-default-libstdcxx-abi=new --enable-libstdcxx-backtrace --enable-gnu-unique-object --disable-vtable-verify --enable-plugin --enable-default-pie --with-system-zlib --enable-libphobos-checking=release --with-target-system-zlib=auto --enable-objc-gc=auto --enable-multiarch --disable-werror --enable-cet --with-arch-32=i686 --with-abi=m64 --with-multilib-list=m32 m64 mx32 --enable-multilib --with-tune=generic --enable-offload-targets=nvptx-none=/build/gcc-13-EldibY/gcc-13-13.3.0/debian/tmp-nvptx/usr amdgcn-amdhsa=/build/gcc-13-EldibY/gcc-13-13.3.0/debian/tmp-gcn/usr --enable-offload-defaulted --without-cuda-driver --enable-checking=release --build=x86_64-linux-gnu --host=x86_64-linux-gnu --target=x86_64-linux-gnu --with-build-config=bootstrap-lto-lean --enable-link-serialization=2]
        ignore line: [Thread model: posix]
        ignore line: [Supported LTO compression algorithms: zlib zstd]
        ignore line: [gcc version 13.3.0 (Ubuntu 13.3.0-6ubuntu2~24.04.1) ]
        ignore line: [COLLECT_GCC_OPTIONS='-v' '-o' 'CMakeFiles/cmTC_b896c.dir/CMakeCCompilerABI.c.o' '-c' '-mtune=generic' '-march=x86-64' '-dumpdir' 'CMakeFiles/cmTC_b896c.dir/']
        ignore line: [ /usr/libexec/gcc/x86_64-linux-gnu/13/cc1 -quiet -v -imultiarch x86_64-linux-gnu /usr/share/cmake-3.28/Modules/CMakeCCompilerABI.c -quiet -dumpdir CMakeFiles/cmTC_b896c.dir/ -dumpbase CMakeCCompilerABI.c.c -dumpbase-ext .c -mtune=generic -march=x86-64 -version -fasynchronous-unwind-tables -fstack-protector-strong -Wformat -Wformat-security -fstack-clash-protection -fcf-protection -o /tmp/ccVDVf0d.s]
        ignore line: [GNU C17 (Ubuntu 13.3.0-6ubuntu2~24.04.1) version 13.3.0 (x86_64-linux-gnu)]
        ignore line: [	compiled by GNU C version 13.3.0  GMP version 6.3.0  MPFR version 4.2.1  MPC version 1.3.1  isl version isl-0.26-GMP]
        ignore line: []
        ignore line: [GGC heuristics: --param ggc-min-expand=100 --param ggc-min-heapsize=131072]
        ignore line: [ignoring nonexistent directory "/usr/local/include/x86_64-linux-gnu"]
        ignore line: [ignoring nonexistent directory "/usr/lib/gcc/x86_64-linux-gnu/13/include-fixed/x86_64-linux-gnu"]
        ignore line: [ignoring nonexistent directory "/usr/lib/gcc/x86_64-linux-gnu/13/include-fixed"]
        ignore line: [ignoring nonexistent directory "/usr/lib/gcc/x86_64-linux-gnu/13/../../../../x86_64-linux-gnu/include"]
        ignore line: [#include "..." search starts here:]
        ignore line: [#include <...> search starts here:]
        ignore line: [ /usr/lib/gcc/x86_64-linux-gnu/13/include]
        ignore line: [ /usr/local/include]
        ignore line: [ /usr/include/x86_64-linux-gnu]
        ignore line: [ /usr/include]
        ignore line: [End of search list.]
        ignore line: [Compiler executable checksum: b220a7f1a1f69970d969d254ad9ec166]
        ignore line: [COLLECT_GCC_OPTIONS='-v' '-o' 'CMakeFiles/cmTC_b896c.dir/CMakeCCompilerABI.c.o' '-c' '-mtune=generic' '-march=x86-64' '-dumpdir' 'CMakeFiles/cmTC_b896c.dir/']
        ignore line: [ as -v --64 -o CMakeFiles/cmTC_b896c.dir/CMakeCCompilerABI.c.o /tmp/ccVDVf0d.s]
        ignore line: [GNU assembler version 2.42 (x86_64-linux-gnu) using BFD version (GNU Binutils for Ubuntu) 2.42]
        ignore line: [COMPILER_PATH=/usr/libexec/gcc/x86_64-linux-gnu/13/:/usr/libexec/gcc/x86_64-linux-gnu/13/:/usr/libexec/gcc/x86_64-linux-gnu/:/usr/lib/gcc/x86_64-linux-gnu/13/:/usr/lib/gcc/x86_64-linux-gnu/]
        ignore line: [LIBRARY_PATH=/usr/lib/gcc/x86_64-linux-gnu/13/:/usr/lib/gcc/x86_64-linux-gnu/13/../../../x86_64-linux-gnu/:/usr/lib/gcc/x86_64-linux-gnu/13/../../../../lib/:/lib/x86_64-linux-gnu/:/lib/../lib/:/usr/lib/x86_64-linux-gnu/:/usr/lib/../lib/:/usr/lib/gcc/x86_64-linux-gnu/13/../../../:/lib/:/usr/lib/]
        ignore line: [COLLECT_GCC_OPTIONS='-v' '-o' 'CMakeFiles/cmTC_b896c.dir/CMakeCCompilerABI.c.o' '-c' '-mtune=generic' '-march=x86-64' '-dumpdir' 'CMakeFiles/cmTC_b896c.dir/CMakeCCompilerABI.c.']
        ignore line: [Linking C executable cmTC_b896c]
        ignore line: [/usr/bin/cmake -E cmake_link_script CMakeFiles/cmTC_b896c.dir/link.txt --verbose=1]
        ignore line: [/usr/bin/cc  -v CMakeFiles/cmTC_b896c.dir/CMakeCCompilerABI.c.o -o cmTC_b896c ]
        ignore line: [Using built-in specs.]
        ignore line: [COLLECT_GCC=/usr/bin/cc]
        ignore line: [COLLECT_LTO_WRAPPER=/usr/libexec/gcc/x86_64-linux-gnu/13/lto-wrapper]
        ignore line: [OFFLOAD_TARGET_NAMES=nvptx-none:amdgcn-amdhsa]
        ignore line: [OFFLOAD_TARGET_DEFAULT=1]
        ignore line: [Target: x86_64-linux-gnu]
        ignore line: [Configured with: ../src/configure -v --with-pkgversion='Ubuntu 13.3.0-6ubuntu2~24.04.1' --with-bugurl=file:///usr/share/doc/gcc-13/README.Bugs --enable-languages=c ada c++ go d fortran objc obj-c++ m2 --prefix=/usr --with-gcc-major-version-only --program-suffix=-13 --program-prefix=x86_64-linux-gnu- --enable-shared --enable-linker-build-id --libexecdir=/usr/libexec --without-included-gettext --enable-threads=posix --libdir=/usr/lib --enable-nls --enable-bootstrap --enable-clocale=gnu --enable-libstdcxx-debug --enable-libstdcxx-time=yes --with-default-libstdcxx-abi=new --enable-libstdcxx-backtrace --enable-gnu-unique-object --disable-vtable-verify --enable-plugin --enable-default-pie --with-system-zlib --enable-libphobos-checking=release --with-target-system-zlib=auto --enable-objc-gc=auto --enable-multiarch --disable-werror --enable-cet --with-arch-32=i686 --with-abi=m64 --with-multilib-list=m32 m64 mx32 --enable-multilib --with-tune=generic --enable-offload-targets=nvptx-none=/build/gcc-13-EldibY/gcc-13-13.3.0/debian/tmp-nvptx/usr amdgcn-amdhsa=/build/gcc-13-EldibY/gcc-13-13.3.0/debian/tmp-gcn/usr --enable-offload-defaulted --without-cuda-driver --enable-checking=release --build=x86_64-linux-gnu --host=x86_64-linux-gnu --target=x86_64-linux-gnu --with-build-config=bootstrap-lto-lean --enable-link-serialization=2]
        ignore line: [Thread model: posix]
        ignore line: [Supported LTO compression algorithms: zlib zstd]
        ignore line: [gcc version 13.3.0 (Ubuntu 13.3.0-6ubuntu2~24.04.1) ]
        ignore line: [COMPILER_PATH=/usr/libexec/gcc/x86_64-linux-gnu/13/:/usr/libexec/gcc/x86_64-linux-gnu/13/:/usr/libexec/gcc/x86_64-linux-gnu/:/usr/lib/gcc/x86_64-linux-gnu/13/:/usr/lib/gcc/x86_64-linux-gnu/]
        ignore line: [LIBRARY_PATH=/usr/lib/gcc/x86_64-linux-gnu/13/:/usr/lib/gcc/x86_64-linux-gnu/13/../../../x86_64-linux-gnu/:/usr/lib/gcc/x86_64-linux-gnu/13/../../../../lib/:/lib/x86_64-linux-gnu/:/lib/../lib/:/usr/lib/x86_64-linux-gnu/:/usr/lib/../lib/:/usr/lib/gcc/x86_64-linux-gnu/13/../../../:/lib/:/usr/lib/]
        ignore line: [COLLECT_GCC_OPTIONS='-v' '-o' 'cmTC_b896c' '-mtune=generic' '-march=x86-64' '-dumpdir' 'cmTC_b896c.']
        link line: [ /usr/libexec/gcc/x86_64-linux-gnu/13/collect2 -plugin /usr/libexec/gcc/x86_64-linux-gnu/13/liblto_plugin.so -plugin-opt=/usr/libexec/gcc/x86_64-linux-gnu/13/lto-wrapper -plugin-opt=-fresolution=/tmp/ccDl1mTN.res -plugin-opt=-pass-through=-lgcc -plugin-opt=-pass-through=-lgcc_s -plugin-opt=-pass-through=-lc -plugin-opt=-pass-through=-lgcc -plugin-opt=-pass-through=-lgcc_s --build-id --eh-frame-hdr -m elf_x86_64 --hash-style=gnu --as-needed -dynamic-linker /lib64/ld-linux-x86-64.so.2 -pie -z now -z relro -o cmTC_b896c /usr/lib/gcc/x86_64-linux-gnu/13/../../../x86_64-linux-gnu/Scrt1.o /usr/lib/gcc/x86_64-linux-gnu/13/../../../x86_64-linux-gnu/crti.o /usr/lib/gcc/x86_64-linux-gnu/13/crtbeginS.o -L/usr/lib/gcc/x86_64-linux-gnu/13 -L/usr/lib/gcc/x86_64-linux-gnu/13/../../../x86_64-linux-gnu -L/usr/lib/gcc/x86_64-linux-gnu/13/../../../../lib -L/lib/x86_64-linux-gnu -L/lib/../lib -L/usr/lib/x86_64-linux-gnu -L/usr/lib/../lib -L/usr/lib/gcc/x86_64-linux-gnu/13/../../.. CMakeFiles/cmTC_b896c.dir/CMakeCCompilerABI.c.o -lgcc --push-state --as-needed -lgcc_s --pop-state -lc -lgcc --push-state --as-needed -lgcc_s --pop-state /usr/lib/gcc/x86_64-linux-gnu/13/crtendS.o /usr/lib/gcc/x86_64-linux-gnu/13/../../../x86_64-linux-gnu/crtn.o]
          arg [/usr/libexec/gcc/x86_64-linux-gnu/13/collect2] ==> ignore
          arg [-plugin] ==> ignore
          arg [/usr/libexec/gcc/x86_64-linux-gnu/13/liblto_plugin.so] ==> ignore
          arg [-plugin-opt=/usr/libexec/gcc/x86_64-linux-gnu/13/lto-wrapper] ==> ignore
          arg [-plugin-opt=-fresolution=/tmp/ccDl1mTN.res] ==> ignore
          arg [-plugin-opt=-pass-through=-lgcc] ==> ignore
          arg [-plugin-opt=-pass-through=-lgcc_s] ==> ignore
          arg [-plugin-opt=-pass-through=-lc] ==> ignore
          arg [-plugin-opt=-pass-through=-lgcc] ==> ignore
          arg [-plugin-opt=-pass-through=-lgcc_s] ==> ignore
          arg [--build-id] ==> ignore
          arg [--eh-frame-hdr] ==> ignore
          arg [-m] ==> ignore
          arg [elf_x86_64] ==> ignore
          arg [--hash-style=gnu] ==> ignore
          arg [--as-needed] ==> ignore
          arg [-dynamic-linker] ==> ignore
          arg [/lib64/ld-linux-x86-64.so.2] ==> ignore
          arg [-pie] ==> ignore
          arg [-znow] ==> ignore
          arg [-zrelro] ==> ignore
          arg [-o] ==> ignore
          arg [cmTC_b896c] ==> ignore
          arg [/usr/lib/gcc/x86_64-linux-gnu/13/../../../x86_64-linux-gnu/Scrt1.o] ==> obj [/usr/lib/gcc/x86_64-linux-gnu/13/../../../x86_64-linux-gnu/Scrt1.o]
          arg [/usr/lib/gcc/x86_64-linux-gnu/13/../../../x86_64-linux-gnu/crti.o] ==> obj [/usr/lib/gcc/x86_64-linux-gnu/13/../../../x86_64-linux-gnu/crti.o]
          arg [/usr/lib/gcc/x86_64-linux-gnu/13/crtbeginS.o] ==> obj [/usr/lib/gcc/x86_64-linux-gnu/13/crtbeginS.o]
          arg [-L/usr/lib/gcc/x86_64-linux-gnu/13] ==> dir [/usr/lib/gcc/x86_64-linux-gnu/13]
          arg [-L/usr/lib/gcc/x86_64-linux-gnu/13/../../../x86_64-linux-gnu] ==> dir [/usr/lib/gcc/x86_64-linux-gnu/13/../../../x86_64-linux-gnu]
          arg [-L/usr/lib/gcc/x86_64-linux-gnu/13/../../../../lib] ==> dir [/usr/lib/gcc/x86_64-linux-gnu/13/../../../../lib]
          arg [-L/lib/x86_64-linux-gnu] ==> dir [/lib/x86_64-linux-gnu]
          arg [-L/lib/../lib] ==> dir [/lib/../lib]
          arg [-L/usr/lib/x86_64-linux-gnu] ==> dir [/usr/lib/x86_64-linux-gnu]
          arg [-L/usr/lib/../lib] ==> dir [/usr/lib/../lib]
          arg [-L/usr/lib/gcc/x86_64-linux-gnu/13/../../..] ==> dir [/usr/lib/gcc/x86_64-linux-gnu/13/../../..]
          arg [CMakeFiles/cmTC_b896c.dir/CMakeCCompilerABI.c.o] ==> ignore
          arg [-lgcc] ==> lib [gcc]
          arg [--push-state] ==> ignore
          arg [--as-needed] ==> ignore
          arg [-lgcc_s] ==> lib [gcc_s]
          arg [--pop-state] ==> ignore
          arg [-lc] ==> lib [c]
          arg [-lgcc] ==> lib [gcc]
          arg [--push-state] ==> ignore
          arg [--as-needed] ==> ignore
          arg [-lgcc_s] ==> lib [gcc_s]
          arg [--pop-state] ==> ignore
          arg [/usr/lib/gcc/x86_64-linux-gnu/13/crtendS.o] ==> obj [/usr/lib/gcc/x86_64-linux-gnu/13/crtendS.o]
          arg [/usr/lib/gcc/x86_64-linux-gnu/13/../../../x86_64-linux-gnu/crtn.o] ==> obj [/usr/lib/gcc/x86_64-linux-gnu/13/../../../x86_64-linux-gnu/crtn.o]
        collapse obj [/usr/lib/gcc/x86_64-linux-gnu/13/../../../x86_64-linux-gnu/Scrt1.o] ==> [/usr/lib/x86_64-linux-gnu/Scrt1.o]
        collapse obj [/usr/lib/gcc/x86_64-linux-gnu/13/../../../x86_64-linux-gnu/crti.o] ==> [/usr/lib/x86_64-linux-gnu/crti.o]
        collapse obj [/usr/lib/gcc/x86_64-linux-gnu/13/../../../x86_64-linux-gnu/crtn.o] ==> [/usr/lib/x86_64-linux-gnu/crtn.o]
        collapse library dir [/usr/lib/gcc/x86_64-linux-gnu/13] ==> [/usr/lib/gcc/x86_64-linux-gnu/13]
        collapse library dir [/usr/lib/gcc/x86_64-linux-gnu/13/../../../x86_64-linux-gnu] ==> [/usr/lib/x86_64-linux-gnu]
        collapse library dir [/usr/lib/gcc/x86_64-linux-gnu/13/../../../../lib] ==> [/usr/lib]
        collapse library dir [/lib/x86_64-linux-gnu] ==> [/lib/x86_64-linux-gnu]
        collapse library dir [/lib/../lib] ==> [/lib]
        collapse library dir [/usr/lib/x86_64-linux-gnu] ==> [/usr/lib/x86_64-linux-gnu]
        collapse library dir [/usr/lib/../lib] ==> [/usr/lib]
        collapse library dir [/usr/lib/gcc/x86_64-linux-gnu/13/../../..] ==> [/usr/lib]
        implicit libs: [gcc;gcc_s;c;gcc;gcc_s]
        implicit objs: [/usr/lib/x86_64-linux-gnu/Scrt1.o;/usr/lib/x86_64-linux-gnu/crti.o;/usr/lib/gcc/x86_64-linux-gnu/13/crtbeginS.o;/usr/lib/gcc/x86_64-linux-gnu/13/crtendS.o;/usr/lib/x86_64-linux-gnu/crtn.o]
        implicit dirs: [/usr/lib/gcc/x86_64-linux-gnu/13;/usr/lib/x86_64-linux-gnu;/usr/lib;/lib/x86_64-linux-gnu;/lib]
        implicit fwks: []
      
      
  -
    kind: "try_compile-v1"
    backtrace:
      - "/usr/share/cmake-3.28/Modules/CMakeDetermineCompilerABI.cmake:57 (try_compile)"
      - "/usr/share/cmake-3.28/Modules/CMakeTestCXXCompiler.cmake:26 (CMAKE_DETERMINE_COMPILER_ABI)"
      - "CMakeLists.txt:2 (project)"
    checks:
      - "Detecting CXX compiler ABI info"
    directories:
      source: "/home/student31/leo_ws/build/leo_lidar_sim/CMakeFiles/CMakeScratch/TryCompile-x8n4hu"
      binary: "/home/student31/leo_ws/build/leo_lidar_sim/CMakeFiles/CMakeScratch/TryCompile-x8n4hu"
    cmakeVariables:
      CMAKE_CXX_FLAGS: ""
      CMAKE_CXX_FLAGS_DEBUG: "-g"
      CMAKE_EXE_LINKER_FLAGS: ""
    buildResult:
      variable: "CMAKE_CXX_ABI_COMPILED"
      cached: true
      stdout: |
        Change Dir: '/home/student31/leo_ws/build/leo_lidar_sim/CMakeFiles/CMakeScratch/TryCompile-x8n4hu'
        
        Run Build Command(s): /usr/bin/cmake -E env VERBOSE=1 /usr/bin/gmake -f Makefile cmTC_07f7d/fast
        /usr/bin/gmake  -f CMakeFiles/cmTC_07f7d.dir/build.make CMakeFiles/cmTC_07f7d.dir/build
        gmake[1]: Entering directory '/home/student31/leo_ws/build/leo_lidar_sim/CMakeFiles/CMakeScratch/TryCompile-x8n4hu'
        Building CXX object CMakeFiles/cmTC_07f7d.dir/CMakeCXXCompilerABI.cpp.o
        /usr/bin/c++   -v -o CMakeFiles/cmTC_07f7d.dir/CMakeCXXCompilerABI.cpp.o -c /usr/share/cmake-3.28/Modules/CMakeCXXCompilerABI.cpp
        Using built-in specs.
        COLLECT_GCC=/usr/bin/c++
        OFFLOAD_TARGET_NAMES=nvptx-none:amdgcn-amdhsa
        OFFLOAD_TARGET_DEFAULT=1
        Target: x86_64-linux-gnu
        Configured with: ../src/configure -v --with-pkgversion='Ubuntu 13.3.0-6ubuntu2~24.04.1' --with-bugurl=file:///usr/share/doc/gcc-13/README.Bugs --enable-languages=c,ada,c++,go,d,fortran,objc,obj-c++,m2 --prefix=/usr --with-gcc-major-version-only --program-suffix=-13 --program-prefix=x86_64-linux-gnu- --enable-shared --enable-linker-build-id --libexecdir=/usr/libexec --without-included-gettext --enable-threads=posix --libdir=/usr/lib --enable-nls --enable-bootstrap --enable-clocale=gnu --enable-libstdcxx-debug --enable-libstdcxx-time=yes --with-default-libstdcxx-abi=new --enable-libstdcxx-backtrace --enable-gnu-unique-object --disable-vtable-verify --enable-plugin --enable-default-pie --with-system-zlib --enable-libphobos-checking=release --with-target-system-zlib=auto --enable-objc-gc=auto --enable-multiarch --disable-werror --enable-cet --with-arch-32=i686 --with-abi=m64 --with-multilib-list=m32,m64,mx32 --enable-multilib --with-tune=generic --enable-offload-targets=nvptx-none=/build/gcc-13-EldibY/gcc-13-13.3.0/debian/tmp-nvptx/usr,amdgcn-amdhsa=/build/gcc-13-EldibY/gcc-13-13.3.0/debian/tmp-gcn/usr --enable-offload-defaulted --without-cuda-driver --enable-checking=release --build=x86_64-linux-gnu --host=x86_64-linux-gnu --target=x86_64-linux-gnu --with-build-config=bootstrap-lto-lean --enable-link-serialization=2
        Thread model: posix
        Supported LTO compression algorithms: zlib zstd
        gcc version 13.3.0 (Ubuntu 13.3.0-6ubuntu2~24.04.1) 
        COLLECT_GCC_OPTIONS='-v' '-o' 'CMakeFiles/cmTC_07f7d.dir/CMakeCXXCompilerABI.cpp.o' '-c' '-shared-libgcc' '-mtune=generic' '-march=x86-64' '-dumpdir' 'CMakeFiles/cmTC_07f7d.dir/'
         /usr/libexec/gcc/x86_64-linux-gnu/13/cc1plus -quiet -v -imultiarch x86_64-linux-gnu -D_GNU_SOURCE /usr/share/cmake-3.28/Modules/CMakeCXXCompilerABI.cpp -quiet -dumpdir CMakeFiles/cmTC_07f7d.dir/ -dumpbase CMakeCXXCompilerABI.cpp.cpp -dumpbase-ext .cpp -mtune=generic -march=x86-64 -version -fasynchronous-unwind-tables -fstack-protector-strong -Wformat -Wformat-security -fstack-clash-protection -fcf-protection -o /tmp/cc2yMtVe.s
        GNU C++17 (Ubuntu 13.3.0-6ubuntu2~24.04.1) version 13.3.0 (x86_64-linux-gnu)
        	compiled by GNU C version 13.3.0, GMP version 6.3.0, MPFR version 4.2.1, MPC version 1.3.1, isl version isl-0.26-GMP
        
        GGC heuristics: --param ggc-min-expand=100 --param ggc-min-heapsize=131072
        ignoring duplicate directory "/usr/include/x86_64-linux-gnu/c++/13"
        ignoring nonexistent directory "/usr/local/include/x86_64-linux-gnu"
        ignoring nonexistent directory "/usr/lib/gcc/x86_64-linux-gnu/13/include-fixed/x86_64-linux-gnu"
        ignoring nonexistent directory "/usr/lib/gcc/x86_64-linux-gnu/13/include-fixed"
        ignoring nonexistent directory "/usr/lib/gcc/x86_64-linux-gnu/13/../../../../x86_64-linux-gnu/include"
        #include "..." search starts here:
        #include <...> search starts here:
         /usr/include/c++/13
         /usr/include/x86_64-linux-gnu/c++/13
         /usr/include/c++/13/backward
         /usr/lib/gcc/x86_64-linux-gnu/13/include
         /usr/local/include
         /usr/include/x86_64-linux-gnu
         /usr/include
        End of search list.
        Compiler executable checksum: 7896445e4990772fdae9dc0659a99266
        COLLECT_GCC_OPTIONS='-v' '-o' 'CMakeFiles/cmTC_07f7d.dir/CMakeCXXCompilerABI.cpp.o' '-c' '-shared-libgcc' '-mtune=generic' '-march=x86-64' '-dumpdir' 'CMakeFiles/cmTC_07f7d.dir/'
         as -v --64 -o CMakeFiles/cmTC_07f7d.dir/CMakeCXXCompilerABI.cpp.o /tmp/cc2yMtVe.s
        GNU assembler version 2.42 (x86_64-linux-gnu) using BFD version (GNU Binutils for Ubuntu) 2.42
        COMPILER_PATH=/usr/libexec/gcc/x86_64-linux-gnu/13/:/usr/libexec/gcc/x86_64-linux-gnu/13/:/usr/libexec/gcc/x86_64-linux-gnu/:/usr/lib/gcc/x86_64-linux-gnu/13/:/usr/lib/gcc/x86_64-linux-gnu/
        LIBRARY_PATH=/usr/lib/gcc/x86_64-linux-gnu/13/:/usr/lib/gcc/x86_64-linux-gnu/13/../../../x86_64-linux-gnu/:/usr/lib/gcc/x86_64-linux-gnu/13/../../../../lib/:/lib/x86_64-linux-gnu/:/lib/../lib/:/usr/lib/x86_64-linux-gnu/:/usr/lib/../lib/:/usr/lib/gcc/x86_64-linux-gnu/13/../../../:/lib/:/usr/lib/
        COLLECT_GCC_OPTIONS='-v' '-o' 'CMakeFiles/cmTC_07f7d.dir/CMakeCXXCompilerABI.cpp.o' '-c' '-shared-libgcc' '-mtune=generic' '-march=x86-64' '-dumpdir' 'CMakeFiles/cmTC_07f7d.dir/CMakeCXXCompilerABI.cpp.'
        Linking CXX executable cmTC_07f7d
        /usr/bin/cmake -E cmake_link_script CMakeFiles/cmTC_07f7d.dir/link.txt --verbose=1
        /usr/bin/c++  -v CMakeFiles/cmTC_07f7d.dir/CMakeCXXCompilerABI.cpp.o -o cmTC_07f7d 
        Using built-in specs.
        COLLECT_GCC=/usr/bin/c++
        COLLECT_LTO_WRAPPER=/usr/libexec/gcc/x86_64-linux-gnu/13/lto-wrapper
        OFFLOAD_TARGET_NAMES=nvptx-none:amdgcn-amdhsa
        OFFLOAD_TARGET_DEFAULT=1
        Target: x86_64-linux-gnu
        Configured with: ../src/configure -v --with-pkgversion='Ubuntu 13.3.0-6ubuntu2~24.04.1' --with-bugurl=file:///usr/share/doc/gcc-13/README.Bugs --enable-languages=c,ada,c++,go,d,fortran,objc,obj-c++,m2 --prefix=/usr --with-gcc-major-version-only --program-suffix=-13 --program-prefix=x86_64-linux-gnu- --enable-shared --enable-linker-build-id --libexecdir=/usr/libexec --without-included-gettext --enable-threads=posix --libdir=/usr/lib --enable-nls --enable-bootstrap --enable-clocale=gnu --enable-libstdcxx-debug --enable-libstdcxx-time=yes --with-default-libstdcxx-abi=new --enable-libstdcxx-backtrace --enable-gnu-unique-object --disable-vtable-verify --enable-plugin --enable-default-pie --with-system-zlib --enable-libphobos-checking=release --with-target-system-zlib=auto --enable-objc-gc=auto --enable-multiarch --disable-werror --enable-cet --with-arch-32=i686 --with-abi=m64 --with-multilib-list=m32,m64,mx32 --enable-multilib --with-tune=generic --enable-offload-targets=nvptx-none=/build/gcc-13-EldibY/gcc-13-13.3.0/debian/tmp-nvptx/usr,amdgcn-amdhsa=/build/gcc-13-EldibY/gcc-13-13.3.0/debian/tmp-gcn/usr --enable-offload-defaulted --without-cuda-driver --enable-checking=release --build=x86_64-linux-gnu --host=x86_64-linux-gnu --target=x86_64-linux-gnu --with-build-config=bootstrap-lto-lean --enable-link-serialization=2
        Thread model: posix
        Supported LTO compression algorithms: zlib zstd
        gcc version 13.3.0 (Ubuntu 13.3.0-6ubuntu2~24.04.1) 
        COMPILER_PATH=/usr/libexec/gcc/x86_64-linux-gnu/13/:/usr/libexec/gcc/x86_64-linux-gnu/13/:/usr/libexec/gcc/x86_64-linux-gnu/:/usr/lib/gcc/x86_64-linux-gnu/13/:/usr/lib/gcc/x86_64-linux-gnu/
        LIBRARY_PATH=/usr/lib/gcc/x86_64-linux-gnu/13/:/usr/lib/gcc/x86_64-linux-gnu/13/../../../x86_64-linux-gnu/:/usr/lib/gcc/x86_64-linux-gnu/13/../../../../lib/:/lib/x86_64-linux-gnu/:/lib/../lib/:/usr/lib/x86_64-linux-gnu/:/usr/lib/../lib/:/usr/lib/gcc/x86_64-linux-gnu/13/../../../:/lib/:/usr/lib/
        COLLECT_GCC_OPTIONS='-v' '-o' 'cmTC_07f7d' '-shared-libgcc' '-mtune=generic' '-march=x86-64' '-dumpdir' 'cmTC_07f7d.'
         /usr/libexec/gcc/x86_64-linux-gnu/13/collect2 -plugin /usr/libexec/gcc/x86_64-linux-gnu/13/liblto_plugin.so -plugin-opt=/usr/libexec/gcc/x86_64-linux-gnu/13/lto-wrapper -plugin-opt=-fresolution=/tmp/ccXTKFH9.res -plugin-opt=-pass-through=-lgcc_s -plugin-opt=-pass-through=-lgcc -plugin-opt=-pass-through=-lc -plugin-opt=-pass-through=-lgcc_s -plugin-opt=-pass-through=-lgcc --build-id --eh-frame-hdr -m elf_x86_64 --hash-style=gnu --as-needed -dynamic-linker /lib64/ld-linux-x86-64.so.2 -pie -z now -z relro -o cmTC_07f7d /usr/lib/gcc/x86_64-linux-gnu/13/../../../x86_64-linux-gnu/Scrt1.o /usr/lib/gcc/x86_64-linux-gnu/13/../../../x86_64-linux-gnu/crti.o /usr/lib/gcc/x86_64-linux-gnu/13/crtbeginS.o -L/usr/lib/gcc/x86_64-linux-gnu/13 -L/usr/lib/gcc/x86_64-linux-gnu/13/../../../x86_64-linux-gnu -L/usr/lib/gcc/x86_64-linux-gnu/13/../../../../lib -L/lib/x86_64-linux-gnu -L/lib/../lib -L/usr/lib/x86_64-linux-gnu -L/usr/lib/../lib -L/usr/lib/gcc/x86_64-linux-gnu/13/../../.. CMakeFiles/cmTC_07f7d.dir/CMakeCXXCompilerABI.cpp.o -lstdc++ -lm -lgcc_s -lgcc -lc -lgcc_s -lgcc /usr/lib/gcc/x86_64-linux-gnu/13/crtendS.o /usr/lib/gcc/x86_64-linux-gnu/13/../../../x86_64-linux-gnu/crtn.o
        COLLECT_GCC_OPTIONS='-v' '-o' 'cmTC_07f7d' '-shared-libgcc' '-mtune=generic' '-march=x86-64' '-dumpdir' 'cmTC_07f7d.'
        gmake[1]: Leaving directory '/home/student31/leo_ws/build/leo_lidar_sim/CMakeFiles/CMakeScratch/TryCompile-x8n4hu'
        
      exitCode: 0
  -
    kind: "message-v1"
    backtrace:
      - "/usr/share/cmake-3.28/Modules/CMakeDetermineCompilerABI.cmake:127 (message)"
      - "/usr/share/cmake-3.28/Modules/CMakeTestCXXCompiler.cmake:26 (CMAKE_DETERMINE_COMPILER_ABI)"
      - "CMakeLists.txt:2 (project)"
    message: |
      Parsed CXX implicit include dir info: rv=done
        found start of include info
        found start of implicit include info
          add: [/usr/include/c++/13]
          add: [/usr/include/x86_64-linux-gnu/c++/13]
          add: [/usr/include/c++/13/backward]
          add: [/usr/lib/gcc/x86_64-linux-gnu/13/include]
          add: [/usr/local/include]
          add: [/usr/include/x86_64-linux-gnu]
          add: [/usr/include]
        end of search list found
        collapse include dir [/usr/include/c++/13] ==> [/usr/include/c++/13]
        collapse include dir [/usr/include/x86_64-linux-gnu/c++/13] ==> [/usr/include/x86_64-linux-gnu/c++/13]
        collapse include dir [/usr/include/c++/13/backward] ==> [/usr/include/c++/13/backward]
        collapse include dir [/usr/lib/gcc/x86_64-linux-gnu/13/include] ==> [/usr/lib/gcc/x86_64-linux-gnu/13/include]
        collapse include dir [/usr/local/include] ==> [/usr/local/include]
        collapse include dir [/usr/include/x86_64-linux-gnu] ==> [/usr/include/x86_64-linux-gnu]
        collapse include dir [/usr/include] ==> [/usr/include]
        implicit include dirs: [/usr/include/c++/13;/usr/include/x86_64-linux-gnu/c++/13;/usr/include/c++/13/backward;/usr/lib/gcc/x86_64-linux-gnu/13/include;/usr/local/include;/usr/include/x86_64-linux-gnu;/usr/include]
      
      
  -
    kind: "message-v1"
    backtrace:
      - "/usr/share/cmake-3.28/Modules/CMakeDetermineCompilerABI.cmake:159 (message)"
      - "/usr/share/cmake-3.28/Modules/CMakeTestCXXCompiler.cmake:26 (CMAKE_DETERMINE_COMPILER_ABI)"
      - "CMakeLists.txt:2 (project)"
    message: |
      Parsed CXX implicit link information:
        link line regex: [^( *|.*[/\\])(ld|CMAKE_LINK_STARTFILE-NOTFOUND|([^/\\]+-)?ld|collect2)[^/\\]*( |$)]
        ignore line: [Change Dir: '/home/student31/leo_ws/build/leo_lidar_sim/CMakeFiles/CMakeScratch/TryCompile-x8n4hu']
        ignore line: []
        ignore line: [Run Build Command(s): /usr/bin/cmake -E env VERBOSE=1 /usr/bin/gmake -f Makefile cmTC_07f7d/fast]
        ignore line: [/usr/bin/gmake  -f CMakeFiles/cmTC_07f7d.dir/build.make CMakeFiles/cmTC_07f7d.dir/build]
        ignore line: [gmake[1]: Entering directory '/home/student31/leo_ws/build/leo_lidar_sim/CMakeFiles/CMakeScratch/TryCompile-x8n4hu']
        ignore line: [Building CXX object CMakeFiles/cmTC_07f7d.dir/CMakeCXXCompilerABI.cpp.o]
        ignore line: [/usr/bin/c++   -v -o CMakeFiles/cmTC_07f7d.dir/CMakeCXXCompilerABI.cpp.o -c /usr/share/cmake-3.28/Modules/CMakeCXXCompilerABI.cpp]
        ignore line: [Using built-in specs.]
        ignore line: [COLLECT_GCC=/usr/bin/c++]
        ignore line: [OFFLOAD_TARGET_NAMES=nvptx-none:amdgcn-amdhsa]
        ignore line: [OFFLOAD_TARGET_DEFAULT=1]
        ignore line: [Target: x86_64-linux-gnu]
        ignore line: [Configured with: ../src/configure -v --with-pkgversion='Ubuntu 13.3.0-6ubuntu2~24.04.1' --with-bugurl=file:///usr/share/doc/gcc-13/README.Bugs --enable-languages=c ada c++ go d fortran objc obj-c++ m2 --prefix=/usr --with-gcc-major-version-only --program-suffix=-13 --program-prefix=x86_64-linux-gnu- --enable-shared --enable-linker-build-id --libexecdir=/usr/libexec --without-included-gettext --enable-threads=posix --libdir=/usr/lib --enable-nls --enable-bootstrap --enable-clocale=gnu --enable-libstdcxx-debug --enable-libstdcxx-time=yes --with-default-libstdcxx-abi=new --enable-libstdcxx-backtrace --enable-gnu-unique-object --disable-vtable-verify --enable-plugin --enable-default-pie --with-system-zlib --enable-libphobos-checking=release --with-target-system-zlib=auto --enable-objc-gc=auto --enable-multiarch --disable-werror --enable-cet --with-arch-32=i686 --with-abi=m64 --with-multilib-list=m32 m64 mx32 --enable-multilib --with-tune=generic --enable-offload-targets=nvptx-none=/build/gcc-13-EldibY/gcc-13-13.3.0/debian/tmp-nvptx/usr amdgcn-amdhsa=/build/gcc-13-EldibY/gcc-13-13.3.0/debian/tmp-gcn/usr --enable-offload-defaulted --without-cuda-driver --enable-checking=release --build=x86_64-linux-gnu --host=x86_64-linux-gnu --target=x86_64-linux-gnu --with-build-config=bootstrap-lto-lean --enable-link-serialization=2]
        ignore line: [Thread model: posix]
        ignore line: [Supported LTO compression algorithms: zlib zstd]
        ignore line: [gcc version 13.3.0 (Ubuntu 13.3.0-6ubuntu2~24.04.1) ]
        ignore line: [COLLECT_GCC_OPTIONS='-v' '-o' 'CMakeFiles/cmTC_07f7d.dir/CMakeCXXCompilerABI.cpp.o' '-c' '-shared-libgcc' '-mtune=generic' '-march=x86-64' '-dumpdir' 'CMakeFiles/cmTC_07f7d.dir/']
        ignore line: [ /usr/libexec/gcc/x86_64-linux-gnu/13/cc1plus -quiet -v -imultiarch x86_64-linux-gnu -D_GNU_SOURCE /usr/share/cmake-3.28/Modules/CMakeCXXCompilerABI.cpp -quiet -dumpdir CMakeFiles/cmTC_07f7d.dir/ -dumpbase CMakeCXXCompilerABI.cpp.cpp -dumpbase-ext .cpp -mtune=generic -march=x86-64 -version -fasynchronous-unwind-tables -fstack-protector-strong -Wformat -Wformat-security -fstack-clash-protection -fcf-protection -o /tmp/cc2yMtVe.s]
        ignore line: [GNU C++17 (Ubuntu 13.3.0-6ubuntu2~24.04.1) version 13.3.0 (x86_64-linux-gnu)]
        ignore line: [	compiled by GNU C version 13.3.0  GMP version 6.3.0  MPFR version 4.2.1  MPC version 1.3.1  isl version isl-0.26-GMP]
        ignore line: []
        ignore line: [GGC heuristics: --param ggc-min-expand=100 --param ggc-min-heapsize=131072]
        ignore line: [ignoring duplicate directory "/usr/include/x86_64-linux-gnu/c++/13"]
        ignore line: [ignoring nonexistent directory "/usr/local/include/x86_64-linux-gnu"]
        ignore line: [ignoring nonexistent directory "/usr/lib/gcc/x86_64-linux-gnu/13/include-fixed/x86_64-linux-gnu"]
        ignore line: [ignoring nonexistent directory "/usr/lib/gcc/x86_64-linux-gnu/13/include-fixed"]
        ignore line: [ignoring nonexistent directory "/usr/lib/gcc/x86_64-linux-gnu/13/../../../../x86_64-linux-gnu/include"]
        ignore line: [#include "..." search starts here:]
        ignore line: [#include <...> search starts here:]
        ignore line: [ /usr/include/c++/13]
        ignore line: [ /usr/include/x86_64-linux-gnu/c++/13]
        ignore line: [ /usr/include/c++/13/backward]
        ignore line: [ /usr/lib/gcc/x86_64-linux-gnu/13/include]
        ignore line: [ /usr/local/include]
        ignore line: [ /usr/include/x86_64-linux-gnu]
        ignore line: [ /usr/include]
        ignore line: [End of search list.]
        ignore line: [Compiler executable checksum: 7896445e4990772fdae9dc0659a99266]
        ignore line: [COLLECT_GCC_OPTIONS='-v' '-o' 'CMakeFiles/cmTC_07f7d.dir/CMakeCXXCompilerABI.cpp.o' '-c' '-shared-libgcc' '-mtune=generic' '-march=x86-64' '-dumpdir' 'CMakeFiles/cmTC_07f7d.dir/']
        ignore line: [ as -v --64 -o CMakeFiles/cmTC_07f7d.dir/CMakeCXXCompilerABI.cpp.o /tmp/cc2yMtVe.s]
        ignore line: [GNU assembler version 2.42 (x86_64-linux-gnu) using BFD version (GNU Binutils for Ubuntu) 2.42]
        ignore line: [COMPILER_PATH=/usr/libexec/gcc/x86_64-linux-gnu/13/:/usr/libexec/gcc/x86_64-linux-gnu/13/:/usr/libexec/gcc/x86_64-linux-gnu/:/usr/lib/gcc/x86_64-linux-gnu/13/:/usr/lib/gcc/x86_64-linux-gnu/]
        ignore line: [LIBRARY_PATH=/usr/lib/gcc/x86_64-linux-gnu/13/:/usr/lib/gcc/x86_64-linux-gnu/13/../../../x86_64-linux-gnu/:/usr/lib/gcc/x86_64-linux-gnu/13/../../../../lib/:/lib/x86_64-linux-gnu/:/lib/../lib/:/usr/lib/x86_64-linux-gnu/:/usr/lib/../lib/:/usr/lib/gcc/x86_64-linux-gnu/13/../../../:/lib/:/usr/lib/]
        ignore line: [COLLECT_GCC_OPTIONS='-v' '-o' 'CMakeFiles/cmTC_07f7d.dir/CMakeCXXCompilerABI.cpp.o' '-c' '-shared-libgcc' '-mtune=generic' '-march=x86-64' '-dumpdir' 'CMakeFiles/cmTC_07f7d.dir/CMakeCXXCompilerABI.cpp.']
        ignore line: [Linking CXX executable cmTC_07f7d]
        ignore line: [/usr/bin/cmake -E cmake_link_script CMakeFiles/cmTC_07f7d.dir/link.txt --verbose=1]
        ignore line: [/usr/bin/c++  -v CMakeFiles/cmTC_07f7d.dir/CMakeCXXCompilerABI.cpp.o -o cmTC_07f7d ]
        ignore line: [Using built-in specs.]
        ignore line: [COLLECT_GCC=/usr/bin/c++]
        ignore line: [COLLECT_LTO_WRAPPER=/usr/libexec/gcc/x86_64-linux-gnu/13/lto-wrapper]
        ignore line: [OFFLOAD_TARGET_NAMES=nvptx-none:amdgcn-amdhsa]
        ignore line: [OFFLOAD_TARGET_DEFAULT=1]
        ignore line: [Target: x86_64-linux-gnu]
        ignore line: [Configured with: ../src/configure -v --with-pkgversion='Ubuntu 13.3.0-6ubuntu2~24.04.1' --with-bugurl=file:///usr/share/doc/gcc-13/README.Bugs --enable-languages=c ada c++ go d fortran objc obj-c++ m2 --prefix=/usr --with-gcc-major-version-only --program-suffix=-13 --program-prefix=x86_64-linux-gnu- --enable-shared --enable-linker-build-id --libexecdir=/usr/libexec --without-included-gettext --enable-threads=posix --libdir=/usr/lib --enable-nls --enable-bootstrap --enable-clocale=gnu --enable-libstdcxx-debug --enable-libstdcxx-time=yes --with-default-libstdcxx-abi=new --enable-libstdcxx-backtrace --enable-gnu-unique-object --disable-vtable-verify --enable-plugin --enable-default-pie --with-system-zlib --enable-libphobos-checking=release --with-target-system-zlib=auto --enable-objc-gc=auto --enable-multiarch --disable-werror --enable-cet --with-arch-32=i686 --with-abi=m64 --with-multilib-list=m32 m64 mx32 --enable-multilib --with-tune=generic --enable-offload-targets=nvptx-none=/build/gcc-13-EldibY/gcc-13-13.3.0/debian/tmp-nvptx/usr amdgcn-amdhsa=/build/gcc-13-EldibY/gcc-13-13.3.0/debian/tmp-gcn/usr --enable-offload-defaulted --without-cuda-driver --enable-checking=release --build=x86_64-linux-gnu --host=x86_64-linux-gnu --target=x86_64-linux-gnu --with-build-config=bootstrap-lto-lean --enable-link-serialization=2]
        ignore line: [Thread model: posix]
        ignore line: [Supported LTO compression algorithms: zlib zstd]
        ignore line: [gcc version 13.3.0 (Ubuntu 13.3.0-6ubuntu2~24.04.1) ]
        ignore line: [COMPILER_PATH=/usr/libexec/gcc/x86_64-linux-gnu/13/:/usr/libexec/gcc/x86_64-linux-gnu/13/:/usr/libexec/gcc/x86_64-linux-gnu/:/usr/lib/gcc/x86_64-linux-gnu/13/:/usr/lib/gcc/x86_64-linux-gnu/]
        ignore line: [LIBRARY_PATH=/usr/lib/gcc/x86_64-linux-gnu/13/:/usr/lib/gcc/x86_64-linux-gnu/13/../../../x86_64-linux-gnu/:/usr/lib/gcc/x86_64-linux-gnu/13/../../../../lib/:/lib/x86_64-linux-gnu/:/lib/../lib/:/usr/lib/x86_64-linux-gnu/:/usr/lib/../lib/:/usr/lib/gcc/x86_64-linux-gnu/13/../../../:/lib/:/usr/lib/]
        ignore line: [COLLECT_GCC_OPTIONS='-v' '-o' 'cmTC_07f7d' '-shared-libgcc' '-mtune=generic' '-march=x86-64' '-dumpdir' 'cmTC_07f7d.']
        link line: [ /usr/libexec/gcc/x86_64-linux-gnu/13/collect2 -plugin /usr/libexec/gcc/x86_64-linux-gnu/13/liblto_plugin.so -plugin-opt=/usr/libexec/gcc/x86_64-linux-gnu/13/lto-wrapper -plugin-opt=-fresolution=/tmp/ccXTKFH9.res -plugin-opt=-pass-through=-lgcc_s -plugin-opt=-pass-through=-lgcc -plugin-opt=-pass-through=-lc -plugin-opt=-pass-through=-lgcc_s -plugin-opt=-pass-through=-lgcc --build-id --eh-frame-hdr -m elf_x86_64 --hash-style=gnu --as-needed -dynamic-linker /lib64/ld-linux-x86-64.so.2 -pie -z now -z relro -o cmTC_07f7d /usr/lib/gcc/x86_64-linux-gnu/13/../../../x86_64-linux-gnu/Scrt1.o /usr/lib/gcc/x86_64-linux-gnu/13/../../../x86_64-linux-gnu/crti.o /usr/lib/gcc/x86_64-linux-gnu/13/crtbeginS.o -L/usr/lib/gcc/x86_64-linux-gnu/13 -L/usr/lib/gcc/x86_64-linux-gnu/13/../../../x86_64-linux-gnu -L/usr/lib/gcc/x86_64-linux-gnu/13/../../../../lib -L/lib/x86_64-linux-gnu -L/lib/../lib -L/usr/lib/x86_64-linux-gnu -L/usr/lib/../lib -L/usr/lib/gcc/x86_64-linux-gnu/13/../../.. CMakeFiles/cmTC_07f7d.dir/CMakeCXXCompilerABI.cpp.o -lstdc++ -lm -lgcc_s -lgcc -lc -lgcc_s -lgcc /usr/lib/gcc/x86_64-linux-gnu/13/crtendS.o /usr/lib/gcc/x86_64-linux-gnu/13/../../../x86_64-linux-gnu/crtn.o]
          arg [/usr/libexec/gcc/x86_64-linux-gnu/13/collect2] ==> ignore
          arg [-plugin] ==> ignore
          arg [/usr/libexec/gcc/x86_64-linux-gnu/13/liblto_plugin.so] ==> ignore
          arg [-plugin-opt=/usr/libexec/gcc/x86_64-linux-gnu/13/lto-wrapper] ==> ignore
          arg [-plugin-opt=-fresolution=/tmp/ccXTKFH9.res] ==> ignore
          arg [-plugin-opt=-pass-through=-lgcc_s] ==> ignore
          arg [-plugin-opt=-pass-through=-lgcc] ==> ignore
          arg [-plugin-opt=-pass-through=-lc] ==> ignore
          arg [-plugin-opt=-pass-through=-lgcc_s] ==> ignore
          arg [-plugin-opt=-pass-through=-lgcc] ==> ignore
          arg [--build-id] ==> ignore
          arg [--eh-frame-hdr] ==> ignore
          arg [-m] ==> ignore
          arg [elf_x86_64] ==> ignore
          arg [--hash-style=gnu] ==> ignore
          arg [--as-needed] ==> ignore
          arg [-dynamic-linker] ==> ignore
          arg [/lib64/ld-linux-x86-64.so.2] ==> ignore
          arg [-pie] ==> ignore
          arg [-znow] ==> ignore
          arg [-zrelro] ==> ignore
          arg [-o] ==> ignore
          arg [cmTC_07f7d] ==> ignore
          arg [/usr/lib/gcc/x86_64-linux-gnu/13/../../../x86_64-linux-gnu/Scrt1.o] ==> obj [/usr/lib/gcc/x86_64-linux-gnu/13/../../../x86_64-linux-gnu/Scrt1.o]
          arg [/usr/lib/gcc/x86_64-linux-gnu/13/../../../x86_64-linux-gnu/crti.o] ==> obj [/usr/lib/gcc/x86_64-linux-gnu/13/../../../x86_64-linux-gnu/crti.o]
          arg [/usr/lib/gcc/x86_64-linux-gnu/13/crtbeginS.o] ==> obj [/usr/lib/gcc/x86_64-linux-gnu/13/crtbeginS.o]
          arg [-L/usr/lib/gcc/x86_64-linux-gnu/13] ==> dir [/usr/lib/gcc/x86_64-linux-gnu/13]
          arg [-L/usr/lib/gcc/x86_64-linux-gnu/13/../../../x86_64-linux-gnu] ==> dir [/usr/lib/gcc/x86_64-linux-gnu/13/../../../x86_64-linux-gnu]
          arg [-L/usr/lib/gcc/x86_64-linux-gnu/13/../../../../lib] ==> dir [/usr/lib/gcc/x86_64-linux-gnu/13/../../../../lib]
          arg [-L/lib/x86_64-linux-gnu] ==> dir [/lib/x86_64-linux-gnu]
          arg [-L/lib/../lib] ==> dir [/lib/../lib]
          arg [-L/usr/lib/x86_64-linux-gnu] ==> dir [/usr/lib/x86_64-linux-gnu]
          arg [-L/usr/lib/../lib] ==> dir [/usr/lib/../lib]
          arg [-L/usr/lib/gcc/x86_64-linux-gnu/13/../../..] ==> dir [/usr/lib/gcc/x86_64-linux-gnu/13/../../..]
          arg [CMakeFiles/cmTC_07f7d.dir/CMakeCXXCompilerABI.cpp.o] ==> ignore
          arg [-lstdc++] ==> lib [stdc++]
          arg [-lm] ==> lib [m]
          arg [-lgcc_s] ==> lib [gcc_s]
          arg [-lgcc] ==> lib [gcc]
          arg [-lc] ==> lib [c]
          arg [-lgcc_s] ==> lib [gcc_s]
          arg [-lgcc] ==> lib [gcc]
          arg [/usr/lib/gcc/x86_64-linux-gnu/13/crtendS.o] ==> obj [/usr/lib/gcc/x86_64-linux-gnu/13/crtendS.o]
          arg [/usr/lib/gcc/x86_64-linux-gnu/13/../../../x86_64-linux-gnu/crtn.o] ==> obj [/usr/lib/gcc/x86_64-linux-gnu/13/../../../x86_64-linux-gnu/crtn.o]
        collapse obj [/usr/lib/gcc/x86_64-linux-gnu/13/../../../x86_64-linux-gnu/Scrt1.o] ==> [/usr/lib/x86_64-linux-gnu/Scrt1.o]
        collapse obj [/usr/lib/gcc/x86_64-linux-gnu/13/../../../x86_64-linux-gnu/crti.o] ==> [/usr/lib/x86_64-linux-gnu/crti.o]
        collapse obj [/usr/lib/gcc/x86_64-linux-gnu/13/../../../x86_64-linux-gnu/crtn.o] ==> [/usr/lib/x86_64-linux-gnu/crtn.o]
        collapse library dir [/usr/lib/gcc/x86_64-linux-gnu/13] ==> [/usr/lib/gcc/x86_64-linux-gnu/13]
        collapse library dir [/usr/lib/gcc/x86_64-linux-gnu/13/../../../x86_64-linux-gnu] ==> [/usr/lib/x86_64-linux-gnu]
        collapse library dir [/usr/lib/gcc/x86_64-linux-gnu/13/../../../../lib] ==> [/usr/lib]
        collapse library dir [/lib/x86_64-linux-gnu] ==> [/lib/x86_64-linux-gnu]
        collapse library dir [/lib/../lib] ==> [/lib]
        collapse library dir [/usr/lib/x86_64-linux-gnu] ==> [/usr/lib/x86_64-linux-gnu]
        collapse library dir [/usr/lib/../lib] ==> [/usr/lib]
        collapse library dir [/usr/lib/gcc/x86_64-linux-gnu/13/../../..] ==> [/usr/lib]
        implicit libs: [stdc++;m;gcc_s;gcc;c;gcc_s;gcc]
        implicit objs: [/usr/lib/x86_64-linux-gnu/Scrt1.o;/usr/lib/x86_64-linux-gnu/crti.o;/usr/lib/gcc/x86_64-linux-gnu/13/crtbeginS.o;/usr/lib/gcc/x86_64-linux-gnu/13/crtendS.o;/usr/lib/x86_64-linux-gnu/crtn.o]
        implicit dirs: [/usr/lib/gcc/x86_64-linux-gnu/13;/usr/lib/x86_64-linux-gnu;/usr/lib;/lib/x86_64-linux-gnu;/lib]
        implicit fwks: []
      
      
  -
    kind: "try_compile-v1"
    backtrace:
      - "/usr/lib/x86_64-linux-gnu/cmake/pcl/Modules/FindOpenMP.cmake:206 (try_compile)"
      - "/usr/lib/x86_64-linux-gnu/cmake/pcl/Modules/FindOpenMP.cmake:451 (_OPENMP_GET_FLAGS)"
      - "/usr/lib/x86_64-linux-gnu/cmake/pcl/PCLConfig.cmake:336 (find_package)"
      - "/usr/lib/x86_64-linux-gnu/cmake/pcl/PCLConfig.cmake:567 (find_external_library)"
      - "/opt/ros/jazzy/share/pcl_conversions/cmake/ament_cmake_export_dependencies-extras.cmake:21 (find_package)"
      - "/opt/ros/jazzy/share/pcl_conversions/cmake/pcl_conversionsConfig.cmake:41 (include)"
      - "/opt/ros/jazzy/share/pcl_ros/cmake/ament_cmake_export_dependencies-extras.cmake:21 (find_package)"
      - "/opt/ros/jazzy/share/pcl_ros/cmake/pcl_rosConfig.cmake:41 (include)"
      - "CMakeLists.txt:19 (find_package)"
    directories:
      source: "/home/student31/leo_ws/build/leo_lidar_sim/CMakeFiles/CMakeTmp"
      binary: "/home/student31/leo_ws/build/leo_lidar_sim/CMakeFiles/CMakeTmp"
    cmakeVariables:
      CMAKE_C_FLAGS: ""
      CMAKE_C_FLAGS_DEBUG: "-g"
      CMAKE_EXE_LINKER_FLAGS: ""
      CMAKE_MODULE_PATH: "/opt/ros/jazzy/share/fastrtps_cmake_module/cmake/Modules;/usr/lib/x86_64-linux-gnu/cmake/pcl/Modules"
    buildResult:
      variable: "OpenMP_COMPILE_RESULT_C_fopenmp"
      cached: true
      stdout: |
        Change Dir: '/home/student31/leo_ws/build/leo_lidar_sim/CMakeFiles/CMakeTmp'
        
        Run Build Command(s): /usr/bin/cmake -E env VERBOSE=1 /usr/bin/gmake -f Makefile cmTC_4ef0a/fast
        /usr/bin/gmake  -f CMakeFiles/cmTC_4ef0a.dir/build.make CMakeFiles/cmTC_4ef0a.dir/build
        gmake[1]: Entering directory '/home/student31/leo_ws/build/leo_lidar_sim/CMakeFiles/CMakeTmp'
        Building C object CMakeFiles/cmTC_4ef0a.dir/OpenMPTryFlag.c.o
        /usr/bin/cc   -fopenmp -v -o CMakeFiles/cmTC_4ef0a.dir/OpenMPTryFlag.c.o -c /home/student31/leo_ws/build/leo_lidar_sim/CMakeFiles/FindOpenMP/OpenMPTryFlag.c
        Using built-in specs.
        COLLECT_GCC=/usr/bin/cc
        OFFLOAD_TARGET_NAMES=nvptx-none:amdgcn-amdhsa
        OFFLOAD_TARGET_DEFAULT=1
        Target: x86_64-linux-gnu
        Configured with: ../src/configure -v --with-pkgversion='Ubuntu 13.3.0-6ubuntu2~24.04.1' --with-bugurl=file:///usr/share/doc/gcc-13/README.Bugs --enable-languages=c,ada,c++,go,d,fortran,objc,obj-c++,m2 --prefix=/usr --with-gcc-major-version-only --program-suffix=-13 --program-prefix=x86_64-linux-gnu- --enable-shared --enable-linker-build-id --libexecdir=/usr/libexec --without-included-gettext --enable-threads=posix --libdir=/usr/lib --enable-nls --enable-bootstrap --enable-clocale=gnu --enable-libstdcxx-debug --enable-libstdcxx-time=yes --with-default-libstdcxx-abi=new --enable-libstdcxx-backtrace --enable-gnu-unique-object --disable-vtable-verify --enable-plugin --enable-default-pie --with-system-zlib --enable-libphobos-checking=release --with-target-system-zlib=auto --enable-objc-gc=auto --enable-multiarch --disable-werror --enable-cet --with-arch-32=i686 --with-abi=m64 --with-multilib-list=m32,m64,mx32 --enable-multilib --with-tune=generic --enable-offload-targets=nvptx-none=/build/gcc-13-EldibY/gcc-13-13.3.0/debian/tmp-nvptx/usr,amdgcn-amdhsa=/build/gcc-13-EldibY/gcc-13-13.3.0/debian/tmp-gcn/usr --enable-offload-defaulted --without-cuda-driver --enable-checking=release --build=x86_64-linux-gnu --host=x86_64-linux-gnu --target=x86_64-linux-gnu --with-build-config=bootstrap-lto-lean --enable-link-serialization=2
        Thread model: posix
        Supported LTO compression algorithms: zlib zstd
        gcc version 13.3.0 (Ubuntu 13.3.0-6ubuntu2~24.04.1) 
        COLLECT_GCC_OPTIONS='-fopenmp' '-v' '-o' 'CMakeFiles/cmTC_4ef0a.dir/OpenMPTryFlag.c.o' '-c' '-mtune=generic' '-march=x86-64' '-pthread' '-dumpdir' 'CMakeFiles/cmTC_4ef0a.dir/'
         /usr/libexec/gcc/x86_64-linux-gnu/13/cc1 -quiet -v -imultiarch x86_64-linux-gnu -D_REENTRANT /home/student31/leo_ws/build/leo_lidar_sim/CMakeFiles/FindOpenMP/OpenMPTryFlag.c -quiet -dumpdir CMakeFiles/cmTC_4ef0a.dir/ -dumpbase OpenMPTryFlag.c.c -dumpbase-ext .c -mtune=generic -march=x86-64 -version -fopenmp -fasynchronous-unwind-tables -fstack-protector-strong -Wformat -Wformat-security -fstack-clash-protection -fcf-protection -o /tmp/ccb1GByZ.s
        GNU C17 (Ubuntu 13.3.0-6ubuntu2~24.04.1) version 13.3.0 (x86_64-linux-gnu)
        	compiled by GNU C version 13.3.0, GMP version 6.3.0, MPFR version 4.2.1, MPC version 1.3.1, isl version isl-0.26-GMP
        
        GGC heuristics: --param ggc-min-expand=100 --param ggc-min-heapsize=131072
        ignoring nonexistent directory "/usr/local/include/x86_64-linux-gnu"
        ignoring nonexistent directory "/usr/lib/gcc/x86_64-linux-gnu/13/include-fixed/x86_64-linux-gnu"
        ignoring nonexistent directory "/usr/lib/gcc/x86_64-linux-gnu/13/include-fixed"
        ignoring nonexistent directory "/usr/lib/gcc/x86_64-linux-gnu/13/../../../../x86_64-linux-gnu/include"
        #include "..." search starts here:
        #include <...> search starts here:
         /usr/lib/gcc/x86_64-linux-gnu/13/include
         /usr/local/include
         /usr/include/x86_64-linux-gnu
         /usr/include
        End of search list.
        Compiler executable checksum: b220a7f1a1f69970d969d254ad9ec166
        COLLECT_GCC_OPTIONS='-fopenmp' '-v' '-o' 'CMakeFiles/cmTC_4ef0a.dir/OpenMPTryFlag.c.o' '-c' '-mtune=generic' '-march=x86-64' '-pthread' '-dumpdir' 'CMakeFiles/cmTC_4ef0a.dir/'
         as -v --64 -o CMakeFiles/cmTC_4ef0a.dir/OpenMPTryFlag.c.o /tmp/ccb1GByZ.s
        GNU assembler version 2.42 (x86_64-linux-gnu) using BFD version (GNU Binutils for Ubuntu) 2.42
        COMPILER_PATH=/usr/libexec/gcc/x86_64-linux-gnu/13/:/usr/libexec/gcc/x86_64-linux-gnu/13/:/usr/libexec/gcc/x86_64-linux-gnu/:/usr/lib/gcc/x86_64-linux-gnu/13/:/usr/lib/gcc/x86_64-linux-gnu/
        LIBRARY_PATH=/usr/lib/gcc/x86_64-linux-gnu/13/:/usr/lib/gcc/x86_64-linux-gnu/13/../../../x86_64-linux-gnu/:/usr/lib/gcc/x86_64-linux-gnu/13/../../../../lib/:/lib/x86_64-linux-gnu/:/lib/../lib/:/usr/lib/x86_64-linux-gnu/:/usr/lib/../lib/:/usr/lib/gcc/x86_64-linux-gnu/13/../../../:/lib/:/usr/lib/
        COLLECT_GCC_OPTIONS='-fopenmp' '-v' '-o' 'CMakeFiles/cmTC_4ef0a.dir/OpenMPTryFlag.c.o' '-c' '-mtune=generic' '-march=x86-64' '-pthread' '-dumpdir' 'CMakeFiles/cmTC_4ef0a.dir/OpenMPTryFlag.c.'
        Linking C executable cmTC_4ef0a
        /usr/bin/cmake -E cmake_link_script CMakeFiles/cmTC_4ef0a.dir/link.txt --verbose=1
        /usr/bin/cc  -fopenmp -v CMakeFiles/cmTC_4ef0a.dir/OpenMPTryFlag.c.o -o cmTC_4ef0a  -v 
        Using built-in specs.
        COLLECT_GCC=/usr/bin/cc
        COLLECT_LTO_WRAPPER=/usr/libexec/gcc/x86_64-linux-gnu/13/lto-wrapper
        OFFLOAD_TARGET_NAMES=nvptx-none:amdgcn-amdhsa
        OFFLOAD_TARGET_DEFAULT=1
        Target: x86_64-linux-gnu
        Configured with: ../src/configure -v --with-pkgversion='Ubuntu 13.3.0-6ubuntu2~24.04.1' --with-bugurl=file:///usr/share/doc/gcc-13/README.Bugs --enable-languages=c,ada,c++,go,d,fortran,objc,obj-c++,m2 --prefix=/usr --with-gcc-major-version-only --program-suffix=-13 --program-prefix=x86_64-linux-gnu- --enable-shared --enable-linker-build-id --libexecdir=/usr/libexec --without-included-gettext --enable-threads=posix --libdir=/usr/lib --enable-nls --enable-bootstrap --enable-clocale=gnu --enable-libstdcxx-debug --enable-libstdcxx-time=yes --with-default-libstdcxx-abi=new --enable-libstdcxx-backtrace --enable-gnu-unique-object --disable-vtable-verify --enable-plugin --enable-default-pie --with-system-zlib --enable-libphobos-checking=release --with-target-system-zlib=auto --enable-objc-gc=auto --enable-multiarch --disable-werror --enable-cet --with-arch-32=i686 --with-abi=m64 --with-multilib-list=m32,m64,mx32 --enable-multilib --with-tune=generic --enable-offload-targets=nvptx-none=/build/gcc-13-EldibY/gcc-13-13.3.0/debian/tmp-nvptx/usr,amdgcn-amdhsa=/build/gcc-13-EldibY/gcc-13-13.3.0/debian/tmp-gcn/usr --enable-offload-defaulted --without-cuda-driver --enable-checking=release --build=x86_64-linux-gnu --host=x86_64-linux-gnu --target=x86_64-linux-gnu --with-build-config=bootstrap-lto-lean --enable-link-serialization=2
        Thread model: posix
        Supported LTO compression algorithms: zlib zstd
        gcc version 13.3.0 (Ubuntu 13.3.0-6ubuntu2~24.04.1) 
        COMPILER_PATH=/usr/libexec/gcc/x86_64-linux-gnu/13/:/usr/libexec/gcc/x86_64-linux-gnu/13/:/usr/libexec/gcc/x86_64-linux-gnu/:/usr/lib/gcc/x86_64-linux-gnu/13/:/usr/lib/gcc/x86_64-linux-gnu/
        LIBRARY_PATH=/usr/lib/gcc/x86_64-linux-gnu/13/:/usr/lib/gcc/x86_64-linux-gnu/13/../../../x86_64-linux-gnu/:/usr/lib/gcc/x86_64-linux-gnu/13/../../../../lib/:/lib/x86_64-linux-gnu/:/lib/../lib/:/usr/lib/x86_64-linux-gnu/:/usr/lib/../lib/:/usr/lib/gcc/x86_64-linux-gnu/13/../../../:/lib/:/usr/lib/
        Reading specs from /usr/lib/gcc/x86_64-linux-gnu/13/libgomp.spec
        COLLECT_GCC_OPTIONS='-fopenmp' '-v' '-o' 'cmTC_4ef0a' '-v' '-mtune=generic' '-march=x86-64' '-pthread' '-dumpdir' 'cmTC_4ef0a.'
         /usr/libexec/gcc/x86_64-linux-gnu/13/collect2 -plugin /usr/libexec/gcc/x86_64-linux-gnu/13/liblto_plugin.so -plugin-opt=/usr/libexec/gcc/x86_64-linux-gnu/13/lto-wrapper -plugin-opt=-fresolution=/tmp/ccAIvHP0.res -plugin-opt=-pass-through=-lgcc -plugin-opt=-pass-through=-lgcc_s -plugin-opt=-pass-through=-lpthread -plugin-opt=-pass-through=-lc -plugin-opt=-pass-through=-lgcc -plugin-opt=-pass-through=-lgcc_s --build-id --eh-frame-hdr -m elf_x86_64 --hash-style=gnu --as-needed -dynamic-linker /lib64/ld-linux-x86-64.so.2 -pie -z now -z relro -o cmTC_4ef0a /usr/lib/gcc/x86_64-linux-gnu/13/../../../x86_64-linux-gnu/Scrt1.o /usr/lib/gcc/x86_64-linux-gnu/13/../../../x86_64-linux-gnu/crti.o /usr/lib/gcc/x86_64-linux-gnu/13/crtbeginS.o /usr/lib/gcc/x86_64-linux-gnu/13/crtoffloadbegin.o -L/usr/lib/gcc/x86_64-linux-gnu/13 -L/usr/lib/gcc/x86_64-linux-gnu/13/../../../x86_64-linux-gnu -L/usr/lib/gcc/x86_64-linux-gnu/13/../../../../lib -L/lib/x86_64-linux-gnu -L/lib/../lib -L/usr/lib/x86_64-linux-gnu -L/usr/lib/../lib -L/usr/lib/gcc/x86_64-linux-gnu/13/../../.. CMakeFiles/cmTC_4ef0a.dir/OpenMPTryFlag.c.o -lgomp -lgcc --push-state --as-needed -lgcc_s --pop-state -lpthread -lc -lgcc --push-state --as-needed -lgcc_s --pop-state /usr/lib/gcc/x86_64-linux-gnu/13/crtendS.o /usr/lib/gcc/x86_64-linux-gnu/13/../../../x86_64-linux-gnu/crtn.o /usr/lib/gcc/x86_64-linux-gnu/13/crtoffloadend.o
        COLLECT_GCC_OPTIONS='-fopenmp' '-v' '-o' 'cmTC_4ef0a' '-v' '-mtune=generic' '-march=x86-64' '-pthread' '-dumpdir' 'cmTC_4ef0a.'
        gmake[1]: Leaving directory '/home/student31/leo_ws/build/leo_lidar_sim/CMakeFiles/CMakeTmp'
        
      exitCode: 0
  -
    kind: "try_compile-v1"
    backtrace:
      - "/usr/lib/x86_64-linux-gnu/cmake/pcl/Modules/FindOpenMP.cmake:206 (try_compile)"
      - "/usr/lib/x86_64-linux-gnu/cmake/pcl/Modules/FindOpenMP.cmake:451 (_OPENMP_GET_FLAGS)"
      - "/usr/lib/x86_64-linux-gnu/cmake/pcl/PCLConfig.cmake:336 (find_package)"
      - "/usr/lib/x86_64-linux-gnu/cmake/pcl/PCLConfig.cmake:567 (find_external_library)"
      - "/opt/ros/jazzy/share/pcl_conversions/cmake/ament_cmake_export_dependencies-extras.cmake:21 (find_package)"
      - "/opt/ros/jazzy/share/pcl_conversions/cmake/pcl_conversionsConfig.cmake:41 (include)"
      - "/opt/ros/jazzy/share/pcl_ros/cmake/ament_cmake_export_dependencies-extras.cmake:21 (find_package)"
      - "/opt/ros/jazzy/share/pcl_ros/cmake/pcl_rosConfig.cmake:41 (include)"
      - "CMakeLists.txt:19 (find_package)"
    directories:
      source: "/home/student31/leo_ws/build/leo_lidar_sim/CMakeFiles/CMakeTmp"
      binary: "/home/student31/leo_ws/build/leo_lidar_sim/CMakeFiles/CMakeTmp"
    cmakeVariables:
      CMAKE_CXX_FLAGS: ""
      CMAKE_CXX_FLAGS_DEBUG: "-g"
      CMAKE_EXE_LINKER_FLAGS: ""
      CMAKE_MODULE_PATH: "/opt/ros/jazzy/share/fastrtps_cmake_module/cmake/Modules;/usr/lib/x86_64-linux-gnu/cmake/pcl/Modules"
    buildResult:
      variable: "OpenMP_COMPILE_RESULT_CXX_fopenmp"
      cached: true
      stdout: |
        Change Dir: '/home/student31/leo_ws/build/leo_lidar_sim/CMakeFiles/CMakeTmp'
        
        Run Build Command(s): /usr/bin/cmake -E env VERBOSE=1 /usr/bin/gmake -f Makefile cmTC_e3bed/fast
        /usr/bin/gmake  -f CMakeFiles/cmTC_e3bed.dir/build.make CMakeFiles/cmTC_e3bed.dir/build
        gmake[1]: Entering directory '/home/student31/leo_ws/build/leo_lidar_sim/CMakeFiles/CMakeTmp'
        Building CXX object CMakeFiles/cmTC_e3bed.dir/OpenMPTryFlag.cpp.o
        /usr/bin/c++   -fopenmp -v -o CMakeFiles/cmTC_e3bed.dir/OpenMPTryFlag.cpp.o -c /home/student31/leo_ws/build/leo_lidar_sim/CMakeFiles/FindOpenMP/OpenMPTryFlag.cpp
        Using built-in specs.
        COLLECT_GCC=/usr/bin/c++
        OFFLOAD_TARGET_NAMES=nvptx-none:amdgcn-amdhsa
        OFFLOAD_TARGET_DEFAULT=1
        Target: x86_64-linux-gnu
        Configured with: ../src/configure -v --with-pkgversion='Ubuntu 13.3.0-6ubuntu2~24.04.1' --with-bugurl=file:///usr/share/doc/gcc-13/README.Bugs --enable-languages=c,ada,c++,go,d,fortran,objc,obj-c++,m2 --prefix=/usr --with-gcc-major-version-only --program-suffix=-13 --program-prefix=x86_64-linux-gnu- --enable-shared --enable-linker-build-id --libexecdir=/usr/libexec --without-included-gettext --enable-threads=posix --libdir=/usr/lib --enable-nls --enable-bootstrap --enable-clocale=gnu --enable-libstdcxx-debug --enable-libstdcxx-time=yes --with-default-libstdcxx-abi=new --enable-libstdcxx-backtrace --enable-gnu-unique-object --disable-vtable-verify --enable-plugin --enable-default-pie --with-system-zlib --enable-libphobos-checking=release --with-target-system-zlib=auto --enable-objc-gc=auto --enable-multiarch --disable-werror --enable-cet --with-arch-32=i686 --with-abi=m64 --with-multilib-list=m32,m64,mx32 --enable-multilib --with-tune=generic --enable-offload-targets=nvptx-none=/build/gcc-13-EldibY/gcc-13-13.3.0/debian/tmp-nvptx/usr,amdgcn-amdhsa=/build/gcc-13-EldibY/gcc-13-13.3.0/debian/tmp-gcn/usr --enable-offload-defaulted --without-cuda-driver --enable-checking=release --build=x86_64-linux-gnu --host=x86_64-linux-gnu --target=x86_64-linux-gnu --with-build-config=bootstrap-lto-lean --enable-link-serialization=2
        Thread model: posix
        Supported LTO compression algorithms: zlib zstd
        gcc version 13.3.0 (Ubuntu 13.3.0-6ubuntu2~24.04.1) 
        COLLECT_GCC_OPTIONS='-fopenmp' '-v' '-o' 'CMakeFiles/cmTC_e3bed.dir/OpenMPTryFlag.cpp.o' '-c' '-shared-libgcc' '-mtune=generic' '-march=x86-64' '-pthread' '-dumpdir' 'CMakeFiles/cmTC_e3bed.dir/'
         /usr/libexec/gcc/x86_64-linux-gnu/13/cc1plus -quiet -v -imultiarch x86_64-linux-gnu -D_GNU_SOURCE -D_REENTRANT /home/student31/leo_ws/build/leo_lidar_sim/CMakeFiles/FindOpenMP/OpenMPTryFlag.cpp -quiet -dumpdir CMakeFiles/cmTC_e3bed.dir/ -dumpbase OpenMPTryFlag.cpp.cpp -dumpbase-ext .cpp -mtune=generic -march=x86-64 -version -fopenmp -fasynchronous-unwind-tables -fstack-protector-strong -Wformat -Wformat-security -fstack-clash-protection -fcf-protection -o /tmp/cc6UplPh.s
        GNU C++17 (Ubuntu 13.3.0-6ubuntu2~24.04.1) version 13.3.0 (x86_64-linux-gnu)
        	compiled by GNU C version 13.3.0, GMP version 6.3.0, MPFR version 4.2.1, MPC version 1.3.1, isl version isl-0.26-GMP
        
        GGC heuristics: --param ggc-min-expand=100 --param ggc-min-heapsize=131072
        ignoring duplicate directory "/usr/include/x86_64-linux-gnu/c++/13"
        ignoring nonexistent directory "/usr/local/include/x86_64-linux-gnu"
        ignoring nonexistent directory "/usr/lib/gcc/x86_64-linux-gnu/13/include-fixed/x86_64-linux-gnu"
        ignoring nonexistent directory "/usr/lib/gcc/x86_64-linux-gnu/13/include-fixed"
        ignoring nonexistent directory "/usr/lib/gcc/x86_64-linux-gnu/13/../../../../x86_64-linux-gnu/include"
        #include "..." search starts here:
        #include <...> search starts here:
         /usr/include/c++/13
         /usr/include/x86_64-linux-gnu/c++/13
         /usr/include/c++/13/backward
         /usr/lib/gcc/x86_64-linux-gnu/13/include
         /usr/local/include
         /usr/include/x86_64-linux-gnu
         /usr/include
        End of search list.
        Compiler executable checksum: 7896445e4990772fdae9dc0659a99266
        COLLECT_GCC_OPTIONS='-fopenmp' '-v' '-o' 'CMakeFiles/cmTC_e3bed.dir/OpenMPTryFlag.cpp.o' '-c' '-shared-libgcc' '-mtune=generic' '-march=x86-64' '-pthread' '-dumpdir' 'CMakeFiles/cmTC_e3bed.dir/'
         as -v --64 -o CMakeFiles/cmTC_e3bed.dir/OpenMPTryFlag.cpp.o /tmp/cc6UplPh.s
        GNU assembler version 2.42 (x86_64-linux-gnu) using BFD version (GNU Binutils for Ubuntu) 2.42
        COMPILER_PATH=/usr/libexec/gcc/x86_64-linux-gnu/13/:/usr/libexec/gcc/x86_64-linux-gnu/13/:/usr/libexec/gcc/x86_64-linux-gnu/:/usr/lib/gcc/x86_64-linux-gnu/13/:/usr/lib/gcc/x86_64-linux-gnu/
        LIBRARY_PATH=/usr/lib/gcc/x86_64-linux-gnu/13/:/usr/lib/gcc/x86_64-linux-gnu/13/../../../x86_64-linux-gnu/:/usr/lib/gcc/x86_64-linux-gnu/13/../../../../lib/:/lib/x86_64-linux-gnu/:/lib/../lib/:/usr/lib/x86_64-linux-gnu/:/usr/lib/../lib/:/usr/lib/gcc/x86_64-linux-gnu/13/../../../:/lib/:/usr/lib/
        COLLECT_GCC_OPTIONS='-fopenmp' '-v' '-o' 'CMakeFiles/cmTC_e3bed.dir/OpenMPTryFlag.cpp.o' '-c' '-shared-libgcc' '-mtune=generic' '-march=x86-64' '-pthread' '-dumpdir' 'CMakeFiles/cmTC_e3bed.dir/OpenMPTryFlag.cpp.'
        Linking CXX executable cmTC_e3bed
        /usr/bin/cmake -E cmake_link_script CMakeFiles/cmTC_e3bed.dir/link.txt --verbose=1
        /usr/bin/c++  -fopenmp -v CMakeFiles/cmTC_e3bed.dir/OpenMPTryFlag.cpp.o -o cmTC_e3bed  -v 
        Using built-in specs.
        COLLECT_GCC=/usr/bin/c++
        COLLECT_LTO_WRAPPER=/usr/libexec/gcc/x86_64-linux-gnu/13/lto-wrapper
        OFFLOAD_TARGET_NAMES=nvptx-none:amdgcn-amdhsa
        OFFLOAD_TARGET_DEFAULT=1
        Target: x86_64-linux-gnu
        Configured with: ../src/configure -v --with-pkgversion='Ubuntu 13.3.0-6ubuntu2~24.04.1' --with-bugurl=file:///usr/share/doc/gcc-13/README.Bugs --enable-languages=c,ada,c++,go,d,fortran,objc,obj-c++,m2 --prefix=/usr --with-gcc-major-version-only --program-suffix=-13 --program-prefix=x86_64-linux-gnu- --enable-shared --enable-linker-build-id --libexecdir=/usr/libexec --without-included-gettext --enable-threads=posix --libdir=/usr/lib --enable-nls --enable-bootstrap --enable-clocale=gnu --enable-libstdcxx-debug --enable-libstdcxx-time=yes --with-default-libstdcxx-abi=new --enable-libstdcxx-backtrace --enable-gnu-unique-object --disable-vtable-verify --enable-plugin --enable-default-pie --with-system-zlib --enable-libphobos-checking=release --with-target-system-zlib=auto --enable-objc-gc=auto --enable-multiarch --disable-werror --enable-cet --with-arch-32=i686 --with-abi=m64 --with-multilib-list=m32,m64,mx32 --enable-multilib --with-tune=generic --enable-offload-targets=nvptx-none=/build/gcc-13-EldibY/gcc-13-13.3.0/debian/tmp-nvptx/usr,amdgcn-amdhsa=/build/gcc-13-EldibY/gcc-13-13.3.0/debian/tmp-gcn/usr --enable-offload-defaulted --without-cuda-driver --enable-checking=release --build=x86_64-linux-gnu --host=x86_64-linux-gnu --target=x86_64-linux-gnu --with-build-config=bootstrap-lto-lean --enable-link-serialization=2
        Thread model: posix
        Supported LTO compression algorithms: zlib zstd
        gcc version 13.3.0 (Ubuntu 13.3.0-6ubuntu2~24.04.1) 
        COMPILER_PATH=/usr/libexec/gcc/x86_64-linux-gnu/13/:/usr/libexec/gcc/x86_64-linux-gnu/13/:/usr/libexec/gcc/x86_64-linux-gnu/:/usr/lib/gcc/x86_64-linux-gnu/13/:/usr/lib/gcc/x86_64-linux-gnu/
        LIBRARY_PATH=/usr/lib/gcc/x86_64-linux-gnu/13/:/usr/lib/gcc/x86_64-linux-gnu/13/../../../x86_64-linux-gnu/:/usr/lib/gcc/x86_64-linux-gnu/13/../../../../lib/:/lib/x86_64-linux-gnu/:/lib/../lib/:/usr/lib/x86_64-linux-gnu/:/usr/lib/../lib/:/usr/lib/gcc/x86_64-linux-gnu/13/../../../:/lib/:/usr/lib/
        Reading specs from /usr/lib/gcc/x86_64-linux-gnu/13/libgomp.spec
        COLLECT_GCC_OPTIONS='-fopenmp' '-v' '-o' 'cmTC_e3bed' '-v' '-shared-libgcc' '-mtune=generic' '-march=x86-64' '-pthread' '-dumpdir' 'cmTC_e3bed.'
         /usr/libexec/gcc/x86_64-linux-gnu/13/collect2 -plugin /usr/libexec/gcc/x86_64-linux-gnu/13/liblto_plugin.so -plugin-opt=/usr/libexec/gcc/x86_64-linux-gnu/13/lto-wrapper -plugin-opt=-fresolution=/tmp/ccEkTfcw.res -plugin-opt=-pass-through=-lgcc_s -plugin-opt=-pass-through=-lgcc -plugin-opt=-pass-through=-lpthread -plugin-opt=-pass-through=-lc -plugin-opt=-pass-through=-lgcc_s -plugin-opt=-pass-through=-lgcc --build-id --eh-frame-hdr -m elf_x86_64 --hash-style=gnu --as-needed -dynamic-linker /lib64/ld-linux-x86-64.so.2 -pie -z now -z relro -o cmTC_e3bed /usr/lib/gcc/x86_64-linux-gnu/13/../../../x86_64-linux-gnu/Scrt1.o /usr/lib/gcc/x86_64-linux-gnu/13/../../../x86_64-linux-gnu/crti.o /usr/lib/gcc/x86_64-linux-gnu/13/crtbeginS.o /usr/lib/gcc/x86_64-linux-gnu/13/crtoffloadbegin.o -L/usr/lib/gcc/x86_64-linux-gnu/13 -L/usr/lib/gcc/x86_64-linux-gnu/13/../../../x86_64-linux-gnu -L/usr/lib/gcc/x86_64-linux-gnu/13/../../../../lib -L/lib/x86_64-linux-gnu -L/lib/../lib -L/usr/lib/x86_64-linux-gnu -L/usr/lib/../lib -L/usr/lib/gcc/x86_64-linux-gnu/13/../../.. CMakeFiles/cmTC_e3bed.dir/OpenMPTryFlag.cpp.o -lstdc++ -lm -lgomp -lgcc_s -lgcc -lpthread -lc -lgcc_s -lgcc /usr/lib/gcc/x86_64-linux-gnu/13/crtendS.o /usr/lib/gcc/x86_64-linux-gnu/13/../../../x86_64-linux-gnu/crtn.o /usr/lib/gcc/x86_64-linux-gnu/13/crtoffloadend.o
        COLLECT_GCC_OPTIONS='-fopenmp' '-v' '-o' 'cmTC_e3bed' '-v' '-shared-libgcc' '-mtune=generic' '-march=x86-64' '-pthread' '-dumpdir' 'cmTC_e3bed.'
        gmake[1]: Leaving directory '/home/student31/leo_ws/build/leo_lidar_sim/CMakeFiles/CMakeTmp'
        
      exitCode: 0
  -
    kind: "try_compile-v1"
    backtrace:
      - "/usr/lib/x86_64-linux-gnu/cmake/pcl/Modules/FindOpenMP.cmake:386 (try_compile)"
      - "/usr/lib/x86_64-linux-gnu/cmake/pcl/Modules/FindOpenMP.cmake:514 (_OPENMP_GET_SPEC_DATE)"
      - "/usr/lib/x86_64-linux-gnu/cmake/pcl/PCLConfig.cmake:336 (find_package)"
      - "/usr/lib/x86_64-linux-gnu/cmake/pcl/PCLConfig.cmake:567 (find_external_library)"
      - "/opt/ros/jazzy/share/pcl_conversions/cmake/ament_cmake_export_dependencies-extras.cmake:21 (find_package)"
      - "/opt/ros/jazzy/share/pcl_conversions/cmake/pcl_conversionsConfig.cmake:41 (include)"
      - "/opt/ros/jazzy/share/pcl_ros/cmake/ament_cmake_export_dependencies-extras.cmake:21 (find_package)"
      - "/opt/ros/jazzy/share/pcl_ros/cmake/pcl_rosConfig.cmake:41 (include)"
      - "CMakeLists.txt:19 (find_package)"
    directories:
      source: "/home/student31/leo_ws/build/leo_lidar_sim/CMakeFiles/CMakeTmp"
      binary: "/home/student31/leo_ws/build/leo_lidar_sim/CMakeFiles/CMakeTmp"
    cmakeVariables:
      CMAKE_CXX_FLAGS: ""
      CMAKE_CXX_FLAGS_DEBUG: "-g"
      CMAKE_EXE_LINKER_FLAGS: ""
      CMAKE_MODULE_PATH: "/opt/ros/jazzy/share/fastrtps_cmake_module/cmake/Modules;/usr/lib/x86_64-linux-gnu/cmake/pcl/Modules"
    buildResult:
      variable: "OpenMP_SPECTEST_CXX_"
      cached: true
      stdout: |
        Change Dir: '/home/student31/leo_ws/build/leo_lidar_sim/CMakeFiles/CMakeTmp'
        
        Run Build Command(s): /usr/bin/cmake -E env VERBOSE=1 /usr/bin/gmake -f Makefile cmTC_cc6b7/fast
        /usr/bin/gmake  -f CMakeFiles/cmTC_cc6b7.dir/build.make CMakeFiles/cmTC_cc6b7.dir/build
        gmake[1]: Entering directory '/home/student31/leo_ws/build/leo_lidar_sim/CMakeFiles/CMakeTmp'
        Building CXX object CMakeFiles/cmTC_cc6b7.dir/OpenMPCheckVersion.cpp.o
        /usr/bin/c++   -fopenmp -o CMakeFiles/cmTC_cc6b7.dir/OpenMPCheckVersion.cpp.o -c /home/student31/leo_ws/build/leo_lidar_sim/CMakeFiles/FindOpenMP/OpenMPCheckVersion.cpp
        Linking CXX executable cmTC_cc6b7
        /usr/bin/cmake -E cmake_link_script CMakeFiles/cmTC_cc6b7.dir/link.txt --verbose=1
        /usr/bin/c++  -fopenmp CMakeFiles/cmTC_cc6b7.dir/OpenMPCheckVersion.cpp.o -o cmTC_cc6b7 
        gmake[1]: Leaving directory '/home/student31/leo_ws/build/leo_lidar_sim/CMakeFiles/CMakeTmp'
        
      exitCode: 0
  -
    kind: "try_compile-v1"
    backtrace:
      - "/usr/share/cmake-3.28/Modules/Internal/CheckSourceCompiles.cmake:101 (try_compile)"
      - "/usr/share/cmake-3.28/Modules/CheckCSourceCompiles.cmake:52 (cmake_check_source_compiles)"
      - "/usr/share/cmake-3.28/Modules/FindThreads.cmake:97 (CHECK_C_SOURCE_COMPILES)"
      - "/usr/share/cmake-3.28/Modules/FindThreads.cmake:163 (_threads_check_libc)"
      - "/usr/lib/x86_64-linux-gnu/cmake/vtk-9.1/VTK-vtk-module-find-packages.cmake:162 (find_package)"
      - "/usr/lib/x86_64-linux-gnu/cmake/vtk-9.1/vtk-config.cmake:150 (include)"
      - "/usr/lib/x86_64-linux-gnu/cmake/pcl/PCLConfig.cmake:275 (find_package)"
      - "/usr/lib/x86_64-linux-gnu/cmake/pcl/PCLConfig.cmake:324 (find_VTK)"
      - "/usr/lib/x86_64-linux-gnu/cmake/pcl/PCLConfig.cmake:567 (find_external_library)"
      - "/opt/ros/jazzy/share/pcl_conversions/cmake/ament_cmake_export_dependencies-extras.cmake:21 (find_package)"
      - "/opt/ros/jazzy/share/pcl_conversions/cmake/pcl_conversionsConfig.cmake:41 (include)"
      - "/opt/ros/jazzy/share/pcl_ros/cmake/ament_cmake_export_dependencies-extras.cmake:21 (find_package)"
      - "/opt/ros/jazzy/share/pcl_ros/cmake/pcl_rosConfig.cmake:41 (include)"
      - "CMakeLists.txt:19 (find_package)"
    directories:
      source: "/home/student31/leo_ws/build/leo_lidar_sim/CMakeFiles/CMakeScratch/TryCompile-vLBTKb"
      binary: "/home/student31/leo_ws/build/leo_lidar_sim/CMakeFiles/CMakeScratch/TryCompile-vLBTKb"
    cmakeVariables:
      CMAKE_C_FLAGS: ""
      CMAKE_C_FLAGS_DEBUG: "-g"
      CMAKE_EXE_LINKER_FLAGS: ""
      CMAKE_MODULE_PATH: "/usr/lib/x86_64-linux-gnu/cmake/vtk-9.1/patches/99;/usr/lib/x86_64-linux-gnu/cmake/vtk-9.1;/opt/ros/jazzy/share/fastrtps_cmake_module/cmake/Modules;/usr/lib/x86_64-linux-gnu/cmake/pcl/Modules"
    buildResult:
      variable: "CMAKE_HAVE_LIBC_PTHREAD"
      cached: true
      stdout: |
        Change Dir: '/home/student31/leo_ws/build/leo_lidar_sim/CMakeFiles/CMakeScratch/TryCompile-vLBTKb'
        
        Run Build Command(s): /usr/bin/cmake -E env VERBOSE=1 /usr/bin/gmake -f Makefile cmTC_991a3/fast
        /usr/bin/gmake  -f CMakeFiles/cmTC_991a3.dir/build.make CMakeFiles/cmTC_991a3.dir/build
        gmake[1]: Entering directory '/home/student31/leo_ws/build/leo_lidar_sim/CMakeFiles/CMakeScratch/TryCompile-vLBTKb'
        Building C object CMakeFiles/cmTC_991a3.dir/src.c.o
        /usr/bin/cc -DCMAKE_HAVE_LIBC_PTHREAD   -o CMakeFiles/cmTC_991a3.dir/src.c.o -c /home/student31/leo_ws/build/leo_lidar_sim/CMakeFiles/CMakeScratch/TryCompile-vLBTKb/src.c
        Linking C executable cmTC_991a3
        /usr/bin/cmake -E cmake_link_script CMakeFiles/cmTC_991a3.dir/link.txt --verbose=1
        /usr/bin/cc CMakeFiles/cmTC_991a3.dir/src.c.o -o cmTC_991a3 
        gmake[1]: Leaving directory '/home/student31/leo_ws/build/leo_lidar_sim/CMakeFiles/CMakeScratch/TryCompile-vLBTKb'
        
      exitCode: 0
  -
    kind: "try_compile-v1"
    backtrace:
      - "/usr/share/cmake-3.28/Modules/FindMPI.cmake:1278 (try_compile)"
      - "/usr/share/cmake-3.28/Modules/FindMPI.cmake:1322 (_MPI_try_staged_settings)"
      - "/usr/share/cmake-3.28/Modules/FindMPI.cmake:1645 (_MPI_check_lang_works)"
      - "/usr/lib/x86_64-linux-gnu/cmake/vtk-9.1/VTK-vtk-module-find-packages.cmake:397 (find_package)"
      - "/usr/lib/x86_64-linux-gnu/cmake/vtk-9.1/vtk-config.cmake:150 (include)"
      - "/usr/lib/x86_64-linux-gnu/cmake/pcl/PCLConfig.cmake:275 (find_package)"
      - "/usr/lib/x86_64-linux-gnu/cmake/pcl/PCLConfig.cmake:324 (find_VTK)"
      - "/usr/lib/x86_64-linux-gnu/cmake/pcl/PCLConfig.cmake:567 (find_external_library)"
      - "/opt/ros/jazzy/share/pcl_conversions/cmake/ament_cmake_export_dependencies-extras.cmake:21 (find_package)"
      - "/opt/ros/jazzy/share/pcl_conversions/cmake/pcl_conversionsConfig.cmake:41 (include)"
      - "/opt/ros/jazzy/share/pcl_ros/cmake/ament_cmake_export_dependencies-extras.cmake:21 (find_package)"
      - "/opt/ros/jazzy/share/pcl_ros/cmake/pcl_rosConfig.cmake:41 (include)"
      - "CMakeLists.txt:19 (find_package)"
    description: "The MPI test test_mpi for C in mode normal"
    directories:
      source: "/home/student31/leo_ws/build/leo_lidar_sim/CMakeFiles/CMakeScratch/TryCompile-NfAfLC"
      binary: "/home/student31/leo_ws/build/leo_lidar_sim/CMakeFiles/CMakeScratch/TryCompile-NfAfLC"
    cmakeVariables:
      CMAKE_C_FLAGS: ""
      CMAKE_C_FLAGS_DEBUG: "-g"
      CMAKE_EXE_LINKER_FLAGS: ""
      CMAKE_MODULE_PATH: "/usr/lib/x86_64-linux-gnu/cmake/vtk-9.1/patches/99;/usr/lib/x86_64-linux-gnu/cmake/vtk-9.1;/opt/ros/jazzy/share/fastrtps_cmake_module/cmake/Modules;/usr/lib/x86_64-linux-gnu/cmake/pcl/Modules"
    buildResult:
      variable: "MPI_RESULT_C_test_mpi_normal"
      cached: true
      stdout: |
        Change Dir: '/home/student31/leo_ws/build/leo_lidar_sim/CMakeFiles/CMakeScratch/TryCompile-NfAfLC'
        
        Run Build Command(s): /usr/bin/cmake -E env VERBOSE=1 /usr/bin/gmake -f Makefile cmTC_7752f/fast
        /usr/bin/gmake  -f CMakeFiles/cmTC_7752f.dir/build.make CMakeFiles/cmTC_7752f.dir/build
        gmake[1]: Entering directory '/home/student31/leo_ws/build/leo_lidar_sim/CMakeFiles/CMakeScratch/TryCompile-NfAfLC'
        Building C object CMakeFiles/cmTC_7752f.dir/test_mpi.c.o
        /usr/bin/cc  -isystem /usr/lib/x86_64-linux-gnu/openmpi/include -isystem /usr/lib/x86_64-linux-gnu/openmpi/include/openmpi  -o CMakeFiles/cmTC_7752f.dir/test_mpi.c.o -c /home/student31/leo_ws/build/leo_lidar_sim/CMakeFiles/CMakeScratch/TryCompile-NfAfLC/test_mpi.c
        Linking C executable cmTC_7752f
        /usr/bin/cmake -E cmake_link_script CMakeFiles/cmTC_7752f.dir/link.txt --verbose=1
        /usr/bin/cc -L/usr/lib/x86_64-linux-gnu/openmpi/lib CMakeFiles/cmTC_7752f.dir/test_mpi.c.o -o cmTC_7752f  /usr/lib/x86_64-linux-gnu/libmpi.so 
        gmake[1]: Leaving directory '/home/student31/leo_ws/build/leo_lidar_sim/CMakeFiles/CMakeScratch/TryCompile-NfAfLC'
        
      exitCode: 0
  -
    kind: "try_compile-v1"
    backtrace:
      - "/usr/share/cmake-3.28/Modules/CheckLibraryExists.cmake:69 (try_compile)"
      - "/usr/share/cmake-3.28/Modules/FindX11.cmake:682 (check_library_exists)"
      - "/usr/lib/x86_64-linux-gnu/cmake/vtk-9.1/VTK-vtk-module-find-packages.cmake:1149 (find_package)"
      - "/usr/lib/x86_64-linux-gnu/cmake/vtk-9.1/vtk-config.cmake:150 (include)"
      - "/usr/lib/x86_64-linux-gnu/cmake/pcl/PCLConfig.cmake:275 (find_package)"
      - "/usr/lib/x86_64-linux-gnu/cmake/pcl/PCLConfig.cmake:324 (find_VTK)"
      - "/usr/lib/x86_64-linux-gnu/cmake/pcl/PCLConfig.cmake:567 (find_external_library)"
      - "/opt/ros/jazzy/share/pcl_conversions/cmake/ament_cmake_export_dependencies-extras.cmake:21 (find_package)"
      - "/opt/ros/jazzy/share/pcl_conversions/cmake/pcl_conversionsConfig.cmake:41 (include)"
      - "/opt/ros/jazzy/share/pcl_ros/cmake/ament_cmake_export_dependencies-extras.cmake:21 (find_package)"
      - "/opt/ros/jazzy/share/pcl_ros/cmake/pcl_rosConfig.cmake:41 (include)"
      - "CMakeLists.txt:19 (find_package)"
    directories:
      source: "/home/student31/leo_ws/build/leo_lidar_sim/CMakeFiles/CMakeScratch/TryCompile-hHQvCc"
      binary: "/home/student31/leo_ws/build/leo_lidar_sim/CMakeFiles/CMakeScratch/TryCompile-hHQvCc"
    cmakeVariables:
      CMAKE_C_FLAGS: ""
      CMAKE_C_FLAGS_DEBUG: "-g"
      CMAKE_EXE_LINKER_FLAGS: ""
      CMAKE_MODULE_PATH: "/usr/lib/x86_64-linux-gnu/cmake/vtk-9.1/patches/99;/usr/lib/x86_64-linux-gnu/cmake/vtk-9.1;/opt/ros/jazzy/share/fastrtps_cmake_module/cmake/Modules;/usr/lib/x86_64-linux-gnu/cmake/pcl/Modules"
    buildResult:
      variable: "X11_LIB_X11_SOLO"
      cached: true
      stdout: |
        Change Dir: '/home/student31/leo_ws/build/leo_lidar_sim/CMakeFiles/CMakeScratch/TryCompile-hHQvCc'
        
        Run Build Command(s): /usr/bin/cmake -E env VERBOSE=1 /usr/bin/gmake -f Makefile cmTC_bab07/fast
        /usr/bin/gmake  -f CMakeFiles/cmTC_bab07.dir/build.make CMakeFiles/cmTC_bab07.dir/build
        gmake[1]: Entering directory '/home/student31/leo_ws/build/leo_lidar_sim/CMakeFiles/CMakeScratch/TryCompile-hHQvCc'
        Building C object CMakeFiles/cmTC_bab07.dir/CheckFunctionExists.c.o
        /usr/bin/cc   -DCHECK_FUNCTION_EXISTS=XOpenDisplay -o CMakeFiles/cmTC_bab07.dir/CheckFunctionExists.c.o -c /home/student31/leo_ws/build/leo_lidar_sim/CMakeFiles/CMakeScratch/TryCompile-hHQvCc/CheckFunctionExists.c
        Linking C executable cmTC_bab07
        /usr/bin/cmake -E cmake_link_script CMakeFiles/cmTC_bab07.dir/link.txt --verbose=1
        /usr/bin/cc  -DCHECK_FUNCTION_EXISTS=XOpenDisplay CMakeFiles/cmTC_bab07.dir/CheckFunctionExists.c.o -o cmTC_bab07  /usr/lib/x86_64-linux-gnu/libX11.so /usr/lib/x86_64-linux-gnu/libXext.so 
        gmake[1]: Leaving directory '/home/student31/leo_ws/build/leo_lidar_sim/CMakeFiles/CMakeScratch/TryCompile-hHQvCc'
        
      exitCode: 0
  -
    kind: "try_compile-v1"
    backtrace:
      - "/usr/share/cmake-3.28/Modules/CheckFunctionExists.cmake:86 (try_compile)"
      - "/usr/share/cmake-3.28/Modules/FindX11.cmake:697 (check_function_exists)"
      - "/usr/lib/x86_64-linux-gnu/cmake/vtk-9.1/VTK-vtk-module-find-packages.cmake:1149 (find_package)"
      - "/usr/lib/x86_64-linux-gnu/cmake/vtk-9.1/vtk-config.cmake:150 (include)"
      - "/usr/lib/x86_64-linux-gnu/cmake/pcl/PCLConfig.cmake:275 (find_package)"
      - "/usr/lib/x86_64-linux-gnu/cmake/pcl/PCLConfig.cmake:324 (find_VTK)"
      - "/usr/lib/x86_64-linux-gnu/cmake/pcl/PCLConfig.cmake:567 (find_external_library)"
      - "/opt/ros/jazzy/share/pcl_conversions/cmake/ament_cmake_export_dependencies-extras.cmake:21 (find_package)"
      - "/opt/ros/jazzy/share/pcl_conversions/cmake/pcl_conversionsConfig.cmake:41 (include)"
      - "/opt/ros/jazzy/share/pcl_ros/cmake/ament_cmake_export_dependencies-extras.cmake:21 (find_package)"
      - "/opt/ros/jazzy/share/pcl_ros/cmake/pcl_rosConfig.cmake:41 (include)"
      - "CMakeLists.txt:19 (find_package)"
    directories:
      source: "/home/student31/leo_ws/build/leo_lidar_sim/CMakeFiles/CMakeScratch/TryCompile-5bHKVQ"
      binary: "/home/student31/leo_ws/build/leo_lidar_sim/CMakeFiles/CMakeScratch/TryCompile-5bHKVQ"
    cmakeVariables:
      CMAKE_C_FLAGS: ""
      CMAKE_C_FLAGS_DEBUG: "-g"
      CMAKE_EXE_LINKER_FLAGS: ""
      CMAKE_MODULE_PATH: "/usr/lib/x86_64-linux-gnu/cmake/vtk-9.1/patches/99;/usr/lib/x86_64-linux-gnu/cmake/vtk-9.1;/opt/ros/jazzy/share/fastrtps_cmake_module/cmake/Modules;/usr/lib/x86_64-linux-gnu/cmake/pcl/Modules"
    buildResult:
      variable: "CMAKE_HAVE_GETHOSTBYNAME"
      cached: true
      stdout: |
        Change Dir: '/home/student31/leo_ws/build/leo_lidar_sim/CMakeFiles/CMakeScratch/TryCompile-5bHKVQ'
        
        Run Build Command(s): /usr/bin/cmake -E env VERBOSE=1 /usr/bin/gmake -f Makefile cmTC_a386b/fast
        /usr/bin/gmake  -f CMakeFiles/cmTC_a386b.dir/build.make CMakeFiles/cmTC_a386b.dir/build
        gmake[1]: Entering directory '/home/student31/leo_ws/build/leo_lidar_sim/CMakeFiles/CMakeScratch/TryCompile-5bHKVQ'
        Building C object CMakeFiles/cmTC_a386b.dir/CheckFunctionExists.c.o
        /usr/bin/cc   -DCHECK_FUNCTION_EXISTS=gethostbyname -o CMakeFiles/cmTC_a386b.dir/CheckFunctionExists.c.o -c /home/student31/leo_ws/build/leo_lidar_sim/CMakeFiles/CMakeScratch/TryCompile-5bHKVQ/CheckFunctionExists.c
        Linking C executable cmTC_a386b
        /usr/bin/cmake -E cmake_link_script CMakeFiles/cmTC_a386b.dir/link.txt --verbose=1
        /usr/bin/cc  -DCHECK_FUNCTION_EXISTS=gethostbyname CMakeFiles/cmTC_a386b.dir/CheckFunctionExists.c.o -o cmTC_a386b 
        gmake[1]: Leaving directory '/home/student31/leo_ws/build/leo_lidar_sim/CMakeFiles/CMakeScratch/TryCompile-5bHKVQ'
        
      exitCode: 0
  -
    kind: "try_compile-v1"
    backtrace:
      - "/usr/share/cmake-3.28/Modules/CheckFunctionExists.cmake:86 (try_compile)"
      - "/usr/share/cmake-3.28/Modules/FindX11.cmake:711 (check_function_exists)"
      - "/usr/lib/x86_64-linux-gnu/cmake/vtk-9.1/VTK-vtk-module-find-packages.cmake:1149 (find_package)"
      - "/usr/lib/x86_64-linux-gnu/cmake/vtk-9.1/vtk-config.cmake:150 (include)"
      - "/usr/lib/x86_64-linux-gnu/cmake/pcl/PCLConfig.cmake:275 (find_package)"
      - "/usr/lib/x86_64-linux-gnu/cmake/pcl/PCLConfig.cmake:324 (find_VTK)"
      - "/usr/lib/x86_64-linux-gnu/cmake/pcl/PCLConfig.cmake:567 (find_external_library)"
      - "/opt/ros/jazzy/share/pcl_conversions/cmake/ament_cmake_export_dependencies-extras.cmake:21 (find_package)"
      - "/opt/ros/jazzy/share/pcl_conversions/cmake/pcl_conversionsConfig.cmake:41 (include)"
      - "/opt/ros/jazzy/share/pcl_ros/cmake/ament_cmake_export_dependencies-extras.cmake:21 (find_package)"
      - "/opt/ros/jazzy/share/pcl_ros/cmake/pcl_rosConfig.cmake:41 (include)"
      - "CMakeLists.txt:19 (find_package)"
    directories:
      source: "/home/student31/leo_ws/build/leo_lidar_sim/CMakeFiles/CMakeScratch/TryCompile-FpTwRC"
      binary: "/home/student31/leo_ws/build/leo_lidar_sim/CMakeFiles/CMakeScratch/TryCompile-FpTwRC"
    cmakeVariables:
      CMAKE_C_FLAGS: ""
      CMAKE_C_FLAGS_DEBUG: "-g"
      CMAKE_EXE_LINKER_FLAGS: ""
      CMAKE_MODULE_PATH: "/usr/lib/x86_64-linux-gnu/cmake/vtk-9.1/patches/99;/usr/lib/x86_64-linux-gnu/cmake/vtk-9.1;/opt/ros/jazzy/share/fastrtps_cmake_module/cmake/Modules;/usr/lib/x86_64-linux-gnu/cmake/pcl/Modules"
    buildResult:
      variable: "CMAKE_HAVE_CONNECT"
      cached: true
      stdout: |
        Change Dir: '/home/student31/leo_ws/build/leo_lidar_sim/CMakeFiles/CMakeScratch/TryCompile-FpTwRC'
        
        Run Build Command(s): /usr/bin/cmake -E env VERBOSE=1 /usr/bin/gmake -f Makefile cmTC_35ca6/fast
        /usr/bin/gmake  -f CMakeFiles/cmTC_35ca6.dir/build.make CMakeFiles/cmTC_35ca6.dir/build
        gmake[1]: Entering directory '/home/student31/leo_ws/build/leo_lidar_sim/CMakeFiles/CMakeScratch/TryCompile-FpTwRC'
        Building C object CMakeFiles/cmTC_35ca6.dir/CheckFunctionExists.c.o
        /usr/bin/cc   -DCHECK_FUNCTION_EXISTS=connect -o CMakeFiles/cmTC_35ca6.dir/CheckFunctionExists.c.o -c /home/student31/leo_ws/build/leo_lidar_sim/CMakeFiles/CMakeScratch/TryCompile-FpTwRC/CheckFunctionExists.c
        Linking C executable cmTC_35ca6
        /usr/bin/cmake -E cmake_link_script CMakeFiles/cmTC_35ca6.dir/link.txt --verbose=1
        /usr/bin/cc  -DCHECK_FUNCTION_EXISTS=connect CMakeFiles/cmTC_35ca6.dir/CheckFunctionExists.c.o -o cmTC_35ca6 
        gmake[1]: Leaving directory '/home/student31/leo_ws/build/leo_lidar_sim/CMakeFiles/CMakeScratch/TryCompile-FpTwRC'
        
      exitCode: 0
  -
    kind: "try_compile-v1"
    backtrace:
      - "/usr/share/cmake-3.28/Modules/CheckFunctionExists.cmake:86 (try_compile)"
      - "/usr/share/cmake-3.28/Modules/FindX11.cmake:720 (check_function_exists)"
      - "/usr/lib/x86_64-linux-gnu/cmake/vtk-9.1/VTK-vtk-module-find-packages.cmake:1149 (find_package)"
      - "/usr/lib/x86_64-linux-gnu/cmake/vtk-9.1/vtk-config.cmake:150 (include)"
      - "/usr/lib/x86_64-linux-gnu/cmake/pcl/PCLConfig.cmake:275 (find_package)"
      - "/usr/lib/x86_64-linux-gnu/cmake/pcl/PCLConfig.cmake:324 (find_VTK)"
      - "/usr/lib/x86_64-linux-gnu/cmake/pcl/PCLConfig.cmake:567 (find_external_library)"
      - "/opt/ros/jazzy/share/pcl_conversions/cmake/ament_cmake_export_dependencies-extras.cmake:21 (find_package)"
      - "/opt/ros/jazzy/share/pcl_conversions/cmake/pcl_conversionsConfig.cmake:41 (include)"
      - "/opt/ros/jazzy/share/pcl_ros/cmake/ament_cmake_export_dependencies-extras.cmake:21 (find_package)"
      - "/opt/ros/jazzy/share/pcl_ros/cmake/pcl_rosConfig.cmake:41 (include)"
      - "CMakeLists.txt:19 (find_package)"
    directories:
      source: "/home/student31/leo_ws/build/leo_lidar_sim/CMakeFiles/CMakeScratch/TryCompile-zXJbwo"
      binary: "/home/student31/leo_ws/build/leo_lidar_sim/CMakeFiles/CMakeScratch/TryCompile-zXJbwo"
    cmakeVariables:
      CMAKE_C_FLAGS: ""
      CMAKE_C_FLAGS_DEBUG: "-g"
      CMAKE_EXE_LINKER_FLAGS: ""
      CMAKE_MODULE_PATH: "/usr/lib/x86_64-linux-gnu/cmake/vtk-9.1/patches/99;/usr/lib/x86_64-linux-gnu/cmake/vtk-9.1;/opt/ros/jazzy/share/fastrtps_cmake_module/cmake/Modules;/usr/lib/x86_64-linux-gnu/cmake/pcl/Modules"
    buildResult:
      variable: "CMAKE_HAVE_REMOVE"
      cached: true
      stdout: |
        Change Dir: '/home/student31/leo_ws/build/leo_lidar_sim/CMakeFiles/CMakeScratch/TryCompile-zXJbwo'
        
        Run Build Command(s): /usr/bin/cmake -E env VERBOSE=1 /usr/bin/gmake -f Makefile cmTC_e8019/fast
        /usr/bin/gmake  -f CMakeFiles/cmTC_e8019.dir/build.make CMakeFiles/cmTC_e8019.dir/build
        gmake[1]: Entering directory '/home/student31/leo_ws/build/leo_lidar_sim/CMakeFiles/CMakeScratch/TryCompile-zXJbwo'
        Building C object CMakeFiles/cmTC_e8019.dir/CheckFunctionExists.c.o
        /usr/bin/cc   -DCHECK_FUNCTION_EXISTS=remove -o CMakeFiles/cmTC_e8019.dir/CheckFunctionExists.c.o -c /home/student31/leo_ws/build/leo_lidar_sim/CMakeFiles/CMakeScratch/TryCompile-zXJbwo/CheckFunctionExists.c
        Linking C executable cmTC_e8019
        /usr/bin/cmake -E cmake_link_script CMakeFiles/cmTC_e8019.dir/link.txt --verbose=1
        /usr/bin/cc  -DCHECK_FUNCTION_EXISTS=remove CMakeFiles/cmTC_e8019.dir/CheckFunctionExists.c.o -o cmTC_e8019 
        gmake[1]: Leaving directory '/home/student31/leo_ws/build/leo_lidar_sim/CMakeFiles/CMakeScratch/TryCompile-zXJbwo'
        
      exitCode: 0
  -
    kind: "try_compile-v1"
    backtrace:
      - "/usr/share/cmake-3.28/Modules/CheckFunctionExists.cmake:86 (try_compile)"
      - "/usr/share/cmake-3.28/Modules/FindX11.cmake:729 (check_function_exists)"
      - "/usr/lib/x86_64-linux-gnu/cmake/vtk-9.1/VTK-vtk-module-find-packages.cmake:1149 (find_package)"
      - "/usr/lib/x86_64-linux-gnu/cmake/vtk-9.1/vtk-config.cmake:150 (include)"
      - "/usr/lib/x86_64-linux-gnu/cmake/pcl/PCLConfig.cmake:275 (find_package)"
      - "/usr/lib/x86_64-linux-gnu/cmake/pcl/PCLConfig.cmake:324 (find_VTK)"
      - "/usr/lib/x86_64-linux-gnu/cmake/pcl/PCLConfig.cmake:567 (find_external_library)"
      - "/opt/ros/jazzy/share/pcl_conversions/cmake/ament_cmake_export_dependencies-extras.cmake:21 (find_package)"
      - "/opt/ros/jazzy/share/pcl_conversions/cmake/pcl_conversionsConfig.cmake:41 (include)"
      - "/opt/ros/jazzy/share/pcl_ros/cmake/ament_cmake_export_dependencies-extras.cmake:21 (find_package)"
      - "/opt/ros/jazzy/share/pcl_ros/cmake/pcl_rosConfig.cmake:41 (include)"
      - "CMakeLists.txt:19 (find_package)"
    directories:
      source: "/home/student31/leo_ws/build/leo_lidar_sim/CMakeFiles/CMakeScratch/TryCompile-uWRMRn"
      binary: "/home/student31/leo_ws/build/leo_lidar_sim/CMakeFiles/CMakeScratch/TryCompile-uWRMRn"
    cmakeVariables:
      CMAKE_C_FLAGS: ""
      CMAKE_C_FLAGS_DEBUG: "-g"
      CMAKE_EXE_LINKER_FLAGS: ""
      CMAKE_MODULE_PATH: "/usr/lib/x86_64-linux-gnu/cmake/vtk-9.1/patches/99;/usr/lib/x86_64-linux-gnu/cmake/vtk-9.1;/opt/ros/jazzy/share/fastrtps_cmake_module/cmake/Modules;/usr/lib/x86_64-linux-gnu/cmake/pcl/Modules"
    buildResult:
      variable: "CMAKE_HAVE_SHMAT"
      cached: true
      stdout: |
        Change Dir: '/home/student31/leo_ws/build/leo_lidar_sim/CMakeFiles/CMakeScratch/TryCompile-uWRMRn'
        
        Run Build Command(s): /usr/bin/cmake -E env VERBOSE=1 /usr/bin/gmake -f Makefile cmTC_d7722/fast
        /usr/bin/gmake  -f CMakeFiles/cmTC_d7722.dir/build.make CMakeFiles/cmTC_d7722.dir/build
        gmake[1]: Entering directory '/home/student31/leo_ws/build/leo_lidar_sim/CMakeFiles/CMakeScratch/TryCompile-uWRMRn'
        Building C object CMakeFiles/cmTC_d7722.dir/CheckFunctionExists.c.o
        /usr/bin/cc   -DCHECK_FUNCTION_EXISTS=shmat -o CMakeFiles/cmTC_d7722.dir/CheckFunctionExists.c.o -c /home/student31/leo_ws/build/leo_lidar_sim/CMakeFiles/CMakeScratch/TryCompile-uWRMRn/CheckFunctionExists.c
        Linking C executable cmTC_d7722
        /usr/bin/cmake -E cmake_link_script CMakeFiles/cmTC_d7722.dir/link.txt --verbose=1
        /usr/bin/cc  -DCHECK_FUNCTION_EXISTS=shmat CMakeFiles/cmTC_d7722.dir/CheckFunctionExists.c.o -o cmTC_d7722 
        gmake[1]: Leaving directory '/home/student31/leo_ws/build/leo_lidar_sim/CMakeFiles/CMakeScratch/TryCompile-uWRMRn'
        
      exitCode: 0
  -
    kind: "try_compile-v1"
    backtrace:
      - "/usr/share/cmake-3.28/Modules/CheckLibraryExists.cmake:69 (try_compile)"
      - "/usr/share/cmake-3.28/Modules/FindX11.cmake:739 (check_library_exists)"
      - "/usr/lib/x86_64-linux-gnu/cmake/vtk-9.1/VTK-vtk-module-find-packages.cmake:1149 (find_package)"
      - "/usr/lib/x86_64-linux-gnu/cmake/vtk-9.1/vtk-config.cmake:150 (include)"
      - "/usr/lib/x86_64-linux-gnu/cmake/pcl/PCLConfig.cmake:275 (find_package)"
      - "/usr/lib/x86_64-linux-gnu/cmake/pcl/PCLConfig.cmake:324 (find_VTK)"
      - "/usr/lib/x86_64-linux-gnu/cmake/pcl/PCLConfig.cmake:567 (find_external_library)"
      - "/opt/ros/jazzy/share/pcl_conversions/cmake/ament_cmake_export_dependencies-extras.cmake:21 (find_package)"
      - "/opt/ros/jazzy/share/pcl_conversions/cmake/pcl_conversionsConfig.cmake:41 (include)"
      - "/opt/ros/jazzy/share/pcl_ros/cmake/ament_cmake_export_dependencies-extras.cmake:21 (find_package)"
      - "/opt/ros/jazzy/share/pcl_ros/cmake/pcl_rosConfig.cmake:41 (include)"
      - "CMakeLists.txt:19 (find_package)"
    directories:
      source: "/home/student31/leo_ws/build/leo_lidar_sim/CMakeFiles/CMakeScratch/TryCompile-LZPpgr"
      binary: "/home/student31/leo_ws/build/leo_lidar_sim/CMakeFiles/CMakeScratch/TryCompile-LZPpgr"
    cmakeVariables:
      CMAKE_C_FLAGS: ""
      CMAKE_C_FLAGS_DEBUG: "-g"
      CMAKE_EXE_LINKER_FLAGS: ""
      CMAKE_MODULE_PATH: "/usr/lib/x86_64-linux-gnu/cmake/vtk-9.1/patches/99;/usr/lib/x86_64-linux-gnu/cmake/vtk-9.1;/opt/ros/jazzy/share/fastrtps_cmake_module/cmake/Modules;/usr/lib/x86_64-linux-gnu/cmake/pcl/Modules"
    buildResult:
      variable: "CMAKE_LIB_ICE_HAS_ICECONNECTIONNUMBER"
      cached: true
      stdout: |
        Change Dir: '/home/student31/leo_ws/build/leo_lidar_sim/CMakeFiles/CMakeScratch/TryCompile-LZPpgr'
        
        Run Build Command(s): /usr/bin/cmake -E env VERBOSE=1 /usr/bin/gmake -f Makefile cmTC_055b2/fast
        /usr/bin/gmake  -f CMakeFiles/cmTC_055b2.dir/build.make CMakeFiles/cmTC_055b2.dir/build
        gmake[1]: Entering directory '/home/student31/leo_ws/build/leo_lidar_sim/CMakeFiles/CMakeScratch/TryCompile-LZPpgr'
        Building C object CMakeFiles/cmTC_055b2.dir/CheckFunctionExists.c.o
        /usr/bin/cc   -DCHECK_FUNCTION_EXISTS=IceConnectionNumber -o CMakeFiles/cmTC_055b2.dir/CheckFunctionExists.c.o -c /home/student31/leo_ws/build/leo_lidar_sim/CMakeFiles/CMakeScratch/TryCompile-LZPpgr/CheckFunctionExists.c
        Linking C executable cmTC_055b2
        /usr/bin/cmake -E cmake_link_script CMakeFiles/cmTC_055b2.dir/link.txt --verbose=1
        /usr/bin/cc  -DCHECK_FUNCTION_EXISTS=IceConnectionNumber CMakeFiles/cmTC_055b2.dir/CheckFunctionExists.c.o -o cmTC_055b2  -lICE 
        gmake[1]: Leaving directory '/home/student31/leo_ws/build/leo_lidar_sim/CMakeFiles/CMakeScratch/TryCompile-LZPpgr'
        
      exitCode: 0
  -
    kind: "try_compile-v1"
    backtrace:
      - "/usr/share/cmake-3.28/Modules/FindMPI.cmake:1278 (try_compile)"
      - "/usr/share/cmake-3.28/Modules/FindMPI.cmake:1322 (_MPI_try_staged_settings)"
      - "/usr/share/cmake-3.28/Modules/FindMPI.cmake:1645 (_MPI_check_lang_works)"
      - "/usr/lib/x86_64-linux-gnu/cmake/vtk-9.1/VTK-vtk-module-find-packages.cmake:397 (find_package)"
      - "/usr/lib/x86_64-linux-gnu/cmake/vtk-9.1/vtk-config.cmake:150 (include)"
      - "/usr/lib/x86_64-linux-gnu/cmake/pcl/PCLConfig.cmake:275 (find_package)"
      - "/usr/lib/x86_64-linux-gnu/cmake/pcl/PCLConfig.cmake:324 (find_VTK)"
      - "/usr/lib/x86_64-linux-gnu/cmake/pcl/PCLConfig.cmake:567 (find_external_library)"
      - "/opt/ros/jazzy/share/pcl_conversions/cmake/ament_cmake_export_dependencies-extras.cmake:21 (find_package)"
      - "/opt/ros/jazzy/share/pcl_conversions/cmake/pcl_conversionsConfig.cmake:41 (include)"
      - "/opt/ros/jazzy/share/pcl_ros/cmake/ament_cmake_export_dependencies-extras.cmake:21 (find_package)"
      - "/opt/ros/jazzy/share/pcl_ros/cmake/pcl_rosConfig.cmake:41 (include)"
      - "CMakeLists.txt:19 (find_package)"
    description: "The MPI test test_mpi for C in mode normal"
    directories:
      source: "/home/student31/leo_ws/build/leo_lidar_sim/CMakeFiles/CMakeScratch/TryCompile-q8o8qA"
      binary: "/home/student31/leo_ws/build/leo_lidar_sim/CMakeFiles/CMakeScratch/TryCompile-q8o8qA"
    cmakeVariables:
      CMAKE_C_FLAGS: ""
      CMAKE_C_FLAGS_DEBUG: "-g"
      CMAKE_EXE_LINKER_FLAGS: ""
      CMAKE_MODULE_PATH: "/usr/lib/x86_64-linux-gnu/cmake/vtk-9.1/patches/99;/usr/lib/x86_64-linux-gnu/cmake/vtk-9.1;/opt/ros/jazzy/share/fastrtps_cmake_module/cmake/Modules;/usr/lib/x86_64-linux-gnu/cmake/pcl/Modules"
    buildResult:
      variable: "MPI_RESULT_C_test_mpi_normal"
      cached: true
      stdout: |
        Change Dir: '/home/student31/leo_ws/build/leo_lidar_sim/CMakeFiles/CMakeScratch/TryCompile-q8o8qA'
        
        Run Build Command(s): /usr/bin/cmake -E env VERBOSE=1 /usr/bin/gmake -f Makefile cmTC_b3ced/fast
        /usr/bin/gmake  -f CMakeFiles/cmTC_b3ced.dir/build.make CMakeFiles/cmTC_b3ced.dir/build
        gmake[1]: Entering directory '/home/student31/leo_ws/build/leo_lidar_sim/CMakeFiles/CMakeScratch/TryCompile-q8o8qA'
        Building C object CMakeFiles/cmTC_b3ced.dir/test_mpi.c.o
        /usr/bin/cc  -isystem /usr/lib/x86_64-linux-gnu/openmpi/include -isystem /usr/lib/x86_64-linux-gnu/openmpi/include/openmpi  -o CMakeFiles/cmTC_b3ced.dir/test_mpi.c.o -c /home/student31/leo_ws/build/leo_lidar_sim/CMakeFiles/CMakeScratch/TryCompile-q8o8qA/test_mpi.c
        Linking C executable cmTC_b3ced
        /usr/bin/cmake -E cmake_link_script CMakeFiles/cmTC_b3ced.dir/link.txt --verbose=1
        /usr/bin/cc -L/usr/lib/x86_64-linux-gnu/openmpi/lib CMakeFiles/cmTC_b3ced.dir/test_mpi.c.o -o cmTC_b3ced  /usr/lib/x86_64-linux-gnu/libmpi.so 
        gmake[1]: Leaving directory '/home/student31/leo_ws/build/leo_lidar_sim/CMakeFiles/CMakeScratch/TryCompile-q8o8qA'
        
      exitCode: 0
  -
    kind: "try_compile-v1"
    backtrace:
      - "/usr/share/cmake-3.28/Modules/FindMPI.cmake:1278 (try_compile)"
      - "/usr/share/cmake-3.28/Modules/FindMPI.cmake:1322 (_MPI_try_staged_settings)"
      - "/usr/share/cmake-3.28/Modules/FindMPI.cmake:1645 (_MPI_check_lang_works)"
      - "/usr/lib/x86_64-linux-gnu/cmake/vtk-9.1/VTK-vtk-module-find-packages.cmake:397 (find_package)"
      - "/usr/lib/x86_64-linux-gnu/cmake/vtk-9.1/vtk-config.cmake:150 (include)"
      - "/usr/lib/x86_64-linux-gnu/cmake/pcl/PCLConfig.cmake:275 (find_package)"
      - "/usr/lib/x86_64-linux-gnu/cmake/pcl/PCLConfig.cmake:324 (find_VTK)"
      - "/usr/lib/x86_64-linux-gnu/cmake/pcl/PCLConfig.cmake:570 (find_external_library)"
      - "/opt/ros/jazzy/share/pcl_conversions/cmake/ament_cmake_export_dependencies-extras.cmake:21 (find_package)"
      - "/opt/ros/jazzy/share/pcl_conversions/cmake/pcl_conversionsConfig.cmake:41 (include)"
      - "/opt/ros/jazzy/share/pcl_ros/cmake/ament_cmake_export_dependencies-extras.cmake:21 (find_package)"
      - "/opt/ros/jazzy/share/pcl_ros/cmake/pcl_rosConfig.cmake:41 (include)"
      - "CMakeLists.txt:19 (find_package)"
    description: "The MPI test test_mpi for C in mode normal"
    directories:
      source: "/home/student31/leo_ws/build/leo_lidar_sim/CMakeFiles/CMakeScratch/TryCompile-Kau1XW"
      binary: "/home/student31/leo_ws/build/leo_lidar_sim/CMakeFiles/CMakeScratch/TryCompile-Kau1XW"
    cmakeVariables:
      CMAKE_C_FLAGS: ""
      CMAKE_C_FLAGS_DEBUG: "-g"
      CMAKE_EXE_LINKER_FLAGS: ""
      CMAKE_MODULE_PATH: "/usr/lib/x86_64-linux-gnu/cmake/vtk-9.1/patches/99;/usr/lib/x86_64-linux-gnu/cmake/vtk-9.1;/opt/ros/jazzy/share/fastrtps_cmake_module/cmake/Modules;/usr/lib/x86_64-linux-gnu/cmake/pcl/Modules"
    buildResult:
      variable: "MPI_RESULT_C_test_mpi_normal"
      cached: true
      stdout: |
        Change Dir: '/home/student31/leo_ws/build/leo_lidar_sim/CMakeFiles/CMakeScratch/TryCompile-Kau1XW'
        
        Run Build Command(s): /usr/bin/cmake -E env VERBOSE=1 /usr/bin/gmake -f Makefile cmTC_8ea3b/fast
        /usr/bin/gmake  -f CMakeFiles/cmTC_8ea3b.dir/build.make CMakeFiles/cmTC_8ea3b.dir/build
        gmake[1]: Entering directory '/home/student31/leo_ws/build/leo_lidar_sim/CMakeFiles/CMakeScratch/TryCompile-Kau1XW'
        Building C object CMakeFiles/cmTC_8ea3b.dir/test_mpi.c.o
        /usr/bin/cc  -isystem /usr/lib/x86_64-linux-gnu/openmpi/include -isystem /usr/lib/x86_64-linux-gnu/openmpi/include/openmpi  -o CMakeFiles/cmTC_8ea3b.dir/test_mpi.c.o -c /home/student31/leo_ws/build/leo_lidar_sim/CMakeFiles/CMakeScratch/TryCompile-Kau1XW/test_mpi.c
        Linking C executable cmTC_8ea3b
        /usr/bin/cmake -E cmake_link_script CMakeFiles/cmTC_8ea3b.dir/link.txt --verbose=1
        /usr/bin/cc -L/usr/lib/x86_64-linux-gnu/openmpi/lib CMakeFiles/cmTC_8ea3b.dir/test_mpi.c.o -o cmTC_8ea3b  /usr/lib/x86_64-linux-gnu/libmpi.so 
        gmake[1]: Leaving directory '/home/student31/leo_ws/build/leo_lidar_sim/CMakeFiles/CMakeScratch/TryCompile-Kau1XW'
        
      exitCode: 0
  -
    kind: "try_compile-v1"
    backtrace:
      - "/usr/share/cmake-3.28/Modules/FindMPI.cmake:1278 (try_compile)"
      - "/usr/share/cmake-3.28/Modules/FindMPI.cmake:1322 (_MPI_try_staged_settings)"
      - "/usr/share/cmake-3.28/Modules/FindMPI.cmake:1645 (_MPI_check_lang_works)"
      - "/usr/lib/x86_64-linux-gnu/cmake/vtk-9.1/VTK-vtk-module-find-packages.cmake:397 (find_package)"
      - "/usr/lib/x86_64-linux-gnu/cmake/vtk-9.1/vtk-config.cmake:150 (include)"
      - "/usr/lib/x86_64-linux-gnu/cmake/pcl/PCLConfig.cmake:275 (find_package)"
      - "/usr/lib/x86_64-linux-gnu/cmake/pcl/PCLConfig.cmake:324 (find_VTK)"
      - "/usr/lib/x86_64-linux-gnu/cmake/pcl/PCLConfig.cmake:567 (find_external_library)"
      - "/opt/ros/jazzy/share/pcl_conversions/cmake/ament_cmake_export_dependencies-extras.cmake:21 (find_package)"
      - "/opt/ros/jazzy/share/pcl_conversions/cmake/pcl_conversionsConfig.cmake:41 (include)"
      - "/opt/ros/jazzy/share/pcl_ros/cmake/ament_cmake_export_dependencies-extras.cmake:21 (find_package)"
      - "/opt/ros/jazzy/share/pcl_ros/cmake/pcl_rosConfig.cmake:41 (include)"
      - "CMakeLists.txt:19 (find_package)"
    description: "The MPI test test_mpi for C in mode normal"
    directories:
      source: "/home/student31/leo_ws/build/leo_lidar_sim/CMakeFiles/CMakeScratch/TryCompile-eKhNUX"
      binary: "/home/student31/leo_ws/build/leo_lidar_sim/CMakeFiles/CMakeScratch/TryCompile-eKhNUX"
    cmakeVariables:
      CMAKE_C_FLAGS: ""
      CMAKE_C_FLAGS_DEBUG: "-g"
      CMAKE_EXE_LINKER_FLAGS: ""
      CMAKE_MODULE_PATH: "/usr/lib/x86_64-linux-gnu/cmake/vtk-9.1/patches/99;/usr/lib/x86_64-linux-gnu/cmake/vtk-9.1;/opt/ros/jazzy/share/fastrtps_cmake_module/cmake/Modules;/usr/lib/x86_64-linux-gnu/cmake/pcl/Modules"
    buildResult:
      variable: "MPI_RESULT_C_test_mpi_normal"
      cached: true
      stdout: |
        Change Dir: '/home/student31/leo_ws/build/leo_lidar_sim/CMakeFiles/CMakeScratch/TryCompile-eKhNUX'
        
        Run Build Command(s): /usr/bin/cmake -E env VERBOSE=1 /usr/bin/gmake -f Makefile cmTC_b6e31/fast
        /usr/bin/gmake  -f CMakeFiles/cmTC_b6e31.dir/build.make CMakeFiles/cmTC_b6e31.dir/build
        gmake[1]: Entering directory '/home/student31/leo_ws/build/leo_lidar_sim/CMakeFiles/CMakeScratch/TryCompile-eKhNUX'
        Building C object CMakeFiles/cmTC_b6e31.dir/test_mpi.c.o
        /usr/bin/cc  -isystem /usr/lib/x86_64-linux-gnu/openmpi/include -isystem /usr/lib/x86_64-linux-gnu/openmpi/include/openmpi  -o CMakeFiles/cmTC_b6e31.dir/test_mpi.c.o -c /home/student31/leo_ws/build/leo_lidar_sim/CMakeFiles/CMakeScratch/TryCompile-eKhNUX/test_mpi.c
        Linking C executable cmTC_b6e31
        /usr/bin/cmake -E cmake_link_script CMakeFiles/cmTC_b6e31.dir/link.txt --verbose=1
        /usr/bin/cc -L/usr/lib/x86_64-linux-gnu/openmpi/lib CMakeFiles/cmTC_b6e31.dir/test_mpi.c.o -o cmTC_b6e31  /usr/lib/x86_64-linux-gnu/libmpi.so 
        gmake[1]: Leaving directory '/home/student31/leo_ws/build/leo_lidar_sim/CMakeFiles/CMakeScratch/TryCompile-eKhNUX'
        
      exitCode: 0
  -
    kind: "try_compile-v1"
    backtrace:
      - "/usr/share/cmake-3.28/Modules/FindMPI.cmake:1278 (try_compile)"
      - "/usr/share/cmake-3.28/Modules/FindMPI.cmake:1322 (_MPI_try_staged_settings)"
      - "/usr/share/cmake-3.28/Modules/FindMPI.cmake:1645 (_MPI_check_lang_works)"
      - "/usr/lib/x86_64-linux-gnu/cmake/vtk-9.1/VTK-vtk-module-find-packages.cmake:397 (find_package)"
      - "/usr/lib/x86_64-linux-gnu/cmake/vtk-9.1/vtk-config.cmake:150 (include)"
      - "/usr/lib/x86_64-linux-gnu/cmake/pcl/PCLConfig.cmake:275 (find_package)"
      - "/usr/lib/x86_64-linux-gnu/cmake/pcl/PCLConfig.cmake:324 (find_VTK)"
      - "/usr/lib/x86_64-linux-gnu/cmake/pcl/PCLConfig.cmake:567 (find_external_library)"
      - "/opt/ros/jazzy/share/pcl_conversions/cmake/ament_cmake_export_dependencies-extras.cmake:21 (find_package)"
      - "/opt/ros/jazzy/share/pcl_conversions/cmake/pcl_conversionsConfig.cmake:41 (include)"
      - "/opt/ros/jazzy/share/pcl_ros/cmake/ament_cmake_export_dependencies-extras.cmake:21 (find_package)"
      - "/opt/ros/jazzy/share/pcl_ros/cmake/pcl_rosConfig.cmake:41 (include)"
      - "CMakeLists.txt:19 (find_package)"
    description: "The MPI test test_mpi for C in mode normal"
    directories:
      source: "/home/student31/leo_ws/build/leo_lidar_sim/CMakeFiles/CMakeScratch/TryCompile-ATDr63"
      binary: "/home/student31/leo_ws/build/leo_lidar_sim/CMakeFiles/CMakeScratch/TryCompile-ATDr63"
    cmakeVariables:
      CMAKE_C_FLAGS: ""
      CMAKE_C_FLAGS_DEBUG: "-g"
      CMAKE_EXE_LINKER_FLAGS: ""
      CMAKE_MODULE_PATH: "/usr/lib/x86_64-linux-gnu/cmake/vtk-9.1/patches/99;/usr/lib/x86_64-linux-gnu/cmake/vtk-9.1;/opt/ros/jazzy/share/fastrtps_cmake_module/cmake/Modules;/usr/lib/x86_64-linux-gnu/cmake/pcl/Modules"
    buildResult:
      variable: "MPI_RESULT_C_test_mpi_normal"
      cached: true
      stdout: |
        Change Dir: '/home/student31/leo_ws/build/leo_lidar_sim/CMakeFiles/CMakeScratch/TryCompile-ATDr63'
        
        Run Build Command(s): /usr/bin/cmake -E env VERBOSE=1 /usr/bin/gmake -f Makefile cmTC_6c85a/fast
        /usr/bin/gmake  -f CMakeFiles/cmTC_6c85a.dir/build.make CMakeFiles/cmTC_6c85a.dir/build
        gmake[1]: Entering directory '/home/student31/leo_ws/build/leo_lidar_sim/CMakeFiles/CMakeScratch/TryCompile-ATDr63'
        Building C object CMakeFiles/cmTC_6c85a.dir/test_mpi.c.o
        /usr/bin/cc  -isystem /usr/lib/x86_64-linux-gnu/openmpi/include -isystem /usr/lib/x86_64-linux-gnu/openmpi/include/openmpi  -o CMakeFiles/cmTC_6c85a.dir/test_mpi.c.o -c /home/student31/leo_ws/build/leo_lidar_sim/CMakeFiles/CMakeScratch/TryCompile-ATDr63/test_mpi.c
        Linking C executable cmTC_6c85a
        /usr/bin/cmake -E cmake_link_script CMakeFiles/cmTC_6c85a.dir/link.txt --verbose=1
        /usr/bin/cc -L/usr/lib/x86_64-linux-gnu/openmpi/lib CMakeFiles/cmTC_6c85a.dir/test_mpi.c.o -o cmTC_6c85a  /usr/lib/x86_64-linux-gnu/libmpi.so 
        gmake[1]: Leaving directory '/home/student31/leo_ws/build/leo_lidar_sim/CMakeFiles/CMakeScratch/TryCompile-ATDr63'
        
      exitCode: 0
...

---
events:
  -
    kind: "try_compile-v1"
    backtrace:
      - "/usr/share/cmake-3.28/Modules/FindMPI.cmake:1278 (try_compile)"
      - "/usr/share/cmake-3.28/Modules/FindMPI.cmake:1322 (_MPI_try_staged_settings)"
      - "/usr/share/cmake-3.28/Modules/FindMPI.cmake:1645 (_MPI_check_lang_works)"
      - "/usr/lib/x86_64-linux-gnu/cmake/vtk-9.1/VTK-vtk-module-find-packages.cmake:397 (find_package)"
      - "/usr/lib/x86_64-linux-gnu/cmake/vtk-9.1/vtk-config.cmake:150 (include)"
      - "/usr/lib/x86_64-linux-gnu/cmake/pcl/PCLConfig.cmake:275 (find_package)"
      - "/usr/lib/x86_64-linux-gnu/cmake/pcl/PCLConfig.cmake:324 (find_VTK)"
      - "/usr/lib/x86_64-linux-gnu/cmake/pcl/PCLConfig.cmake:567 (find_external_library)"
      - "/opt/ros/jazzy/share/pcl_conversions/cmake/ament_cmake_export_dependencies-extras.cmake:21 (find_package)"
      - "/opt/ros/jazzy/share/pcl_conversions/cmake/pcl_conversionsConfig.cmake:41 (include)"
      - "/opt/ros/jazzy/share/pcl_ros/cmake/ament_cmake_export_dependencies-extras.cmake:21 (find_package)"
      - "/opt/ros/jazzy/share/pcl_ros/cmake/pcl_rosConfig.cmake:41 (include)"
      - "CMakeLists.txt:19 (find_package)"
    description: "The MPI test test_mpi for C in mode normal"
    directories:
      source: "/home/student31/leo_ws/build/leo_lidar_sim/CMakeFiles/CMakeScratch/TryCompile-ly0X5G"
      binary: "/home/student31/leo_ws/build/leo_lidar_sim/CMakeFiles/CMakeScratch/TryCompile-ly0X5G"
    cmakeVariables:
      CMAKE_C_FLAGS: ""
      CMAKE_C_FLAGS_DEBUG: "-g"
      CMAKE_EXE_LINKER_FLAGS: ""
      CMAKE_MODULE_PATH: "/usr/lib/x86_64-linux-gnu/cmake/vtk-9.1/patches/99;/usr/lib/x86_64-linux-gnu/cmake/vtk-9.1;/opt/ros/jazzy/share/fastrtps_cmake_module/cmake/Modules;/usr/lib/x86_64-linux-gnu/cmake/pcl/Modules"
    buildResult:
      variable: "MPI_RESULT_C_test_mpi_normal"
      cached: true
      stdout: |
        Change Dir: '/home/student31/leo_ws/build/leo_lidar_sim/CMakeFiles/CMakeScratch/TryCompile-ly0X5G'
        
        Run Build Command(s): /usr/bin/cmake -E env VERBOSE=1 /usr/bin/gmake -f Makefile cmTC_e0998/fast
        /usr/bin/gmake  -f CMakeFiles/cmTC_e0998.dir/build.make CMakeFiles/cmTC_e0998.dir/build
        gmake[1]: Entering directory '/home/student31/leo_ws/build/leo_lidar_sim/CMakeFiles/CMakeScratch/TryCompile-ly0X5G'
        Building C object CMakeFiles/cmTC_e0998.dir/test_mpi.c.o
        /usr/bin/cc  -isystem /usr/lib/x86_64-linux-gnu/openmpi/include -isystem /usr/lib/x86_64-linux-gnu/openmpi/include/openmpi  -o CMakeFiles/cmTC_e0998.dir/test_mpi.c.o -c /home/student31/leo_ws/build/leo_lidar_sim/CMakeFiles/CMakeScratch/TryCompile-ly0X5G/test_mpi.c
        Linking C executable cmTC_e0998
        /usr/bin/cmake -E cmake_link_script CMakeFiles/cmTC_e0998.dir/link.txt --verbose=1
        /usr/bin/cc -L/usr/lib/x86_64-linux-gnu/openmpi/lib CMakeFiles/cmTC_e0998.dir/test_mpi.c.o -o cmTC_e0998  /usr/lib/x86_64-linux-gnu/libmpi.so 
        gmake[1]: Leaving directory '/home/student31/leo_ws/build/leo_lidar_sim/CMakeFiles/CMakeScratch/TryCompile-ly0X5G'
        
      exitCode: 0
  -
    kind: "try_compile-v1"
    backtrace:
      - "/usr/share/cmake-3.28/Modules/FindMPI.cmake:1278 (try_compile)"
      - "/usr/share/cmake-3.28/Modules/FindMPI.cmake:1322 (_MPI_try_staged_settings)"
      - "/usr/share/cmake-3.28/Modules/FindMPI.cmake:1645 (_MPI_check_lang_works)"
      - "/usr/lib/x86_64-linux-gnu/cmake/vtk-9.1/VTK-vtk-module-find-packages.cmake:397 (find_package)"
      - "/usr/lib/x86_64-linux-gnu/cmake/vtk-9.1/vtk-config.cmake:150 (include)"
      - "/usr/lib/x86_64-linux-gnu/cmake/pcl/PCLConfig.cmake:275 (find_package)"
      - "/usr/lib/x86_64-linux-gnu/cmake/pcl/PCLConfig.cmake:324 (find_VTK)"
      - "/usr/lib/x86_64-linux-gnu/cmake/pcl/PCLConfig.cmake:567 (find_external_library)"
      - "/opt/ros/jazzy/share/pcl_conversions/cmake/ament_cmake_export_dependencies-extras.cmake:21 (find_package)"
      - "/opt/ros/jazzy/share/pcl_conversions/cmake/pcl_conversionsConfig.cmake:41 (include)"
      - "/opt/ros/jazzy/share/pcl_ros/cmake/ament_cmake_export_dependencies-extras.cmake:21 (find_package)"
      - "/opt/ros/jazzy/share/pcl_ros/cmake/pcl_rosConfig.cmake:41 (include)"
      - "CMakeLists.txt:19 (find_package)"
    description: "The MPI test test_mpi for C in mode normal"
    directories:
      source: "/home/student31/leo_ws/build/leo_lidar_sim/CMakeFiles/CMakeScratch/TryCompile-eZ5KSf"
      binary: "/home/student31/leo_ws/build/leo_lidar_sim/CMakeFiles/CMakeScratch/TryCompile-eZ5KSf"
    cmakeVariables:
      CMAKE_C_FLAGS: ""
      CMAKE_C_FLAGS_DEBUG: "-g"
      CMAKE_EXE_LINKER_FLAGS: ""
      CMAKE_MODULE_PATH: "/usr/lib/x86_64-linux-gnu/cmake/vtk-9.1/patches/99;/usr/lib/x86_64-linux-gnu/cmake/vtk-9.1;/opt/ros/jazzy/share/fastrtps_cmake_module/cmake/Modules;/usr/lib/x86_64-linux-gnu/cmake/pcl/Modules"
    buildResult:
      variable: "MPI_RESULT_C_test_mpi_normal"
      cached: true
      stdout: |
        Change Dir: '/home/student31/leo_ws/build/leo_lidar_sim/CMakeFiles/CMakeScratch/TryCompile-eZ5KSf'
        
        Run Build Command(s): /usr/bin/cmake -E env VERBOSE=1 /usr/bin/gmake -f Makefile cmTC_0277f/fast
        /usr/bin/gmake  -f CMakeFiles/cmTC_0277f.dir/build.make CMakeFiles/cmTC_0277f.dir/build
        gmake[1]: Entering directory '/home/student31/leo_ws/build/leo_lidar_sim/CMakeFiles/CMakeScratch/TryCompile-eZ5KSf'
        Building C object CMakeFiles/cmTC_0277f.dir/test_mpi.c.o
        /usr/bin/cc  -isystem /usr/lib/x86_64-linux-gnu/openmpi/include -isystem /usr/lib/x86_64-linux-gnu/openmpi/include/openmpi  -o CMakeFiles/cmTC_0277f.dir/test_mpi.c.o -c /home/student31/leo_ws/build/leo_lidar_sim/CMakeFiles/CMakeScratch/TryCompile-eZ5KSf/test_mpi.c
        Linking C executable cmTC_0277f
        /usr/bin/cmake -E cmake_link_script CMakeFiles/cmTC_0277f.dir/link.txt --verbose=1
        Error opening link script "CMakeFiles/cmTC_0277f.dir/link.txt"
        gmake[1]: *** [CMakeFiles/cmTC_0277f.dir/build.make:100: cmTC_0277f] Error 1
        gmake[1]: Leaving directory '/home/student31/leo_ws/build/leo_lidar_sim/CMakeFiles/CMakeScratch/TryCompile-eZ5KSf'
        gmake: *** [Makefile:127: cmTC_0277f/fast] Error 2
        
      exitCode: 2
  -
    kind: "try_compile-v1"
    backtrace:
      - "/usr/share/cmake-3.28/Modules/FindMPI.cmake:1278 (try_compile)"
      - "/usr/share/cmake-3.28/Modules/FindMPI.cmake:1322 (_MPI_try_staged_settings)"
      - "/usr/share/cmake-3.28/Modules/FindMPI.cmake:1645 (_MPI_check_lang_works)"
      - "/usr/lib/x86_64-linux-gnu/cmake/vtk-9.1/VTK-vtk-module-find-packages.cmake:397 (find_package)"
      - "/usr/lib/x86_64-linux-gnu/cmake/vtk-9.1/vtk-config.cmake:150 (include)"
      - "/usr/lib/x86_64-linux-gnu/cmake/pcl/PCLConfig.cmake:275 (find_package)"
      - "/usr/lib/x86_64-linux-gnu/cmake/pcl/PCLConfig.cmake:324 (find_VTK)"
      - "/usr/lib/x86_64-linux-gnu/cmake/pcl/PCLConfig.cmake:570 (find_external_library)"
      - "/opt/ros/jazzy/share/pcl_conversions/cmake/ament_cmake_export_dependencies-extras.cmake:21 (find_package)"
      - "/opt/ros/jazzy/share/pcl_conversions/cmake/pcl_conversionsConfig.cmake:41 (include)"
      - "/opt/ros/jazzy/share/pcl_ros/cmake/ament_cmake_export_dependencies-extras.cmake:21 (find_package)"
      - "/opt/ros/jazzy/share/pcl_ros/cmake/pcl_rosConfig.cmake:41 (include)"
      - "CMakeLists.txt:19 (find_package)"
    description: "The MPI test test_mpi for C in mode normal"
    directories:
      source: "/home/student31/leo_ws/build/leo_lidar_sim/CMakeFiles/CMakeScratch/TryCompile-kn56qf"
      binary: "/home/student31/leo_ws/build/leo_lidar_sim/CMakeFiles/CMakeScratch/TryCompile-kn56qf"
    cmakeVariables:
      CMAKE_C_FLAGS: ""
      CMAKE_C_FLAGS_DEBUG: "-g"
      CMAKE_EXE_LINKER_FLAGS: ""
      CMAKE_MODULE_PATH: "/usr/lib/x86_64-linux-gnu/cmake/vtk-9.1/patches/99;/usr/lib/x86_64-linux-gnu/cmake/vtk-9.1;/opt/ros/jazzy/share/fastrtps_cmake_module/cmake/Modules;/usr/lib/x86_64-linux-gnu/cmake/pcl/Modules"
    buildResult:
      variable: "MPI_RESULT_C_test_mpi_normal"
      cached: true
      stdout: |
        Change Dir: '/home/student31/leo_ws/build/leo_lidar_sim/CMakeFiles/CMakeScratch/TryCompile-kn56qf'
        
        Run Build Command(s): /usr/bin/cmake -E env VERBOSE=1 /usr/bin/gmake -f Makefile cmTC_0cff3/fast
        /usr/bin/gmake  -f CMakeFiles/cmTC_0cff3.dir/build.make CMakeFiles/cmTC_0cff3.dir/build
        gmake[1]: Entering directory '/home/student31/leo_ws/build/leo_lidar_sim/CMakeFiles/CMakeScratch/TryCompile-kn56qf'
        Building C object CMakeFiles/cmTC_0cff3.dir/test_mpi.c.o
        /usr/bin/cc  -isystem /usr/lib/x86_64-linux-gnu/openmpi/include -isystem /usr/lib/x86_64-linux-gnu/openmpi/include/openmpi  -o CMakeFiles/cmTC_0cff3.dir/test_mpi.c.o -c /home/student31/leo_ws/build/leo_lidar_sim/CMakeFiles/CMakeScratch/TryCompile-kn56qf/test_mpi.c
        Linking C executable cmTC_0cff3
        /usr/bin/cmake -E cmake_link_script CMakeFiles/cmTC_0cff3.dir/link.txt --verbose=1
        /usr/bin/cc -L/usr/lib/x86_64-linux-gnu/openmpi/lib CMakeFiles/cmTC_0cff3.dir/test_mpi.c.o -o cmTC_0cff3  /usr/lib/x86_64-linux-gnu/libmpi.so 
        gmake[1]: Leaving directory '/home/student31/leo_ws/build/leo_lidar_sim/CMakeFiles/CMakeScratch/TryCompile-kn56qf'
        
      exitCode: 0

---
events:
  -
    kind: "try_compile-v1"
    backtrace:
      - "/usr/share/cmake-3.28/Modules/FindMPI.cmake:1278 (try_compile)"
      - "/usr/share/cmake-3.28/Modules/FindMPI.cmake:1322 (_MPI_try_staged_settings)"
      - "/usr/share/cmake-3.28/Modules/FindMPI.cmake:1645 (_MPI_check_lang_works)"
      - "/usr/lib/x86_64-linux-gnu/cmake/vtk-9.1/VTK-vtk-module-find-packages.cmake:397 (find_package)"
      - "/usr/lib/x86_64-linux-gnu/cmake/vtk-9.1/vtk-config.cmake:150 (include)"
      - "/usr/lib/x86_64-linux-gnu/cmake/pcl/PCLConfig.cmake:275 (find_package)"
      - "/usr/lib/x86_64-linux-gnu/cmake/pcl/PCLConfig.cmake:324 (find_VTK)"
      - "/usr/lib/x86_64-linux-gnu/cmake/pcl/PCLConfig.cmake:567 (find_external_library)"
      - "/opt/ros/jazzy/share/pcl_conversions/cmake/ament_cmake_export_dependencies-extras.cmake:21 (find_package)"
      - "/opt/ros/jazzy/share/pcl_conversions/cmake/pcl_conversionsConfig.cmake:41 (include)"
      - "/opt/ros/jazzy/share/pcl_ros/cmake/ament_cmake_export_dependencies-extras.cmake:21 (find_package)"
      - "/opt/ros/jazzy/share/pcl_ros/cmake/pcl_rosConfig.cmake:41 (include)"
      - "CMakeLists.txt:19 (find_package)"
    description: "The MPI test test_mpi for C in mode normal"
    directories:
      source: "/home/student31/leo_ws/build/leo_lidar_sim/CMakeFiles/CMakeScratch/TryCompile-nG5Qdz"
      binary: "/home/student31/leo_ws/build/leo_lidar_sim/CMakeFiles/CMakeScratch/TryCompile-nG5Qdz"
    cmakeVariables:
      CMAKE_C_FLAGS: ""
      CMAKE_C_FLAGS_DEBUG: "-g"
      CMAKE_EXE_LINKER_FLAGS: ""
      CMAKE_MODULE_PATH: "/usr/lib/x86_64-linux-gnu/cmake/vtk-9.1/patches/99;/usr/lib/x86_64-linux-gnu/cmake/vtk-9.1;/opt/ros/jazzy/share/fastrtps_cmake_module/cmake/Modules;/usr/lib/x86_64-linux-gnu/cmake/pcl/Modules"
    buildResult:
      variable: "MPI_RESULT_C_test_mpi_normal"
      cached: true
      stdout: |
        Change Dir: '/home/student31/leo_ws/build/leo_lidar_sim/CMakeFiles/CMakeScratch/TryCompile-nG5Qdz'
        
        Run Build Command(s): /usr/bin/cmake -E env VERBOSE=1 /usr/bin/gmake -f Makefile cmTC_800d2/fast
        gmake[1]: Entering directory '/home/student31/leo_ws/build/leo_lidar_sim/CMakeFiles/CMakeScratch/TryCompile-nG5Qdz'
        /usr/bin/gmake  -f CMakeFiles/cmTC_800d2.dir/build.make CMakeFiles/cmTC_800d2.dir/build
        gmake[2]: Entering directory '/home/student31/leo_ws/build/leo_lidar_sim/CMakeFiles/CMakeScratch/TryCompile-nG5Qdz'
        Building C object CMakeFiles/cmTC_800d2.dir/test_mpi.c.o
        /usr/bin/cc  -isystem /usr/lib/x86_64-linux-gnu/openmpi/include -isystem /usr/lib/x86_64-linux-gnu/openmpi/include/openmpi  -o CMakeFiles/cmTC_800d2.dir/test_mpi.c.o -c /home/student31/leo_ws/build/leo_lidar_sim/CMakeFiles/CMakeScratch/TryCompile-nG5Qdz/test_mpi.c
        Linking C executable cmTC_800d2
        /usr/bin/cmake -E cmake_link_script CMakeFiles/cmTC_800d2.dir/link.txt --verbose=1
        /usr/bin/cc -L/usr/lib/x86_64-linux-gnu/openmpi/lib CMakeFiles/cmTC_800d2.dir/test_mpi.c.o -o cmTC_800d2  /usr/lib/x86_64-linux-gnu/libmpi.so 
        gmake[2]: Leaving directory '/home/student31/leo_ws/build/leo_lidar_sim/CMakeFiles/CMakeScratch/TryCompile-nG5Qdz'
        gmake[1]: Leaving directory '/home/student31/leo_ws/build/leo_lidar_sim/CMakeFiles/CMakeScratch/TryCompile-nG5Qdz'
        
      exitCode: 0
  -
    kind: "try_compile-v1"
    backtrace:
      - "/usr/share/cmake-3.28/Modules/FindMPI.cmake:1278 (try_compile)"
      - "/usr/share/cmake-3.28/Modules/FindMPI.cmake:1322 (_MPI_try_staged_settings)"
      - "/usr/share/cmake-3.28/Modules/FindMPI.cmake:1645 (_MPI_check_lang_works)"
      - "/usr/lib/x86_64-linux-gnu/cmake/vtk-9.1/VTK-vtk-module-find-packages.cmake:397 (find_package)"
      - "/usr/lib/x86_64-linux-gnu/cmake/vtk-9.1/vtk-config.cmake:150 (include)"
      - "/usr/lib/x86_64-linux-gnu/cmake/pcl/PCLConfig.cmake:275 (find_package)"
      - "/usr/lib/x86_64-linux-gnu/cmake/pcl/PCLConfig.cmake:324 (find_VTK)"
      - "/usr/lib/x86_64-linux-gnu/cmake/pcl/PCLConfig.cmake:567 (find_external_library)"
      - "/opt/ros/jazzy/share/pcl_conversions/cmake/ament_cmake_export_dependencies-extras.cmake:21 (find_package)"
      - "/opt/ros/jazzy/share/pcl_conversions/cmake/pcl_conversionsConfig.cmake:41 (include)"
      - "/opt/ros/jazzy/share/pcl_ros/cmake/ament_cmake_export_dependencies-extras.cmake:21 (find_package)"
      - "/opt/ros/jazzy/share/pcl_ros/cmake/pcl_rosConfig.cmake:41 (include)"
      - "CMakeLists.txt:19 (find_package)"
    description: "The MPI test test_mpi for C in mode normal"
    directories:
      source: "/home/student31/leo_ws/build/leo_lidar_sim/CMakeFiles/CMakeScratch/TryCompile-7LCtMM"
      binary: "/home/student31/leo_ws/build/leo_lidar_sim/CMakeFiles/CMakeScratch/TryCompile-7LCtMM"
    cmakeVariables:
      CMAKE_C_FLAGS: ""
      CMAKE_C_FLAGS_DEBUG: "-g"
      CMAKE_EXE_LINKER_FLAGS: ""
      CMAKE_MODULE_PATH: "/usr/lib/x86_64-linux-gnu/cmake/vtk-9.1/patches/99;/usr/lib/x86_64-linux-gnu/cmake/vtk-9.1;/opt/ros/jazzy/share/fastrtps_cmake_module/cmake/Modules;/usr/lib/x86_64-linux-gnu/cmake/pcl/Modules"
    buildResult:
      variable: "MPI_RESULT_C_test_mpi_normal"
      cached: true
      stdout: |
        Change Dir: '/home/student31/leo_ws/build/leo_lidar_sim/CMakeFiles/CMakeScratch/TryCompile-7LCtMM'
        
        Run Build Command(s): /usr/bin/cmake -E env VERBOSE=1 /usr/bin/gmake -f Makefile cmTC_c31fb/fast
        /usr/bin/gmake  -f CMakeFiles/cmTC_c31fb.dir/build.make CMakeFiles/cmTC_c31fb.dir/build
        gmake[1]: Entering directory '/home/student31/leo_ws/build/leo_lidar_sim/CMakeFiles/CMakeScratch/TryCompile-7LCtMM'
        Building C object CMakeFiles/cmTC_c31fb.dir/test_mpi.c.o
        /usr/bin/cc  -isystem /usr/lib/x86_64-linux-gnu/openmpi/include -isystem /usr/lib/x86_64-linux-gnu/openmpi/include/openmpi  -o CMakeFiles/cmTC_c31fb.dir/test_mpi.c.o -c /home/student31/leo_ws/build/leo_lidar_sim/CMakeFiles/CMakeScratch/TryCompile-7LCtMM/test_mpi.c
        Linking C executable cmTC_c31fb
        /usr/bin/cmake -E cmake_link_script CMakeFiles/cmTC_c31fb.dir/link.txt --verbose=1
        /usr/bin/cc -L/usr/lib/x86_64-linux-gnu/openmpi/lib CMakeFiles/cmTC_c31fb.dir/test_mpi.c.o -o cmTC_c31fb  /usr/lib/x86_64-linux-gnu/libmpi.so 
        gmake[1]: Leaving directory '/home/student31/leo_ws/build/leo_lidar_sim/CMakeFiles/CMakeScratch/TryCompile-7LCtMM'
        
      exitCode: 0
  -
    kind: "try_compile-v1"
    backtrace:
      - "/usr/share/cmake-3.28/Modules/FindMPI.cmake:1278 (try_compile)"
      - "/usr/share/cmake-3.28/Modules/FindMPI.cmake:1322 (_MPI_try_staged_settings)"
      - "/usr/share/cmake-3.28/Modules/FindMPI.cmake:1645 (_MPI_check_lang_works)"
      - "/usr/lib/x86_64-linux-gnu/cmake/vtk-9.1/VTK-vtk-module-find-packages.cmake:397 (find_package)"
      - "/usr/lib/x86_64-linux-gnu/cmake/vtk-9.1/vtk-config.cmake:150 (include)"
      - "/usr/lib/x86_64-linux-gnu/cmake/pcl/PCLConfig.cmake:275 (find_package)"
      - "/usr/lib/x86_64-linux-gnu/cmake/pcl/PCLConfig.cmake:324 (find_VTK)"
      - "/usr/lib/x86_64-linux-gnu/cmake/pcl/PCLConfig.cmake:567 (find_external_library)"
      - "/opt/ros/jazzy/share/pcl_conversions/cmake/ament_cmake_export_dependencies-extras.cmake:21 (find_package)"
      - "/opt/ros/jazzy/share/pcl_conversions/cmake/pcl_conversionsConfig.cmake:41 (include)"
      - "/opt/ros/jazzy/share/pcl_ros/cmake/ament_cmake_export_dependencies-extras.cmake:21 (find_package)"
      - "/opt/ros/jazzy/share/pcl_ros/cmake/pcl_rosConfig.cmake:41 (include)"
      - "CMakeLists.txt:19 (find_package)"
    description: "The MPI test test_mpi for C in mode normal"
    directories:
      source: "/home/student31/leo_ws/build/leo_lidar_sim/CMakeFiles/CMakeScratch/TryCompile-Xq416K"
      binary: "/home/student31/leo_ws/build/leo_lidar_sim/CMakeFiles/CMakeScratch/TryCompile-Xq416K"
    cmakeVariables:
      CMAKE_C_FLAGS: ""
      CMAKE_C_FLAGS_DEBUG: "-g"
      CMAKE_EXE_LINKER_FLAGS: ""
      CMAKE_MODULE_PATH: "/usr/lib/x86_64-linux-gnu/cmake/vtk-9.1/patches/99;/usr/lib/x86_64-linux-gnu/cmake/vtk-9.1;/opt/ros/jazzy/share/fastrtps_cmake_module/cmake/Modules;/usr/lib/x86_64-linux-gnu/cmake/pcl/Modules"
    buildResult:
      variable: "MPI_RESULT_C_test_mpi_normal"
      cached: true
      stdout: |
        Change Dir: '/home/student31/leo_ws/build/leo_lidar_sim/CMakeFiles/CMakeScratch/TryCompile-Xq416K'
        
        Run Build Command(s): /usr/bin/cmake -E env VERBOSE=1 /usr/bin/gmake -f Makefile cmTC_a0112/fast
        /usr/bin/gmake  -f CMakeFiles/cmTC_a0112.dir/build.make CMakeFiles/cmTC_a0112.dir/build
        gmake[1]: Entering directory '/home/student31/leo_ws/build/leo_lidar_sim/CMakeFiles/CMakeScratch/TryCompile-Xq416K'
        Building C object CMakeFiles/cmTC_a0112.dir/test_mpi.c.o
        /usr/bin/cc  -isystem /usr/lib/x86_64-linux-gnu/openmpi/include -isystem /usr/lib/x86_64-linux-gnu/openmpi/include/openmpi  -o CMakeFiles/cmTC_a0112.dir/test_mpi.c.o -c /home/student31/leo_ws/build/leo_lidar_sim/CMakeFiles/CMakeScratch/TryCompile-Xq416K/test_mpi.c
        Linking C executable cmTC_a0112
        /usr/bin/cmake -E cmake_link_script CMakeFiles/cmTC_a0112.dir/link.txt --verbose=1
        /usr/bin/cc -L/usr/lib/x86_64-linux-gnu/openmpi/lib CMakeFiles/cmTC_a0112.dir/test_mpi.c.o -o cmTC_a0112  /usr/lib/x86_64-linux-gnu/libmpi.so 
        gmake[1]: Leaving directory '/home/student31/leo_ws/build/leo_lidar_sim/CMakeFiles/CMakeScratch/TryCompile-Xq416K'
        
      exitCode: 0
  -
    kind: "try_compile-v1"
    backtrace:
      - "/usr/share/cmake-3.28/Modules/FindMPI.cmake:1278 (try_compile)"
      - "/usr/share/cmake-3.28/Modules/FindMPI.cmake:1322 (_MPI_try_staged_settings)"
      - "/usr/share/cmake-3.28/Modules/FindMPI.cmake:1645 (_MPI_check_lang_works)"
      - "/usr/lib/x86_64-linux-gnu/cmake/vtk-9.1/VTK-vtk-module-find-packages.cmake:397 (find_package)"
      - "/usr/lib/x86_64-linux-gnu/cmake/vtk-9.1/vtk-config.cmake:150 (include)"
      - "/usr/lib/x86_64-linux-gnu/cmake/pcl/PCLConfig.cmake:275 (find_package)"
      - "/usr/lib/x86_64-linux-gnu/cmake/pcl/PCLConfig.cmake:324 (find_VTK)"
      - "/usr/lib/x86_64-linux-gnu/cmake/pcl/PCLConfig.cmake:567 (find_external_library)"
      - "/opt/ros/jazzy/share/pcl_conversions/cmake/ament_cmake_export_dependencies-extras.cmake:21 (find_package)"
      - "/opt/ros/jazzy/share/pcl_conversions/cmake/pcl_conversionsConfig.cmake:41 (include)"
      - "/opt/ros/jazzy/share/pcl_ros/cmake/ament_cmake_export_dependencies-extras.cmake:21 (find_package)"
      - "/opt/ros/jazzy/share/pcl_ros/cmake/pcl_rosConfig.cmake:41 (include)"
      - "CMakeLists.txt:19 (find_package)"
    description: "The MPI test test_mpi for C in mode normal"
    directories:
      source: "/home/student31/leo_ws/build/leo_lidar_sim/CMakeFiles/CMakeScratch/TryCompile-kyNsfZ"
      binary: "/home/student31/leo_ws/build/leo_lidar_sim/CMakeFiles/CMakeScratch/TryCompile-kyNsfZ"
    cmakeVariables:
      CMAKE_C_FLAGS: ""
      CMAKE_C_FLAGS_DEBUG: "-g"
      CMAKE_EXE_LINKER_FLAGS: ""
      CMAKE_MODULE_PATH: "/usr/lib/x86_64-linux-gnu/cmake/vtk-9.1/patches/99;/usr/lib/x86_64-linux-gnu/cmake/vtk-9.1;/opt/ros/jazzy/share/fastrtps_cmake_module/cmake/Modules;/usr/lib/x86_64-linux-gnu/cmake/pcl/Modules"
    buildResult:
      variable: "MPI_RESULT_C_test_mpi_normal"
      cached: true
      stdout: |
        Change Dir: '/home/student31/leo_ws/build/leo_lidar_sim/CMakeFiles/CMakeScratch/TryCompile-kyNsfZ'
        
        Run Build Command(s): /usr/bin/cmake -E env VERBOSE=1 /usr/bin/gmake -f Makefile cmTC_e2a19/fast
        gmake[1]: Entering directory '/home/student31/leo_ws/build/leo_lidar_sim/CMakeFiles/CMakeScratch/TryCompile-kyNsfZ'
        /usr/bin/gmake  -f CMakeFiles/cmTC_e2a19.dir/build.make CMakeFiles/cmTC_e2a19.dir/build
        gmake[2]: Entering directory '/home/student31/leo_ws/build/leo_lidar_sim/CMakeFiles/CMakeScratch/TryCompile-kyNsfZ'
        Building C object CMakeFiles/cmTC_e2a19.dir/test_mpi.c.o
        /usr/bin/cc  -isystem /usr/lib/x86_64-linux-gnu/openmpi/include -isystem /usr/lib/x86_64-linux-gnu/openmpi/include/openmpi  -o CMakeFiles/cmTC_e2a19.dir/test_mpi.c.o -c /home/student31/leo_ws/build/leo_lidar_sim/CMakeFiles/CMakeScratch/TryCompile-kyNsfZ/test_mpi.c
        Linking C executable cmTC_e2a19
        /usr/bin/cmake -E cmake_link_script CMakeFiles/cmTC_e2a19.dir/link.txt --verbose=1
        /usr/bin/cc -L/usr/lib/x86_64-linux-gnu/openmpi/lib CMakeFiles/cmTC_e2a19.dir/test_mpi.c.o -o cmTC_e2a19  /usr/lib/x86_64-linux-gnu/libmpi.so 
        gmake[2]: Leaving directory '/home/student31/leo_ws/build/leo_lidar_sim/CMakeFiles/CMakeScratch/TryCompile-kyNsfZ'
        gmake[1]: Leaving directory '/home/student31/leo_ws/build/leo_lidar_sim/CMakeFiles/CMakeScratch/TryCompile-kyNsfZ'
        
      exitCode: 0
...
  -
    kind: "try_compile-v1"
    backtrace:
      - "/usr/share/cmake-3.28/Modules/FindMPI.cmake:1278 (try_compile)"
      - "/usr/share/cmake-3.28/Modules/FindMPI.cmake:1322 (_MPI_try_staged_settings)"
      - "/usr/share/cmake-3.28/Modules/FindMPI.cmake:1645 (_MPI_check_lang_works)"
      - "/usr/lib/x86_64-linux-gnu/cmake/vtk-9.1/VTK-vtk-module-find-packages.cmake:397 (find_package)"
      - "/usr/lib/x86_64-linux-gnu/cmake/vtk-9.1/vtk-config.cmake:150 (include)"
      - "/usr/lib/x86_64-linux-gnu/cmake/pcl/PCLConfig.cmake:275 (find_package)"
      - "/usr/lib/x86_64-linux-gnu/cmake/pcl/PCLConfig.cmake:324 (find_VTK)"
      - "/usr/lib/x86_64-linux-gnu/cmake/pcl/PCLConfig.cmake:570 (find_external_library)"
      - "/opt/ros/jazzy/share/pcl_conversions/cmake/ament_cmake_export_dependencies-extras.cmake:21 (find_package)"
      - "/opt/ros/jazzy/share/pcl_conversions/cmake/pcl_conversionsConfig.cmake:41 (include)"
      - "/opt/ros/jazzy/share/pcl_ros/cmake/ament_cmake_export_dependencies-extras.cmake:21 (find_package)"
      - "/opt/ros/jazzy/share/pcl_ros/cmake/pcl_rosConfig.cmake:41 (include)"
      - "CMakeLists.txt:19 (find_package)"
    description: "The MPI test test_mpi for C in mode normal"
    directories:
      source: "/home/student31/leo_ws/build/leo_lidar_sim/CMakeFiles/CMakeScratch/TryCompile-hqnXzX"
      binary: "/home/student31/leo_ws/build/leo_lidar_sim/CMakeFiles/CMakeScratch/TryCompile-hqnXzX"
    cmakeVariables:
      CMAKE_C_FLAGS: ""
      CMAKE_C_FLAGS_DEBUG: "-g"
      CMAKE_EXE_LINKER_FLAGS: ""
      CMAKE_MODULE_PATH: "/usr/lib/x86_64-linux-gnu/cmake/vtk-9.1/patches/99;/usr/lib/x86_64-linux-gnu/cmake/vtk-9.1;/opt/ros/jazzy/share/fastrtps_cmake_module/cmake/Modules;/usr/lib/x86_64-linux-gnu/cmake/pcl/Modules"
    buildResult:
      variable: "MPI_RESULT_C_test_mpi_normal"
      cached: true
      stdout: |
        Change Dir: '/home/student31/leo_ws/build/leo_lidar_sim/CMakeFiles/CMakeScratch/TryCompile-hqnXzX'
        
        Run Build Command(s): /usr/bin/cmake -E env VERBOSE=1 /usr/bin/gmake -f Makefile cmTC_76c46/fast
        gmake[1]: Entering directory '/home/student31/leo_ws/build/leo_lidar_sim/CMakeFiles/CMakeScratch/TryCompile-hqnXzX'
        /usr/bin/gmake  -f CMakeFiles/cmTC_76c46.dir/build.make CMakeFiles/cmTC_76c46.dir/build
        gmake[2]: Entering directory '/home/student31/leo_ws/build/leo_lidar_sim/CMakeFiles/CMakeScratch/TryCompile-hqnXzX'
        Building C object CMakeFiles/cmTC_76c46.dir/test_mpi.c.o
        /usr/bin/cc  -isystem /usr/lib/x86_64-linux-gnu/openmpi/include -isystem /usr/lib/x86_64-linux-gnu/openmpi/include/openmpi  -o CMakeFiles/cmTC_76c46.dir/test_mpi.c.o -c /home/student31/leo_ws/build/leo_lidar_sim/CMakeFiles/CMakeScratch/TryCompile-hqnXzX/test_mpi.c
        cc1: fatal error: /home/student31/leo_ws/build/leo_lidar_sim/CMakeFiles/CMakeScratch/TryCompile-hqnXzX/test_mpi.c: No such file or directory
        compilation terminated.
        gmake[2]: *** [CMakeFiles/cmTC_76c46.dir/build.make:78: CMakeFiles/cmTC_76c46.dir/test_mpi.c.o] Error 1
        gmake[2]: Leaving directory '/home/student31/leo_ws/build/leo_lidar_sim/CMakeFiles/CMakeScratch/TryCompile-hqnXzX'
        gmake[1]: *** [Makefile:127: cmTC_76c46/fast] Error 2
        gmake[1]: Leaving directory '/home/student31/leo_ws/build/leo_lidar_sim/CMakeFiles/CMakeScratch/TryCompile-hqnXzX'
        
      exitCode: 2
  -
    kind: "try_compile-v1"
    backtrace:
      - "/usr/share/cmake-3.28/Modules/FindMPI.cmake:1278 (try_compile)"
      - "/usr/share/cmake-3.28/Modules/FindMPI.cmake:1322 (_MPI_try_staged_settings)"
      - "/usr/share/cmake-3.28/Modules/FindMPI.cmake:1645 (_MPI_check_lang_works)"
      - "/usr/lib/x86_64-linux-gnu/cmake/vtk-9.1/VTK-vtk-module-find-packages.cmake:397 (find_package)"
      - "/usr/lib/x86_64-linux-gnu/cmake/vtk-9.1/vtk-config.cmake:150 (include)"
      - "/usr/lib/x86_64-linux-gnu/cmake/pcl/PCLConfig.cmake:275 (find_package)"
      - "/usr/lib/x86_64-linux-gnu/cmake/pcl/PCLConfig.cmake:324 (find_VTK)"
      - "/usr/lib/x86_64-linux-gnu/cmake/pcl/PCLConfig.cmake:567 (find_external_library)"
      - "/opt/ros/jazzy/share/pcl_conversions/cmake/ament_cmake_export_dependencies-extras.cmake:21 (find_package)"
      - "/opt/ros/jazzy/share/pcl_conversions/cmake/pcl_conversionsConfig.cmake:41 (include)"
      - "/opt/ros/jazzy/share/pcl_ros/cmake/ament_cmake_export_dependencies-extras.cmake:21 (find_package)"
      - "/opt/ros/jazzy/share/pcl_ros/cmake/pcl_rosConfig.cmake:41 (include)"
      - "CMakeLists.txt:19 (find_package)"
    description: "The MPI test test_mpi for C in mode normal"
    directories:
      source: "/home/student31/leo_ws/build/leo_lidar_sim/CMakeFiles/CMakeScratch/TryCompile-ThSzE4"
      binary: "/home/student31/leo_ws/build/leo_lidar_sim/CMakeFiles/CMakeScratch/TryCompile-ThSzE4"
    cmakeVariables:
      CMAKE_C_FLAGS: ""
      CMAKE_C_FLAGS_DEBUG: "-g"
      CMAKE_EXE_LINKER_FLAGS: ""
      CMAKE_MODULE_PATH: "/usr/lib/x86_64-linux-gnu/cmake/vtk-9.1/patches/99;/usr/lib/x86_64-linux-gnu/cmake/vtk-9.1;/opt/ros/jazzy/share/fastrtps_cmake_module/cmake/Modules;/usr/lib/x86_64-linux-gnu/cmake/pcl/Modules"
    buildResult:
      variable: "MPI_RESULT_C_test_mpi_normal"
      cached: true
      stdout: |
        Change Dir: '/home/student31/leo_ws/build/leo_lidar_sim/CMakeFiles/CMakeScratch/TryCompile-ThSzE4'
        
        Run Build Command(s): /usr/bin/cmake -E env VERBOSE=1 /usr/bin/gmake -f Makefile cmTC_aa4b6/fast
        gmake[1]: Entering directory '/home/student31/leo_ws/build/leo_lidar_sim/CMakeFiles/CMakeScratch/TryCompile-ThSzE4'
        /usr/bin/gmake  -f CMakeFiles/cmTC_aa4b6.dir/build.make CMakeFiles/cmTC_aa4b6.dir/build
        gmake[2]: Entering directory '/home/student31/leo_ws/build/leo_lidar_sim/CMakeFiles/CMakeScratch/TryCompile-ThSzE4'
        Building C object CMakeFiles/cmTC_aa4b6.dir/test_mpi.c.o
        /usr/bin/cc  -isystem /usr/lib/x86_64-linux-gnu/openmpi/include -isystem /usr/lib/x86_64-linux-gnu/openmpi/include/openmpi  -o CMakeFiles/cmTC_aa4b6.dir/test_mpi.c.o -c /home/student31/leo_ws/build/leo_lidar_sim/CMakeFiles/CMakeScratch/TryCompile-ThSzE4/test_mpi.c
        Linking C executable cmTC_aa4b6
        /usr/bin/cmake -E cmake_link_script CMakeFiles/cmTC_aa4b6.dir/link.txt --verbose=1
        /usr/bin/cc -L/usr/lib/x86_64-linux-gnu/openmpi/lib CMakeFiles/cmTC_aa4b6.dir/test_mpi.c.o -o cmTC_aa4b6  /usr/lib/x86_64-linux-gnu/libmpi.so 
        gmake[2]: Leaving directory '/home/student31/leo_ws/build/leo_lidar_sim/CMakeFiles/CMakeScratch/TryCompile-ThSzE4'
        gmake[1]: Leaving directory '/home/student31/leo_ws/build/leo_lidar_sim/CMakeFiles/CMakeScratch/TryCompile-ThSzE4'
        
      exitCode: 0

---
events:
  -
    kind: "try_compile-v1"
    backtrace:
      - "/usr/share/cmake-3.28/Modules/FindMPI.cmake:1278 (try_compile)"
      - "/usr/share/cmake-3.28/Modules/FindMPI.cmake:1322 (_MPI_try_staged_settings)"
      - "/usr/share/cmake-3.28/Modules/FindMPI.cmake:1645 (_MPI_check_lang_works)"
      - "/usr/lib/x86_64-linux-gnu/cmake/vtk-9.1/VTK-vtk-module-find-packages.cmake:397 (find_package)"
      - "/usr/lib/x86_64-linux-gnu/cmake/vtk-9.1/vtk-config.cmake:150 (include)"
      - "/usr/lib/x86_64-linux-gnu/cmake/pcl/PCLConfig.cmake:275 (find_package)"
      - "/usr/lib/x86_64-linux-gnu/cmake/pcl/PCLConfig.cmake:324 (find_VTK)"
      - "/usr/lib/x86_64-linux-gnu/cmake/pcl/PCLConfig.cmake:567 (find_external_library)"
      - "/opt/ros/jazzy/share/pcl_conversions/cmake/ament_cmake_export_dependencies-extras.cmake:21 (find_package)"
      - "/opt/ros/jazzy/share/pcl_conversions/cmake/pcl_conversionsConfig.cmake:41 (include)"
      - "/opt/ros/jazzy/share/pcl_ros/cmake/ament_cmake_export_dependencies-extras.cmake:21 (find_package)"
      - "/opt/ros/jazzy/share/pcl_ros/cmake/pcl_rosConfig.cmake:41 (include)"
      - "CMakeLists.txt:19 (find_package)"
    description: "The MPI test test_mpi for C in mode normal"
    directories:
      source: "/home/student31/leo_ws/build/leo_lidar_sim/CMakeFiles/CMakeScratch/TryCompile-rOFEld"
      binary: "/home/student31/leo_ws/build/leo_lidar_sim/CMakeFiles/CMakeScratch/TryCompile-rOFEld"
    cmakeVariables:
      CMAKE_C_FLAGS: ""
      CMAKE_C_FLAGS_DEBUG: "-g"
      CMAKE_EXE_LINKER_FLAGS: ""
      CMAKE_MODULE_PATH: "/usr/lib/x86_64-linux-gnu/cmake/vtk-9.1/patches/99;/usr/lib/x86_64-linux-gnu/cmake/vtk-9.1;/opt/ros/jazzy/share/fastrtps_cmake_module/cmake/Modules;/usr/lib/x86_64-linux-gnu/cmake/pcl/Modules"
    buildResult:
      variable: "MPI_RESULT_C_test_mpi_normal"
      cached: true
      stdout: |
        Change Dir: '/home/student31/leo_ws/build/leo_lidar_sim/CMakeFiles/CMakeScratch/TryCompile-rOFEld'
        
        Run Build Command(s): /usr/bin/cmake -E env VERBOSE=1 /usr/bin/gmake -f Makefile cmTC_a8a4f/fast
        gmake[1]: Entering directory '/home/student31/leo_ws/build/leo_lidar_sim/CMakeFiles/CMakeScratch/TryCompile-rOFEld'
        /usr/bin/gmake  -f CMakeFiles/cmTC_a8a4f.dir/build.make CMakeFiles/cmTC_a8a4f.dir/build
        gmake[2]: Entering directory '/home/student31/leo_ws/build/leo_lidar_sim/CMakeFiles/CMakeScratch/TryCompile-rOFEld'
        Building C object CMakeFiles/cmTC_a8a4f.dir/test_mpi.c.o
        /usr/bin/cc  -isystem /usr/lib/x86_64-linux-gnu/openmpi/include -isystem /usr/lib/x86_64-linux-gnu/openmpi/include/openmpi  -o CMakeFiles/cmTC_a8a4f.dir/test_mpi.c.o -c /home/student31/leo_ws/build/leo_lidar_sim/CMakeFiles/CMakeScratch/TryCompile-rOFEld/test_mpi.c
        Linking C executable cmTC_a8a4f
        /usr/bin/cmake -E cmake_link_script CMakeFiles/cmTC_a8a4f.dir/link.txt --verbose=1
        /usr/bin/cc -L/usr/lib/x86_64-linux-gnu/openmpi/lib CMakeFiles/cmTC_a8a4f.dir/test_mpi.c.o -o cmTC_a8a4f  /usr/lib/x86_64-linux-gnu/libmpi.so 
        gmake[2]: Leaving directory '/home/student31/leo_ws/build/leo_lidar_sim/CMakeFiles/CMakeScratch/TryCompile-rOFEld'
        gmake[1]: Leaving directory '/home/student31/leo_ws/build/leo_lidar_sim/CMakeFiles/CMakeScratch/TryCompile-rOFEld'
        
      exitCode: 0
  -
    kind: "try_compile-v1"
    backtrace:
      - "/usr/share/cmake-3.28/Modules/FindMPI.cmake:1278 (try_compile)"
      - "/usr/share/cmake-3.28/Modules/FindMPI.cmake:1322 (_MPI_try_staged_settings)"
      - "/usr/share/cmake-3.28/Modules/FindMPI.cmake:1645 (_MPI_check_lang_works)"
      - "/usr/lib/x86_64-linux-gnu/cmake/vtk-9.1/VTK-vtk-module-find-packages.cmake:397 (find_package)"
      - "/usr/lib/x86_64-linux-gnu/cmake/vtk-9.1/vtk-config.cmake:150 (include)"
      - "/usr/lib/x86_64-linux-gnu/cmake/pcl/PCLConfig.cmake:275 (find_package)"
      - "/usr/lib/x86_64-linux-gnu/cmake/pcl/PCLConfig.cmake:324 (find_VTK)"
      - "/usr/lib/x86_64-linux-gnu/cmake/pcl/PCLConfig.cmake:567 (find_external_library)"
      - "/opt/ros/jazzy/share/pcl_conversions/cmake/ament_cmake_export_dependencies-extras.cmake:21 (find_package)"
      - "/opt/ros/jazzy/share/pcl_conversions/cmake/pcl_conversionsConfig.cmake:41 (include)"
      - "/opt/ros/jazzy/share/pcl_ros/cmake/ament_cmake_export_dependencies-extras.cmake:21 (find_package)"
      - "/opt/ros/jazzy/share/pcl_ros/cmake/pcl_rosConfig.cmake:41 (include)"
      - "CMakeLists.txt:19 (find_package)"
    description: "The MPI test test_mpi for C in mode normal"
    directories:
      source: "/home/student31/leo_ws/build/leo_lidar_sim/CMakeFiles/CMakeScratch/TryCompile-zDRi2R"
      binary: "/home/student31/leo_ws/build/leo_lidar_sim/CMakeFiles/CMakeScratch/TryCompile-zDRi2R"
    cmakeVariables:
      CMAKE_C_FLAGS: ""
      CMAKE_C_FLAGS_DEBUG: "-g"
      CMAKE_EXE_LINKER_FLAGS: ""
      CMAKE_MODULE_PATH: "/usr/lib/x86_64-linux-gnu/cmake/vtk-9.1/patches/99;/usr/lib/x86_64-linux-gnu/cmake/vtk-9.1;/opt/ros/jazzy/share/fastrtps_cmake_module/cmake/Modules;/usr/lib/x86_64-linux-gnu/cmake/pcl/Modules"
    buildResult:
      variable: "MPI_RESULT_C_test_mpi_normal"
      cached: true
      stdout: |
        Change Dir: '/home/student31/leo_ws/build/leo_lidar_sim/CMakeFiles/CMakeScratch/TryCompile-zDRi2R'
        
        Run Build Command(s): /usr/bin/cmake -E env VERBOSE=1 /usr/bin/gmake -f Makefile cmTC_d97ec/fast
        gmake[1]: Entering directory '/home/student31/leo_ws/build/leo_lidar_sim/CMakeFiles/CMakeScratch/TryCompile-zDRi2R'
        /usr/bin/gmake  -f CMakeFiles/cmTC_d97ec.dir/build.make CMakeFiles/cmTC_d97ec.dir/build
        gmake[2]: Entering directory '/home/student31/leo_ws/build/leo_lidar_sim/CMakeFiles/CMakeScratch/TryCompile-zDRi2R'
        Building C object CMakeFiles/cmTC_d97ec.dir/test_mpi.c.o
        /usr/bin/cc  -isystem /usr/lib/x86_64-linux-gnu/openmpi/include -isystem /usr/lib/x86_64-linux-gnu/openmpi/include/openmpi  -o CMakeFiles/cmTC_d97ec.dir/test_mpi.c.o -c /home/student31/leo_ws/build/leo_lidar_sim/CMakeFiles/CMakeScratch/TryCompile-zDRi2R/test_mpi.c
        Linking C executable cmTC_d97ec
        /usr/bin/cmake -E cmake_link_script CMakeFiles/cmTC_d97ec.dir/link.txt --verbose=1
        /usr/bin/cc -L/usr/lib/x86_64-linux-gnu/openmpi/lib CMakeFiles/cmTC_d97ec.dir/test_mpi.c.o -o cmTC_d97ec  /usr/lib/x86_64-linux-gnu/libmpi.so 
        gmake[2]: Leaving directory '/home/student31/leo_ws/build/leo_lidar_sim/CMakeFiles/CMakeScratch/TryCompile-zDRi2R'
        gmake[1]: Leaving directory '/home/student31/leo_ws/build/leo_lidar_sim/CMakeFiles/CMakeScratch/TryCompile-zDRi2R'
        
      exitCode: 0
  -
    kind: "try_compile-v1"
    backtrace:
      - "/usr/share/cmake-3.28/Modules/FindMPI.cmake:1278 (try_compile)"
      - "/usr/share/cmake-3.28/Modules/FindMPI.cmake:1322 (_MPI_try_staged_settings)"
      - "/usr/share/cmake-3.28/Modules/FindMPI.cmake:1645 (_MPI_check_lang_works)"
      - "/usr/lib/x86_64-linux-gnu/cmake/vtk-9.1/VTK-vtk-module-find-packages.cmake:397 (find_package)"
      - "/usr/lib/x86_64-linux-gnu/cmake/vtk-9.1/vtk-config.cmake:150 (include)"
      - "/usr/lib/x86_64-linux-gnu/cmake/pcl/PCLConfig.cmake:275 (find_package)"
      - "/usr/lib/x86_64-linux-gnu/cmake/pcl/PCLConfig.cmake:324 (find_VTK)"
      - "/usr/lib/x86_64-linux-gnu/cmake/pcl/PCLConfig.cmake:567 (find_external_library)"
      - "/opt/ros/jazzy/share/pcl_conversions/cmake/ament_cmake_export_dependencies-extras.cmake:21 (find_package)"
      - "/opt/ros/jazzy/share/pcl_conversions/cmake/pcl_conversionsConfig.cmake:41 (include)"
      - "/opt/ros/jazzy/share/pcl_ros/cmake/ament_cmake_export_dependencies-extras.cmake:21 (find_package)"
      - "/opt/ros/jazzy/share/pcl_ros/cmake/pcl_rosConfig.cmake:41 (include)"
      - "CMakeLists.txt:19 (find_package)"
    description: "The MPI test test_mpi for C in mode normal"
    directories:
      source: "/home/student31/leo_ws/build/leo_lidar_sim/CMakeFiles/CMakeScratch/TryCompile-UvUUbK"
      binary: "/home/student31/leo_ws/build/leo_lidar_sim/CMakeFiles/CMakeScratch/TryCompile-UvUUbK"
    cmakeVariables:
      CMAKE_C_FLAGS: ""
      CMAKE_C_FLAGS_DEBUG: "-g"
      CMAKE_EXE_LINKER_FLAGS: ""
      CMAKE_MODULE_PATH: "/usr/lib/x86_64-linux-gnu/cmake/vtk-9.1/patches/99;/usr/lib/x86_64-linux-gnu/cmake/vtk-9.1;/opt/ros/jazzy/share/fastrtps_cmake_module/cmake/Modules;/usr/lib/x86_64-linux-gnu/cmake/pcl/Modules"
    buildResult:
      variable: "MPI_RESULT_C_test_mpi_normal"
      cached: true
      stdout: |
        Change Dir: '/home/student31/leo_ws/build/leo_lidar_sim/CMakeFiles/CMakeScratch/TryCompile-UvUUbK'
        
        Run Build Command(s): /usr/bin/cmake -E env VERBOSE=1 /usr/bin/gmake -f Makefile cmTC_835c0/fast
        gmake[1]: Entering directory '/home/student31/leo_ws/build/leo_lidar_sim/CMakeFiles/CMakeScratch/TryCompile-UvUUbK'
        /usr/bin/gmake  -f CMakeFiles/cmTC_835c0.dir/build.make CMakeFiles/cmTC_835c0.dir/build
        gmake[2]: Entering directory '/home/student31/leo_ws/build/leo_lidar_sim/CMakeFiles/CMakeScratch/TryCompile-UvUUbK'
        Building C object CMakeFiles/cmTC_835c0.dir/test_mpi.c.o
        /usr/bin/cc  -isystem /usr/lib/x86_64-linux-gnu/openmpi/include -isystem /usr/lib/x86_64-linux-gnu/openmpi/include/openmpi  -o CMakeFiles/cmTC_835c0.dir/test_mpi.c.o -c /home/student31/leo_ws/build/leo_lidar_sim/CMakeFiles/CMakeScratch/TryCompile-UvUUbK/test_mpi.c
        Linking C executable cmTC_835c0
        /usr/bin/cmake -E cmake_link_script CMakeFiles/cmTC_835c0.dir/link.txt --verbose=1
        /usr/bin/cc -L/usr/lib/x86_64-linux-gnu/openmpi/lib CMakeFiles/cmTC_835c0.dir/test_mpi.c.o -o cmTC_835c0  /usr/lib/x86_64-linux-gnu/libmpi.so 
        gmake[2]: Leaving directory '/home/student31/leo_ws/build/leo_lidar_sim/CMakeFiles/CMakeScratch/TryCompile-UvUUbK'
        gmake[1]: Leaving directory '/home/student31/leo_ws/build/leo_lidar_sim/CMakeFiles/CMakeScratch/TryCompile-UvUUbK'
        
      exitCode: 0
...
  -
    kind: "try_compile-v1"
    backtrace:
      - "/usr/share/cmake-3.28/Modules/FindMPI.cmake:1278 (try_compile)"
      - "/usr/share/cmake-3.28/Modules/FindMPI.cmake:1322 (_MPI_try_staged_settings)"
      - "/usr/share/cmake-3.28/Modules/FindMPI.cmake:1645 (_MPI_check_lang_works)"
      - "/usr/lib/x86_64-linux-gnu/cmake/vtk-9.1/VTK-vtk-module-find-packages.cmake:397 (find_package)"
      - "/usr/lib/x86_64-linux-gnu/cmake/vtk-9.1/vtk-config.cmake:150 (include)"
      - "/usr/lib/x86_64-linux-gnu/cmake/pcl/PCLConfig.cmake:275 (find_package)"
      - "/usr/lib/x86_64-linux-gnu/cmake/pcl/PCLConfig.cmake:324 (find_VTK)"
      - "/usr/lib/x86_64-linux-gnu/cmake/pcl/PCLConfig.cmake:570 (find_external_library)"
      - "/opt/ros/jazzy/share/pcl_conversions/cmake/ament_cmake_export_dependencies-extras.cmake:21 (find_package)"
      - "/opt/ros/jazzy/share/pcl_conversions/cmake/pcl_conversionsConfig.cmake:41 (include)"
      - "/opt/ros/jazzy/share/pcl_ros/cmake/ament_cmake_export_dependencies-extras.cmake:21 (find_package)"
      - "/opt/ros/jazzy/share/pcl_ros/cmake/pcl_rosConfig.cmake:41 (include)"
      - "CMakeLists.txt:19 (find_package)"
    description: "The MPI test test_mpi for C in mode normal"
    directories:
      source: "/home/student31/leo_ws/build/leo_lidar_sim/CMakeFiles/CMakeScratch/TryCompile-Ygumj8"
      binary: "/home/student31/leo_ws/build/leo_lidar_sim/CMakeFiles/CMakeScratch/TryCompile-Ygumj8"
    cmakeVariables:
      CMAKE_C_FLAGS: ""
      CMAKE_C_FLAGS_DEBUG: "-g"
      CMAKE_EXE_LINKER_FLAGS: ""
      CMAKE_MODULE_PATH: "/usr/lib/x86_64-linux-gnu/cmake/vtk-9.1/patches/99;/usr/lib/x86_64-linux-gnu/cmake/vtk-9.1;/opt/ros/jazzy/share/fastrtps_cmake_module/cmake/Modules;/usr/lib/x86_64-linux-gnu/cmake/pcl/Modules"
    buildResult:
      variable: "MPI_RESULT_C_test_mpi_normal"
      cached: true
      stdout: |
        Change Dir: '/home/student31/leo_ws/build/leo_lidar_sim/CMakeFiles/CMakeScratch/TryCompile-Ygumj8'
        
        Run Build Command(s): /usr/bin/cmake -E env VERBOSE=1 /usr/bin/gmake -f Makefile cmTC_d0ea9/fast
        gmake[1]: Entering directory '/home/student31/leo_ws/build/leo_lidar_sim/CMakeFiles/CMakeScratch/TryCompile-Ygumj8'
        /usr/bin/gmake  -f CMakeFiles/cmTC_d0ea9.dir/build.make CMakeFiles/cmTC_d0ea9.dir/build
        gmake[2]: Entering directory '/home/student31/leo_ws/build/leo_lidar_sim/CMakeFiles/CMakeScratch/TryCompile-Ygumj8'
        Building C object CMakeFiles/cmTC_d0ea9.dir/test_mpi.c.o
        /usr/bin/cc  -isystem /usr/lib/x86_64-linux-gnu/openmpi/include -isystem /usr/lib/x86_64-linux-gnu/openmpi/include/openmpi  -o CMakeFiles/cmTC_d0ea9.dir/test_mpi.c.o -c /home/student31/leo_ws/build/leo_lidar_sim/CMakeFiles/CMakeScratch/TryCompile-Ygumj8/test_mpi.c
        Linking C executable cmTC_d0ea9
        /usr/bin/cmake -E cmake_link_script CMakeFiles/cmTC_d0ea9.dir/link.txt --verbose=1
        /usr/bin/cc -L/usr/lib/x86_64-linux-gnu/openmpi/lib CMakeFiles/cmTC_d0ea9.dir/test_mpi.c.o -o cmTC_d0ea9  /usr/lib/x86_64-linux-gnu/libmpi.so 
        gmake[2]: Leaving directory '/home/student31/leo_ws/build/leo_lidar_sim/CMakeFiles/CMakeScratch/TryCompile-Ygumj8'
        gmake[1]: Leaving directory '/home/student31/leo_ws/build/leo_lidar_sim/CMakeFiles/CMakeScratch/TryCompile-Ygumj8'
        
      exitCode: 0

---
events:
  -
    kind: "try_compile-v1"
    backtrace:
      - "/usr/share/cmake-3.28/Modules/FindMPI.cmake:1278 (try_compile)"
      - "/usr/share/cmake-3.28/Modules/FindMPI.cmake:1322 (_MPI_try_staged_settings)"
      - "/usr/share/cmake-3.28/Modules/FindMPI.cmake:1645 (_MPI_check_lang_works)"
      - "/usr/lib/x86_64-linux-gnu/cmake/vtk-9.1/VTK-vtk-module-find-packages.cmake:397 (find_package)"
      - "/usr/lib/x86_64-linux-gnu/cmake/vtk-9.1/vtk-config.cmake:150 (include)"
      - "/usr/lib/x86_64-linux-gnu/cmake/pcl/PCLConfig.cmake:275 (find_package)"
      - "/usr/lib/x86_64-linux-gnu/cmake/pcl/PCLConfig.cmake:324 (find_VTK)"
      - "/usr/lib/x86_64-linux-gnu/cmake/pcl/PCLConfig.cmake:567 (find_external_library)"
      - "/opt/ros/jazzy/share/pcl_conversions/cmake/ament_cmake_export_dependencies-extras.cmake:21 (find_package)"
      - "/opt/ros/jazzy/share/pcl_conversions/cmake/pcl_conversionsConfig.cmake:41 (include)"
      - "/opt/ros/jazzy/share/pcl_ros/cmake/ament_cmake_export_dependencies-extras.cmake:21 (find_package)"
      - "/opt/ros/jazzy/share/pcl_ros/cmake/pcl_rosConfig.cmake:41 (include)"
      - "CMakeLists.txt:19 (find_package)"
    description: "The MPI test test_mpi for C in mode normal"
    directories:
      source: "/home/student31/leo_ws/build/leo_lidar_sim/CMakeFiles/CMakeScratch/TryCompile-iNlkT6"
      binary: "/home/student31/leo_ws/build/leo_lidar_sim/CMakeFiles/CMakeScratch/TryCompile-iNlkT6"
    cmakeVariables:
      CMAKE_C_FLAGS: ""
      CMAKE_C_FLAGS_DEBUG: "-g"
      CMAKE_EXE_LINKER_FLAGS: ""
      CMAKE_MODULE_PATH: "/usr/lib/x86_64-linux-gnu/cmake/vtk-9.1/patches/99;/usr/lib/x86_64-linux-gnu/cmake/vtk-9.1;/opt/ros/jazzy/share/fastrtps_cmake_module/cmake/Modules;/usr/lib/x86_64-linux-gnu/cmake/pcl/Modules"
    buildResult:
      variable: "MPI_RESULT_C_test_mpi_normal"
      cached: true
      stdout: |
        Change Dir: '/home/student31/leo_ws/build/leo_lidar_sim/CMakeFiles/CMakeScratch/TryCompile-iNlkT6'
        
        Run Build Command(s): /usr/bin/cmake -E env VERBOSE=1 /usr/bin/gmake -f Makefile cmTC_56b57/fast
        gmake[1]: Entering directory '/home/student31/leo_ws/build/leo_lidar_sim/CMakeFiles/CMakeScratch/TryCompile-iNlkT6'
        /usr/bin/gmake  -f CMakeFiles/cmTC_56b57.dir/build.make CMakeFiles/cmTC_56b57.dir/build
        gmake[2]: Entering directory '/home/student31/leo_ws/build/leo_lidar_sim/CMakeFiles/CMakeScratch/TryCompile-iNlkT6'
        Building C object CMakeFiles/cmTC_56b57.dir/test_mpi.c.o
        /usr/bin/cc  -isystem /usr/lib/x86_64-linux-gnu/openmpi/include -isystem /usr/lib/x86_64-linux-gnu/openmpi/include/openmpi  -o CMakeFiles/cmTC_56b57.dir/test_mpi.c.o -c /home/student31/leo_ws/build/leo_lidar_sim/CMakeFiles/CMakeScratch/TryCompile-iNlkT6/test_mpi.c
        Linking C executable cmTC_56b57
        /usr/bin/cmake -E cmake_link_script CMakeFiles/cmTC_56b57.dir/link.txt --verbose=1
        /usr/bin/cc -L/usr/lib/x86_64-linux-gnu/openmpi/lib CMakeFiles/cmTC_56b57.dir/test_mpi.c.o -o cmTC_56b57  /usr/lib/x86_64-linux-gnu/libmpi.so 
        gmake[2]: Leaving directory '/home/student31/leo_ws/build/leo_lidar_sim/CMakeFiles/CMakeScratch/TryCompile-iNlkT6'
        gmake[1]: Leaving directory '/home/student31/leo_ws/build/leo_lidar_sim/CMakeFiles/CMakeScratch/TryCompile-iNlkT6'
        
      exitCode: 0
  -
    kind: "try_compile-v1"
    backtrace:
      - "/usr/share/cmake-3.28/Modules/FindMPI.cmake:1278 (try_compile)"
      - "/usr/share/cmake-3.28/Modules/FindMPI.cmake:1322 (_MPI_try_staged_settings)"
      - "/usr/share/cmake-3.28/Modules/FindMPI.cmake:1645 (_MPI_check_lang_works)"
      - "/usr/lib/x86_64-linux-gnu/cmake/vtk-9.1/VTK-vtk-module-find-packages.cmake:397 (find_package)"
      - "/usr/lib/x86_64-linux-gnu/cmake/vtk-9.1/vtk-config.cmake:150 (include)"
      - "/usr/lib/x86_64-linux-gnu/cmake/pcl/PCLConfig.cmake:275 (find_package)"
      - "/usr/lib/x86_64-linux-gnu/cmake/pcl/PCLConfig.cmake:324 (find_VTK)"
      - "/usr/lib/x86_64-linux-gnu/cmake/pcl/PCLConfig.cmake:567 (find_external_library)"
      - "/opt/ros/jazzy/share/pcl_conversions/cmake/ament_cmake_export_dependencies-extras.cmake:21 (find_package)"
      - "/opt/ros/jazzy/share/pcl_conversions/cmake/pcl_conversionsConfig.cmake:41 (include)"
      - "/opt/ros/jazzy/share/pcl_ros/cmake/ament_cmake_export_dependencies-extras.cmake:21 (find_package)"
      - "/opt/ros/jazzy/share/pcl_ros/cmake/pcl_rosConfig.cmake:41 (include)"
      - "CMakeLists.txt:19 (find_package)"
    description: "The MPI test test_mpi for C in mode normal"
    directories:
      source: "/home/student31/leo_ws/build/leo_lidar_sim/CMakeFiles/CMakeScratch/TryCompile-7qbqlN"
      binary: "/home/student31/leo_ws/build/leo_lidar_sim/CMakeFiles/CMakeScratch/TryCompile-7qbqlN"
    cmakeVariables:
      CMAKE_C_FLAGS: ""
      CMAKE_C_FLAGS_DEBUG: "-g"
      CMAKE_EXE_LINKER_FLAGS: ""
      CMAKE_MODULE_PATH: "/usr/lib/x86_64-linux-gnu/cmake/vtk-9.1/patches/99;/usr/lib/x86_64-linux-gnu/cmake/vtk-9.1;/opt/ros/jazzy/share/fastrtps_cmake_module/cmake/Modules;/usr/lib/x86_64-linux-gnu/cmake/pcl/Modules"
    buildResult:
      variable: "MPI_RESULT_C_test_mpi_normal"
      cached: true
      stdout: |
        Change Dir: '/home/student31/leo_ws/build/leo_lidar_sim/CMakeFiles/CMakeScratch/TryCompile-7qbqlN'
        
        Run Build Command(s): /usr/bin/cmake -E env VERBOSE=1 /usr/bin/gmake -f Makefile cmTC_3f87f/fast
        gmake[1]: Entering directory '/home/student31/leo_ws/build/leo_lidar_sim/CMakeFiles/CMakeScratch/TryCompile-7qbqlN'
        /usr/bin/gmake  -f CMakeFiles/cmTC_3f87f.dir/build.make CMakeFiles/cmTC_3f87f.dir/build
        gmake[2]: Entering directory '/home/student31/leo_ws/build/leo_lidar_sim/CMakeFiles/CMakeScratch/TryCompile-7qbqlN'
        Building C object CMakeFiles/cmTC_3f87f.dir/test_mpi.c.o
        /usr/bin/cc  -isystem /usr/lib/x86_64-linux-gnu/openmpi/include -isystem /usr/lib/x86_64-linux-gnu/openmpi/include/openmpi  -o CMakeFiles/cmTC_3f87f.dir/test_mpi.c.o -c /home/student31/leo_ws/build/leo_lidar_sim/CMakeFiles/CMakeScratch/TryCompile-7qbqlN/test_mpi.c
        Linking C executable cmTC_3f87f
        /usr/bin/cmake -E cmake_link_script CMakeFiles/cmTC_3f87f.dir/link.txt --verbose=1
        /usr/bin/cc -L/usr/lib/x86_64-linux-gnu/openmpi/lib CMakeFiles/cmTC_3f87f.dir/test_mpi.c.o -o cmTC_3f87f  /usr/lib/x86_64-linux-gnu/libmpi.so 
        gmake[2]: Leaving directory '/home/student31/leo_ws/build/leo_lidar_sim/CMakeFiles/CMakeScratch/TryCompile-7qbqlN'
        gmake[1]: Leaving directory '/home/student31/leo_ws/build/leo_lidar_sim/CMakeFiles/CMakeScratch/TryCompile-7qbqlN'
        
      exitCode: 0
  -
    kind: "try_compile-v1"
    backtrace:
      - "/usr/share/cmake-3.28/Modules/FindMPI.cmake:1278 (try_compile)"
      - "/usr/share/cmake-3.28/Modules/FindMPI.cmake:1322 (_MPI_try_staged_settings)"
      - "/usr/share/cmake-3.28/Modules/FindMPI.cmake:1645 (_MPI_check_lang_works)"
      - "/usr/lib/x86_64-linux-gnu/cmake/vtk-9.1/VTK-vtk-module-find-packages.cmake:397 (find_package)"
      - "/usr/lib/x86_64-linux-gnu/cmake/vtk-9.1/vtk-config.cmake:150 (include)"
      - "/usr/lib/x86_64-linux-gnu/cmake/pcl/PCLConfig.cmake:275 (find_package)"
      - "/usr/lib/x86_64-linux-gnu/cmake/pcl/PCLConfig.cmake:324 (find_VTK)"
      - "/usr/lib/x86_64-linux-gnu/cmake/pcl/PCLConfig.cmake:567 (find_external_library)"
      - "/opt/ros/jazzy/share/pcl_conversions/cmake/ament_cmake_export_dependencies-extras.cmake:21 (find_package)"
      - "/opt/ros/jazzy/share/pcl_conversions/cmake/pcl_conversionsConfig.cmake:41 (include)"
      - "/opt/ros/jazzy/share/pcl_ros/cmake/ament_cmake_export_dependencies-extras.cmake:21 (find_package)"
      - "/opt/ros/jazzy/share/pcl_ros/cmake/pcl_rosConfig.cmake:41 (include)"
      - "CMakeLists.txt:19 (find_package)"
    description: "The MPI test test_mpi for C in mode normal"
    directories:
      source: "/home/student31/leo_ws/build/leo_lidar_sim/CMakeFiles/CMakeScratch/TryCompile-wNQW36"
      binary: "/home/student31/leo_ws/build/leo_lidar_sim/CMakeFiles/CMakeScratch/TryCompile-wNQW36"
    cmakeVariables:
      CMAKE_C_FLAGS: ""
      CMAKE_C_FLAGS_DEBUG: "-g"
      CMAKE_EXE_LINKER_FLAGS: ""
      CMAKE_MODULE_PATH: "/usr/lib/x86_64-linux-gnu/cmake/vtk-9.1/patches/99;/usr/lib/x86_64-linux-gnu/cmake/vtk-9.1;/opt/ros/jazzy/share/fastrtps_cmake_module/cmake/Modules;/usr/lib/x86_64-linux-gnu/cmake/pcl/Modules"
    buildResult:
      variable: "MPI_RESULT_C_test_mpi_normal"
      cached: true
      stdout: |
        Change Dir: '/home/student31/leo_ws/build/leo_lidar_sim/CMakeFiles/CMakeScratch/TryCompile-wNQW36'
        
        Run Build Command(s): /usr/bin/cmake -E env VERBOSE=1 /usr/bin/gmake -f Makefile cmTC_4ff59/fast
        gmake[1]: Entering directory '/home/student31/leo_ws/build/leo_lidar_sim/CMakeFiles/CMakeScratch/TryCompile-wNQW36'
        /usr/bin/gmake  -f CMakeFiles/cmTC_4ff59.dir/build.make CMakeFiles/cmTC_4ff59.dir/build
        gmake[2]: Entering directory '/home/student31/leo_ws/build/leo_lidar_sim/CMakeFiles/CMakeScratch/TryCompile-wNQW36'
        Building C object CMakeFiles/cmTC_4ff59.dir/test_mpi.c.o
        /usr/bin/cc  -isystem /usr/lib/x86_64-linux-gnu/openmpi/include -isystem /usr/lib/x86_64-linux-gnu/openmpi/include/openmpi  -o CMakeFiles/cmTC_4ff59.dir/test_mpi.c.o -c /home/student31/leo_ws/build/leo_lidar_sim/CMakeFiles/CMakeScratch/TryCompile-wNQW36/test_mpi.c
        Linking C executable cmTC_4ff59
        /usr/bin/cmake -E cmake_link_script CMakeFiles/cmTC_4ff59.dir/link.txt --verbose=1
        /usr/bin/cc -L/usr/lib/x86_64-linux-gnu/openmpi/lib CMakeFiles/cmTC_4ff59.dir/test_mpi.c.o -o cmTC_4ff59  /usr/lib/x86_64-linux-gnu/libmpi.so 
        gmake[2]: Leaving directory '/home/student31/leo_ws/build/leo_lidar_sim/CMakeFiles/CMakeScratch/TryCompile-wNQW36'
        gmake[1]: Leaving directory '/home/student31/leo_ws/build/leo_lidar_sim/CMakeFiles/CMakeScratch/TryCompile-wNQW36'
        
      exitCode: 0
  -
    kind: "try_compile-v1"
    backtrace:
      - "/usr/share/cmake-3.28/Modules/FindMPI.cmake:1278 (try_compile)"
      - "/usr/share/cmake-3.28/Modules/FindMPI.cmake:1322 (_MPI_try_staged_settings)"
      - "/usr/share/cmake-3.28/Modules/FindMPI.cmake:1645 (_MPI_check_lang_works)"
      - "/usr/lib/x86_64-linux-gnu/cmake/vtk-9.1/VTK-vtk-module-find-packages.cmake:397 (find_package)"
      - "/usr/lib/x86_64-linux-gnu/cmake/vtk-9.1/vtk-config.cmake:150 (include)"
      - "/usr/lib/x86_64-linux-gnu/cmake/pcl/PCLConfig.cmake:275 (find_package)"
      - "/usr/lib/x86_64-linux-gnu/cmake/pcl/PCLConfig.cmake:324 (find_VTK)"
      - "/usr/lib/x86_64-linux-gnu/cmake/pcl/PCLConfig.cmake:567 (find_external_library)"
      - "/opt/ros/jazzy/share/pcl_conversions/cmake/ament_cmake_export_dependencies-extras.cmake:21 (find_package)"
      - "/opt/ros/jazzy/share/pcl_conversions/cmake/pcl_conversionsConfig.cmake:41 (include)"
      - "/opt/ros/jazzy/share/pcl_ros/cmake/ament_cmake_export_dependencies-extras.cmake:21 (find_package)"
      - "/opt/ros/jazzy/share/pcl_ros/cmake/pcl_rosConfig.cmake:41 (include)"
      - "CMakeLists.txt:19 (find_package)"
    description: "The MPI test test_mpi for C in mode normal"
    directories:
      source: "/home/student31/leo_ws/build/leo_lidar_sim/CMakeFiles/CMakeScratch/TryCompile-baDdly"
      binary: "/home/student31/leo_ws/build/leo_lidar_sim/CMakeFiles/CMakeScratch/TryCompile-baDdly"
    cmakeVariables:
      CMAKE_C_FLAGS: ""
      CMAKE_C_FLAGS_DEBUG: "-g"
      CMAKE_EXE_LINKER_FLAGS: ""
      CMAKE_MODULE_PATH: "/usr/lib/x86_64-linux-gnu/cmake/vtk-9.1/patches/99;/usr/lib/x86_64-linux-gnu/cmake/vtk-9.1;/opt/ros/jazzy/share/fastrtps_cmake_module/cmake/Modules;/usr/lib/x86_64-linux-gnu/cmake/pcl/Modules"
    buildResult:
      variable: "MPI_RESULT_C_test_mpi_normal"
      cached: true
      stdout: |
        Change Dir: '/home/student31/leo_ws/build/leo_lidar_sim/CMakeFiles/CMakeScratch/TryCompile-baDdly'
        
        Run Build Command(s): /usr/bin/cmake -E env VERBOSE=1 /usr/bin/gmake -f Makefile cmTC_b902b/fast
        gmake[1]: Entering directory '/home/student31/leo_ws/build/leo_lidar_sim/CMakeFiles/CMakeScratch/TryCompile-baDdly'
        /usr/bin/gmake  -f CMakeFiles/cmTC_b902b.dir/build.make CMakeFiles/cmTC_b902b.dir/build
        gmake[2]: Entering directory '/home/student31/leo_ws/build/leo_lidar_sim/CMakeFiles/CMakeScratch/TryCompile-baDdly'
        Building C object CMakeFiles/cmTC_b902b.dir/test_mpi.c.o
        /usr/bin/cc  -isystem /usr/lib/x86_64-linux-gnu/openmpi/include -isystem /usr/lib/x86_64-linux-gnu/openmpi/include/openmpi  -o CMakeFiles/cmTC_b902b.dir/test_mpi.c.o -c /home/student31/leo_ws/build/leo_lidar_sim/CMakeFiles/CMakeScratch/TryCompile-baDdly/test_mpi.c
        Linking C executable cmTC_b902b
        /usr/bin/cmake -E cmake_link_script CMakeFiles/cmTC_b902b.dir/link.txt --verbose=1
        /usr/bin/cc -L/usr/lib/x86_64-linux-gnu/openmpi/lib CMakeFiles/cmTC_b902b.dir/test_mpi.c.o -o cmTC_b902b  /usr/lib/x86_64-linux-gnu/libmpi.so 
        gmake[2]: Leaving directory '/home/student31/leo_ws/build/leo_lidar_sim/CMakeFiles/CMakeScratch/TryCompile-baDdly'
        gmake[1]: Leaving directory '/home/student31/leo_ws/build/leo_lidar_sim/CMakeFiles/CMakeScratch/TryCompile-baDdly'
        
      exitCode: 0
  -
    kind: "try_compile-v1"
    backtrace:
      - "/usr/share/cmake-3.28/Modules/FindMPI.cmake:1278 (try_compile)"
      - "/usr/share/cmake-3.28/Modules/FindMPI.cmake:1322 (_MPI_try_staged_settings)"
      - "/usr/share/cmake-3.28/Modules/FindMPI.cmake:1645 (_MPI_check_lang_works)"
      - "/usr/lib/x86_64-linux-gnu/cmake/vtk-9.1/VTK-vtk-module-find-packages.cmake:397 (find_package)"
      - "/usr/lib/x86_64-linux-gnu/cmake/vtk-9.1/vtk-config.cmake:150 (include)"
      - "/usr/lib/x86_64-linux-gnu/cmake/pcl/PCLConfig.cmake:275 (find_package)"
      - "/usr/lib/x86_64-linux-gnu/cmake/pcl/PCLConfig.cmake:324 (find_VTK)"
      - "/usr/lib/x86_64-linux-gnu/cmake/pcl/PCLConfig.cmake:570 (find_external_library)"
      - "/opt/ros/jazzy/share/pcl_conversions/cmake/ament_cmake_export_dependencies-extras.cmake:21 (find_package)"
      - "/opt/ros/jazzy/share/pcl_conversions/cmake/pcl_conversionsConfig.cmake:41 (include)"
      - "/opt/ros/jazzy/share/pcl_ros/cmake/ament_cmake_export_dependencies-extras.cmake:21 (find_package)"
      - "/opt/ros/jazzy/share/pcl_ros/cmake/pcl_rosConfig.cmake:41 (include)"
      - "CMakeLists.txt:19 (find_package)"
    description: "The MPI test test_mpi for C in mode normal"
    directories:
      source: "/home/student31/leo_ws/build/leo_lidar_sim/CMakeFiles/CMakeScratch/TryCompile-52nKVC"
      binary: "/home/student31/leo_ws/build/leo_lidar_sim/CMakeFiles/CMakeScratch/TryCompile-52nKVC"
    cmakeVariables:
      CMAKE_C_FLAGS: ""
      CMAKE_C_FLAGS_DEBUG: "-g"
      CMAKE_EXE_LINKER_FLAGS: ""
      CMAKE_MODULE_PATH: "/usr/lib/x86_64-linux-gnu/cmake/vtk-9.1/patches/99;/usr/lib/x86_64-linux-gnu/cmake/vtk-9.1;/opt/ros/jazzy/share/fastrtps_cmake_module/cmake/Modules;/usr/lib/x86_64-linux-gnu/cmake/pcl/Modules"
    buildResult:
      variable: "MPI_RESULT_C_test_mpi_normal"
      cached: true
      stdout: |
        Change Dir: '/home/student31/leo_ws/build/leo_lidar_sim/CMakeFiles/CMakeScratch/TryCompile-52nKVC'
        
        Run Build Command(s): /usr/bin/cmake -E env VERBOSE=1 /usr/bin/gmake -f Makefile cmTC_c6e4c/fast
        gmake[1]: Entering directory '/home/student31/leo_ws/build/leo_lidar_sim/CMakeFiles/CMakeScratch/TryCompile-52nKVC'
        /usr/bin/gmake  -f CMakeFiles/cmTC_c6e4c.dir/build.make CMakeFiles/cmTC_c6e4c.dir/build
        gmake[2]: Entering directory '/home/student31/leo_ws/build/leo_lidar_sim/CMakeFiles/CMakeScratch/TryCompile-52nKVC'
        Building C object CMakeFiles/cmTC_c6e4c.dir/test_mpi.c.o
        /usr/bin/cc  -isystem /usr/lib/x86_64-linux-gnu/openmpi/include -isystem /usr/lib/x86_64-linux-gnu/openmpi/include/openmpi  -o CMakeFiles/cmTC_c6e4c.dir/test_mpi.c.o -c /home/student31/leo_ws/build/leo_lidar_sim/CMakeFiles/CMakeScratch/TryCompile-52nKVC/test_mpi.c
        Linking C executable cmTC_c6e4c
        /usr/bin/cmake -E cmake_link_script CMakeFiles/cmTC_c6e4c.dir/link.txt --verbose=1
        /usr/bin/cc -L/usr/lib/x86_64-linux-gnu/openmpi/lib CMakeFiles/cmTC_c6e4c.dir/test_mpi.c.o -o cmTC_c6e4c  /usr/lib/x86_64-linux-gnu/libmpi.so 
        gmake[2]: Leaving directory '/home/student31/leo_ws/build/leo_lidar_sim/CMakeFiles/CMakeScratch/TryCompile-52nKVC'
        gmake[1]: Leaving directory '/home/student31/leo_ws/build/leo_lidar_sim/CMakeFiles/CMakeScratch/TryCompile-52nKVC'
        
      exitCode: 0
...
  -
    kind: "try_compile-v1"
    backtrace:
      - "/usr/share/cmake-3.28/Modules/FindMPI.cmake:1278 (try_compile)"
      - "/usr/share/cmake-3.28/Modules/FindMPI.cmake:1322 (_MPI_try_staged_settings)"
      - "/usr/share/cmake-3.28/Modules/FindMPI.cmake:1645 (_MPI_check_lang_works)"
      - "/usr/lib/x86_64-linux-gnu/cmake/vtk-9.1/VTK-vtk-module-find-packages.cmake:397 (find_package)"
      - "/usr/lib/x86_64-linux-gnu/cmake/vtk-9.1/vtk-config.cmake:150 (include)"
      - "/usr/lib/x86_64-linux-gnu/cmake/pcl/PCLConfig.cmake:275 (find_package)"
      - "/usr/lib/x86_64-linux-gnu/cmake/pcl/PCLConfig.cmake:324 (find_VTK)"
      - "/usr/lib/x86_64-linux-gnu/cmake/pcl/PCLConfig.cmake:567 (find_external_library)"
      - "/opt/ros/jazzy/share/pcl_conversions/cmake/ament_cmake_export_dependencies-extras.cmake:21 (find_package)"
      - "/opt/ros/jazzy/share/pcl_conversions/cmake/pcl_conversionsConfig.cmake:41 (include)"
      - "/opt/ros/jazzy/share/pcl_ros/cmake/ament_cmake_export_dependencies-extras.cmake:21 (find_package)"
      - "/opt/ros/jazzy/share/pcl_ros/cmake/pcl_rosConfig.cmake:41 (include)"
      - "CMakeLists.txt:19 (find_package)"
    description: "The MPI test test_mpi for C in mode normal"
    directories:
      source: "/home/student31/leo_ws/build/leo_lidar_sim/CMakeFiles/CMakeScratch/TryCompile-NF88sT"
      binary: "/home/student31/leo_ws/build/leo_lidar_sim/CMakeFiles/CMakeScratch/TryCompile-NF88sT"
    cmakeVariables:
      CMAKE_C_FLAGS: ""
      CMAKE_C_FLAGS_DEBUG: "-g"
      CMAKE_EXE_LINKER_FLAGS: ""
      CMAKE_MODULE_PATH: "/usr/lib/x86_64-linux-gnu/cmake/vtk-9.1/patches/99;/usr/lib/x86_64-linux-gnu/cmake/vtk-9.1;/opt/ros/jazzy/share/fastrtps_cmake_module/cmake/Modules;/usr/lib/x86_64-linux-gnu/cmake/pcl/Modules"
    buildResult:
      variable: "MPI_RESULT_C_test_mpi_normal"
      cached: true
      stdout: |
        Change Dir: '/home/student31/leo_ws/build/leo_lidar_sim/CMakeFiles/CMakeScratch/TryCompile-NF88sT'
        
        Run Build Command(s): /usr/bin/cmake -E env VERBOSE=1 /usr/bin/gmake -f Makefile cmTC_ff507/fast
        gmake[1]: Entering directory '/home/student31/leo_ws/build/leo_lidar_sim/CMakeFiles/CMakeScratch/TryCompile-NF88sT'
        /usr/bin/gmake  -f CMakeFiles/cmTC_ff507.dir/build.make CMakeFiles/cmTC_ff507.dir/build
        gmake[2]: Entering directory '/home/student31/leo_ws/build/leo_lidar_sim/CMakeFiles/CMakeScratch/TryCompile-NF88sT'
        Building C object CMakeFiles/cmTC_ff507.dir/test_mpi.c.o
        /usr/bin/cc  -isystem /usr/lib/x86_64-linux-gnu/openmpi/include -isystem /usr/lib/x86_64-linux-gnu/openmpi/include/openmpi  -o CMakeFiles/cmTC_ff507.dir/test_mpi.c.o -c /home/student31/leo_ws/build/leo_lidar_sim/CMakeFiles/CMakeScratch/TryCompile-NF88sT/test_mpi.c
        Linking C executable cmTC_ff507
        /usr/bin/cmake -E cmake_link_script CMakeFiles/cmTC_ff507.dir/link.txt --verbose=1
        /usr/bin/cc -L/usr/lib/x86_64-linux-gnu/openmpi/lib CMakeFiles/cmTC_ff507.dir/test_mpi.c.o -o cmTC_ff507  /usr/lib/x86_64-linux-gnu/libmpi.so 
        gmake[2]: Leaving directory '/home/student31/leo_ws/build/leo_lidar_sim/CMakeFiles/CMakeScratch/TryCompile-NF88sT'
        gmake[1]: Leaving directory '/home/student31/leo_ws/build/leo_lidar_sim/CMakeFiles/CMakeScratch/TryCompile-NF88sT'
        
      exitCode: 0
  -
    kind: "try_compile-v1"
    backtrace:
      - "/usr/share/cmake-3.28/Modules/FindMPI.cmake:1278 (try_compile)"
      - "/usr/share/cmake-3.28/Modules/FindMPI.cmake:1322 (_MPI_try_staged_settings)"
      - "/usr/share/cmake-3.28/Modules/FindMPI.cmake:1645 (_MPI_check_lang_works)"
      - "/usr/lib/x86_64-linux-gnu/cmake/vtk-9.1/VTK-vtk-module-find-packages.cmake:397 (find_package)"
      - "/usr/lib/x86_64-linux-gnu/cmake/vtk-9.1/vtk-config.cmake:150 (include)"
      - "/usr/lib/x86_64-linux-gnu/cmake/pcl/PCLConfig.cmake:275 (find_package)"
      - "/usr/lib/x86_64-linux-gnu/cmake/pcl/PCLConfig.cmake:324 (find_VTK)"
      - "/usr/lib/x86_64-linux-gnu/cmake/pcl/PCLConfig.cmake:567 (find_external_library)"
      - "/opt/ros/jazzy/share/pcl_conversions/cmake/ament_cmake_export_dependencies-extras.cmake:21 (find_package)"
      - "/opt/ros/jazzy/share/pcl_conversions/cmake/pcl_conversionsConfig.cmake:41 (include)"
      - "/opt/ros/jazzy/share/pcl_ros/cmake/ament_cmake_export_dependencies-extras.cmake:21 (find_package)"
      - "/opt/ros/jazzy/share/pcl_ros/cmake/pcl_rosConfig.cmake:41 (include)"
      - "CMakeLists.txt:19 (find_package)"
    description: "The MPI test test_mpi for C in mode normal"
    directories:
      source: "/home/student31/leo_ws/build/leo_lidar_sim/CMakeFiles/CMakeScratch/TryCompile-mEXlhO"
      binary: "/home/student31/leo_ws/build/leo_lidar_sim/CMakeFiles/CMakeScratch/TryCompile-mEXlhO"
    cmakeVariables:
      CMAKE_C_FLAGS: ""
      CMAKE_C_FLAGS_DEBUG: "-g"
      CMAKE_EXE_LINKER_FLAGS: ""
      CMAKE_MODULE_PATH: "/usr/lib/x86_64-linux-gnu/cmake/vtk-9.1/patches/99;/usr/lib/x86_64-linux-gnu/cmake/vtk-9.1;/opt/ros/jazzy/share/fastrtps_cmake_module/cmake/Modules;/usr/lib/x86_64-linux-gnu/cmake/pcl/Modules"
    buildResult:
      variable: "MPI_RESULT_C_test_mpi_normal"
      cached: true
      stdout: |
        Change Dir: '/home/student31/leo_ws/build/leo_lidar_sim/CMakeFiles/CMakeScratch/TryCompile-mEXlhO'
        
        Run Build Command(s): /usr/bin/cmake -E env VERBOSE=1 /usr/bin/gmake -f Makefile cmTC_6f173/fast
        gmake[1]: Entering directory '/home/student31/leo_ws/build/leo_lidar_sim/CMakeFiles/CMakeScratch/TryCompile-mEXlhO'
        /usr/bin/gmake  -f CMakeFiles/cmTC_6f173.dir/build.make CMakeFiles/cmTC_6f173.dir/build
        gmake[2]: Entering directory '/home/student31/leo_ws/build/leo_lidar_sim/CMakeFiles/CMakeScratch/TryCompile-mEXlhO'
        Building C object CMakeFiles/cmTC_6f173.dir/test_mpi.c.o
        /usr/bin/cc  -isystem /usr/lib/x86_64-linux-gnu/openmpi/include -isystem /usr/lib/x86_64-linux-gnu/openmpi/include/openmpi  -o CMakeFiles/cmTC_6f173.dir/test_mpi.c.o -c /home/student31/leo_ws/build/leo_lidar_sim/CMakeFiles/CMakeScratch/TryCompile-mEXlhO/test_mpi.c
        Linking C executable cmTC_6f173
        /usr/bin/cmake -E cmake_link_script CMakeFiles/cmTC_6f173.dir/link.txt --verbose=1
        /usr/bin/cc -L/usr/lib/x86_64-linux-gnu/openmpi/lib CMakeFiles/cmTC_6f173.dir/test_mpi.c.o -o cmTC_6f173  /usr/lib/x86_64-linux-gnu/libmpi.so 
        gmake[2]: Leaving directory '/home/student31/leo_ws/build/leo_lidar_sim/CMakeFiles/CMakeScratch/TryCompile-mEXlhO'
        gmake[1]: Leaving directory '/home/student31/leo_ws/build/leo_lidar_sim/CMakeFiles/CMakeScratch/TryCompile-mEXlhO'
        
      exitCode: 0
...

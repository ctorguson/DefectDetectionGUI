# MatlabEnvironmentForGUI
cd /Users/ciaratorguson/point_processing/build

export DYLD_LIBRARY_PATH=/Applications/MATLAB_R2023b.app/bin/maci64:/Applications/MATLAB_R2023b.app/extern/bin/maci64:$DYLD_LIBRARY_PATH

echo $DYLD_LIBRARY_PATH

otool -L /Users/ciaratorguson/point_processing/build/PointProcessing.mexmaci64

install_name_tool -change @rpath/libMatlabEngine.dylib /Applications/MATLAB_R2023b.app/extern/bin/maci64/libMatlabEngine.dylib /Users/ciaratorguson/point_processing/build/PointProcessing.mexmaci64

arch -x86_64 /Applications/MATLAB_R2023b.app/bin/matlab &

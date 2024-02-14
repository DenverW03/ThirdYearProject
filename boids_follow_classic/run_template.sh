# Ensure that the version number is correct of the fltk library will fail to be found
export DYLD_LIBRARY_PATH="/opt/homebrew/Cellar/fltk/HEAD-ecb3e40/lib:$DYLD_LIBRARY_PATH"
export DYLD_LIBRARY_PATH="/usr/local/lib:$DYLD_LIBRARY_PATH"
export STAGE_HEADER=/usr/local/include/Stage-4.3;
export STAGEPATH=/usr/local/lib;
# This line is to be replaced in generation of run.sh file
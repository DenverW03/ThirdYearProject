export DYLD_LIBRARY_PATH=/opt/homebrew/Cellar/fltk/HEAD-4964a15_1/lib;
export DYLD_LIBRARY_PATH=/usr/local/lib;
export STAGE_HEADER=/usr/local/include/Stage-4.3;
export STAGEPATH=/usr/local/lib;
g++ main.cpp -o main -I /opt/homebrew/Cellar/fltk/HEAD-4964a15_1/include -L /opt/homebrew/Cellar/fltk/HEAD-4964a15_1/lib -l fltk -I /usr/local/include/Stage-4.3/ -L /usr/local/lib/ -l stage.4.3.0;
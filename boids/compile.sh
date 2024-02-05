# fltk version has to match version installed on device otherwise dependency will not be sourced
export DYLD_LIBRARY_PATH=/opt/homebrew/Cellar/fltk/HEAD-ecb3e40/lib;
export DYLD_LIBRARY_PATH=/usr/local/lib;
export STAGE_HEADER=/usr/local/include/Stage-4.3;
export STAGEPATH=/usr/local/lib;
g++ -std=c++11 main.cpp ./bot/SimpleRobot.cpp -o main -I /opt/homebrew/Cellar/fltk/HEAD-ecb3e40/include -L /opt/homebrew/Cellar/fltk/HEAD-ecb3e40/lib -l fltk -I /usr/local/include/Stage-4.3/ -L /usr/local/lib/ -l stage.4.3.0;
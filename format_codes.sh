export ARTISTIC_STYLE_OPTIONS=.astylerc

SOURCES_PATH='./MAIN'

find $SOURCES_PATH -name '*.c' -exec astyle {} \;
find $SOURCES_PATH -name '*.h' -exec astyle {} \;

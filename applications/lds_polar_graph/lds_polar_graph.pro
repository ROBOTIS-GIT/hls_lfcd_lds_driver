QT += charts

HEADERS += \
    lds_polar_graph.h

SOURCES += \
    main.cpp \
    lds_polar_graph.cpp

unix {
    LIBS += -lboost_system
}

win32 {
    INCLUDEPATH += C:/boost/
    LIBS += -lws2_32
    LIBS += C:/boost/stage/lib/libboost_system-mgw53-mt-sd-x32-1_66.a
}

macx {
    INCLUDEPATH += /usr/local/Cellar/boost/1.66.0/include
    LIBS += -L/usr/local/Cellar/boost/1.66.0/lib -lboost_system
}

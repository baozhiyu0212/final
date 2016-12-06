INCLUDE = $(shell pkg-config --cflags opencv)
LIBS = $(shell pkg-config --libs opencv)
SOURCES = webcam.cpp
    # 目标文件
OBJECTS = $(SOURCES:.cpp=.o)
    # 可执行文件
TARGET = webcam
$(TARGET):$(OBJECTS)
		g++ -o $(TARGET) $(OBJECTS) -I $(INCLUDE) $(LIBS)
$(OBJECTS):$(SOURCES)
		g++ -c $(SOURCES)
clean:
		rm $(OBJECTS) $(TARGET)
    # 编译规则 $@代表目标文件 [        DISCUZ_CODE_272        ]lt; 代表第一个依赖文件
%.o:%.cpp
		g++ -I $(INCLUDE) -o $[url=home.php?mod=space&uid=72445]@[/url] -c [        DISCUZ_CODE_272        ]lt;

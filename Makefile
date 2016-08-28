CXX = g++

PREFIX ?= /usr/local
HEADER = Astar.hpp
DEMO = astar

demo : $(TARGET)
	$(CXX) -std=c++14 -fPIC demo.cpp -o $(DEMO)
    
install :
	install -m 644 $(HEADER) $(PREFIX)/include

uninstall :
	rm -f $(PREFIX)/include/$(HEADER)

clean :
	-rm -f $(DEMO)
